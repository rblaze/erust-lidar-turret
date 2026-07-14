use core::cell::Cell;

use async_scheduler::sync::mailbox::Mailbox;
use critical_section::Mutex;
use firmware::error::Error;
use rtt_target::debug_rprintln;
use stm32g0_hal::gpio::Alternate;
use stm32g0_hal::gpio::gpiob::{PB10, PB11};
use stm32g0_hal::pac::dma1::CH;
use stm32g0_hal::pac::{DMA1, DMAMUX, Interrupt, NVIC, USART3, interrupt};
use stm32g0_hal::rcc::{Rcc, ResetEnable};

use crate::lidar_reader::DISTANCE_QUEUE;

pub struct HostUsart<'a> {
    tx_dma: &'a CH,
}

impl<'a> HostUsart<'a> {
    pub fn new(
        _usart_tx: PB10<Alternate<4>>,
        _usart_rx: PB11<Alternate<4>>,
        usart: USART3,
        dma: &'a DMA1,
        dmamux: &DMAMUX,
        rcc: &Rcc,
    ) -> Self {
        let tx_dma = dma.ch2();
        Self::init_usart(&usart, tx_dma, dmamux, rcc);

        Self { tx_dma }
    }

    fn init_usart(usart: &USART3, tx_dma: &CH, dmamux: &DMAMUX, rcc: &Rcc) {
        #[allow(unsafe_code)]
        unsafe {
            tx_dma
                .par()
                .write(|w| w.pa().bits(core::ptr::from_ref(usart.tdr()) as u32));
            critical_section::with(|cs| {
                let ptr = HOST_TX_BUFFER.borrow(cs).as_ptr();
                tx_dma.mar().write(|w| w.ma().bits(ptr as u32));
            });
        }
        tx_dma.cr().write(|w| {
            w.mem2mem()
                .disabled()
                .msize()
                .bits8()
                .minc()
                .enabled()
                .psize()
                .bits32()
                .pinc()
                .disabled()
                .circ()
                .disabled()
                .dir()
                .from_memory()
                .htie()
                .disabled()
                .tcie()
                .enabled()
        });

        #[allow(unsafe_code)]
        unsafe {
            // USART3 TX DMA request id is 55
            dmamux.ccr(1).write(|w| w.dmareq_id().bits(55));
        }

        // USART for host communication
        USART3::enable(rcc);
        USART3::reset(rcc);

        // Set 8 bit word length, no parity
        usart
            .cr1()
            .write(|w| w.m1().clear_bit().m0().clear_bit().pce().disabled());
        // Set 1 stop bit
        usart.cr2().write(|w| w.stop().stop1());
        // Set bit rate to 230400. Assume usart_presc=1, USART running at MCU clock.
        let usart_div = rcc.sysclk().to_Hz() / 230400;
        usart.brr().write(|w| w.brr().set(usart_div as u16));
        // Swap TX/RX - breakout board has TX and RX swapped on debug connector.
        usart.cr2().modify(|_, w| w.swap().set_bit());
        // Enable USART
        usart.cr1().modify(|_, w| w.ue().enabled());
        // Enable TX DMA
        usart.cr3().write(|w| w.dmat().enabled());
        // Enable transmitter.
        usart.cr1().modify(|_, w| w.te().enabled());

        // Enable interrupts
        #[allow(unsafe_code)]
        unsafe {
            NVIC::unmask(Interrupt::DMA1_CHANNEL2_3);
        }
    }

    pub async fn task(&self) -> Result<(), Error> {
        loop {
            critical_section::with(|cs| {
                if let Some(value) = DISTANCE_QUEUE.borrow_ref_mut(cs).read_for_host_usart() {
                    self.write(value)?;
                }
                Ok::<(), Error>(())
            })?;

            HOST_USART_EVENT.read().await?;
        }
    }

    fn write(&self, value: u16) -> Result<(), Error> {
        let btrem = self.tx_dma.ndtr().read().ndt().bits();
        if self.tx_dma.ndtr().read().ndt() != 0 {
            debug_rprintln!("host tx dma busy, ndtr={}", btrem);
            return Err(Error::DeviceBusy);
        }

        // Disable DMA channel to allow reconfiguration.
        self.tx_dma.cr().modify(|_, w| w.en().disabled());

        critical_section::with(|cs| {
            HOST_TX_BUFFER.borrow(cs).set(value.to_le_bytes());
        });

        self.tx_dma.ndtr().write(|w| w.ndt().set(2));
        self.tx_dma.cr().modify(|_, w| w.en().enabled());

        Ok(())
    }
}

const TX_BUFFER_SIZE: usize = 2;
static HOST_TX_BUFFER: Mutex<Cell<[u8; TX_BUFFER_SIZE]>> = Mutex::new(Cell::new([0; _]));
pub static HOST_USART_EVENT: Mailbox<()> = Mailbox::new();

#[interrupt]
fn DMA1_CHANNEL2_3() {
    #[allow(unsafe_code)]
    let dma = unsafe { DMA1::steal() };

    if dma.isr().read().tcif2().is_complete() {
        dma.ifcr().write(|w| w.ctcif2().clear());
        HOST_USART_EVENT.post(());
    }
}
