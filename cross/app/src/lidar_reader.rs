use core::cell::Cell;

use async_scheduler::sync::mailbox::Mailbox;
use critical_section::Mutex;
use rtt_target::debug_rprintln;
use stm32g0_hal::gpio::Alternate;
use stm32g0_hal::gpio::gpioa::{PA2, PA3};
use stm32g0_hal::pac::dma1::CH;
use stm32g0_hal::pac::{DMA1, DMAMUX, Interrupt, NVIC, USART2, interrupt};
use stm32g0_hal::rcc::{Rcc, ResetEnable};

use firmware::error::Error;

pub struct LidarReader {}

impl LidarReader {
    pub fn new(
        _usart_tx: PA2<Alternate<1>>,
        _usart_rx: PA3<Alternate<1>>,
        usart: USART2,
        rx_dma: &CH,
        dmamux: &DMAMUX,
        rcc: &Rcc,
    ) -> Self {
        Self::init_usart(usart, rx_dma, dmamux, rcc);

        Self {}
    }

    fn init_usart(usart: USART2, rx_dma: &CH, dmamux: &DMAMUX,rcc: &Rcc) {
        #[allow(unsafe_code)]
        unsafe {
            rx_dma
                .par()
                .write(|w| w.pa().bits(usart.rdr() as *const _ as u32));
            critical_section::with(|cs| {
                let ptr = LIDAR_DMA_BUFFER.borrow(cs).as_ptr();
                rx_dma.mar().write(|w| w.ma().bits(ptr as u32));
            });
        }
        rx_dma
            .ndtr()
            .write(|w| w.ndt().set(LIDAR_BUFFER_SIZE as u16));
        rx_dma.cr().write(|w| {
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
                .enabled()
                .dir()
                .from_peripheral()
                .htie()
                .enabled()
                .tcie()
                .enabled()
        });

        #[allow(unsafe_code)]
        unsafe {
            // USART2 DMA request id is 52
            dmamux.ccr(0).write(|w| w.dmareq_id().bits(52));
        }
        // Enable DMA channel
        rx_dma.cr().modify(|_, w| w.en().enabled());

        // USART for communicating with LIDAR
        USART2::enable(rcc);
        USART2::reset(rcc);

        // Set 8 bit word length, no parity
        usart
            .cr1()
            .write(|w| w.m1().clear_bit().m0().clear_bit().pce().disabled());
        // Set 1 stop bit
        usart.cr2().write(|w| w.stop().stop1());
        // Set bit rate to 115200. Assume usart_presc=1, USART running at MCU clock.
        let usart_div = rcc.sysclk().to_Hz() / 115200;
        usart.brr().write(|w| w.brr().set(usart_div as u16));
        // Enable USART
        usart.cr1().modify(|_, w| w.ue().enabled());
        // Enable RX DMA
        usart.cr3().write(|w| w.dmar().enabled());
        // Enable transmitter and receiver.
        usart.cr1().modify(|_, w| w.te().enabled().re().enabled());

        // Enable interrupts
        #[allow(unsafe_code)]
        unsafe {
            NVIC::unmask(Interrupt::DMA1_CHANNEL1);
        }
    }

    pub async fn task(&self) -> Result<(), Error> {
        loop {
            LIDAR_UPDATE_EVENT.read().await?;
            debug_rprintln!("dma event");
        }
    }
}

const LIDAR_BUFFER_SIZE: usize = 1024;
static LIDAR_DMA_BUFFER: Mutex<Cell<[u8; LIDAR_BUFFER_SIZE]>> =
    Mutex::new(Cell::new([0; LIDAR_BUFFER_SIZE]));

static LIDAR_UPDATE_EVENT: Mailbox<()> = Mailbox::new();

#[interrupt]
fn DMA1_CHANNEL1() {
    #[allow(unsafe_code)]
    let rx_dma = unsafe { DMA1::steal() };

    if rx_dma.isr().read().htif1().is_half() {
        // TODO: copy half data
        rx_dma.ifcr().write(|w| w.chtif1().clear());
    }

    if rx_dma.isr().read().tcif1().is_complete() {
        // TODO: copy second half data
        rx_dma.ifcr().write(|w| w.ctcif1().clear());
    }

    LIDAR_UPDATE_EVENT.post(());
}
