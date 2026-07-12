use core::cell::RefCell;

use async_scheduler::sync::mailbox::Mailbox;
use critical_section::Mutex;
use rtt_target::debug_rprintln;
use stm32g0_hal::gpio::Alternate;
use stm32g0_hal::gpio::gpioa::{PA2, PA3};
use stm32g0_hal::pac::dma1::CH;
use stm32g0_hal::pac::{DMA1, DMAMUX, Interrupt, NVIC, USART2, interrupt};
use stm32g0_hal::rcc::{Rcc, ResetEnable};

use firmware::error::Error;

use crate::host_usart::HostUsart;

pub struct LidarReader<'a> {
    host_usart: &'a HostUsart<'a>,
}

impl<'a> LidarReader<'a> {
    pub fn new(
        host_usart: &'a HostUsart,
        _usart_tx: PA2<Alternate<1>>,
        _usart_rx: PA3<Alternate<1>>,
        usart: USART2,
        dma: &DMA1,
        dmamux: &DMAMUX,
        rcc: &Rcc,
    ) -> Self {
        Self::init_usart(&usart, dma.ch1(), dmamux, rcc);
        Self::init_lidar(&usart);

        Self { host_usart }
    }

    fn init_usart(usart: &USART2, rx_dma: &CH, dmamux: &DMAMUX, rcc: &Rcc) {
        #[allow(unsafe_code)]
        unsafe {
            rx_dma
                .par()
                .write(|w| w.pa().bits(core::ptr::from_ref(usart.rdr()) as u32));
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
            // USART2 RX DMA request id is 52
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

    fn send_lidar_command(usart: &USART2, cmd: &[u8]) {
        for b in cmd {
            while usart.isr().read().txe().is_full() {}
            usart.tdr().write(|w| w.tdr().set((*b).into()));
            while usart.isr().read().tc().is_tx_not_complete() {}
        }
    }

    fn init_lidar(usart: &USART2) {
        // debug_rprintln!("reset lidar");
        // Self::send_lidar_command(usart, &[0x5a, 4, 0x02, 0x00]);
        debug_rprintln!("get version");
        Self::send_lidar_command(usart, &[0x5a, 4, 0x01, 0x00]);
        // debug_rprintln!("set freq");
        // Self::send_lidar_command(usart, &[0x5a, 6, 0x03, 0xFA, 0x00, 0x00]);
        // debug_rprintln!("set format");
        // Self::send_lidar_command(usart, &[0x5a, 5, 0x05, 0x01, 0x00]);
        // debug_rprintln!("enable output");
        // Self::send_lidar_command(usart, &[0x5a, 5, 0x07, 0x01, 0x00]);
    }

    pub async fn task(&mut self) -> Result<(), Error> {
        const REPORT_LENGTH: usize = 9;
        const HALF_BUF_LEN: usize = LIDAR_BUFFER_SIZE / 2;
        let mut buf = [0; HALF_BUF_LEN + REPORT_LENGTH];
        let mut rem = 0;
        loop {
            let event = LIDAR_UPDATE_EVENT.read().await?;
            // debug_rprintln!("dma event {:?}", event);
            critical_section::with(|cs| {
                let dmabuf = LIDAR_DMA_BUFFER.borrow_ref(cs);
                match event {
                    LidarBufferEvent::FirstHalf => {
                        buf[rem..rem + HALF_BUF_LEN].copy_from_slice(&dmabuf[..HALF_BUF_LEN])
                    }
                    LidarBufferEvent::SecondHalf => {
                        buf[rem..rem + HALF_BUF_LEN].copy_from_slice(&dmabuf[HALF_BUF_LEN..])
                    }
                }
            });
            let mut databuf = [0; 2 + 2 * HALF_BUF_LEN / REPORT_LENGTH];
            let mut di = 0;

            let mut s = buf[..rem + HALF_BUF_LEN].as_ref();
            while s.len() >= REPORT_LENGTH {
                if s[0] != 0x59 || s[1] != 0x59 {
                    s = &s[1..];
                    continue;
                }
                databuf[di] = s[2];
                databuf[di + 1] = s[3];
                di += 2;
                s = &s[REPORT_LENGTH..];
            }

            let tail_start = rem + HALF_BUF_LEN - s.len();
            rem = s.len();
            buf.copy_within(tail_start..tail_start + rem, 0);

            // debug_rprintln!(
            //     "distance count {}, sample value {}",
            //     di,
            //     u16::from_le_bytes([databuf[0], databuf[1]])
            // );

            self.host_usart.write(&databuf[..di])?;
        }
    }
}

const LIDAR_BUFFER_SIZE: usize = 1024;
static LIDAR_DMA_BUFFER: Mutex<RefCell<[u8; LIDAR_BUFFER_SIZE]>> =
    Mutex::new(RefCell::new([1; LIDAR_BUFFER_SIZE]));

#[derive(PartialEq, Eq, Debug)]
enum LidarBufferEvent {
    FirstHalf,
    SecondHalf,
}
static LIDAR_UPDATE_EVENT: Mailbox<LidarBufferEvent> = Mailbox::new();

#[interrupt]
fn DMA1_CHANNEL1() {
    #[allow(unsafe_code)]
    let rx_dma = unsafe { DMA1::steal() };

    if rx_dma.isr().read().htif1().is_half() {
        rx_dma.ifcr().write(|w| w.chtif1().clear());
        LIDAR_UPDATE_EVENT.post(LidarBufferEvent::FirstHalf);
    }

    if rx_dma.isr().read().tcif1().is_complete() {
        rx_dma.ifcr().write(|w| w.ctcif1().clear());
        LIDAR_UPDATE_EVENT.post(LidarBufferEvent::SecondHalf);
    }
}
