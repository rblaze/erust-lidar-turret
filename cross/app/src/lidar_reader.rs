use core::cell::RefCell;

use async_scheduler::sync::mailbox::Mailbox;
use critical_section::Mutex;
use firmware::distance_queue::DistanceQueue;
use rtt_target::debug_rprintln;
use stm32g0_hal::gpio::Alternate;
use stm32g0_hal::gpio::gpioa::{PA2, PA3};
use stm32g0_hal::pac::{Interrupt, NVIC, USART2, interrupt};
use stm32g0_hal::rcc::{Rcc, ResetEnable};

use firmware::error::Error;

use crate::host_usart::HOST_USART_EVENT;

pub struct LidarReader {}

impl LidarReader {
    pub fn new(
        _usart_tx: PA2<Alternate<1>>,
        _usart_rx: PA3<Alternate<1>>,
        usart: USART2,
        rcc: &Rcc,
    ) -> Self {
        Self::init_usart(&usart, rcc);
        Self::init_lidar(&usart);

        Self {}
    }

    fn init_usart(usart: &USART2, rcc: &Rcc) {
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
        // Enable RX FIFO
        usart.cr1().modify(|_, w| w.fifoen().enabled());
        usart
            .cr3()
            .modify(|_, w| w.rxftie().enabled().rxftcfg().depth_7_8());
        // Enable USART
        usart.cr1().modify(|_, w| w.ue().enabled());
        // Enable transmitter and receiver.
        usart.cr1().modify(|_, w| w.te().enabled().re().enabled());

        // Enable interrupts
        #[allow(unsafe_code)]
        unsafe {
            NVIC::unmask(Interrupt::USART2);
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
        debug_rprintln!("set freq");
        Self::send_lidar_command(usart, &[0x5a, 6, 0x03, 0xfa, 0x00, 0x00]);
        // debug_rprintln!("set format");
        // Self::send_lidar_command(usart, &[0x5a, 5, 0x05, 0x01, 0x00]);
        // debug_rprintln!("enable output");
        // Self::send_lidar_command(usart, &[0x5a, 5, 0x07, 0x01, 0x00]);
    }

    pub async fn task(&mut self) -> Result<(), Error> {
        loop {
            critical_section::with(|cs| {
                let mut queue = DISTANCE_QUEUE.borrow_ref_mut(cs);
                while queue.read_for_lidar().is_some() {
                    // TODO: use lidar data
                }
            });

            LIDAR_UPDATE_EVENT.read().await?;
        }
    }
}

pub static DISTANCE_QUEUE: Mutex<RefCell<DistanceQueue>> =
    Mutex::new(RefCell::new(DistanceQueue::new()));
static LIDAR_UPDATE_EVENT: Mailbox<()> = Mailbox::new();

#[interrupt]
fn USART2() {
    #[allow(unsafe_code)]
    let usart = unsafe { USART2::steal() };

    if usart.isr().read().rxft().is_reached() {
        critical_section::with(|cs| {
            let mut queue = DISTANCE_QUEUE.borrow_ref_mut(cs);

            while usart.isr().read().rxne().is_data_ready() {
                let byte = (usart.rdr().read().rdr().bits() & 0xff) as u8;
                if queue.push_byte(byte).expect("distance queue overrun") {
                    LIDAR_UPDATE_EVENT.post(());
                    HOST_USART_EVENT.post(());
                }
            }
        });
    }
}
