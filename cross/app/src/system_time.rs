use core::cell::Cell;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::exception;
use critical_section::Mutex;

use stm32g0_hal::pac::SYST;
use stm32g0_hal::rcc::Rcc;

use firmware::time::{Instant, TimerTicks};

static TICKS: Mutex<Cell<TimerTicks>> = Mutex::new(Cell::new(0));

#[derive(Clone, Copy, Debug)]
pub struct Ticker {}

impl Ticker {
    // Setup SysTick to tick at 100Hz
    pub fn new(mut syst: SYST, rcc: &Rcc) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(rcc.sysclk().to_Hz() / 100);
        syst.clear_current();
        syst.enable_interrupt();
        syst.enable_counter();

        Ticker {}
    }

    // Get current tick count.
    pub fn ticks(&self) -> TimerTicks {
        critical_section::with(|cs| TICKS.borrow(cs).get())
    }

    // Get current timestamp.
    #[allow(unused)]
    pub fn now(&self) -> Instant {
        let ticks = self.ticks();
        Instant::from_ticks(ticks)
    }

    // Wait for the next tick.
    // Makes sure the ticker is enabled.
    pub fn wait_for_tick(&self) {
        cortex_m::asm::wfi();
    }
}

#[exception]
fn SysTick() {
    critical_section::with(|cs| {
        TICKS.borrow(cs).update(|ticks| ticks + 1);
    });
}
