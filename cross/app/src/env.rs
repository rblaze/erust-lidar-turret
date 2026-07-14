use async_scheduler::executor::Environment;
use cortex_m::peripheral::{SCB, scb::VectActive};
use portable_atomic::{AtomicBool, Ordering};

use crate::system_time::Ticker;

#[derive(Debug)]
pub struct Env {
    ticker: Ticker,
}

impl Env {
    pub const fn new(ticker: Ticker) -> Self {
        Self { ticker }
    }
}

impl Environment for Env {
    fn wait_for_event_with_deadline(
        &self,
        event: &AtomicBool,
        tick: Option<async_scheduler::time::Instant>,
    ) {
        debug_assert_eq!(
            SCB::vect_active(),
            VectActive::ThreadMode,
            "calling wait_for_event_with_deadline() in interrupt handler"
        );

        critical_section::with(|_| {
            if event.load(Ordering::Acquire) {
                return;
            }

            if let Some(deadline) = tick
                && deadline <= self.ticks()
            {
                return;
            }

            // Critical section prevents interrupt handler from updating 'event' here.
            // Pending interrupt will wake up CPU and exit critical section.
            self.ticker.wait_for_tick();
        });
    }

    fn ticks(&self) -> async_scheduler::time::Instant {
        async_scheduler::time::Instant::new(self.ticker.ticks() as i64)
    }
}
