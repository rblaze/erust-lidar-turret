use fugit::{TimerDuration, TimerInstant};

const TIMER_FREQ: u64 = 100;

pub type TimerTicks = u32;
pub type Instant = TimerInstant<TimerTicks, TIMER_FREQ>;
pub type Duration = TimerDuration<TimerTicks, TIMER_FREQ>;

/// Sleeps for the specified duration.
pub async fn sleep(duration: Duration) {
    async_scheduler::sleep(async_scheduler::time::Duration::new(
        duration.as_ticks().into(),
    ))
    .await;
}
