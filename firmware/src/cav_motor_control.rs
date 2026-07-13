use embedded_hal::pwm::SetDutyCycle;
use rtt_target::debug_rprintln;

use crate::error::Error;
use crate::time::{Duration, Instant};
use crate::types::EventWaiter;

async fn get_mark_time<Mark: EventWaiter>(mark: &Mark) -> Instant {
    mark.wait().await;
    Instant::from_ticks(async_scheduler::now().await.ticks() as u32)
}

fn get_next_duty(duty: u16, actual_interval: Duration, desired_interval: Duration) -> u16 {
    let error = actual_interval.as_secs_f32() / desired_interval.as_secs_f32();
    (duty as f32 * error.clamp(0.95, 1.05)) as u16
}

pub async fn control_loop<SpeedControl, Mark>(
    desired_interval: Duration,
    initial_duty: u16,
    mark: Mark,
    speed_control: &mut SpeedControl,
) -> Result<(), Error>
where
    SpeedControl: SetDutyCycle,
    Mark: EventWaiter,
    Error: From<SpeedControl::Error>,
{
    let max_duty = speed_control.max_duty_cycle();
    let mut duty = initial_duty;
    let mut last_mark_time = get_mark_time(&mark).await;

    loop {
        for _ in 0..4 {
            // Use average of 5 rotations
            get_mark_time(&mark).await;
        }
        let mark_time = get_mark_time(&mark).await;
        let actual_interval = (mark_time - last_mark_time) / 5;

        // Clamp changes to 5% to avoid overshooting and oscillation.
        duty = get_next_duty(duty, actual_interval, desired_interval).clamp(0, max_duty);
        last_mark_time = mark_time;

        debug_rprintln!(
            "actual: {}, desired: {}, duty: {}",
            actual_interval,
            desired_interval,
            duty
        );

        speed_control.set_duty_cycle(duty)?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_duty_increase() {
        let desired_interval = Duration::from_secs(1);
        let actual_interval = Duration::from_millis(1030);
        let duty = 10000;

        let next_duty = get_next_duty(duty, actual_interval, desired_interval);
        // Actual RPM is 0.9708
        // Desired RPM is 1
        // Duty factor is 1.03
        assert_eq!(next_duty, 10300);
    }

    #[test]
    fn test_duty_decrease() {
        let desired_interval = Duration::from_secs(1);
        let actual_interval = Duration::from_millis(970);
        let duty = 10000;

        let next_duty = get_next_duty(duty, actual_interval, desired_interval);
        // Actual RPM is 1.0309
        // Desired RPM is 1
        // Duty factor is 0.97
        assert_eq!(next_duty, 9700);
    }

    #[test]
    fn test_duty_no_change() {
        let desired_interval = Duration::from_secs(1);
        let actual_interval = Duration::from_millis(1001);
        let duty = 10000;

        let next_duty = get_next_duty(duty, actual_interval, desired_interval);
        assert_eq!(next_duty, duty);
    }

    #[test]
    fn test_duty_clamp() {
        let desired_interval = Duration::from_secs(1);
        let actual_interval = Duration::from_millis(835);
        let duty = 10000;

        let next_duty = get_next_duty(duty, actual_interval, desired_interval);
        // Duty must change by 5% max
        assert_eq!(next_duty, 9500);
    }
}
