use embedded_hal::pwm::SetDutyCycle;
use rtt_target::debug_rprintln;

use crate::error::Error;
use crate::time::{Duration, sleep};

pub async fn control_loop<const NUM_POINTS: usize, SpeedControl>(
    check_interval: Duration,
    desired_interval: u32,
    initial_duty: u16,
    speed_getter: impl Fn() -> [u32; NUM_POINTS],
    speed_control: &mut SpeedControl,
) -> Result<(), Error>
where
    SpeedControl: SetDutyCycle,
    Error: From<SpeedControl::Error>,
{
    let mut duty = initial_duty;

    loop {
        let speed_data = speed_getter();
        let average_interval = speed_data.iter().sum::<u32>() / (NUM_POINTS as u32);
        let error = average_interval as f32 / desired_interval as f32;

        // Clamp changes to 5% to avoid overshooting and oscillation.
        duty = (duty as f32 * error.clamp(0.95, 1.05)) as u16;

        debug_rprintln!(
            "actual: {}, desired: {}, error: {}, duty: {}",
            average_interval,
            desired_interval,
            error,
            duty
        );

        speed_control.set_duty_cycle(duty)?;

        sleep(check_interval).await;
    }
}
