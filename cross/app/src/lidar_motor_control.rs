use core::cell::Cell;

use async_scheduler::sync::mailbox::Mailbox;
use critical_section::Mutex;
use embedded_hal::pwm::SetDutyCycle;
use firmware::types::EventWaiter;
use fugit::HertzU32;
use stm32g0_hal::exti::{Event, ExtiExt};
use stm32g0_hal::gpio::gpioa::{PA1, PA6};
use stm32g0_hal::gpio::{Alternate, Floating, Input, SignalEdge};
use stm32g0_hal::pac::{EXTI, Interrupt, NVIC, TIM3, interrupt};
use stm32g0_hal::timer::Pwm;

use firmware::error::Error;
use firmware::time::{Duration, sleep};

use crate::system_time::Ticker;

pub struct LidarMotorControl {
    motor_pwm: Pwm<TIM3>,
    motor_pin: PA6<Alternate<1>>,
    target_interval: Duration,
}

impl LidarMotorControl {
    pub fn new(
        motor_pwm: Pwm<TIM3>,
        motor_pin: PA6<Alternate<1>>,
        mark_pin: PA1<Input<Floating>>,
        exti: &mut EXTI,
        target_rpm: HertzU32,
    ) -> Self {
        Self::init_speed_timer(mark_pin, exti);

        Self {
            motor_pwm,
            motor_pin,
            target_interval: target_rpm.to_duration(),
        }
    }

    fn init_speed_timer(mut mark_pin: PA1<Input<Floating>>, exti: &mut EXTI) {
        mark_pin.make_interrupt_source(exti);
        mark_pin.trigger_on_edge(SignalEdge::Both, exti);
        exti.listen(Event::Gpio1);

        #[allow(unsafe_code)]
        unsafe {
            NVIC::unmask(Interrupt::EXTI0_1);
        }
    }

    pub async fn task(self, initial_delay: Duration) -> Result<(), Error> {
        let mut motor_pin = self.motor_pwm.bind_pin(self.motor_pin);
        let duty = motor_pin.max_duty_cycle() / 3;
        motor_pin.set_duty_cycle(duty);

        // Let motor spin up
        sleep(initial_delay).await;

        firmware::cav_motor_control::control_loop(
            self.target_interval,
            duty,
            MarkWaiter {},
            &mut motor_pin,
        )
        .await
    }
}

struct MarkWaiter {}

impl EventWaiter for MarkWaiter {
    async fn wait(&self) {
        WHEEL_MARK.read().await.expect("mark wait failed")
    }
}

static WHEEL_MARK: Mailbox<()> = Mailbox::new();
static LAST_RAISE: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

#[interrupt]
fn EXTI0_1() {
    #[allow(unsafe_code)]
    let exti = unsafe { EXTI::steal() };

    let rising = exti.is_pending(Event::Gpio1, SignalEdge::Rising);
    let falling = exti.is_pending(Event::Gpio1, SignalEdge::Falling);
    exti.unpend(Event::Gpio1);

    if rising {
        critical_section::with(|cs| {
            #[allow(unsafe_code)]
            let now = unsafe { Ticker::systicks(cs) };
            LAST_RAISE.borrow(cs).set(now);
        });
    }

    if falling {
        let event_duration = critical_section::with(|cs| {
            #[allow(unsafe_code)]
            let now = unsafe { Ticker::systicks(cs) };
            let last_raise = LAST_RAISE.borrow(cs).replace(now);
            // debug_rprintln!("delta {} last {} now {}", now - last_raise, last_raise, now);

            now - last_raise
        });

        // Debouncing: require at least 1 ms pulse; assume 16MHz CPU freq
        // TODO: get CPU frequency from RCC clocks
        const MIN_DELAY: u64 = 16_000_000 / 1000;
        if event_duration > MIN_DELAY {
            // debug_rprintln!("Mark");
            WHEEL_MARK.post(());
        }
    }
}
