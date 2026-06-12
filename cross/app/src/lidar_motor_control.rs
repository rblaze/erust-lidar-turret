use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::pwm::SetDutyCycle;
use fugit::HertzU32;
use stm32g0_hal::gpio::Alternate;
use stm32g0_hal::gpio::gpioa::{PA0, PA6};
use stm32g0_hal::pac::{Interrupt, NVIC, TIM2, TIM3, interrupt};
use stm32g0_hal::rcc::{Rcc, ResetEnable};
use stm32g0_hal::timer::Pwm;

use firmware::error::Error;
use firmware::time::{Duration, sleep};

pub struct LidarMotorControl {
    motor_pwm: Pwm<TIM3>,
    motor_pin: PA6<Alternate<1>>,
    target_interval: u32,
}

impl LidarMotorControl {
    const NUM_WHEEL_SLOTS: u32 = 25;

    pub fn new(
        motor_pwm: Pwm<TIM3>,
        motor_pin: PA6<Alternate<1>>,
        speed_timer: TIM2,
        _speed_pin: PA0<Alternate<2>>,
        rcc: &Rcc,
        target_rpm: HertzU32,
    ) -> Self {
        Self::init_speed_timer(speed_timer, rcc);

        Self {
            motor_pwm,
            motor_pin,
            target_interval: rcc.sysclk() / target_rpm / Self::NUM_WHEEL_SLOTS,
        }
    }

    // TODO: rewrite with HAL primitives
    fn init_speed_timer(speed_timer: TIM2, rcc: &Rcc) {
        TIM2::enable(rcc);
        TIM2::reset(rcc);

        // 1. Select the proper TI1x source (internal or external) with the
        // TI1SEL[3:0] bits in the TIMx_TISEL register.
        #[allow(unsafe_code)]
        unsafe {
            speed_timer.tisel().write(|w| w.ti1sel().bits(0b0)); // Noop, default value.
        }
        // 2. Select the active input: TIMx_CCR1 must be linked to the TI1
        // input, so write the CC1S bits to 01 in the TIMx_CCMR1 register.
        // As soon as CC1S becomes different from 00, the channel is
        // configured in input and the TIMx_CCR1 register becomes read-only.
        speed_timer.ccmr1_input().write(|w| w.cc1s().ti1());

        // 3. Program the appropriate input filter duration in relation with
        // the signal connected to the timer (when the input is one of the
        // TIx (ICxF bits in the TIMx_CCMRx register). Let’s imagine that,
        // when toggling, the input signal is not stable during at must 5
        // internal clock cycles. We must program a filter duration longer
        // than these 5 clock cycles. We can validate a transition on TI1
        // when 8 consecutive samples with the new level have been detected
        // (sampled at fDTS frequency). Then write IC1F bits to 0011 in the
        // TIMx_CCMR1 register.
        speed_timer
            .ccmr1_input()
            .modify(|_, w| w.ic1f().fck_int_n8());

        // 4. Select the edge of the active transition on the TI1 channel
        // by writing the CC1P and CC1NP bits to 000 in the
        // TIMx_CCER register (rising edge in this case).
        speed_timer.ccer().write(|w| w.cc1p().rising_edge()); // Noop, default value.

        // 5. Program the input prescaler. In our example, we wish the
        // capture to be performed at each valid transition, so the prescaler
        // is disabled (write IC1PS bits to 00 in the TIMx_CCMR1 register).
        speed_timer
            .ccmr1_input()
            .modify(|_, w| w.ic1psc().no_prescaler()); // Noop, default value.

        // 6. Enable capture from the counter into the capture register
        // by setting the CC1E bit in the TIMx_CCER register.
        speed_timer.ccer().modify(|_, w| w.cc1e().enabled());

        // 7. If needed, enable the related interrupt request by setting
        // the CC1IE bit in the TIMx_DIER register, and/or the DMA request
        // by setting the CC1DE bit in the TIMx_DIER register.
        speed_timer.dier().write(|w| w.cc1ie().enabled());

        #[allow(unsafe_code)]
        unsafe {
            // Set slave mode input trigger to TI1FP1
            speed_timer.smcr().write(|w| w.ts().bits(0b00101));
            // Set slave mode to reset
            speed_timer.smcr().modify(|_, w| w.sms().bits(0b100));
        }

        speed_timer.cr1().write(|w| w.cen().enabled());

        #[allow(unsafe_code)]
        unsafe {
            NVIC::unmask(Interrupt::TIM2);
        }
    }

    fn get_speed_data() -> [u32; NUM_SPEED_POINTS] {
        critical_section::with(|cs| SPEED_TICKS.borrow_ref(cs).speed_data)
    }

    pub async fn task(
        self,
        initial_delay: Duration,
        check_interval: Duration,
    ) -> Result<(), Error> {
        let mut motor_pin = self.motor_pwm.bind_pin(self.motor_pin);
        let duty = motor_pin.max_duty_cycle() / 10;
        motor_pin.set_duty_cycle(duty);

        // Let motor spin up
        sleep(initial_delay).await;

        firmware::cav_motor_control::control_loop(
            check_interval,
            self.target_interval,
            duty,
            Self::get_speed_data,
            &mut motor_pin,
        )
        .await
    }
}

const NUM_SPEED_POINTS: usize = 25;

#[derive(Copy, Clone, Debug)]
struct SpeedInfo {
    speed_data: [u32; NUM_SPEED_POINTS],
    next_index: usize,
}

static SPEED_TICKS: Mutex<RefCell<SpeedInfo>> = Mutex::new(RefCell::new(SpeedInfo {
    speed_data: [0; NUM_SPEED_POINTS],
    next_index: 0,
}));

#[interrupt]
fn TIM2() {
    #[allow(unsafe_code)]
    let speed_timer = unsafe { TIM2::steal() };
    let ticks = speed_timer.ccr1().read().ccr().bits();
    critical_section::with(|cs| {
        let mut info = SPEED_TICKS.borrow_ref_mut(cs);
        let index = info.next_index;

        info.speed_data[index] = ticks;
        info.next_index = (index + 1) % NUM_SPEED_POINTS;
    });
}
