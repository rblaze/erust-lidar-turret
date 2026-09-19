use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::digital::{InputPin, OutputPin};
use firmware::error::Error;
use firmware::time::{Duration, sleep};
use rtt_target::debug_rprintln;
use stm32g0_hal::gpio::gpioa::{PA0, PA8, PA9, PA10, PA11};
use stm32g0_hal::gpio::{Floating, Input, Output, PushPull};
use stm32g0_hal::pac::{Interrupt, NVIC, TIM6, interrupt};
use stm32g0_hal::rcc::Rcc;
use stm32g0_hal::timer::{BasicTimEvent, Counter, Timer};

use crate::system_time::Ticker;

#[derive(Debug)]
pub struct LaserMotorControl {}

impl LaserMotorControl {
    pub fn new(
        phase1: PA8<Output<PushPull>>,
        phase2: PA9<Output<PushPull>>,
        phase3: PA10<Output<PushPull>>,
        phase4: PA11<Output<PushPull>>,
        mut sensor: PA0<Input<Floating>>,
        timer: Timer<TIM6>,
        rcc: &Rcc,
    ) -> Self {
        let mut motor = StepperMotor::new(phase1, phase2, phase3, phase4);

        // Make timer tick at 1MHz, so we can use precise timing.
        let prescaler = rcc.sysclk().to_Hz() / 1_000_000;
        // Trigger an interrupt every two milliseconds.
        let counter = timer.upcounter(prescaler as u16 - 1, 2000, rcc);

        counter.listen(BasicTimEvent::Update);
        counter.start();

        // Reset motor to zero position.
        while sensor.is_low().expect("Laser position read failed") {
            motor.move_backward();
            while !counter.is_pending(BasicTimEvent::Update) {}
            counter.unpend(BasicTimEvent::Update);
        }

        critical_section::with(|cs| {
            let mut borrow = STATE.borrow_ref_mut(cs);
            *borrow = Some(StepperData {
                motor,
                timer: counter,
                current_position: 0,
                target_position: 0,
            });
        });

        #[allow(unsafe_code)]
        unsafe {
            NVIC::unmask(Interrupt::TIM6_DAC_LPTIM1);
        }

        Self {}
    }

    // TODO: this function should get a fractional value
    pub fn set_target_position(&self, position: i32) {
        debug_rprintln!("Setting target position to {}", position);
        critical_section::with(|cs| {
            let mut borrow = STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("StepperData not initialized");
            state.target_position = position;
        });
    }

    // TODO: temporary task to move laser here and there.
    pub async fn task(&self, ticker: Ticker) -> Result<(), Error> {
        loop {
            let ticks = ticker.ticks();
            let position = ticks as i32 % TOTAL_STEPS;
            self.set_target_position(position);

            sleep(Duration::from_secs(5)).await;
        }
    }
}

/// Number of total steps for a full rotation of the stepper motor.
const TOTAL_STEPS: i32 = 2048;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Phase {
    Phase1,
    Phase2,
    Phase3,
    Phase4,
}

impl Phase {
    fn next(self) -> Self {
        match self {
            Phase::Phase1 => Phase::Phase2,
            Phase::Phase2 => Phase::Phase3,
            Phase::Phase3 => Phase::Phase4,
            Phase::Phase4 => Phase::Phase1,
        }
    }

    fn prev(self) -> Self {
        match self {
            Phase::Phase1 => Phase::Phase4,
            Phase::Phase2 => Phase::Phase1,
            Phase::Phase3 => Phase::Phase2,
            Phase::Phase4 => Phase::Phase3,
        }
    }
}

#[derive(Debug)]
struct StepperMotor {
    phase1: PA8<Output<PushPull>>,
    phase2: PA9<Output<PushPull>>,
    phase3: PA10<Output<PushPull>>,
    phase4: PA11<Output<PushPull>>,
    current_phase: Phase,
}

impl StepperMotor {
    fn new(
        phase1: PA8<Output<PushPull>>,
        phase2: PA9<Output<PushPull>>,
        phase3: PA10<Output<PushPull>>,
        phase4: PA11<Output<PushPull>>,
    ) -> Self {
        Self {
            phase1,
            phase2,
            phase3,
            phase4,
            current_phase: Phase::Phase1,
        }
    }

    fn move_forward(&mut self) {
        self.current_phase = self.current_phase.next();
        self.update_phases();
    }

    fn move_backward(&mut self) {
        self.current_phase = self.current_phase.prev();
        self.update_phases();
    }

    fn update_phases(&mut self) {
        match self.current_phase {
            Phase::Phase1 => {
                self.phase1.set_high().ok();
                self.phase2.set_low().ok();
                self.phase3.set_low().ok();
                self.phase4.set_low().ok();
            }
            Phase::Phase2 => {
                self.phase1.set_low().ok();
                self.phase2.set_high().ok();
                self.phase3.set_low().ok();
                self.phase4.set_low().ok();
            }
            Phase::Phase3 => {
                self.phase1.set_low().ok();
                self.phase2.set_low().ok();
                self.phase3.set_high().ok();
                self.phase4.set_low().ok();
            }
            Phase::Phase4 => {
                self.phase1.set_low().ok();
                self.phase2.set_low().ok();
                self.phase3.set_low().ok();
                self.phase4.set_high().ok();
            }
        }
    }
}

#[derive(Debug)]
struct StepperData {
    motor: StepperMotor,
    timer: Counter<TIM6>,
    current_position: i32,
    target_position: i32,
}

impl StepperData {
    fn distance_forward(&self) -> i32 {
        let distance = self.target_position - self.current_position;
        if distance >= 0 {
            distance
        } else {
            TOTAL_STEPS + distance
        }
    }

    fn distance_backward(&self) -> i32 {
        let distance = self.current_position - self.target_position;
        if distance >= 0 {
            distance
        } else {
            TOTAL_STEPS + distance
        }
    }
}

static STATE: Mutex<RefCell<Option<StepperData>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn TIM6_DAC_LPTIM1() {
    critical_section::with(|cs| {
        let mut borrow = STATE.borrow_ref_mut(cs);
        let state = borrow
            .as_mut()
            .expect("StepperData must be initialized before TIM6 interrupt");

        debug_assert!(state.current_position >= 0 && state.current_position < TOTAL_STEPS);
        debug_assert!(state.target_position >= 0 && state.target_position < TOTAL_STEPS);

        if state.current_position != state.target_position {
            if state.distance_forward() <= state.distance_backward() {
                state.motor.move_forward();
                state.current_position = (state.current_position + 1) % TOTAL_STEPS;
            } else {
                state.motor.move_backward();
                state.current_position = (state.current_position - 1 + TOTAL_STEPS) % TOTAL_STEPS;
            }
        }

        state.timer.unpend(BasicTimEvent::Update);
    });
}
