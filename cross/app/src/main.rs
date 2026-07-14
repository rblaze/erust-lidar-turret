#![no_std]
#![no_main]
#![deny(unsafe_code)]

mod env;
mod host_usart;
mod lidar_motor_control;
mod lidar_reader;
mod system_time;

use core::panic::PanicInfo;
use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use cortex_m_rt::entry;
use fugit::HertzU32;
use futures::task::LocalFutureObj;
use rtt_target::debug_rprintln;
#[cfg(debug_assertions)]
use rtt_target::rtt_init_print;
use stm32g0_hal::gpio::GpioExt;
use stm32g0_hal::pac::{CorePeripherals, DMA1, Peripherals};
use stm32g0_hal::rcc::config::{Config, Prescaler};
use stm32g0_hal::rcc::{RccExt, ResetEnable};
use stm32g0_hal::timer::TimerExt;

use firmware::error::Error;
use firmware::time::Duration;

use crate::env::Env;
use crate::lidar_motor_control::LidarMotorControl;
use crate::system_time::Ticker;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    debug_rprintln!("{}", info);

    cortex_m::asm::bkpt();
    cortex_m::asm::udf();
}

async fn panic_if_exited<F: core::future::Future<Output: core::fmt::Debug>>(f: F) {
    panic!("future exited with {:?}", f.await)
}

#[entry]
fn main() -> ! {
    move || -> Result<(), Error> {
        #[cfg(debug_assertions)]
        rtt_init_print!(rtt_target::ChannelMode::NoBlockSkip, 4096);

        debug_rprintln!("starting");

        let cp = CorePeripherals::take().ok_or(Error::AlreadyTaken)?;
        let dp = Peripherals::take().ok_or(Error::AlreadyTaken)?;

        // Enable debug while stopped to keep probe-rs happy while WFI
        // Enabling DMA resolves another instability issue:
        // https://github.com/probe-rs/probe-rs/issues/350
        #[cfg(debug_assertions)]
        {
            dp.DBG.cr().modify(|_, w| w.dbg_stop().set_bit());
            dp.RCC.ahbenr().modify(|_, w| w.dmaen().set_bit());
        }

        let clocks = Config::sysclk_hsi(Prescaler::Div1);
        let rcc = dp.RCC.constrain(clocks);
        let mut exti = dp.EXTI;

        let gpioa = dp.GPIOA.split(&rcc);
        let gpiob = dp.GPIOB.split(&rcc);

        // Set PWM cycle to 10000 Hz to reduce power noise
        let max_duty = (rcc.sysclk().to_Hz() / 10000).clamp(0, u16::MAX as u32);
        let motor_pwm = dp.TIM3.constrain().pwm(0, max_duty as u16, &rcc);
        let motor_control = LidarMotorControl::new(
            motor_pwm,
            gpioa.pa6.into_alternate_function(),
            gpioa.pa1.into_floating_input(),
            &mut exti,
            HertzU32::Hz(1),
        );

        DMA1::enable(&rcc);
        DMA1::reset(&rcc);

        let host_usart = host_usart::HostUsart::new(
            gpiob.pb10.into_alternate_function(),
            gpiob.pb11.into_alternate_function(),
            dp.USART3,
            &dp.DMA1,
            &dp.DMAMUX,
            &rcc,
        );

        let mut lidar_reader = lidar_reader::LidarReader::new(
            gpioa.pa2.into_alternate_function(),
            gpioa.pa3.into_alternate_function(),
            dp.USART2,
            &rcc,
        );

        debug_rprintln!("entering control loop");

        let env = Env::new(Ticker::new(cp.SYST, &rcc));
        LocalExecutor::new(&env).run([
            LocalFutureObj::new(pin!(panic_if_exited(host_usart.task()))),
            LocalFutureObj::new(pin!(panic_if_exited(lidar_reader.task()))),
            LocalFutureObj::new(pin!(panic_if_exited(
                motor_control.task(Duration::from_secs(1))
            ))),
        ]);

        unreachable!()
    }()
    .expect("error in main");

    unreachable!()
}
