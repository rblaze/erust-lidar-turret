#![no_std]
#![no_main]
#![deny(unsafe_code)]

mod env;
mod system_time;

use core::cell::{Cell, RefCell};
use core::panic::PanicInfo;
use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use async_scheduler::sync::mailbox::Mailbox;
use cortex_m_rt::entry;
use critical_section::Mutex;
use embedded_hal::pwm::SetDutyCycle;
use futures::task::LocalFutureObj;
use rtt_target::debug_rprintln;
#[cfg(debug_assertions)]
use rtt_target::rtt_init_print;
use stm32g0_hal::gpio::gpioa::{PA0, PA2, PA3};
use stm32g0_hal::gpio::{Alternate, GpioExt};
use stm32g0_hal::pac::{
    CorePeripherals, DMA1, Interrupt, NVIC, Peripherals, TIM2, USART2, interrupt,
};
use stm32g0_hal::rcc::config::{Config, Prescaler};
use stm32g0_hal::rcc::{RccExt, ResetEnable};
use stm32g0_hal::timer::TimerExt;

use firmware::error::Error;

use crate::env::Env;
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

async fn lidar_task() -> Result<(), Error> {
    loop {
        LIDAR_UPDATE_EVENT.read().await?;
        debug_rprintln!("dma event");
    }
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

        let gpioa = dp.GPIOA.split(&rcc);

        // PWM for motor control
        let motor_pwm = dp.TIM3.constrain().pwm(0, u16::MAX, &rcc);
        let mut motor_pin = motor_pwm.bind_pin(gpioa.pa6.into_alternate_function());
        let mut duty = motor_pin.max_duty_cycle() / 10;
        motor_pin.set_duty_cycle(duty);

        // Timer for rotation speed measurement
        let _photointerruptor: PA0<Alternate<2>> = gpioa.pa0.into_alternate_function();

        TIM2::enable(&rcc);
        TIM2::reset(&rcc);
        let speed_timer = dp.TIM2;

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

        // USART RX DMA
        let _tx: PA2<Alternate<1>> = gpioa.pa2.into_alternate_function();
        let _rx: PA3<Alternate<1>> = gpioa.pa3.into_alternate_function();
        let usart = dp.USART2;

        let rx_dma = dp.DMA1.ch1();
        DMA1::enable(&rcc);
        DMA1::reset(&rcc);

        #[allow(unsafe_code)]
        unsafe {
            rx_dma
                .par()
                .write(|w| w.pa().bits(usart.rdr() as *const _ as u32));
            critical_section::with(|cs| {
                let ptr = LIDAR_DMA_BUFFER.borrow(cs).as_ptr();
                rx_dma.mar().write(|w| w.ma().bits(ptr as u32));
            });
        }
        rx_dma
            .ndtr()
            .write(|w| w.ndt().set(LIDAR_BUFFER_SIZE as u16));
        rx_dma.cr().write(|w| {
            w.mem2mem()
                .disabled()
                .msize()
                .bits8()
                .minc()
                .enabled()
                .psize()
                .bits32()
                .pinc()
                .disabled()
                .circ()
                .enabled()
                .dir()
                .from_peripheral()
                .htie()
                .enabled()
                .tcie()
                .enabled()
        });

        let dmamux = dp.DMAMUX;
        #[allow(unsafe_code)]
        unsafe {
            // USART2 DMA request id is 52
            dmamux.ccr(0).write(|w| w.dmareq_id().bits(52));
        }
        // Enable DMA channel
        rx_dma.cr().modify(|_, w| w.en().enabled());

        // USART for communicating with LIDAR
        USART2::enable(&rcc);
        USART2::reset(&rcc);

        // Set 8 bit word length, no parity
        usart
            .cr1()
            .write(|w| w.m1().clear_bit().m0().clear_bit().pce().disabled());
        // Set 1 stop bit
        usart.cr2().write(|w| w.stop().stop1());
        // Set bit rate to 115200. Assume usart_presc=1, USART running at MCU clock.
        let usart_div = rcc.sysclk().to_Hz() / 115200;
        usart.brr().write(|w| w.brr().set(usart_div as u16));
        // Enable USART
        usart.cr1().modify(|_, w| w.ue().enabled());
        // Enable RX DMA
        usart.cr3().write(|w| w.dmar().enabled());
        // Enable transmitter and receiver.
        usart.cr1().modify(|_, w| w.te().enabled().re().enabled());

        // Enable interrupts
        #[allow(unsafe_code)]
        unsafe {
            NVIC::unmask(Interrupt::TIM2);
            NVIC::unmask(Interrupt::DMA1_CHANNEL1);
        }

        // debug_rprintln!("sleeping");
        // delay(rcc.sysclk().to_Hz());
        debug_rprintln!("entering control loop");

        let env = Env::new(Ticker::new(cp.SYST, &rcc));
        LocalExecutor::new(&env).run([LocalFutureObj::new(pin!(panic_if_exited(lidar_task())))]);

        unreachable!()

        // for ch in [0x5a, 0x04, 0x01, 0x00] {
        //     while usart.isr().read().txe().bit_is_clear() {}
        //     debug_rprintln!("sending {:02x}", ch);
        //     usart.tdr().write(|w| w.tdr().set(ch));
        // }

        // while usart.isr().read().tc().bit_is_clear() {}

        // debug_rprintln!("done sending");

        // loop {
        // for _ in 0..NUM_SPEED_POINTS {
        // cortex_m::asm::wfi();
        // }

        // const NUM_WHEEL_SLOTS: u32 = 25;

        // let speed_data = critical_section::with(|cs| SPEED_TICKS.borrow_ref(cs).speed_data);
        // let average_speed = speed_data.iter().sum::<u32>() / (NUM_SPEED_POINTS as u32);
        // let freq = HSI_FREQ.to_Hz() / average_speed;

        // // This calculates new duty cycle
        // let error = freq as f32 / NUM_WHEEL_SLOTS as f32;
        // duty = (duty as f32 / error) as u16;

        // motor_pin.set_duty_cycle(duty);

        // debug_rprintln!("freq: {}, error: {}, new duty: {}", freq, error, duty);
        // }
    }()
    .expect("error in main");

    unreachable!()
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

const LIDAR_BUFFER_SIZE: usize = 1024;
static LIDAR_DMA_BUFFER: Mutex<Cell<[u8; LIDAR_BUFFER_SIZE]>> =
    Mutex::new(Cell::new([0; LIDAR_BUFFER_SIZE]));

static LIDAR_UPDATE_EVENT: Mailbox<()> = Mailbox::new();

#[interrupt]
fn DMA1_CHANNEL1() {
    #[allow(unsafe_code)]
    let rx_dma = unsafe { DMA1::steal() };

    if rx_dma.isr().read().htif1().is_half() {
        // TODO: copy half data
        rx_dma.ifcr().write(|w| w.chtif1().clear());
    }

    if rx_dma.isr().read().tcif1().is_complete() {
        // TODO: copy second half data
        rx_dma.ifcr().write(|w| w.ctcif1().clear());
    }

    LIDAR_UPDATE_EVENT.post(());
}
