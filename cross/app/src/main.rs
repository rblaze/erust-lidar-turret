#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::panic::PanicInfo;

use cortex_m_rt::entry;
use rtt_target::debug_rprintln;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    debug_rprintln!("{}", info);

    cortex_m::asm::bkpt();
    cortex_m::asm::udf();
}

#[entry]
fn main() -> ! {
    todo!()
}
