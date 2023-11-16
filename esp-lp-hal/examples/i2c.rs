#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c6_lp_hal::{
    delay::Delay,
    gpio::{GpioPin, InOut, Unknown},
    i2c::*,
    prelude::*,
};
use esp32c6_lp_hal::uart;

use panic_halt as _;

#[entry]
fn main(mut i2c: LpI2c) -> ! {
    let _peripherals = esp32c6_lp::Peripherals::take().unwrap();

    let mut lp_uart = unsafe { uart::conjour().unwrap() };

    let mut delay = Delay::new();

    // TODO
    // Disable pulldown on the io pin

    loop {
        // let read = nb::block!(lp_uart.read());

        // match read {
        //     Ok(read) => writeln!(lp_uart, "Read 0x{:02x}", read),
        //     Err(err) => writeln!(lp_uart, "Error {:?}", err),
        // };

        let mut data = [0u8; 22];
        // i2c.write(0x77, &[0xaa]);
        // i2c.write(0x77, b"abcd");
        i2c.write_read(0x77, &[0xaa], &mut data).ok();
        // i2c.read(0xaa, &mut data);
        delay.delay_ms(1000u32);
        // i2c.write(0x77, b"efgh");
    }
}
