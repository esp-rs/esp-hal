//! This shows how to read and write text via USB Serial/JTAG using embassy.
//! You need to connect via the Serial/JTAG interface to see any output.
//! Most dev-kits use a USB-UART-bridge - in that case you won't see any output.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};
use esp32h2_hal::{
    clock::ClockControl,
    embassy,
    peripherals::Peripherals,
    prelude::*,
    UsbSerialJtag,
};
use esp_backtrace as _;

#[main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32h2_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(
        &clocks,
        esp32h2_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks),
    );

    let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);

    loop {
        let mut read_buf = [0; 64];

        match AsyncRead::read(&mut usb_serial, &mut read_buf).await {
            Ok(n) => {
                AsyncWrite::write_all(&mut usb_serial, b"echo: ")
                    .await
                    .unwrap();
                AsyncWrite::write_all(&mut usb_serial, &read_buf[..n])
                    .await
                    .unwrap();
                AsyncWrite::write_all(&mut usb_serial, b"\n").await.unwrap();
            }
            Err(e) => esp_println::println!("error: {e}"),
        }
    }
}
