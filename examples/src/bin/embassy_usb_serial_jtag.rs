//! This shows how to read and write text via USB Serial/JTAG using embassy.
//! You need to connect via the Serial/JTAG interface to see any output.
//! Most dev-kits use a USB-UART-bridge - in that case you won't see any output.

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    system::SystemControl,
    timer::timg::TimerGroup,
    usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx},
    Async,
};
use static_cell::StaticCell;

const MAX_BUFFER_SIZE: usize = 512;

#[embassy_executor::task]
async fn writer(
    mut tx: UsbSerialJtagTx<'static, Async>,
    signal: &'static Signal<NoopRawMutex, heapless::String<MAX_BUFFER_SIZE>>,
) {
    use core::fmt::Write;
    embedded_io_async::Write::write_all(
        &mut tx,
        b"Hello async USB Serial JTAG. Type something.\r\n",
    )
    .await
    .unwrap();
    loop {
        let message = signal.wait().await;
        signal.reset();
        write!(&mut tx, "-- received '{}' --\r\n", message).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn reader(
    mut rx: UsbSerialJtagRx<'static, Async>,
    signal: &'static Signal<NoopRawMutex, heapless::String<MAX_BUFFER_SIZE>>,
) {
    let mut rbuf = [0u8; MAX_BUFFER_SIZE];
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf).await;
        match r {
            Ok(len) => {
                let mut string_buffer: heapless::Vec<_, MAX_BUFFER_SIZE> = heapless::Vec::new();
                string_buffer.extend_from_slice(&rbuf[..len]).unwrap();
                signal.signal(heapless::String::from_utf8(string_buffer).unwrap());
            }
            Err(e) => esp_println::println!("RX Error: {:?}", e),
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> () {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    let (tx, rx) = UsbSerialJtag::new_async(peripherals.USB_DEVICE).split();

    static SIGNAL: StaticCell<Signal<NoopRawMutex, heapless::String<MAX_BUFFER_SIZE>>> =
        StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    spawner.spawn(reader(rx, &signal)).unwrap();
    spawner.spawn(writer(tx, &signal)).unwrap();
}
