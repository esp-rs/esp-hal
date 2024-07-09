//! embassy serial
//!
//! This is an example of running the embassy executor and asynchronously
//! writing to and reading from uart

#![no_std]
#![no_main]

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    peripherals::{Peripherals, UART0},
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    uart::{
        config::{AtCmdConfig, Config},
        Uart,
        UartRx,
        UartTx,
    },
    Async,
};
use static_cell::StaticCell;

// rx_fifo_full_threshold
const READ_BUF_SIZE: usize = 64;
// EOT (CTRL-D)
const AT_CMD: u8 = 0x04;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[embassy_executor::task]
async fn writer(
    mut tx: UartTx<'static, UART0, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    use core::fmt::Write;
    embedded_io_async::Write::write(
        &mut tx,
        b"Hello async serial. Enter something ended with EOT (CTRL-D).\r\n",
    )
    .await
    .unwrap();
    embedded_io_async::Write::flush(&mut tx).await.unwrap();
    loop {
        let bytes_read = signal.wait().await;
        signal.reset();
        write!(&mut tx, "\r\n-- received {} bytes --\r\n", bytes_read).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn reader(
    mut rx: UartRx<'static, UART0, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
        match r {
            Ok(len) => {
                offset += len;
                esp_println::println!("Read: {len}, data: {:?}", &rbuf[..offset]);
                offset = 0;
                signal.signal(len);
            }
            Err(e) => esp_println::println!("RX Error: {:?}", e),
        }
    }
}

#[main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timers = [timer0];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Default pins for Uart/Serial communication
    #[cfg(feature = "esp32")]
    let (tx_pin, rx_pin) = (io.pins.gpio1, io.pins.gpio3);
    #[cfg(feature = "esp32c2")]
    let (tx_pin, rx_pin) = (io.pins.gpio20, io.pins.gpio19);
    #[cfg(feature = "esp32c3")]
    let (tx_pin, rx_pin) = (io.pins.gpio21, io.pins.gpio20);
    #[cfg(feature = "esp32c6")]
    let (tx_pin, rx_pin) = (io.pins.gpio16, io.pins.gpio17);
    #[cfg(feature = "esp32h2")]
    let (tx_pin, rx_pin) = (io.pins.gpio24, io.pins.gpio23);
    #[cfg(feature = "esp32s2")]
    let (tx_pin, rx_pin) = (io.pins.gpio43, io.pins.gpio44);
    #[cfg(feature = "esp32s3")]
    let (tx_pin, rx_pin) = (io.pins.gpio43, io.pins.gpio44);

    let config = Config::default().rx_fifo_full_threshold(READ_BUF_SIZE as u16);

    let mut uart0 =
        Uart::new_async_with_config(peripherals.UART0, config, &clocks, tx_pin, rx_pin).unwrap();
    uart0.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));

    let (tx, rx) = uart0.split();

    static SIGNAL: StaticCell<Signal<NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    spawner.spawn(reader(rx, &signal)).ok();
    spawner.spawn(writer(tx, &signal)).ok();
}
