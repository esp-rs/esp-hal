//! embassy serial
//!
//! This is an example of running the embassy executor and asynchronously
//! writing to and reading from UART.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    gpio::Io,
    peripherals::UART0,
    timer::timg::TimerGroup,
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

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let System {
        peripherals,
        clocks,
        ..
    } = esp_hal::init(CpuClock::boot_default());

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Default pins for Uart/Serial communication
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let (tx_pin, rx_pin) = (io.pins.gpio1, io.pins.gpio3);
        } else if #[cfg(feature = "esp32c2")] {
            let (tx_pin, rx_pin) = (io.pins.gpio20, io.pins.gpio19);
        } else if #[cfg(feature = "esp32c3")] {
            let (tx_pin, rx_pin) = (io.pins.gpio21, io.pins.gpio20);
        } else if #[cfg(feature = "esp32c6")] {
            let (tx_pin, rx_pin) = (io.pins.gpio16, io.pins.gpio17);
        } else if #[cfg(feature = "esp32h2")] {
            let (tx_pin, rx_pin) = (io.pins.gpio24, io.pins.gpio23);
        } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
            let (tx_pin, rx_pin) = (io.pins.gpio43, io.pins.gpio44);
        }
    }

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
