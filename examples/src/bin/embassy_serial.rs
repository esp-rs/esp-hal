//! embassy serial
//!
//! This is an example of running the embassy executor and asynchronously
//! writing to and reading from UART.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy esp-hal/unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    Async,
    timer::timg::TimerGroup,
    uart::{AtCmdConfig, Config, RxConfig, Uart, UartRx, UartTx},
};
use static_cell::StaticCell;

// fifo_full_threshold (RX)
const READ_BUF_SIZE: usize = 64;
// EOT (CTRL-D)
const AT_CMD: u8 = 0x04;

esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn writer(mut tx: UartTx<'static, Async>, signal: &'static Signal<NoopRawMutex, usize>) {
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
async fn reader(mut rx: UartRx<'static, Async>, signal: &'static Signal<NoopRawMutex, usize>) {
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
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Default pins for Uart communication
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let (tx_pin, rx_pin) = (peripherals.GPIO1, peripherals.GPIO3);
        } else if #[cfg(feature = "esp32c2")] {
            let (tx_pin, rx_pin) = (peripherals.GPIO20, peripherals.GPIO19);
        } else if #[cfg(feature = "esp32c3")] {
            let (tx_pin, rx_pin) = (peripherals.GPIO21, peripherals.GPIO20);
        } else if #[cfg(feature = "esp32c6")] {
            let (tx_pin, rx_pin) = (peripherals.GPIO16, peripherals.GPIO17);
        } else if #[cfg(feature = "esp32h2")] {
            let (tx_pin, rx_pin) = (peripherals.GPIO24, peripherals.GPIO23);
        } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
            let (tx_pin, rx_pin) = (peripherals.GPIO43, peripherals.GPIO44);
        }
    }

    let config = Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(READ_BUF_SIZE as u16));

    let mut uart0 = Uart::new(peripherals.UART0, config)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin)
        .into_async();
    uart0.set_at_cmd(AtCmdConfig::default().with_cmd_char(AT_CMD));

    let (rx, tx) = uart0.split();

    static SIGNAL: StaticCell<Signal<NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    spawner.spawn(reader(rx, &signal)).ok();
    spawner.spawn(writer(tx, &signal)).ok();
}
