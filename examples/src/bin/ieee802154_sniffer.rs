//! While this can be used as an example it's meant to be used with `extras/ieee802154-sniffer`
//!
//! Besides the runtime changeable channel and the output format it's almost identical to ieee802154_receive_all_frames

//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    reset::software_reset,
    system::SystemControl,
    uart::Uart,
};
use esp_ieee802154::*;
use esp_println::println;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Default pins for Uart/Serial communication
    #[cfg(feature = "esp32c6")]
    let (mut tx_pin, mut rx_pin) = (io.pins.gpio16, io.pins.gpio17);
    #[cfg(feature = "esp32h2")]
    let (mut tx_pin, mut rx_pin) = (io.pins.gpio24, io.pins.gpio23);

    let mut uart0 =
        Uart::new_with_default_pins(peripherals.UART0, &clocks, &mut tx_pin, &mut rx_pin).unwrap();

    // read two characters which get parsed as the channel
    let mut cnt = 0;
    let mut read = [0u8; 2];
    loop {
        let c = nb::block!(uart0.read_byte()).unwrap();
        if c == b'r' {
            continue;
        }

        read[cnt] = c;
        cnt += 1;

        if cnt >= 2 {
            break;
        }
    }
    let channel: u8 = unsafe { core::str::from_utf8_unchecked(&read) }
        .parse()
        .unwrap();

    let radio = peripherals.IEEE802154;
    let mut ieee802154 = Ieee802154::new(radio, &mut peripherals.RADIO_CLK);

    ieee802154.set_config(Config {
        channel,
        promiscuous: true,
        rx_when_idle: true,
        auto_ack_rx: false,
        auto_ack_tx: false,
        ..Config::default()
    });

    ieee802154.start_receive();

    loop {
        if let Some(frame) = ieee802154.get_raw_received() {
            println!("@RAW {:02x?}", &frame.data);
        }

        if let nb::Result::Ok(c) = uart0.read_byte() {
            if c == b'r' {
                software_reset();
            }
        }
    }
}
