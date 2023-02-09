#![no_std]
#![no_main]

// Run this example with the eh1 feature enabled to use embedded-can instead of
// embedded-hal-0.2.7. embedded-can was split off from embedded-hal before it's
// upgrade to 1.0.0. cargo run --example twai --features eh1 --release
#[cfg(feature = "eh1")]
use embedded_can::{nb::Can, Frame, Id};
// Run this example without the eh1 flag to use the embedded-hal 0.2.7 CAN traits.
// cargo run --example twai --release
#[cfg(not(feature = "eh1"))]
use embedded_hal::can::{Can, Frame, Id};
use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    twai,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Use GPIO pins 2 and 3 to connect to the respective pins on the CAN
    // transceiver.
    let can_tx_pin = io.pins.gpio2;
    let can_rx_pin = io.pins.gpio3;

    // The speed of the CAN bus.
    const CAN_BAUDRATE: twai::BaudRate = twai::BaudRate::B1000K;

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    let mut can_config = twai::TwaiConfiguration::new(
        peripherals.TWAI,
        can_tx_pin,
        can_rx_pin,
        &mut system.peripheral_clock_control,
        &clocks,
        CAN_BAUDRATE,
    );

    // Partially filter the incoming messages to reduce overhead of receiving
    // undesired messages. Note that due to how the hardware filters messages,
    // standard ids and extended ids may both match a filter. Frame ids should
    // be explicitly checked in the application instead of fully relying on
    // these partial acceptance filters to exactly match. A filter that matches
    // standard ids of an even value.
    const FILTER: twai::filter::SingleStandardFilter =
        twai::filter::SingleStandardFilter::new(b"xxxxxxxxxx0", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
    can_config.set_filter(FILTER);

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    let mut can = can_config.start();

    loop {
        // Wait for a frame to be received.
        let frame = block!(can.receive()).unwrap();

        println!("Received a frame:");

        // Print different messages based on the frame id type.
        match frame.id() {
            Id::Standard(id) => {
                println!("\tStandard Id: {:?}", id);
            }
            Id::Extended(id) => {
                println!("\tExtended Id: {:?}", id);
            }
        }

        // Print out the frame data or the requested data length code for a remote
        // transmission request frame.
        if frame.is_data_frame() {
            println!("\tData: {:?}", frame.data());
        } else {
            println!("\tRemote Frame. Data Length Code: {}", frame.dlc());
        }

        // Transmit the frame back.
        let _result = block!(can.transmit(&frame)).unwrap();
    }
}
