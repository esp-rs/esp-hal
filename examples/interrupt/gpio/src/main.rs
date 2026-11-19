//! GPIO interrupt
//!
//! This prints "Interrupt" when the boot button is pressed.
//! It also blinks an LED like the blinky example.
//!
//! The following wiring is assumed:
//! - LED => GPIO2
//! - BUTTON:
//!   - ESP32, ESP32-S2, ESP32-S3: GPIO0
//!   - ESP32-C5: GPIO28
//!   - ESP32-P4: GPIO35
//!   - ESP32-H4: GPIO34
//!   - ESP32-S31: GPIO61
//!   - Others: GPIO9

//% CHIP_FILTER: gpio_driver_supported

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    handler,
    main,
    ram,
};

esp_bootloader_esp_idf::esp_app_desc!();

static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Log on UART0's default TX pin.
    let log_tx = cfg_select! {
        feature = "esp32" => peripherals.GPIO1,
        feature = "esp32c2" => peripherals.GPIO20,
        feature = "esp32c3" => peripherals.GPIO21,
        any(feature = "esp32c5", feature = "esp32c61") => peripherals.GPIO11,
        feature = "esp32c6" => peripherals.GPIO16,
        any(feature = "esp32h2", feature = "esp32h4") => peripherals.GPIO24,
        feature = "esp32p4" => peripherals.GPIO37,
        any(feature = "esp32s2", feature = "esp32s3") => peripherals.GPIO43,
        feature = "esp32s31" => peripherals.GPIO58,
    };
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    // Set GPIO2 as an output, and set its state high initially.
    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(handler);

    let mut led = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());

    let button = cfg_select! {
        any(feature = "esp32", feature = "esp32s2", feature = "esp32s3") => peripherals.GPIO0,
        feature = "esp32c5" => peripherals.GPIO28,
        feature = "esp32p4" => peripherals.GPIO35,
        feature = "esp32h4" => peripherals.GPIO34,
        feature = "esp32s31" => peripherals.GPIO61,
        _ => peripherals.GPIO9,
    };

    let config = InputConfig::default().with_pull(Pull::Up);
    let mut button = Input::new(button, config);

    critical_section::with(|cs| {
        button.listen(Event::FallingEdge);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });
    led.set_high();

    let delay = Delay::new();

    loop {
        led.toggle();
        delay.delay_millis(500);
    }
}

#[handler]
#[ram]
fn handler() {
    cfg_select! {
        any(feature = "esp32", feature = "esp32s2", feature = "esp32s3") => {
            log::info!(
                "GPIO Interrupt with priority {}",
                esp_hal::xtensa_lx::interrupt::get_level()
            );
        }
        _ => {
            log::info!("GPIO Interrupt");
        }
    }

    if critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        log::info!("Button was the source of the interrupt");
    } else {
        log::info!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}
