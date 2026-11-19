//! CDC-ACM serial port example using embassy.
//!
//! This example should be built in release mode.
//!
//! The following wiring is assumed:
//! - ESP32-S2/S3:
//!   - DP => GPIO20
//!   - DM => GPIO19
//! - ESP32-H4:
//!   - DP => GPIO22
//!   - DM => GPIO21
//! - ESP32-P4:
//!   - DP => GPIO27
//!   - DM => GPIO26

//% CHIP_FILTER: usb_otg_driver_supported

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, State},
    driver::EndpointError,
};
use esp_backtrace as _;
use esp_hal::{
    timer::timg::TimerGroup,
    usb::otg::{
        Usb,
        embassy_usb_device::{Config, Driver},
    },
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Log on UART0's default TX pin.
    let log_tx = cfg_select! {
        feature = "esp32h4" => peripherals.GPIO24,
        feature = "esp32p4" => peripherals.GPIO37,
        any(feature = "esp32s2", feature = "esp32s3") => peripherals.GPIO43,
    };
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    log::info!("Init!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let (dp, dm) = cfg_select! {
        feature = "esp32h4" => (peripherals.GPIO22, peripherals.GPIO21),
        feature = "esp32p4" => (peripherals.GPIO27, peripherals.GPIO26),
        _ => (peripherals.GPIO20, peripherals.GPIO19),
    };
    let usb = Usb::new_fs(peripherals.USB_FS, dp, dm);

    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 1024];
    let config = Config::default();

    let driver = Driver::new(usb, &mut ep_out_buffer, config);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x303A, 0x3001);
    config.manufacturer = Some("Espressif");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            log::info!("Connected");
            let _ = echo(&mut class).await;
            log::info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using
    // separate tasks instead.
    join(usb_fut, echo_fut).await;
}

async fn echo<'d>(class: &mut CdcAcmClass<'d, Driver<'d>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        // Echo back in upper case
        for c in buf[0..n].iter_mut() {
            if 0x61 <= *c && *c <= 0x7a {
                *c &= !0x20;
            }
        }
        let data = &buf[..n];
        class.write_packet(data).await?;
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}
