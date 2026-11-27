//! Resets into the ROM download mode from a USB OTG application.
//!
//! The example runs a CDC-ACM serial port on the USB OTG controller. Send `b` to the port to reset
//! the chip into the ROM download mode. The chip then shows up as the USB Serial/JTAG device, and
//! a flashing tool can connect to it without the BOOT button.
//!
//! This example should be built in release mode.
//!
//! The following wiring is assumed:
//! - DP => GPIO20
//! - DM => GPIO19

//% CHIP_FILTER: esp32s3

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
    system::software_reset_to_download_mode,
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

    esp_log::init_uart!(peripherals);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let usb = Usb::new_fs(peripherals.USB_FS, peripherals.GPIO20, peripherals.GPIO19);

    let mut ep_out_buffer = [0u8; 1024];
    let driver = Driver::new(usb, &mut ep_out_buffer, Config::default());

    let mut config = embassy_usb::Config::new(0x303A, 0x3001);
    config.manufacturer = Some("Espressif");
    config.product = Some("USB download mode example");
    config.serial_number = Some("12345678");

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    let mut usb = builder.build();

    let usb_fut = usb.run();

    let command_fut = async {
        loop {
            class.wait_connection().await;
            log::info!("Connected, send `b` to reset into the download mode");
            let _ = wait_for_command(&mut class).await;
            log::info!("Disconnected");
        }
    };

    join(usb_fut, command_fut).await;
}

async fn wait_for_command<'d>(
    class: &mut CdcAcmClass<'d, Driver<'d>>,
) -> Result<(), EndpointError> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        if buf[..n].contains(&b'b') {
            log::info!("Resetting into the download mode");
            software_reset_to_download_mode();
        }
        class.write_packet(&buf[..n]).await?;
    }
}
