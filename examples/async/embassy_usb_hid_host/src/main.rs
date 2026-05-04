//! USB HID host example using embassy-usb-host.
//!
//! This example should be built in release mode.
//!
//! Connect a mouse or keyboard to the USB port, and it will log raw HID input reports to the
//! console.
//!
//! The following wiring is assumed:
//! - DP => GPIO20
//! - DM => GPIO19

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_usb_host::{BusRoute, BusState, class::hid::HidHost};
use esp_backtrace as _;
use esp_hal::{
    interrupt::software::SoftwareInterruptControl,
    otg_fs::{Usb, embassy_usb_host::Driver},
    timer::timg::TimerGroup,
};
use log::*;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");

    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);
    static BUS_STATE: BusState = BusState::new();
    let (mut bus_ctrl, bus) = embassy_usb_host::bus(Driver::new(usb), &BUS_STATE);
    info!("USB host initialized, waiting for device...");

    loop {
        // Wait for a device to connect
        let speed = bus_ctrl.wait_for_connection().await;
        info!("Device connected at speed {:?}", speed);

        // Enumerate the device
        let mut config_buf = [0u8; 256];
        let result = bus
            .enumerate(BusRoute::Direct(speed), &mut config_buf)
            .await;

        let (enum_info, config_len) = match result {
            Ok(r) => r,
            Err(e) => {
                error!("Enumeration failed: {:?}", e);
                continue;
            }
        };

        info!(
            "Enumerated: VID={:04x} PID={:04x} addr={}",
            enum_info.device_desc.vendor_id,
            enum_info.device_desc.product_id,
            enum_info.device_address
        );

        // Try to create a HID host driver
        let mut hid = match HidHost::new(&bus, &config_buf[..config_len], &enum_info) {
            Ok(h) => h,
            Err(e) => {
                error!("HID init failed: {:?}", e);
                continue;
            }
        };

        // Disable idle repeat (STALL-tolerant: some devices don't support SET_IDLE)
        if let Err(e) = hid.set_idle(0, 0).await {
            error!("SET_IDLE failed: {:?}", e);
            continue;
        }

        info!("HID device ready, reading reports...");

        // Read loop: log raw HID input reports
        let mut buf = [0u8; 64];
        loop {
            match hid.read(&mut buf).await {
                Ok(n) if n > 0 => {
                    info!("HID report: {:x?}", &buf[..n]);
                }
                Ok(_) => {}
                Err(e) => {
                    error!("HID read failed: {:?}", e);
                    break;
                }
            }
        }

        info!("Device disconnected, waiting for next...");
    }
}
