//! CDC-ACM serial port example using embassy.
//!
//! This example should be built in release mode.
//!
//! The following wiring is assumed:
//! - DP => GPIO20
//! - DM => GPIO19

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_usb_driver::host::UsbHostDriver;
use embassy_usb_host::{
    UsbHost,
    class::gip::{GipEvent, GipHost, RumbleCommand, XboxOneSGamepad},
};
use esp_backtrace as _;
use esp_hal::{
    interrupt::software::SoftwareInterruptControl,
    otg_fs::{Usb, embassy_usb_host::Driver},
    timer::timg::TimerGroup,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);
    let mut host = UsbHost::new(Driver::new(usb));
    info!("USB host initialized, waiting for device...");

    loop {
        // Wait for a device to connect
        let speed = host.wait_for_connection().await;
        info!("Device connected at speed {:?}", speed);

        // Enumerate the device
        let mut config_buf = [0u8; 256];
        let result = host.enumerate(speed, &mut config_buf).await;

        let (dev_desc, addr, config_len) = match result {
            Ok(r) => r,
            Err(e) => {
                error!("Enumeration failed: {:?}", e);
                continue;
            }
        };

        info!(
            "Enumerated: VID={:04x} PID={:04x} addr={}",
            dev_desc.vendor_id, dev_desc.product_id, addr
        );

        let driver = host.driver();

        let poll_for_disconnect = async {
            loop {
                if driver.wait_for_device_event().await
                    == embassy_usb_driver::host::DeviceEvent::Disconnected
                {
                    warn!("Device disconnected");
                    return;
                }
            }
        };
        let handle_gamepad = async {
            let result = GipHost::<_, XboxOneSGamepad>::try_register(
                host.driver(),
                &config_buf[..config_len],
                addr,
                dev_desc.vendor_id,
                dev_desc.product_id,
            )
            .await;
            let mut gip = match result {
                Ok(g) => {
                    info!("GIP device registered successfully");
                    g
                }
                Err(e) => {
                    error!("GIP registration failed: {:?}", e);
                    return;
                }
            };
            loop {
                match gip.poll().await {
                    Ok(GipEvent::Input(report)) => {
                        info!("GIP input report: {}", report);
                        let command = if report.a {
                            RumbleCommand {
                                left_trigger: 10,
                                right_trigger: 10,
                                strong: 10,
                                weak: 10,
                            }
                        } else {
                            RumbleCommand {
                                left_trigger: 0,
                                right_trigger: 0,
                                strong: 0,
                                weak: 0,
                            }
                        };

                        gip.set_rumble(&command).await.unwrap();
                    }
                    Ok(GipEvent::GuideButton(pressed)) => {
                        info!(
                            "Guide button {}",
                            if pressed { "pressed" } else { "released" }
                        );
                    }
                    Err(e) => {
                        error!("GIP poll error: {:?}", e);
                        break;
                    }
                }
            }
        };

        embassy_futures::select::select(poll_for_disconnect, handle_gamepad).await;
        host.free_address(addr);
        info!("Device disconnected, waiting for next...");
    }
}
