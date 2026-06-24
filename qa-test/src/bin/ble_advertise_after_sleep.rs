//! BLE advertising after light-sleep reproducer
//!
//! Reproduces https://github.com/esp-rs/esp-hal/issues/3873 for the ESP32-C6.

//% FEATURES: esp-radio esp-radio/ble esp-radio/unstable esp-hal/unstable
//% CHIP_FILTER: bt_driver_supported && sleep_driver_supported

#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    rtc_cntl::{Rtc, sleep::TimerWakeupSource},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use log::{info, warn};
use trouble_host::prelude::*;

esp_bootloader_esp_idf::esp_app_desc!();

const ADVERTISE_MS: u64 = 8_000;
const SLEEP_SECS: u64 = 5;

/// Max number of connections.
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

// GATT Server definition
#[gatt_server]
struct Server {
    battery_service: BatteryService,
}

/// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, name = "hello", read, value = "Battery Level", type = &'static str)]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
}

#[esp_hal::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mut rtc = Rtc::new(peripherals.LPWR);

    let sleep_config = esp_hal::rtc_cntl::sleep::RtcSleepConfig::default();
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(SLEEP_SECS));

    let connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 1> = ExternalController::new(connector);

    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    // Drive the BLE host forever alongside the advertise/sleep cycle.
    let app = async {
        let mut iteration: u32 = 0;
        loop {
            println!("================ iteration {iteration} ================");
            info!("[iter {iteration}] advertising for {ADVERTISE_MS} ms - scan for \"TrouBLE\"");

            select(
                Timer::after(Duration::from_millis(ADVERTISE_MS)),
                advertise_and_serve(&mut peripheral, &server),
            )
            .await;

            // Dropping the advertiser/connection above stops advertising. Yield
            // long enough (async, so the host task runs) for the controller to
            // actually process the "advertising disable" before we sleep.
            info!("[iter {iteration}] stopping advertising");
            Timer::after(Duration::from_millis(200)).await;

            info!("[iter {iteration}] entering light sleep for {SLEEP_SECS}s");
            rtc.sleep(&sleep_config, &[&timer]);

            println!(
                "[iter {iteration}] woke up - scan for \"TrouBLE\" now: on an affected C6 it will \
                 no longer be visible even though we re-enable advertising below"
            );
            iteration = iteration.wrapping_add(1);
        }
    };

    join(ble_task(runner), app).await;
}

/// Advertise and, if a central connects, stay connected until it disconnects.
async fn advertise_and_serve<'a, C: Controller>(
    peripheral: &mut Peripheral<'a, C, DefaultPacketPool>,
    server: &Server<'a>,
) {
    loop {
        match advertise("TrouBLE", peripheral, server).await {
            Ok(conn) => {
                info!("[ble] connection established");
                loop {
                    if let GattConnectionEvent::Disconnected { reason } = conn.next().await {
                        info!("[ble] disconnected: {:?}", reason);
                        break;
                    }
                }
            }
            Err(e) => {
                warn!("[ble][adv] error: {:?}", e);
                Timer::after(Duration::from_millis(200)).await;
            }
        }
    }
}

/// Background task required to run forever alongside any other BLE tasks.
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            warn!("[ble][ble_task] error: {:?}", e);
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

/// Create an advertiser and wait for a central to connect.
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x0f, 0x18]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[ble][adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}
