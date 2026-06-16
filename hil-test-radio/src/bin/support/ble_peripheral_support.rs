//% CHIP_FILTER(has_wifi_ble): esp32c6
//% SUPPORT-FIRMWARE: true
//% FEATURES: unstable esp-alloc embassy
//% FEATURES(has_wifi_ble): esp-radio/ble esp-radio esp-radio-unstable
//% FEATURES(has_wifi_ble): esp-radio/defmt defmt
//% ENV: ESP_HAL_CONFIG_STACK_GUARD_OFFSET=4

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
};
use esp_radio::ble::controller::BleConnector;
use hil_test as _;
use semihosting as _;
use trouble_host::prelude::*;

extern crate alloc;

const PERIPHERAL_ADDRESS: [u8; 6] = [0xff, 0x48, 0x49, 0x4c, 0x42, 0xff];
const DEVICE_NAME: &str = "HIL BAS";
const INITIAL_BATTERY_LEVEL: u8 = 12;

#[gatt_server]
struct Server {
    battery_service: BatteryService,
}

#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, value = INITIAL_BATTERY_LEVEL)]
    level: u8,
}

fn init_heap() {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c2, esp32c6))] {
            use esp_hal::ram;
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
            esp_alloc::heap_allocator!(size: 36 * 1024);
        } else if #[cfg(any(esp32c5, esp32h2))] {
            esp_alloc::heap_allocator!(size: 72 * 1024);
        }
    }
}

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    init_heap();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    let connector = BleConnector::new(p.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 1> = ExternalController::new(connector);

    run_peripheral(controller).await;
}

async fn run_peripheral<C>(controller: C)
where
    C: Controller,
{
    let address = Address::random(PERIPHERAL_ADDRESS);
    defmt::info!("[BLE-SUPPORT] address configured");

    let mut resources: HostResources<DefaultPacketPool, 1, 2> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: DEVICE_NAME,
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    defmt::info!("[BLE-SUPPORT] connected");
                    gatt_events_task(&server, &conn).await;
                    defmt::info!("[BLE-SUPPORT] disconnected");
                }
                Err(_) => Timer::after(Duration::from_millis(100)).await,
            }
        }
    })
    .await;
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if runner.run().await.is_err() {
            defmt::warn!("[BLE-SUPPORT] runner stopped");
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

async fn advertise<'values, 'server, C>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>>
where
    C: Controller,
{
    let mut adv_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x0f, 0x18]]),
            AdStructure::CompleteLocalName(DEVICE_NAME.as_bytes()),
        ],
        &mut adv_data,
    )?;

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;

    defmt::info!("[BLE-SUPPORT] advertising");
    Ok(advertiser.accept().await?.with_attribute_server(server)?)
}

async fn gatt_events_task<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let level = server.battery_service.level;

    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { .. } => break,
            GattConnectionEvent::Gatt { event } => {
                if let GattEvent::Read(event) = &event
                    && event.handle() == level.handle
                {
                    defmt::info!("[BLE-SUPPORT] battery level read");
                }

                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(_) => defmt::warn!("[BLE-SUPPORT] failed to accept GATT event"),
                }
            }
            _ => {}
        }
    }
}
