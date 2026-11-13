//% CHIP_FILTER(has_wifi_ble): esp32c6 || esp32s3
//% SUPPORT-FIRMWARE: true
//% FEATURES: unstable esp-alloc embassy
//% FEATURES(has_wifi_ble): esp-radio/ble esp-radio esp-radio-unstable
//% ENV: ESP_HAL_CONFIG_STACK_GUARD_OFFSET=4

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_time::{Duration, Timer};
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};
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
    cfg_select! {
        any(esp32, esp32s2, esp32s3, esp32c3, esp32c2, esp32c6) => {
            use esp_hal::ram;
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
            esp_alloc::heap_allocator!(size: 36 * 1024);
        }
        any(esp32c5, esp32h2) => {
            esp_alloc::heap_allocator!(size: 72 * 1024);
        }
        _ => {}
    }
}

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    init_heap();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0);

    let connector = BleConnector::new(p.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 1> = ExternalController::new(connector);

    run_peripheral(controller).await;
}

async fn run_peripheral(controller: ExternalController<BleConnector<'static>, 1>) {
    let address = Address::random(PERIPHERAL_ADDRESS);

    let mut resources: HostResources<DefaultPacketPool, 1, 2> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .register_l2cap_spsm(hil_test::ble::L2CAP_PSM)
        .build();
    let mut peripheral = stack.peripheral();
    let runner = stack.runner();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: DEVICE_NAME,
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    select(
                        gatt_events_task(&server, &conn),
                        send_l2cap_data(&stack, conn.raw()),
                    )
                    .await;
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
            AdStructure::IncompleteServiceUuids16(&[[0x0f, 0x18]]),
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

    hil_test::signal_harness_ready();
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
                {}

                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(_) => {}
                }
            }
            _ => {}
        }
    }
}

/// Sends `L2CAP_BYTES` on the first L2CAP channel the central opens.
async fn send_l2cap_data(
    stack: &Stack<'_, ExternalController<BleConnector<'static>, 1>, DefaultPacketPool>,
    conn: &Connection<'_, DefaultPacketPool>,
) {
    let Ok(mut channel) = L2capChannel::listen(stack, conn)
        .accept(&Default::default())
        .await
    else {
        return core::future::pending().await;
    };
    // 2120 µs is the maximum LE packet time.
    if conn
        .update_data_length(stack, hil_test::ble::DATA_LENGTH, 2120)
        .await
        .is_ok()
    {
        let mut sent = 0;
        while sent < hil_test::ble::L2CAP_BYTES {
            let mut sdu = [0; 240];
            let len = sdu.len().min(hil_test::ble::L2CAP_BYTES - sent);
            for (i, byte) in sdu[..len].iter_mut().enumerate() {
                *byte = hil_test::ble::l2cap_byte(sent + i);
            }
            if channel.send(stack, &sdu[..len]).await.is_err() {
                break;
            }
            sent += len;
        }
    }
    // Keep the channel open until disconnect, so the central receives everything.
    core::future::pending().await
}
