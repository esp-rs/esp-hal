//! Alternating Wi-Fi / BLE radio stress test
//!
//! https://github.com/esp-rs/esp-hal/pull/5670
//!
//! The easiest way to reproduce the crash is to press Ctrl+R during the Wi-Fi phase.
//! In the next iteration, when it enters the BLE phase, it should crash immediately.
//% FEATURES: esp-radio esp-radio/wifi esp-radio/ble esp-radio/unstable esp-hal/unstable
//% CHIP_FILTER: wifi_driver_supported && bt_driver_supported

#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    rng::Rng,
    system::software_reset,
    timer::timg::TimerGroup,
};
use esp_radio::{
    ble::controller::BleConnector,
    wifi::{Config, ControllerConfig, WifiController, ap::AccessPointConfig},
};
use log::{info, warn};
use trouble_host::prelude::*;

esp_bootloader_esp_idf::esp_app_desc!();

/// Iteration counter, retained across software resets.
///
/// Most chips keep it in persistent RTC fast RAM: zeroed once on the initial
/// (power-on) boot, then preserved thereafter. The ESP32-C2 and ESP32-C61 have
/// no persistent RTC fast RAM, so they stash the counter in an otherwise-unused
/// always-on scratch register (`STORE6`) - on C2 in `RTC_CNTL`, on C61 in
/// `LP_AON` - which likewise survives a software reset and is cleared only on
/// power-on reset.
#[cfg(not(any(feature = "esp32c2", feature = "esp32c61")))]
#[ram(unstable(rtc_fast, persistent))]
static mut ITERATION: u32 = 0;

fn load_iteration() -> u32 {
    cfg_select! {
        feature = "esp32c2" => {
            esp_hal::peripherals::LPWR::regs().store6().read().bits()
        },
        feature = "esp32c61" => {
            esp_hal::peripherals::LP_AON::regs().store6().read().bits()
        },
        _ => unsafe { ITERATION },
    }
}
fn store_iteration(value: u32) {
    cfg_select! {
        feature = "esp32c2" => {
            esp_hal::peripherals::LPWR::regs()
                .store6()
                .write(|w| unsafe { w.bits(value) });
        },
        feature = "esp32c61" => {
            esp_hal::peripherals::LP_AON::regs()
                .store6()
                .write(|w| unsafe { w.bits(value) });
        },
        _ => unsafe { ITERATION = value },
    }
}

const AP_SSID: &str = "esp-handoff";

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
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, name = "hello", read, value = "Battery Level", type = &'static str)]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: bool,
}

fn init_heap() {
    cfg_select! {
        feature = "esp32" => {
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 96 * 1024);
            esp_alloc::heap_allocator!(size: 24 * 1024);
        },
        _ => {
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
            esp_alloc::heap_allocator!(size: 64 * 1024);
        },
    }
}

#[esp_hal::main]
async fn main(_s: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    init_heap();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Read the iteration left behind by the previous boot, then bump it so the
    // next boot (after our software reset) runs the other radio.
    let iteration = load_iteration();
    store_iteration(iteration.wrapping_add(1));

    let wifi_phase = iteration % 2 == 0;

    // Pseudo-random active window: 3000..=8000 ms (long enough for a central to
    // find us and connect during the BLE phase).
    let rng = Rng::new();
    let active_ms = 3000 + (rng.random() % 5000);

    info!(
        "iteration {iteration}: {} phase, active for {active_ms} ms",
        if wifi_phase { "WIFI" } else { "BLE" }
    );

    if wifi_phase {
        run_wifi(peripherals.WIFI, active_ms).await;
    } else {
        let connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
        let controller: ExternalController<_, 1> = ExternalController::new(connector);
        run_ble_peripheral(controller, active_ms).await;
    }

    info!("iteration {iteration} done, software reset");
    // Give the logger a moment to flush before we reset.
    Timer::after(Duration::from_millis(50)).await;

    software_reset();
}

/// Bring up a Wi-Fi SoftAP and keep it beaconing for `active_ms`.
async fn run_wifi(wifi: esp_hal::peripherals::WIFI<'static>, active_ms: u32) {
    let ap_config = Config::AccessPoint(AccessPointConfig::default().with_ssid(AP_SSID));

    // Creating the controller in AP mode calls `esp_wifi_start()`, so the radio
    // starts beaconing immediately.
    let controller = WifiController::new(
        wifi,
        ControllerConfig::default().with_initial_config(ap_config),
    )
    .unwrap();

    info!("[WIFI] SoftAP '{AP_SSID}' started, beaconing for {active_ms} ms");
    Timer::after(Duration::from_millis(active_ms as u64)).await;

    // Tear the radio down before the reset so we exercise the deinit path too.
    core::mem::drop(controller);
    info!("[WIFI] controller dropped");
}

/// Run the BLE battery-service peripheral for `active_ms`, then return so the
/// caller can software-reset. Mirrors the `bas_peripheral` example, but bounded
/// by the active window instead of looping forever.
async fn run_ble_peripheral<C>(controller: C, active_ms: u32)
where
    C: Controller,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("[BLE] our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    info!("[BLE] starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let app = async {
        loop {
            match advertise("Trouble Example", &mut peripheral, &server).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't
                    // run when no one is connected.
                    let a = gatt_events_task(&server, &conn);
                    let b = custom_task(&server, &conn, &stack);
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(a, b).await;
                }
                Err(e) => {
                    warn!("[BLE][adv] error: {:?}", e);
                    Timer::after(Duration::from_millis(100)).await;
                }
            }
        }
    };

    // Bound the whole BLE phase by the active window so we always get back to
    // the software reset and alternate with Wi-Fi.
    let window = Timer::after(Duration::from_millis(active_ms as u64));
    select(window, join(ble_task(runner), app)).await;
    info!("[BLE] active window elapsed");
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            warn!("[BLE][ble_task] error: {:?}", e);
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

/// Stream Events until the connection closes.
async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let level = server.battery_service.level;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(event) => {
                        if event.handle() == level.handle {
                            let value = server.get(&level);
                            info!("[BLE][gatt] read level characteristic: {:?}", value);
                        }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == level.handle {
                            info!("[BLE][gatt] write level characteristic: {:?}", event.data());
                        }
                    }
                    _ => {}
                };
                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[BLE][gatt] error sending response: {:?}", e),
                };
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    info!("[BLE][gatt] disconnected: {:?}", reason);
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
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
    info!("[BLE][adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[BLE][adv] connection established");
    Ok(conn)
}

/// Notify the connected central of a counter value and read RSSI every 2 seconds.
async fn custom_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut tick: u8 = 0;
    let level = server.battery_service.level;
    loop {
        tick = tick.wrapping_add(1);
        info!("[BLE][custom_task] notifying connection of tick {}", tick);
        if level.notify(conn, &tick).await.is_err() {
            info!("[BLE][custom_task] error notifying connection");
            break;
        };
        if let Ok(rssi) = conn.raw().rssi(stack).await {
            info!("[BLE][custom_task] RSSI: {:?}", rssi);
        } else {
            info!("[BLE][custom_task] error getting RSSI");
            break;
        };
        Timer::after_secs(2).await;
    }
}
