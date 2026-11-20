//! Embassy Wi-Fi provisioning shell (SoftAP + BLE)
//!
//! Interactive console mirroring the Zephyr `esp_provisioning_shell` sample:
//!
//! 1) Start provisioning (BLE GATT and SoftAP HTTP until the first delivers credentials)
//! 2) Test Wi-Fi connection (last provisioning in RAM, else stored nvs blob)
//! 3) Show current credentials (confirm with `Y`)
//! 4) Save Wi-Fi credentials to the nvs partition and reboot
//! 5) Reboot only
//!
//! Defaults: SoftAP/BLE name `PROV_esp`, PoP `abcd1234`, SoftAP at `192.168.4.1/24`.
//! Use the Espressif ESP BLE Provisioning app (SoftAP or BLE transport).
//!
//! Menu input uses USB Serial/JTAG when available, otherwise UART0. Logging uses
//! `esp_println` (same port `espflash --monitor` attaches to).

//% CHIP_FILTER: wifi_driver_supported && bt_driver_supported

#![no_std]
#![no_main]

mod storage;

use core::future::pending;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::{Runner, StackResources};
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    Blocking,
    clock::CpuClock,
    flash::{Config as FlashConfig, Flash},
    ram,
    rng::Rng,
    system::software_reset,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_provisioning::{
    CryptoRngWrap,
    DEFAULT_AP_GATEWAY,
    DEFAULT_POP,
    DEFAULT_PROV_NAME,
    DEFAULT_TIMEOUT_SECS,
    ProvConfig,
    ProvError,
    WifiCredentials,
    cancel,
    cancel_isr,
    run,
};
use esp_radio::{
    ble::controller::BleConnector,
    wifi::{
        AuthenticationMethodConfig,
        Config,
        ControllerConfig,
        Interface,
        Password,
        Ssid,
        WifiController,
        ap::AccessPointConfig,
        sta::StationConfig,
    },
};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const DRAM2_UNINIT_SIZE: usize = {
    let range: core::ops::Range<usize> = esp_metadata_generated::memory_range!("DRAM2_UNINIT");
    range.end - range.start
};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

struct AppState {
    last_prov: Option<WifiCredentials>,
}

// Classic ESP32 uses UART0 for the USB-UART bridge; other coex chips use USB Serial/JTAG.
#[cfg(feature = "esp32")]
type ConsoleRx = esp_hal::uart::UartRx<'static, esp_hal::Async>;
#[cfg(not(feature = "esp32"))]
type ConsoleRx = esp_hal::usb::usb_serial_jtag::UsbSerialJtagRx<'static, esp_hal::Async>;

async fn read_byte(rx: &mut ConsoleRx) -> u8 {
    let mut b = [0u8; 1];
    loop {
        match rx.read(&mut b).await {
            Ok(0) => Timer::after(Duration::from_millis(10)).await,
            Ok(_) => return b[0],
            Err(_) => Timer::after(Duration::from_millis(10)).await,
        }
    }
}

fn print_menu() {
    println!();
    println!("1) Start provisioning");
    println!("2) Test WiFi connection");
    println!("3) Show current credentials");
    println!("4) Save WiFi credentials and reboot");
    println!("5) Reboot only");
    println!("(Enter or Esc: show menu again)");
    println!("Enter choice 1-5: ");
}

async fn read_menu_digit(rx: &mut ConsoleRx) -> char {
    loop {
        let c = read_byte(rx).await;
        if (b'1'..=b'5').contains(&c) {
            println!("{}", c as char);
            return c as char;
        }
        if c == b'\r' || c == b'\n' || c == 0x1b {
            print_menu();
            continue;
        }
        println!("\nInvalid key.");
        print_menu();
    }
}

async fn confirm_secrets(rx: &mut ConsoleRx) -> bool {
    println!("Show SSID/PSK in plaintext? Press Y to confirm: ");
    loop {
        let c = read_byte(rx).await;
        if c == b'Y' || c == b'y' {
            println!("Y");
            return true;
        }
        if c == b'N' || c == b'n' || c == b'\r' || c == b'\n' || c == 0x1b {
            println!("cancelled");
            return false;
        }
    }
}

/// Drain auto-repeat `C`/`c` until ~200 ms with none, so the menu does not see them.
async fn wait_for_c_release(rx: &mut ConsoleRx) {
    println!("Release C");
    loop {
        match select(read_byte(rx), Timer::after(Duration::from_millis(200))).await {
            Either::First(b) if b == b'C' || b == b'c' => {}
            Either::First(_) => {}
            Either::Second(_) => break,
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await;
}

async fn option_provisioning(
    state: &mut AppState,
    controller: &mut WifiController<'_>,
    ap_device: Interface,
    spawner: &Spawner,
    connector: &mut BleConnector<'static>,
    console_rx: &mut ConsoleRx,
) {
    println!("Starting SoftAP + BLE provisioning (first transport to finish wins)...");
    println!(
        "SoftAP SSID '{}', PoP '{}', BLE name '{}'",
        DEFAULT_PROV_NAME, DEFAULT_POP, DEFAULT_PROV_NAME
    );
    println!("Press Esc to cancel");
    println!("Hold C to cancel");

    let mut rng_ap = CryptoRngWrap(Rng::new());
    let mut rng_ble = CryptoRngWrap(Rng::new());

    let config = ProvConfig {
        pop: DEFAULT_POP,
        softap_ssid: DEFAULT_PROV_NAME,
        ble_name: DEFAULT_PROV_NAME,
        ap_gateway: DEFAULT_AP_GATEWAY,
        timeout: Duration::from_secs(DEFAULT_TIMEOUT_SECS),
        controller,
        ap_device,
        ble_connector: connector,
        spawner,
        rng: &mut rng_ap,
        rng_ble: &mut rng_ble,
    };

    // Esc = event-driven cancel_isr; hold C (~1 s of repeats) = polled cancel.
    let mut held_c_cancel = false;
    let console_cancel = async {
        let mut held_c = 0u32;
        loop {
            match select(
                read_byte(console_rx),
                Timer::after(Duration::from_millis(50)),
            )
            .await
            {
                Either::First(0x1b) => {
                    cancel_isr();
                    pending::<()>().await;
                }
                Either::First(b) if b == b'C' || b == b'c' => {
                    held_c = held_c.saturating_add(1);
                    if held_c >= 20 {
                        held_c_cancel = true;
                        cancel();
                        pending::<()>().await;
                    }
                }
                Either::First(_) => held_c = 0,
                Either::Second(_) => held_c = 0,
            }
        }
    };

    match select(run(config), console_cancel).await {
        Either::Second(_) => unreachable!(),
        Either::First(Ok(creds)) => {
            println!("Provisioning OK — SSID '{}'", creds.ssid.as_str());
            state.last_prov = Some(creds);
        }
        Either::First(Err(ProvError::Timeout)) => println!("Provisioning timed out"),
        Either::First(Err(ProvError::Cancelled)) => {
            println!("Provisioning cancelled");
            if held_c_cancel {
                wait_for_c_release(console_rx).await;
            }
        }
        Either::First(Err(ProvError::Internal)) => {
            println!("Provisioning ended without credentials (internal error)")
        }
    }
}

async fn option_test_wifi(
    state: &AppState,
    flash: &mut Flash<'_, Blocking>,
    controller: &mut WifiController<'_>,
    sta_device: Interface,
    spawner: &Spawner,
) {
    let creds = if let Some(c) = state.last_prov.clone() {
        c
    } else if let Some((s, p)) = storage::load(flash) {
        println!("Using credentials from nvs partition");
        let Ok(ssid) = Ssid::try_from(s.as_str()) else {
            println!("Stored SSID invalid");
            return;
        };
        WifiCredentials { ssid, password: p }
    } else {
        println!("No credentials. Run 1) provisioning first (then optionally 4) save).");
        return;
    };

    println!("Connecting to '{}'...", creds.ssid.as_str());
    let sta_cfg = if creds.password.is_empty() {
        Config::Station(
            StationConfig::default()
                .with_ssid(creds.ssid)
                .with_authentication(AuthenticationMethodConfig::Open),
        )
    } else {
        let Ok(password) = Password::try_from(creds.password.as_str()) else {
            println!("Password invalid");
            return;
        };
        Config::Station(
            StationConfig::default()
                .with_ssid(creds.ssid)
                .with_authentication(AuthenticationMethodConfig::Wpa2Personal(password)),
        )
    };
    if let Err(e) = controller.set_config(&sta_cfg) {
        println!("set_config failed: {:?}", e);
        return;
    }

    let net_cfg = embassy_net::Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let (stack, runner) = embassy_net::new(
        sta_device,
        net_cfg,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );
    spawner.spawn(net_task(runner).unwrap());

    let deadline = embassy_time::Instant::now() + Duration::from_secs(30);
    while embassy_time::Instant::now() < deadline {
        if stack.is_config_up() {
            if let Some(cfg) = stack.config_v4() {
                println!("STA up: {}", cfg.address);
                return;
            }
        }
        Timer::after(Duration::from_millis(250)).await;
    }
    println!("STA connect timed out (30 s)");
}

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: DRAM2_UNINIT_SIZE);
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    #[cfg(feature = "esp32")]
    let mut console_rx = {
        use esp_hal::uart::{Config as UartConfig, Uart};
        Uart::new(peripherals.UART0, UartConfig::default())
            .unwrap()
            .into_async()
            .split()
            .0
    };

    #[cfg(not(feature = "esp32"))]
    let mut console_rx = {
        use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;
        UsbSerialJtag::new(peripherals.USB_DEVICE)
            .into_async()
            .split()
            .0
    };

    let mut flash = Flash::new(peripherals.FLASH, FlashConfig::default()).unwrap();
    let mut state = AppState { last_prov: None };

    let mut sta_device = Some(Interface::station());
    let mut controller = WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(Config::AccessPoint(
            AccessPointConfig::default().with_ssid(DEFAULT_PROV_NAME.try_into().unwrap()),
        )),
    )
    .unwrap();

    let mut ble_connector =
        BleConnector::new(peripherals.BT, esp_radio::ble::Config::default()).unwrap();

    println!("embassy_provisioning ready");
    println!("Transports: BLE + SoftAP");

    loop {
        print_menu();
        match read_menu_digit(&mut console_rx).await {
            '1' => {
                let Some(ap) = Interface::try_access_point() else {
                    println!("SoftAP interface already in use — try again after the session ends");
                    continue;
                };
                option_provisioning(
                    &mut state,
                    &mut controller,
                    ap,
                    &spawner,
                    &mut ble_connector,
                    &mut console_rx,
                )
                .await;
            }
            '2' => {
                let Some(sta) = sta_device.take() else {
                    println!("STA interface already consumed — reboot (5) and try again");
                    continue;
                };
                option_test_wifi(&state, &mut flash, &mut controller, sta, &spawner).await;
            }
            '3' => {
                let ok = confirm_secrets(&mut console_rx).await;
                if !ok {
                    continue;
                }
                if let Some(creds) = &state.last_prov {
                    println!("RAM (last provisioning):");
                    println!("  SSID: {}", creds.ssid.as_str());
                    println!("  PSK:  {}", creds.password.as_str());
                } else {
                    println!("RAM (last provisioning): (none)");
                }
                match storage::load(&mut flash) {
                    Some((ssid, pass)) => {
                        println!("Stored (nvs blob):");
                        println!("  SSID: {}", ssid.as_str());
                        println!("  PSK:  {}", pass.as_str());
                    }
                    None => println!("Stored (nvs blob): (none)"),
                }
            }
            '4' => {
                if let Some(creds) = &state.last_prov {
                    if storage::save(&mut flash, creds.ssid.as_str(), creds.password.as_str())
                        .is_err()
                    {
                        println!("Failed to save credentials");
                        continue;
                    }
                    println!("Rebooting...");
                    Timer::after(Duration::from_millis(200)).await;
                    software_reset();
                } else {
                    println!("No new provisioning detected, nothing to save.");
                }
            }
            '5' => {
                println!("Rebooting...");
                Timer::after(Duration::from_millis(200)).await;
                software_reset();
            }
            _ => {}
        }
    }
}
