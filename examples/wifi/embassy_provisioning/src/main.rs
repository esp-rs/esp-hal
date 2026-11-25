//! SoftAP and BLE Wi-Fi provisioning.
//!
//! Proof of possession defaults to `abcd1234`. Enable `softap`, `ble`, or both.
//! The Espressif BLE Provisioning app uses whichever transport is compiled in.
//!
//! The application owns the Wi-Fi controller, `embassy-net` stacks, BLE connector,
//! pre-scan list, and station verify loop. [`esp_provisioning::softap::serve`] and
//! [`esp_provisioning::ble::serve`] run concurrently with the verify loop so
//! `GetStatus` reports real association + DHCP status and failed attempts can be
//! retried via `prov-ctrl` without dropping the link.
//!
//! Protobuf schema: ESP-IDF v5.4.2 `wifi_provisioning` and `protocomm` (not
//! `network_provisioning`).

//% CHIP_FILTER: wifi_driver_supported && bt_driver_supported

#![no_std]
#![no_main]

#[cfg(not(any(feature = "ble", feature = "softap")))]
compile_error!("enable the `ble` feature, the `softap` feature, or both");

use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
#[cfg(feature = "softap")]
use embassy_net::{Ipv4Cidr, StaticConfigV4};
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::{Duration, Timer, with_timeout};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, ram, rng::TrngSource, timer::timg::TimerGroup};
use esp_println::println;
#[cfg(feature = "ble")]
use esp_provisioning::ble;
#[cfg(feature = "softap")]
use esp_provisioning::softap;
use esp_provisioning::{Config, Session, StaFailReason, StaState, WifiCredentials};
#[cfg(feature = "ble")]
use esp_radio::ble::controller::BleConnector;
#[cfg(feature = "softap")]
use esp_radio::wifi::ap::AccessPointConfig;
use esp_radio::wifi::{
    AuthenticationMethod,
    AuthenticationMethodConfig,
    Config as WifiConfig,
    ConnectionError,
    ControllerConfig,
    DisconnectReason,
    Interface,
    Password,
    WifiController,
    scan::{AccessPointInfo, ScanConfig, ScanTypeConfig},
    sta::{ScanMethod, StationConfig},
};
use heapless::Vec;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const DRAM2_UNINIT_SIZE: usize = {
    let range: core::ops::Range<usize> = esp_metadata_generated::memory_range!("DRAM2_UNINIT");
    range.end - range.start
};

/// Overall provisioning window before the example gives up.
const PROVISIONING_TIMEOUT: Duration = Duration::from_secs(300);

/// Maximum wait for a DHCPv4 lease after STA association.
const DHCP_TIMEOUT: Duration = Duration::from_secs(30);

/// Matches ESP-IDF's `AUTOSTOP_TIMEOUT` so the phone has time to read `Connected`
/// even if the link is slow, while exiting cleanly if the phone vanished.
const AUTOSTOP_TIMEOUT: Duration = Duration::from_secs(30);

/// Coexistence retries on transient association/beacon drops before reporting failure.
const MAX_CONNECT_ATTEMPTS: usize = 3;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let hal_config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(hal_config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: DRAM2_UNINIT_SIZE);
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let _entropy = TrngSource::new(peripherals.RNG, peripherals.ADC1);
    let trng = esp_hal::rng::Trng::try_new().expect("TRNG source is enabled");

    let config = Config::default();
    println!(
        "Provisioning SoftAP '{}' BLE '{}' gateway {}",
        config.softap_ssid().as_str(),
        config.ble_name().as_str(),
        config.gateway()
    );
    println!(
        "{{\"ver\":\"v1\",\"name\":\"{}\",\"pop\":\"{}\",\"transport\":\"softap\"}}",
        config.softap_ssid().as_str(),
        config.pop().as_str()
    );
    println!(
        "{{\"ver\":\"v1\",\"name\":\"{}\",\"pop\":\"{}\",\"transport\":\"ble\"}}",
        config.ble_name().as_str(),
        config.pop().as_str()
    );

    let mut controller =
        WifiController::new(peripherals.WIFI, ControllerConfig::default()).unwrap();

    // Station mode is required to scan nearby networks before SoftAP starts.
    controller
        .set_config(&WifiConfig::Station(StationConfig::default()))
        .expect("STA config");
    let scan_config = ScanConfig::default()
        .with_max(16)
        .with_scan_type(ScanTypeConfig::Active {
            min: esp_hal::time::Duration::from_millis(100),
            max: esp_hal::time::Duration::from_millis(300),
        });
    let scanned = controller
        .scan_async(&scan_config)
        .await
        .unwrap_or_else(|err| {
            println!("Scan failed: {err:?}");
            Default::default()
        });
    println!("Scan found {} APs", scanned.len());
    let mut aps: Vec<AccessPointInfo, 16> = Vec::new();
    for ap in scanned {
        println!(
            "  '{}' ch {} rssi {} auth {:?}",
            ap.ssid.as_str(),
            ap.channel,
            ap.signal_strength,
            ap.auth_method,
        );
        if aps.push(ap).is_err() {
            break;
        }
    }

    // Create the reusable STA network stack once so failed retries do not leak static resources.
    let sta_seed = ((trng.random() as u64) << 32) | trng.random() as u64;
    let (sta_stack, sta_runner) = embassy_net::new(
        Interface::station(),
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<4>, StackResources::<4>::new()),
        sta_seed,
    );
    spawner.spawn(net_task(sta_runner).unwrap());

    // SoftAP keeps the radio in AP+STA from here through station verification.
    #[cfg(feature = "softap")]
    let (ap_config, ap_stack) = {
        let ap = AccessPointConfig::default().with_ssid(config.softap_ssid());
        let open_sta =
            StationConfig::default().with_authentication(AuthenticationMethodConfig::Open);
        controller
            .set_config(&WifiConfig::AccessPointStation(open_sta, ap.clone()))
            .expect("APSTA config");
        // 0.25 dBm units. 44 is 11 dBm (WifiController::new defaults to 20 / 5 dBm).
        controller.set_max_tx_power(44).expect("TX power");

        let ap_seed = ((trng.random() as u64) << 32) | trng.random() as u64;
        let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(config.gateway(), 24),
            gateway: Some(config.gateway()),
            dns_servers: Default::default(),
        });
        let (ap_stack, ap_runner) = embassy_net::new(
            Interface::access_point(),
            net_config,
            mk_static!(StackResources<6>, StackResources::<6>::new()),
            ap_seed,
        );
        spawner.spawn(net_task(ap_runner).unwrap());
        ap_stack.wait_config_up().await;
        (ap, ap_stack)
    };
    #[cfg(not(feature = "softap"))]
    {
        controller.set_max_tx_power(44).expect("TX power");
    }

    let session = Session::new();
    #[cfg(feature = "ble")]
    let mut ble_connector =
        BleConnector::new(peripherals.BT, esp_radio::ble::Config::default()).unwrap();

    let serves = async {
        #[cfg(all(feature = "softap", feature = "ble"))]
        {
            match select(
                softap::serve(ap_stack, &aps, &config, &session, trng.clone()),
                ble::serve(&mut ble_connector, &aps, &config, &session, trng.clone()),
            )
            .await
            {
                Either::First(res) | Either::Second(res) => res,
            }
        }
        #[cfg(all(feature = "softap", not(feature = "ble")))]
        {
            softap::serve(ap_stack, &aps, &config, &session, trng.clone()).await
        }
        #[cfg(all(feature = "ble", not(feature = "softap")))]
        {
            ble::serve(&mut ble_connector, &aps, &config, &session, trng.clone()).await
        }
    };

    let verify = verify_loop(
        &session,
        &mut controller,
        sta_stack,
        &aps,
        #[cfg(feature = "softap")]
        &ap_config,
    );

    match with_timeout(PROVISIONING_TIMEOUT, select(serves, verify)).await {
        Ok(Either::First(Ok(()))) => {
            if let Some(cfg) = sta_stack.config_v4() {
                log::info!("Provisioning complete; STA IP: {}", cfg.address);
            } else {
                log::info!("Provisioning complete");
            }
        }
        Ok(Either::First(Err(err))) => {
            log::warn!("Provisioning transport error: {err}");
        }
        Ok(Either::Second(ipv4)) => {
            log::info!("Provisioning complete (autostop); STA IP: {ipv4}");
        }
        Err(_) => {
            log::warn!(
                "Provisioning timed out after {} s",
                PROVISIONING_TIMEOUT.as_secs()
            );
        }
    }

    // Keep controller (and BLE connector if enabled) alive so the verified STA stays up.
    let _controller = controller;
    #[cfg(feature = "ble")]
    let _ble = ble_connector;
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

async fn verify_loop(
    session: &Session,
    controller: &mut WifiController<'static>,
    sta_stack: Stack<'static>,
    aps: &[AccessPointInfo],
    #[cfg(feature = "softap")] ap: &AccessPointConfig,
) -> Ipv4Addr {
    loop {
        let creds = session.credentials().await;
        log::info!(
            "Verifying SSID '{}' (pass_len={})",
            creds.ssid.as_str(),
            creds.password.len()
        );

        let sta = match build_station_config(&creds, aps) {
            Ok(sta) => sta,
            Err(reason) => {
                log::warn!(
                    "Invalid credentials for '{}': {:?}",
                    creds.ssid.as_str(),
                    reason
                );
                session.report(StaState::Failed(reason));
                continue;
            }
        };

        #[cfg(feature = "softap")]
        let wifi_cfg = WifiConfig::AccessPointStation(sta, ap.clone());
        #[cfg(not(feature = "softap"))]
        let wifi_cfg = WifiConfig::Station(sta);

        if let Err(err) = controller.set_config(&wifi_cfg) {
            log::warn!("set_config failed: {err:?}");
            session.report(StaState::Failed(StaFailReason::AuthError));
            continue;
        }
        let _ = controller.set_max_tx_power(44);

        match connect_with_retries(controller, sta_stack).await {
            Ok((ipv4, info)) => {
                log::info!("STA verified: {ipv4} (ch {})", info.channel);
                session.report(StaState::Connected { ipv4, info });
                // Give the active transport time to deliver Connected GetStatus and win the
                // outer select; exit cleanly after AUTOSTOP_TIMEOUT if the phone vanished.
                Timer::after(AUTOSTOP_TIMEOUT).await;
                return ipv4;
            }
            Err(reason) => {
                log::warn!("STA verify failed: {reason:?}");
                let _ = controller.disconnect_async().await;
                session.report(StaState::Failed(reason));
            }
        }
    }
}

fn build_station_config(
    creds: &WifiCredentials,
    aps: &[AccessPointInfo],
) -> Result<StationConfig, StaFailReason> {
    let auth = if creds.password.is_empty() {
        AuthenticationMethodConfig::Open
    } else {
        let password =
            Password::try_from(creds.password.as_str()).map_err(|_| StaFailReason::AuthError)?;
        let scanned_auth = aps
            .iter()
            .filter(|ap| ap.ssid == creds.ssid)
            .max_by_key(|ap| ap.signal_strength)
            .and_then(|ap| ap.auth_method);

        match scanned_auth {
            Some(AuthenticationMethod::Wep) => AuthenticationMethodConfig::Wep(password),
            Some(AuthenticationMethod::Wpa) => AuthenticationMethodConfig::Wpa(password),
            Some(AuthenticationMethod::Wpa2Personal) => {
                AuthenticationMethodConfig::Wpa2Personal(password)
            }
            Some(AuthenticationMethod::WpaWpa2Personal) => {
                AuthenticationMethodConfig::WpaWpa2Personal(password)
            }
            Some(AuthenticationMethod::Wpa3Personal) => {
                AuthenticationMethodConfig::Wpa3Personal(password)
            }
            Some(AuthenticationMethod::Wpa2Wpa3Personal) => {
                AuthenticationMethodConfig::Wpa2Wpa3Personal(password)
            }
            _ => AuthenticationMethodConfig::WpaWpa2Personal(password),
        }
    };

    Ok(StationConfig::default()
        .with_ssid(creds.ssid.clone())
        .with_authentication(auth)
        .with_scan_method(ScanMethod::AllChannels)
        .with_failure_retry_cnt(2))
}

async fn connect_with_retries(
    controller: &mut WifiController<'static>,
    sta_stack: Stack<'static>,
) -> Result<(Ipv4Addr, esp_radio::wifi::sta::ConnectedInfo), StaFailReason> {
    for attempt in 1..=MAX_CONNECT_ATTEMPTS {
        match controller.connect_async().await {
            Ok(info) => {
                log::info!(
                    "STA associated on channel {}, waiting for DHCP",
                    info.channel
                );
                match with_timeout(DHCP_TIMEOUT, sta_stack.wait_config_up()).await {
                    Ok(()) => {
                        if let Some(cfg) = sta_stack.config_v4() {
                            return Ok((cfg.address.address(), info));
                        }
                        return Err(StaFailReason::AuthError);
                    }
                    Err(_) => {
                        log::warn!("DHCP timed out after {} s", DHCP_TIMEOUT.as_secs());
                        return Err(StaFailReason::AuthError);
                    }
                }
            }
            Err(ConnectionError::Failed(disc)) => {
                log::warn!(
                    "STA connect attempt {attempt}/{MAX_CONNECT_ATTEMPTS} failed: {:?}",
                    disc.reason
                );
                if is_transient_coex_reason(disc.reason) && attempt < MAX_CONNECT_ATTEMPTS {
                    Timer::after(Duration::from_millis(500)).await;
                    continue;
                }
                let reason = match disc.reason {
                    DisconnectReason::NoAccessPointFound => StaFailReason::NetworkNotFound,
                    _ => StaFailReason::AuthError,
                };
                return Err(reason);
            }
            Err(err) => {
                log::warn!("STA connect error: {err:?}");
                return Err(StaFailReason::AuthError);
            }
        }
    }
    Err(StaFailReason::AuthError)
}

fn is_transient_coex_reason(reason: DisconnectReason) -> bool {
    matches!(
        reason,
        DisconnectReason::AuthenticationExpired
            | DisconnectReason::DisassociatedDueToInactivity
            | DisconnectReason::AssociationFailed
            | DisconnectReason::BeaconTimeout
            | DisconnectReason::Timeout
            | DisconnectReason::Unspecified
    )
}
