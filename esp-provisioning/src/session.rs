//! Combined SoftAP + BLE provisioning session (`run` / `cancel` / `cancel_isr`).

use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_radio::{
    ble::controller::BleConnector,
    wifi::{Interface, Ssid, WifiController},
};
use heapless::String;
use rand_core::{CryptoRng, RngCore};

use crate::{
    DEFAULT_AP_GATEWAY,
    DEFAULT_POP,
    DEFAULT_PROV_NAME,
    DEFAULT_TIMEOUT_SECS,
    softap::SoftApSession,
};

static CANCEL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Wi-Fi credentials returned after a successful provisioning session.
///
/// [`Ssid`] is the esp-radio type so STA config can use it directly. The passphrase
/// stays a `String<64>` because `esp_radio::wifi::Password` does not expose bytes to
/// dependents; convert with `Password::try_from(password.as_str())` when joining STA.
#[derive(Clone, Debug)]
#[instability::unstable]
pub struct WifiCredentials {
    /// Network SSID.
    pub ssid: Ssid,
    /// WPA passphrase (empty for open networks).
    pub password: String<64>,
}

/// Outcome of [`run`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[instability::unstable]
pub enum ProvError {
    /// No transport delivered credentials before the configured timeout.
    Timeout,
    /// [`cancel`] or [`cancel_isr`] was called while a session was active.
    Cancelled,
    /// All active transports failed without delivering credentials.
    Internal,
}

/// Configuration for [`run`].
#[instability::unstable]
pub struct ProvConfig<'a, 'b, R, RB = R> {
    /// Proof-of-possession for sec1 (default [`DEFAULT_POP`]).
    pub pop: &'a str,
    /// SoftAP SSID (default [`DEFAULT_PROV_NAME`]).
    pub softap_ssid: &'a str,
    /// BLE advertised name (default [`DEFAULT_PROV_NAME`]).
    pub ble_name: &'a str,
    /// SoftAP gateway / device IP (default [`DEFAULT_AP_GATEWAY`]).
    pub ap_gateway: Ipv4Addr,
    /// Session timeout (default [`DEFAULT_TIMEOUT_SECS`]).
    pub timeout: Duration,
    /// Wi-Fi controller (shared by SoftAP path for scan + AP).
    pub controller: &'a mut WifiController<'b>,
    /// SoftAP network interface (used for the SoftAP session; released when the session ends).
    #[cfg(feature = "softap")]
    pub ap_device: Interface,
    /// BLE controller connector (borrowed for the BLE session; caller owns Drop / deinit).
    #[cfg(feature = "ble")]
    pub ble_connector: &'a mut BleConnector<'static>,
    /// Embassy spawner. SoftAP drives its network inside [`crate::softap::run`]; the
    /// example still uses this for the STA test.
    pub spawner: &'a Spawner,
    /// RNG for the SoftAP sec1 handshake.
    pub rng: &'a mut R,
    /// RNG for the BLE sec1 handshake (required when both `softap` and `ble` are enabled).
    #[cfg(all(feature = "softap", feature = "ble"))]
    pub rng_ble: &'a mut RB,
}

impl<'a, 'b, R, RB> ProvConfig<'a, 'b, R, RB>
where
    R: RngCore + CryptoRng,
    RB: RngCore + CryptoRng,
{
    /// Fills unset fields with library defaults.
    pub fn with_defaults(mut self) -> Self {
        if self.pop.is_empty() {
            self.pop = DEFAULT_POP;
        }
        if self.softap_ssid.is_empty() {
            self.softap_ssid = DEFAULT_PROV_NAME;
        }
        if self.ble_name.is_empty() {
            self.ble_name = DEFAULT_PROV_NAME;
        }
        if self.ap_gateway == Ipv4Addr::UNSPECIFIED {
            self.ap_gateway = DEFAULT_AP_GATEWAY;
        }
        if self.timeout == Duration::from_secs(0) {
            self.timeout = Duration::from_secs(DEFAULT_TIMEOUT_SECS);
        }
        self
    }
}

/// Request cancellation of an in-flight [`run`] from task / thread context.
///
/// Only sets the stop signal. Does not stop SoftAP, tear down Wi-Fi, or deinit BT —
/// the application owns radio policy after [`run`] returns.
#[instability::unstable]
pub fn cancel() {
    CANCEL.signal(());
}

/// Request cancellation of an in-flight [`run`] from interrupt context.
///
/// Only sets the stop signal (no socket or radio work). Same outcome as [`cancel`].
#[instability::unstable]
pub fn cancel_isr() {
    CANCEL.signal(());
}

pub(crate) fn creds_from_pending(ssid: &str, pass: &str) -> Option<WifiCredentials> {
    let mut password = String::new();
    password.push_str(pass).ok()?;
    Some(WifiCredentials {
        ssid: Ssid::try_from(ssid).ok()?,
        password,
    })
}

type CredsPair = Option<WifiCredentials>;

/// Run SoftAP and/or BLE provisioning until credentials arrive, timeout, or cancel.
///
/// When both transports are enabled, the first to deliver credentials wins.
/// SoftAP releases the AP interface singleton when its future ends; BLE borrows the
/// connector so the caller retains Drop / BT deinit. STA join stays with the application.
#[instability::unstable]
pub async fn run<'a, 'b, R, RB>(
    config: ProvConfig<'a, 'b, R, RB>,
) -> Result<WifiCredentials, ProvError>
where
    R: RngCore + CryptoRng,
    RB: RngCore + CryptoRng,
{
    CANCEL.reset();

    let config = config.with_defaults();
    let mut deadline = Timer::after(config.timeout);

    #[cfg(all(feature = "softap", feature = "ble"))]
    {
        let softap_session = SoftApSession {
            pop: config.pop,
            ssid: config.softap_ssid,
            gateway: config.ap_gateway,
            timeout: config.timeout,
        };
        let mut softap_fut = core::pin::pin!(crate::softap::run(
            config.controller,
            config.ap_device,
            config.rng,
            &softap_session,
        ));
        let _ = config.spawner;
        let mut ble_fut = core::pin::pin!(crate::ble::run(
            config.ble_connector,
            config.rng_ble,
            config.ble_name,
            config.pop,
        ));
        let mut softap_active = true;
        let mut ble_active = true;

        loop {
            let mut cancel_wait = CANCEL.wait();
            let transport_done = match (softap_active, ble_active) {
                (true, true) => {
                    let mut race = select(softap_fut.as_mut(), ble_fut.as_mut());
                    match select(select(&mut deadline, &mut cancel_wait), &mut race).await {
                        Either::First(Either::First(_)) => return Err(ProvError::Timeout),
                        Either::First(Either::Second(_)) => return Err(ProvError::Cancelled),
                        Either::Second(Either::First(r)) => TransportDone::SoftAp(r),
                        Either::Second(Either::Second(r)) => TransportDone::Ble(r),
                    }
                }
                (true, false) => {
                    match select(select(&mut deadline, &mut cancel_wait), softap_fut.as_mut()).await
                    {
                        Either::First(Either::First(_)) => return Err(ProvError::Timeout),
                        Either::First(Either::Second(_)) => return Err(ProvError::Cancelled),
                        Either::Second(r) => TransportDone::SoftAp(r),
                    }
                }
                (false, true) => {
                    match select(select(&mut deadline, &mut cancel_wait), ble_fut.as_mut()).await {
                        Either::First(Either::First(_)) => return Err(ProvError::Timeout),
                        Either::First(Either::Second(_)) => return Err(ProvError::Cancelled),
                        Either::Second(r) => TransportDone::Ble(r),
                    }
                }
                (false, false) => TransportDone::Exhausted,
            };

            match transport_done {
                TransportDone::SoftAp(Some(creds)) => return Ok(creds),
                TransportDone::Ble(Some(creds)) => return Ok(creds),
                TransportDone::SoftAp(None) => {
                    softap_active = false;
                    if !ble_active {
                        return Err(ProvError::Internal);
                    }
                }
                TransportDone::Ble(None) => {
                    ble_active = false;
                    if !softap_active {
                        return Err(ProvError::Internal);
                    }
                }
                TransportDone::Exhausted => return Err(ProvError::Internal),
            }
        }
    }

    #[cfg(all(feature = "softap", not(feature = "ble")))]
    {
        let softap_session = SoftApSession {
            pop: config.pop,
            ssid: config.softap_ssid,
            gateway: config.ap_gateway,
            timeout: config.timeout,
        };
        let mut softap_fut = core::pin::pin!(crate::softap::run(
            config.controller,
            config.ap_device,
            config.rng,
            &softap_session,
        ));
        let _ = config.spawner;
        let mut cancel_wait = CANCEL.wait();
        match select(select(&mut deadline, &mut cancel_wait), softap_fut.as_mut()).await {
            Either::First(Either::First(_)) => Err(ProvError::Timeout),
            Either::First(Either::Second(_)) => Err(ProvError::Cancelled),
            Either::Second(Some(creds)) => Ok(creds),
            Either::Second(None) => Err(ProvError::Internal),
        }
    }

    #[cfg(all(feature = "ble", not(feature = "softap")))]
    {
        let mut ble_fut = core::pin::pin!(crate::ble::run(
            config.ble_connector,
            config.rng,
            config.ble_name,
            config.pop,
        ));
        let mut cancel_wait = CANCEL.wait();
        match select(select(&mut deadline, &mut cancel_wait), ble_fut.as_mut()).await {
            Either::First(Either::First(_)) => Err(ProvError::Timeout),
            Either::First(Either::Second(_)) => Err(ProvError::Cancelled),
            Either::Second(Some(creds)) => Ok(creds),
            Either::Second(None) => Err(ProvError::Internal),
        }
    }
}

enum TransportDone {
    SoftAp(CredsPair),
    Ble(CredsPair),
    Exhausted,
}
