//! BLE GATT protocomm provisioning (Espressif `protocomm` over GATT notify).

use bt_hci_transport::{PacketToController, PacketToHost, Transport};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use esp_radio::ble::controller::{BleConnector, BleConnectorError};
use heapless::Vec as HVec;
use rand_core::{CryptoRng, RngCore};
use trouble_host::{
    advertise::{
        AdStructure,
        Advertisement,
        AdvertisementParameters,
        BR_EDR_NOT_SUPPORTED,
        LE_GENERAL_DISCOVERABLE,
    },
    att::AttErrorCode,
    gatt::{GattConnectionEvent, GattEvent},
    prelude::*,
};

use crate::{
    PROTO_VER_JSON,
    PROV_SERVICE_UUID_128,
    WifiCredentials,
    gatt,
    handlers::ProvCommState,
    session::creds_from_pending,
};

const ACL_SLOTS: usize = 8;
const CONNS: usize = 2;
const CHANNELS: usize = 4;

/// Borrowed HCI transport so [`BleConnector`] Drop (BT deinit) stays with the app.
struct BleTransport<'a>(&'a mut BleConnector<'static>);

impl embedded_io::ErrorType for BleTransport<'_> {
    type Error = BleConnectorError;
}

impl Transport for BleTransport<'_> {
    async fn read<'a, P: PacketToHost<'a>>(&self, rx: &'a mut [u8]) -> Result<P, Self::Error> {
        Transport::read(&*self.0, rx).await
    }

    async fn write<P: PacketToController>(&self, tx: &P) -> Result<(), Self::Error> {
        Transport::write(&*self.0, tx).await
    }
}

#[inline]
fn log_notify_err(ctx: &str, r: &Result<(), trouble_host::Error>) {
    if let Err(e) = r {
        warn!("[ble] notify {ctx} failed: {e:?}");
    }
}

async fn send_gatt_reply<P: PacketPool>(
    result: Result<trouble_host::gatt::Reply<'_, P>, trouble_host::Error>,
) {
    match result {
        Ok(reply) => {
            reply.send().await;
        }
        Err(e) => warn!("[ble] gatt accept failed: {e:?}"),
    }
}

/// Run the BLE GATT provisioning server until credentials are received.
///
/// Borrows the connector for the session; the caller owns Drop / BT deinit.
#[instability::unstable]
pub async fn run<R>(
    connector: &mut BleConnector<'static>,
    rng: &mut R,
    ble_name: &str,
    pop: &str,
) -> Option<WifiCredentials>
where
    R: RngCore + CryptoRng,
{
    let controller = ExternalController::<_, ACL_SLOTS>::new(BleTransport(connector));
    let mut resources = HostResources::<DefaultPacketPool, CONNS, CHANNELS>::new();

    let mut addr_bytes = [0u8; 6];
    rng.fill_bytes(&mut addr_bytes);
    addr_bytes[5] = (addr_bytes[5] & 0x3F) | 0xC0;

    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random(addr_bytes))
        .build();
    let mut peripheral = stack.peripheral();
    let mut runner = stack.runner();

    let mut session_buf = [0u8; 256];
    let mut config_buf = [0u8; 256];
    let mut ctrl_buf = [0u8; 32];
    let mut scan_buf = [0u8; 512];
    let mut ver_buf = [0u8; 128];
    let (server, session_char, config_char, scan_char, ver_char) = gatt::build_prov_gatt_server(
        &mut session_buf,
        &mut config_buf,
        &mut ctrl_buf,
        &mut scan_buf,
        &mut ver_buf,
    );
    let session_h = session_char.handle;
    let config_h = config_char.handle;
    let scan_h = scan_char.handle;
    let ver_h = ver_char.handle;

    let name_bytes = ble_name.as_bytes();
    let mut adv_data = [0u8; 31];
    let adv_len = match AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::Unknown {
                ty: 0x07,
                data: &PROV_SERVICE_UUID_128,
            },
        ],
        &mut adv_data,
    ) {
        Ok(n) => n,
        Err(e) => {
            error!("[ble] adv encode failed: {e:?}");
            return None;
        }
    };

    let mut scan_data = [0u8; 31];
    let scan_len = match AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(name_bytes)],
        &mut scan_data,
    ) {
        Ok(n) => n,
        Err(e) => {
            error!("[ble] scan response encode failed: {e:?}");
            return None;
        }
    };

    info!(
        "[ble] advertising '{ble_name}' — QR: {{\"ver\":\"v1\",\"name\":\"{ble_name}\",\"pop\":\"{pop}\",\"transport\":\"ble\"}}"
    );

    let runner_loop = async {
        loop {
            let _ = runner.run().await;
        }
        #[allow(unreachable_code)]
        None::<WifiCredentials>
    };

    let pop_bytes = pop.as_bytes();

    let prov_loop = async {
        loop {
            let adv_params = AdvertisementParameters::default();
            let advertiser = match peripheral
                .advertise(
                    &adv_params,
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data[..adv_len],
                        scan_data: &scan_data[..scan_len],
                    },
                )
                .await
            {
                Ok(a) => a,
                Err(e) => {
                    warn!("[ble] advertise failed: {e:?}");
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }
            };

            let conn = match advertiser.accept().await {
                Ok(c) => c,
                Err(e) => {
                    warn!("[ble] accept failed: {e:?}");
                    continue;
                }
            };
            info!("[ble] central connected");

            let gatt = match conn.with_attribute_server(&server) {
                Ok(g) => g,
                Err(e) => {
                    warn!("[ble] GATT attach failed: {e:?}");
                    continue;
                }
            };
            info!("BLE link up, waiting for prov-session.");

            let mut state = ProvCommState::new();
            let mut mode_announced = false;

            'conn: loop {
                match gatt.next().await {
                    GattConnectionEvent::Disconnected { reason } => {
                        info!(
                            "[ble] disconnected 0x{:02x} — re-advertising",
                            reason.into_inner()
                        );
                        break 'conn;
                    }
                    GattConnectionEvent::Gatt { event } => match event {
                        GattEvent::Write(ev) => {
                            let handle = ev.handle();
                            let mut req = [0u8; 256];
                            let req_len = ev.with_data(|_offset, data| {
                                let n = data.len().min(req.len());
                                req[..n].copy_from_slice(&data[..n]);
                                n
                            });
                            send_gatt_reply(ev.accept()).await;

                            if handle == session_h {
                                if !mode_announced {
                                    info!("Provisioning BLE mode selected.");
                                    mode_announced = true;
                                }
                                let mut resp_buf = [0u8; 256];
                                let wire = state.handle_session(
                                    &req[..req_len],
                                    pop_bytes,
                                    rng,
                                    &mut resp_buf,
                                    "[ble]",
                                );
                                let mut v = HVec::<u8, 256>::new();
                                let _ = v.extend_from_slice(wire.as_bytes());
                                log_notify_err(
                                    "session",
                                    &session_char.notify(&gatt, &v, true).await,
                                );
                            } else if handle == config_h {
                                let mut plain = [0u8; 256];
                                let n = req_len.min(plain.len());
                                plain[..n].copy_from_slice(&req[..n]);
                                state.decrypt_body(&mut plain, n);

                                let mut resp_buf = [0u8; 64];
                                let (done, resp_len) =
                                    state.handle_config(&plain[..n], &mut resp_buf, "[ble]");

                                state.encrypt_response(&mut resp_buf, resp_len);
                                let mut v = HVec::<u8, 256>::new();
                                let _ = v.extend_from_slice(&resp_buf[..resp_len]);
                                log_notify_err(
                                    if done { "config(done)" } else { "config" },
                                    &config_char.notify(&gatt, &v, true).await,
                                );

                                if done {
                                    wait_post_get_status(&gatt, config_h).await;
                                    return creds_from_pending(
                                        state.pending_ssid.as_str(),
                                        state.pending_pass.as_str(),
                                    );
                                }
                            } else if handle == scan_h {
                                let mut plain = [0u8; 64];
                                let n = req_len.min(plain.len());
                                plain[..n].copy_from_slice(&req[..n]);
                                state.decrypt_body(&mut plain, n);

                                let mut scan_resp = [0u8; 512];
                                let sr_len = state.handle_scan(&plain[..n], &mut scan_resp);
                                state.encrypt_response(&mut scan_resp, sr_len);
                                let mut v = HVec::<u8, 512>::new();
                                let _ = v.extend_from_slice(&scan_resp[..sr_len]);
                                log_notify_err("scan", &scan_char.notify(&gatt, &v, true).await);
                            } else if handle == ver_h {
                                let mut v = HVec::<u8, 128>::new();
                                let _ = v.extend_from_slice(PROTO_VER_JSON);
                                log_notify_err(
                                    "proto-ver",
                                    &ver_char.notify(&gatt, &v, true).await,
                                );
                            }
                        }
                        GattEvent::Read(ev) => {
                            send_gatt_reply(ev.accept()).await;
                        }
                        GattEvent::Other(ev) => {
                            send_gatt_reply(ev.accept()).await;
                        }
                        GattEvent::NotAllowed(ev) => {
                            send_gatt_reply(ev.accept()).await;
                        }
                    },
                    GattConnectionEvent::RequestConnectionParams(_) => {}
                    _ => {}
                }
            }

            Timer::after(Duration::from_millis(100)).await;
        }
    };

    match select(runner_loop, prov_loop).await {
        Either::First(_) => None,
        Either::Second(r) => r,
    }
}

/// After sending GetStatus, give the phone time to read the response and disconnect cleanly.
async fn wait_post_get_status<P: PacketPool>(gatt: &GattConnection<'_, '_, P>, config_h: u16) {
    info!("[ble] waiting for config READ / disconnect (max 10 s)");
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let now = Instant::now();
        if now >= deadline {
            warn!("[ble] post-GetStatus timeout — returning credentials");
            break;
        }
        let slice = deadline
            .saturating_duration_since(now)
            .min(Duration::from_millis(500));
        match select(gatt.next(), Timer::after(slice)).await {
            Either::Second(_) => continue,
            Either::First(evt) => match evt {
                GattConnectionEvent::Disconnected { reason } => {
                    info!(
                        "[ble] phone disconnected 0x{:02x} (provisioning complete)",
                        reason.into_inner()
                    );
                    break;
                }
                GattConnectionEvent::Gatt { event } => match event {
                    GattEvent::Read(ev) => {
                        let h = ev.handle();
                        send_gatt_reply(ev.accept()).await;
                        if h == config_h {
                            info!("[ble] GetStatus READ received");
                        }
                    }
                    GattEvent::Write(ev) => {
                        send_gatt_reply(ev.reject(AttErrorCode::REQUEST_NOT_SUPPORTED)).await;
                    }
                    GattEvent::Other(ev) => {
                        send_gatt_reply(ev.accept()).await;
                    }
                    GattEvent::NotAllowed(ev) => {
                        send_gatt_reply(ev.accept()).await;
                    }
                },
                _ => {}
            },
        }
    }
}
