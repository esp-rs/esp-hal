//! BLE GATT protocomm (Write-then-Read).
//!
//! The application keeps [`BleConnector`] so Bluetooth deinit stays with the caller.

use bt_hci_transport::{PacketToController, PacketToHost, Transport};
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_time::{Duration, Instant, Timer};
use esp_radio::{
    ble::controller::{BleConnector, BleConnectorError},
    wifi::scan::AccessPointInfo,
};
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
    prelude::*,
};

use crate::{
    Config,
    ProvError,
    Session,
    gatt::ProvServer,
    handlers::{ConnState, Endpoint},
    sec1::{self, Sec1},
    types::{PROTO_VER_JSON, PROV_SERVICE_UUID_128},
};

const ACL_SLOTS: usize = 8;
const CONNS: usize = 2;
const CHANNELS: usize = 4;

/// How long to wait for the phone to issue `GattEvent::Read` after a `Connected` `GetStatus`.
const READ_DEADLINE: Duration = Duration::from_secs(3);

/// Short grace window after `prov-config` is read so the phone can disconnect cleanly.
const DISCONNECT_GRACE: Duration = Duration::from_secs(3);

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

/// Serves the provisioning GATT service until [`Session`] finishes or this future is dropped.
///
/// `aps` is the same scan list SoftAP serves.
///
/// # Errors
///
/// [`ProvError::Transport`] when the host runner stops or advertisement data does not encode.
///
/// # Cancellation Safety
///
/// Dropping this future stops advertising. The other transport and the session keep running.
/// The caller's [`BleConnector`] drop deinits Bluetooth.
#[instability::unstable]
pub async fn serve<R>(
    connector: &mut BleConnector<'static>,
    aps: &[AccessPointInfo],
    config: &Config,
    session: &Session,
    mut rng: R,
) -> Result<(), ProvError>
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

    let server = ProvServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: config.ble_name().as_str(),
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .map_err(|_| ProvError::Transport)?;

    let mut proto_ver = HVec::<u8, 128>::new();
    let _ = proto_ver.extend_from_slice(PROTO_VER_JSON);
    let _ = server.set(&server.service.proto_ver, &proto_ver);

    let name = config.ble_name();
    let mut adv_data = [0u8; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteServiceUuids128(&[PROV_SERVICE_UUID_128]),
        ],
        &mut adv_data,
    )
    .map_err(|_| ProvError::Transport)?;
    let mut scan_data = [0u8; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(name.as_bytes())],
        &mut scan_data,
    )
    .map_err(|_| ProvError::Transport)?;

    info!("[ble] advertising '{}'", name.as_str());

    match select3(
        runner.run(),
        advertise(
            &mut peripheral,
            &server,
            &adv_data[..adv_len],
            &scan_data[..scan_len],
            aps,
            session,
            config.pop().as_str(),
            &mut rng,
        ),
        session.wait_finished(),
    )
    .await
    {
        Either3::First(_) => Err(ProvError::Transport),
        Either3::Second(result) => result,
        Either3::Third(_) => Ok(()),
    }
}

async fn advertise<C, R>(
    peripheral: &mut trouble_host::peripheral::Peripheral<'_, C, DefaultPacketPool>,
    server: &ProvServer<'_>,
    adv_data: &[u8],
    scan_data: &[u8],
    aps: &[AccessPointInfo],
    session: &Session,
    pop: &str,
    rng: &mut R,
) -> Result<(), ProvError>
where
    C: Controller,
    R: RngCore + CryptoRng,
{
    loop {
        let advertiser = match peripheral
            .advertise(
                &AdvertisementParameters::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data,
                    scan_data,
                },
            )
            .await
        {
            Ok(advertiser) => advertiser,
            Err(err) => {
                warn!("[ble] advertise failed: {}", crate::fmt::Debug2Format(&err));
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };
        let conn = match advertiser.accept().await {
            Ok(conn) => conn,
            Err(err) => {
                warn!("[ble] accept failed: {}", crate::fmt::Debug2Format(&err));
                continue;
            }
        };
        let gatt = match conn.with_attribute_server(server) {
            Ok(gatt) => gatt,
            Err(err) => {
                warn!(
                    "[ble] GATT attach failed: {}",
                    crate::fmt::Debug2Format(&err)
                );
                continue;
            }
        };
        debug!("[ble] central connected");
        if connection(&gatt, server, aps, session, pop, rng).await {
            return Ok(());
        }
    }
}

/// Phase tracked after `GetStatus` returns `Connected`.
#[derive(Clone, Copy)]
enum CompletionGate {
    /// Normal provisioning (including failures and `prov-ctrl` retries).
    Running,
    /// `Connected` response is stored in `prov-config`; waiting for the phone to read it.
    AwaitingConfigRead(Instant),
    /// Phone read `prov-config`; waiting briefly for a clean disconnect before finishing.
    DisconnectGrace(Instant),
}

async fn connection<P, R>(
    gatt: &GattConnection<'_, '_, P>,
    server: &ProvServer<'_>,
    aps: &[AccessPointInfo],
    session: &Session,
    pop: &str,
    rng: &mut R,
) -> bool
where
    P: PacketPool,
    R: RngCore + CryptoRng,
{
    let mut state = ConnState::<Sec1>::new();
    state.set_transport("ble");
    let pop = pop.as_bytes();
    let mut gate = CompletionGate::Running;

    loop {
        let next = match gate {
            CompletionGate::Running => gatt.next().await,
            CompletionGate::AwaitingConfigRead(deadline) => {
                match select(gatt.next(), Timer::at(deadline)).await {
                    Either::First(ev) => ev,
                    Either::Second(_) => {
                        info!("[ble] config read timed out");
                        session.finish();
                        return true;
                    }
                }
            }
            CompletionGate::DisconnectGrace(deadline) => {
                match select(gatt.next(), Timer::at(deadline)).await {
                    Either::First(ev) => ev,
                    Either::Second(_) => {
                        info!("[ble] disconnect grace elapsed");
                        session.finish();
                        return true;
                    }
                }
            }
        };

        match next {
            GattConnectionEvent::Disconnected { reason } => {
                debug!("[ble] disconnected 0x{:02x}", reason.into_inner());
                if !matches!(gate, CompletionGate::Running) {
                    session.finish();
                    return true;
                }
                return false;
            }
            GattConnectionEvent::Gatt { event } => match event {
                GattEvent::Write(ev) => {
                    let handle = ev.handle();
                    let Some(endpoint) = endpoint_for_handle(server, handle) else {
                        send_reply(ev.accept()).await;
                        continue;
                    };

                    let mut req = [0u8; 512];
                    let mut too_long = false;
                    let req_len = ev.with_data(|_offset, data| {
                        if data.len() > req.len() {
                            too_long = true;
                            return 0;
                        }
                        req[..data.len()].copy_from_slice(data);
                        data.len()
                    });
                    if too_long {
                        warn!("[ble] request exceeds buffer");
                        send_reply(ev.reject(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH)).await;
                        continue;
                    }

                    let cap = endpoint_capacity(endpoint);
                    let mut response = [0u8; 512];
                    match state.handle(
                        endpoint,
                        &req[..req_len],
                        &mut response[..cap],
                        session,
                        aps,
                        rng,
                        pop,
                    ) {
                        Ok(reply) => {
                            // Store response bytes BEFORE sending ATT_WRITE_RSP so the phone's
                            // immediate GATT Read always sees the response, never echoed request
                            // bytes (`accept_unprocessed` skips copying request data into the DB).
                            if let Err(err) =
                                set_characteristic(server, endpoint, &response[..reply.len])
                            {
                                warn!(
                                    "[ble] set characteristic failed: {}",
                                    crate::fmt::Debug2Format(&err)
                                );
                                send_reply(ev.reject(AttErrorCode::UNLIKELY_ERROR)).await;
                                continue;
                            }
                            send_reply(ev.accept_unprocessed()).await;
                            if reply.complete {
                                sec1::log_report("ble", "GetStatus sent");
                                gate = CompletionGate::AwaitingConfigRead(
                                    Instant::now() + READ_DEADLINE,
                                );
                            }
                        }
                        Err(err) => {
                            warn!("[ble] request failed: {}", crate::fmt::Debug2Format(&err));
                            send_reply(ev.reject(AttErrorCode::UNLIKELY_ERROR)).await;
                        }
                    }
                }
                GattEvent::Read(ev) => {
                    let handle = ev.handle();
                    send_reply(ev.accept()).await;
                    if matches!(gate, CompletionGate::AwaitingConfigRead(_))
                        && handle == server.service.config.handle()
                    {
                        info!("[ble] config read done — waiting for disconnect");
                        gate = CompletionGate::DisconnectGrace(Instant::now() + DISCONNECT_GRACE);
                    }
                }
                GattEvent::Other(ev) => send_reply(ev.accept()).await,
                GattEvent::NotAllowed(ev) => send_reply(ev.accept()).await,
            },
            GattConnectionEvent::RequestConnectionParams(_) => {}
            _ => {}
        }
    }
}

fn endpoint_for_handle(server: &ProvServer<'_>, handle: u16) -> Option<Endpoint> {
    if handle == server.service.session.handle() {
        Some(Endpoint::Session)
    } else if handle == server.service.config.handle() {
        Some(Endpoint::Config)
    } else if handle == server.service.scan.handle() {
        Some(Endpoint::Scan)
    } else if handle == server.service.proto_ver.handle() {
        Some(Endpoint::ProtoVer)
    } else if handle == server.service.ctrl.handle() {
        Some(Endpoint::Ctrl)
    } else {
        None
    }
}

fn endpoint_capacity(endpoint: Endpoint) -> usize {
    match endpoint {
        Endpoint::Ctrl => 64,
        Endpoint::ProtoVer => 128,
        Endpoint::Session | Endpoint::Config => 256,
        Endpoint::Scan => 512,
    }
}

fn set_characteristic(
    server: &ProvServer<'_>,
    endpoint: Endpoint,
    bytes: &[u8],
) -> Result<(), trouble_host::Error> {
    match endpoint {
        Endpoint::Session => {
            let mut value = HVec::<u8, 256>::new();
            value
                .extend_from_slice(bytes)
                .map_err(|_| trouble_host::Error::InsufficientSpace)?;
            server.set(&server.service.session, &value)
        }
        Endpoint::Config => {
            let mut value = HVec::<u8, 256>::new();
            value
                .extend_from_slice(bytes)
                .map_err(|_| trouble_host::Error::InsufficientSpace)?;
            server.set(&server.service.config, &value)
        }
        Endpoint::Scan => {
            let mut value = HVec::<u8, 512>::new();
            value
                .extend_from_slice(bytes)
                .map_err(|_| trouble_host::Error::InsufficientSpace)?;
            server.set(&server.service.scan, &value)
        }
        Endpoint::ProtoVer => {
            let mut value = HVec::<u8, 128>::new();
            value
                .extend_from_slice(bytes)
                .map_err(|_| trouble_host::Error::InsufficientSpace)?;
            server.set(&server.service.proto_ver, &value)
        }
        Endpoint::Ctrl => {
            let mut value = HVec::<u8, 64>::new();
            value
                .extend_from_slice(bytes)
                .map_err(|_| trouble_host::Error::InsufficientSpace)?;
            server.set(&server.service.ctrl, &value)
        }
    }
}

async fn send_reply<P: PacketPool>(
    result: Result<trouble_host::gatt::Reply<'_, P>, trouble_host::Error>,
) {
    match result {
        Ok(reply) => {
            reply.send().await;
        }
        Err(err) => warn!(
            "[ble] gatt reply failed: {}",
            crate::fmt::Debug2Format(&err)
        ),
    }
}
