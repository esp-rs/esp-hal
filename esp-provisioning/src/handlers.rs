//! Per-connection security state and the shared plaintext request handler.

use heapless::String;
use rand_core::{CryptoRng, RngCore};

use crate::{
    proto::Status,
    protocomm::{self, ConfigEvent, ConfigStatus, ConnectedWire, CtrlCommand},
    sec1::{Sec1, Security, SessionError},
    session::{Session, SessionStatus, WifiCredentials},
    types::PROTO_VER_JSON,
};

/// protocomm endpoint, after the transport has stripped HTTP or GATT framing.
#[derive(Clone, Copy)]
pub(crate) enum Endpoint {
    ProtoVer,
    Session,
    Config,
    Scan,
    Ctrl,
}

/// Bytes to write back on the same connection.
pub(crate) struct Reply {
    pub len: usize,
    /// JSON `proto-ver` body. Otherwise the body is protobuf.
    #[cfg_attr(not(feature = "softap"), allow(dead_code))]
    pub json: bool,
    /// `GetStatus` carried `Connected`. The transport should finish
    /// the serve future after this reply is delivered to the client.
    pub complete: bool,
}

/// One phone connection. Reset when the transport accepts a new socket or link.
///
/// `S` is the protocomm security scheme. Only [`Sec1`] is implemented.
pub(crate) struct ConnState<S: Security = Sec1> {
    security: S,
    pending_ssid: String<32>,
    pending_pass: String<64>,
}

impl<S: Security> ConnState<S> {
    pub(crate) fn new() -> Self {
        Self {
            security: S::default(),
            pending_ssid: String::new(),
            pending_pass: String::new(),
        }
    }

    pub(crate) fn set_transport(&mut self, name: &'static str) {
        self.security.set_transport(name);
    }

    pub(crate) fn handle<R: RngCore + CryptoRng>(
        &mut self,
        endpoint: Endpoint,
        request: &[u8],
        response: &mut [u8],
        session: &Session,
        aps: &[esp_radio::wifi::scan::AccessPointInfo],
        rng: &mut R,
        pop: &[u8],
    ) -> Result<Reply, crate::ProvError> {
        #[cfg(feature = "debug")]
        match endpoint {
            Endpoint::ProtoVer => debug!("endpoint proto-ver"),
            Endpoint::Session => debug!("endpoint prov-session"),
            Endpoint::Config => debug!("endpoint prov-config"),
            Endpoint::Scan => debug!("endpoint prov-scan"),
            Endpoint::Ctrl => debug!("endpoint prov-ctrl"),
        }

        match endpoint {
            Endpoint::ProtoVer => {
                if response.len() < PROTO_VER_JSON.len() {
                    return Err(crate::ProvError::Protocol);
                }
                let n = PROTO_VER_JSON.len();
                response[..n].copy_from_slice(PROTO_VER_JSON);
                Ok(Reply {
                    len: n,
                    json: true,
                    complete: false,
                })
            }
            Endpoint::Session => {
                let len = self
                    .security
                    .handshake(request, rng, pop, response)
                    .map_err(handshake_error)?;
                Ok(Reply {
                    len,
                    json: false,
                    complete: false,
                })
            }
            Endpoint::Config => self.config(request, response, session),
            Endpoint::Scan => self.scan(request, response, aps),
            Endpoint::Ctrl => self.ctrl(request, response, session),
        }
    }

    fn config(
        &mut self,
        request: &[u8],
        response: &mut [u8],
        session: &Session,
    ) -> Result<Reply, crate::ProvError> {
        if !self.security.established() {
            warn!("Secure session not established");
            let len = protocomm::encode_config_status(
                response,
                Status::InvalidSession,
                ConfigStatus::Disconnected,
            )
            .map_err(|_| crate::ProvError::Protocol)?;
            return Ok(Reply {
                len,
                json: false,
                complete: false,
            });
        }

        let mut scratch = [0u8; 512];
        let plain = decrypt_exact(&mut self.security, request, &mut scratch)?;

        let status = session.status();
        let config_status = match &status {
            SessionStatus::Idle => ConfigStatus::Disconnected,
            SessionStatus::Connecting => ConfigStatus::Connecting,
            SessionStatus::Failed(reason) => ConfigStatus::Failed(*reason),
            SessionStatus::Connected(snap) => ConfigStatus::Connected(ConnectedWire {
                ipv4: snap.ipv4,
                auth_mode: protocomm::wifi_auth(Some(snap.info.authmode)),
                ssid: snap.info.ssid.as_bytes(),
                bssid: snap.info.bssid,
                channel: snap.info.channel,
            }),
        };

        let (event, len) = protocomm::handle_config(
            plain,
            &mut self.pending_ssid,
            &mut self.pending_pass,
            response,
            config_status,
        )
        .map_err(|_| crate::ProvError::Protocol)?;

        let mut complete = false;
        match event {
            ConfigEvent::Apply => {
                if let Some(creds) = credentials_from(&self.pending_ssid, &self.pending_pass) {
                    info!(
                        "ApplyConfig ssid='{}' pass_len={}",
                        creds.ssid.as_str(),
                        creds.password.len()
                    );
                    let _ = session.submit(creds);
                }
            }
            ConfigEvent::Status | ConfigEvent::Ignored => {
                if matches!(status, SessionStatus::Connected(_)) {
                    complete = true;
                }
            }
            ConfigEvent::Set | ConfigEvent::Rejected => {}
        }

        self.security
            .encrypt(&mut response[..len])
            .map_err(|_| crate::ProvError::Handshake)?;
        Ok(Reply {
            len,
            json: false,
            complete,
        })
    }

    fn scan(
        &mut self,
        request: &[u8],
        response: &mut [u8],
        aps: &[esp_radio::wifi::scan::AccessPointInfo],
    ) -> Result<Reply, crate::ProvError> {
        if !self.security.established() {
            warn!("Secure session not established");
            let len = protocomm::encode_scan_unsecured(response)
                .map_err(|_| crate::ProvError::Protocol)?;
            return Ok(Reply {
                len,
                json: false,
                complete: false,
            });
        }
        let mut scratch = [0u8; 256];
        let plain = decrypt_exact(&mut self.security, request, &mut scratch)?;
        let len =
            protocomm::handle_scan(plain, aps, response).map_err(|_| crate::ProvError::Protocol)?;
        self.security
            .encrypt(&mut response[..len])
            .map_err(|_| crate::ProvError::Handshake)?;
        Ok(Reply {
            len,
            json: false,
            complete: false,
        })
    }

    fn ctrl(
        &mut self,
        request: &[u8],
        response: &mut [u8],
        session: &Session,
    ) -> Result<Reply, crate::ProvError> {
        if !self.security.established() {
            warn!("Secure session not established");
            let len = protocomm::encode_ctrl_unsecured(response)
                .map_err(|_| crate::ProvError::Protocol)?;
            return Ok(Reply {
                len,
                json: false,
                complete: false,
            });
        }

        let mut scratch = [0u8; protocomm::MAX_CTRL_LEN];
        let plain = decrypt_exact(&mut self.security, request, &mut scratch)?;
        let cmd = protocomm::decode_ctrl(plain).map_err(|_| crate::ProvError::Protocol)?;

        let status = match cmd {
            CtrlCommand::Reset => {
                if session.reset_on_failure() {
                    self.pending_ssid.clear();
                    self.pending_pass.clear();
                    info!("CtrlReset accepted");
                    Status::Success
                } else {
                    warn!("CtrlReset rejected in current state");
                    Status::InternalError
                }
            }
            CtrlCommand::Reprov => {
                warn!("CtrlReprov unsupported");
                Status::InternalError
            }
        };

        let len = protocomm::encode_ctrl_response(cmd, status, response)
            .map_err(|_| crate::ProvError::Protocol)?;
        self.security
            .encrypt(&mut response[..len])
            .map_err(|_| crate::ProvError::Handshake)?;
        Ok(Reply {
            len,
            json: false,
            complete: false,
        })
    }
}

/// Decrypt `request` in `scratch` when it fits. A longer body is rejected
/// before the keystream moves.
fn decrypt_exact<'a, S: Security>(
    security: &mut S,
    request: &[u8],
    scratch: &'a mut [u8],
) -> Result<&'a mut [u8], crate::ProvError> {
    let plain =
        protocomm::copy_request(request, scratch).map_err(|_| crate::ProvError::Protocol)?;
    security
        .decrypt(plain)
        .map_err(|_| crate::ProvError::Handshake)?;
    Ok(plain)
}

fn handshake_error(err: SessionError) -> crate::ProvError {
    warn!("sec1 handshake failed: {}", crate::fmt::Debug2Format(&err));
    match err {
        SessionError::BufferTooSmall | SessionError::Parse => crate::ProvError::Protocol,
        SessionError::NotEstablished => crate::ProvError::Handshake,
    }
}

fn credentials_from(ssid: &str, pass: &str) -> Option<WifiCredentials> {
    let mut password = String::new();
    password.push_str(pass).ok()?;
    Some(WifiCredentials {
        ssid: esp_radio::wifi::Ssid::try_from(ssid).ok()?,
        password,
    })
}
