//! Plaintext `prov-config`, `prov-scan`, and `prov-ctrl` using the committed micropb messages.

use core::{fmt::Write as _, net::Ipv4Addr};

use heapless::String;

use crate::{
    proto::{
        self,
        RespApplyConfig,
        RespCtrlReprov,
        RespCtrlReset,
        RespGetStatus,
        RespGetStatus_,
        RespSetConfig,
        Status,
        WiFiConfigMsgType,
        WiFiConfigPayload,
        WiFiConfigPayload_,
        WiFiCtrlMsgType,
        WiFiCtrlPayload,
        WiFiCtrlPayload_,
        WifiAuthMode,
        WifiConnectFailedReason,
        WifiConnectedState,
        WifiStationState,
    },
    state::StaFailReason,
};

/// Maximum plaintext/ciphertext request size accepted on `prov-ctrl`.
pub(crate) const MAX_CTRL_LEN: usize = 64;

/// What a decrypted `prov-config` request asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ConfigEvent {
    /// Credentials were stored. The phone has not applied them yet.
    Set,
    /// The phone asked the device to use the stored credentials.
    Apply,
    /// The phone is polling station status.
    Status,
    /// The message was not a config command we act on.
    Ignored,
    /// `SetConfig` was rejected. Pending credentials are unchanged.
    Rejected,
}

/// Wire-ready connected station details for `RespGetStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ConnectedWire<'a> {
    pub ipv4: Ipv4Addr,
    pub auth_mode: WifiAuthMode,
    pub ssid: &'a [u8],
    pub bssid: [u8; 6],
    pub channel: u8,
}

/// Station state to encode into `RespGetStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ConfigStatus<'a> {
    Disconnected,
    Connecting,
    Connected(ConnectedWire<'a>),
    Failed(StaFailReason),
}

/// Decoded `prov-ctrl` command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CtrlCommand {
    Reset,
    Reprov,
}

pub(crate) fn encode_config_status(
    out: &mut [u8],
    status: Status,
    config_status: ConfigStatus<'_>,
) -> Result<usize, ()> {
    let (sta_state, state) = match config_status {
        ConfigStatus::Disconnected => (WifiStationState::Disconnected, None),
        ConfigStatus::Connecting => (WifiStationState::Connecting, None),
        ConfigStatus::Failed(reason) => {
            let wire_reason = match reason {
                StaFailReason::AuthError => WifiConnectFailedReason::AuthError,
                StaFailReason::NetworkNotFound => WifiConnectFailedReason::NetworkNotFound,
            };
            (
                WifiStationState::ConnectionFailed,
                Some(RespGetStatus_::State::FailReason(wire_reason)),
            )
        }
        ConfigStatus::Connected(info) => {
            let mut ip4_addr = heapless::String::<64>::new();
            let [a, b, c, d] = info.ipv4.octets();
            let _ = write!(ip4_addr, "{a}.{b}.{c}.{d}");

            let mut ssid = heapless::Vec::<u8, 64>::new();
            let _ = ssid.extend_from_slice(info.ssid);

            let mut bssid = heapless::Vec::<u8, 64>::new();
            let _ = bssid.extend_from_slice(&info.bssid);

            (
                WifiStationState::Connected,
                Some(RespGetStatus_::State::Connected(WifiConnectedState {
                    ip4_addr,
                    auth_mode: info.auth_mode,
                    ssid,
                    bssid,
                    channel: i32::from(info.channel),
                })),
            )
        }
    };
    let msg = WiFiConfigPayload {
        msg: WiFiConfigMsgType::TypeRespGetStatus,
        payload: Some(WiFiConfigPayload_::Payload::RespGetStatus(RespGetStatus {
            status,
            sta_state,
            state,
        })),
    };
    proto::encode(&msg, out)
}

pub(crate) fn handle_config(
    plain: &[u8],
    pending_ssid: &mut String<32>,
    pending_pass: &mut String<64>,
    out: &mut [u8],
    config_status: ConfigStatus<'_>,
) -> Result<(ConfigEvent, usize), ()> {
    let payload: WiFiConfigPayload = proto::decode(plain)?;
    let event = match payload.msg {
        WiFiConfigMsgType::TypeCmdGetStatus => ConfigEvent::Status,
        WiFiConfigMsgType::TypeCmdSetConfig => ConfigEvent::Set,
        WiFiConfigMsgType::TypeCmdApplyConfig => ConfigEvent::Apply,
        _ => ConfigEvent::Ignored,
    };

    if event == ConfigEvent::Set
        && let Some(WiFiConfigPayload_::Payload::CmdSetConfig(cmd)) = payload.payload
    {
        match (
            copy_utf8(cmd.ssid.as_slice()),
            copy_utf8(cmd.passphrase.as_slice()),
        ) {
            (Ok(ssid), Ok(pass)) => {
                *pending_ssid = ssid;
                *pending_pass = pass;
            }
            _ => {
                let len = encode_set_status(out, Status::InvalidArgument)?;
                return Ok((ConfigEvent::Rejected, len));
            }
        }
    }

    let len = match event {
        ConfigEvent::Set => {
            let msg = WiFiConfigPayload {
                msg: WiFiConfigMsgType::TypeRespSetConfig,
                payload: Some(WiFiConfigPayload_::Payload::RespSetConfig(RespSetConfig {
                    status: Status::Success,
                })),
            };
            proto::encode(&msg, out)?
        }
        ConfigEvent::Apply => {
            let msg = WiFiConfigPayload {
                msg: WiFiConfigMsgType::TypeRespApplyConfig,
                payload: Some(WiFiConfigPayload_::Payload::RespApplyConfig(
                    RespApplyConfig {
                        status: Status::Success,
                    },
                )),
            };
            proto::encode(&msg, out)?
        }
        ConfigEvent::Status | ConfigEvent::Ignored => {
            encode_config_status(out, Status::Success, config_status)?
        }
        // `SetConfig` that does not fit returns before this match.
        ConfigEvent::Rejected => unreachable!(),
    };
    Ok((event, len))
}

fn encode_set_status(out: &mut [u8], status: Status) -> Result<usize, ()> {
    let msg = WiFiConfigPayload {
        msg: WiFiConfigMsgType::TypeRespSetConfig,
        payload: Some(WiFiConfigPayload_::Payload::RespSetConfig(RespSetConfig {
            status,
        })),
    };
    proto::encode(&msg, out)
}

/// Plaintext `prov-ctrl` error reply when sec1 is not yet established.
pub(crate) fn encode_ctrl_unsecured(out: &mut [u8]) -> Result<usize, ()> {
    let msg = WiFiCtrlPayload {
        msg: WiFiCtrlMsgType::TypeRespCtrlReset,
        status: Status::InvalidSession,
        payload: None,
    };
    proto::encode(&msg, out)
}

/// Decodes a decrypted `prov-ctrl` request into a supported command.
pub(crate) fn decode_ctrl(plain: &[u8]) -> Result<CtrlCommand, ()> {
    let payload: WiFiCtrlPayload = proto::decode(plain)?;
    match payload.msg {
        WiFiCtrlMsgType::TypeCmdCtrlReset => Ok(CtrlCommand::Reset),
        WiFiCtrlMsgType::TypeCmdCtrlReprov => Ok(CtrlCommand::Reprov),
        _ => Err(()),
    }
}

/// Encodes the plaintext `prov-ctrl` response for `cmd` and `status`.
pub(crate) fn encode_ctrl_response(
    cmd: CtrlCommand,
    status: Status,
    out: &mut [u8],
) -> Result<usize, ()> {
    let msg = match cmd {
        CtrlCommand::Reset => WiFiCtrlPayload {
            msg: WiFiCtrlMsgType::TypeRespCtrlReset,
            status,
            payload: Some(WiFiCtrlPayload_::Payload::RespCtrlReset(RespCtrlReset {})),
        },
        CtrlCommand::Reprov => WiFiCtrlPayload {
            msg: WiFiCtrlMsgType::TypeRespCtrlReprov,
            status,
            payload: Some(WiFiCtrlPayload_::Payload::RespCtrlReprov(RespCtrlReprov {})),
        },
    };
    proto::encode(&msg, out)
}

/// Copy `request` into `buf` only when the whole request fits.
///
/// A longer body is a protocol error. Decrypting a prefix and parsing it
/// would accept a truncated protobuf.
pub(crate) fn copy_request<'a>(request: &[u8], buf: &'a mut [u8]) -> Result<&'a mut [u8], ()> {
    let Some(dst) = buf.get_mut(..request.len()) else {
        return Err(());
    };
    dst.copy_from_slice(request);
    Ok(dst)
}

/// Copy `bytes` only when they are valid UTF-8 and fit in `N`.
fn copy_utf8<const N: usize>(bytes: &[u8]) -> Result<String<N>, ()> {
    let text = core::str::from_utf8(bytes).map_err(|_| ())?;
    let mut dst = String::new();
    dst.push_str(text).map_err(|_| ())?;
    Ok(dst)
}

#[cfg(any(feature = "softap", feature = "ble"))]
use crate::proto::{
    RespScanResult,
    RespScanStatus,
    WiFiScanMsgType,
    WiFiScanPayload,
    WiFiScanPayload_,
};

#[cfg(any(feature = "softap", feature = "ble"))]
pub(crate) fn encode_scan_unsecured(out: &mut [u8]) -> Result<usize, ()> {
    let msg = WiFiScanPayload {
        msg: WiFiScanMsgType::TypeRespScanStatus,
        status: Status::InvalidSession,
        payload: None,
    };
    proto::encode(&msg, out)
}

#[cfg(any(feature = "softap", feature = "ble"))]
pub(crate) fn handle_scan(
    plain: &[u8],
    aps: &[esp_radio::wifi::scan::AccessPointInfo],
    out: &mut [u8],
) -> Result<usize, ()> {
    let payload: WiFiScanPayload = proto::decode(plain).unwrap_or_default();
    match payload.msg {
        WiFiScanMsgType::TypeCmdScanResult => encode_scan_entries(payload, aps, out),
        _ => {
            let msg = WiFiScanPayload {
                msg: WiFiScanMsgType::TypeRespScanStatus,
                status: Status::Success,
                payload: Some(WiFiScanPayload_::Payload::RespScanStatus(RespScanStatus {
                    scan_finished: true,
                    result_count: aps.len() as u32,
                })),
            };
            proto::encode(&msg, out)
        }
    }
}

#[cfg(any(feature = "softap", feature = "ble"))]
fn encode_scan_entries(
    request: WiFiScanPayload,
    aps: &[esp_radio::wifi::scan::AccessPointInfo],
    out: &mut [u8],
) -> Result<usize, ()> {
    let (start, count) = match request.payload {
        Some(WiFiScanPayload_::Payload::CmdScanResult(cmd)) => {
            (cmd.start_index as usize, cmd.count as usize)
        }
        _ => (0, aps.len()),
    };
    let count = if count == 0 {
        aps.len().saturating_sub(start)
    } else {
        count
    };
    let mut entries = heapless::Vec::new();
    for ap in aps.iter().skip(start).take(count) {
        if entries.len() == entries.capacity() {
            break;
        }
        let mut ssid = heapless::Vec::new();
        let _ = ssid.extend_from_slice(ap.ssid.as_bytes());
        let mut bssid = heapless::Vec::new();
        let _ = bssid.extend_from_slice(&ap.bssid);
        let item = crate::proto::WiFiScanResult {
            ssid,
            channel: ap.channel as u32,
            rssi: ap.signal_strength as i32,
            bssid,
            auth: wifi_auth(ap.auth_method),
        };
        let _ = entries.push(item);
    }
    let msg = WiFiScanPayload {
        msg: WiFiScanMsgType::TypeRespScanResult,
        status: Status::Success,
        payload: Some(WiFiScanPayload_::Payload::RespScanResult(RespScanResult {
            entries,
        })),
    };
    proto::encode(&msg, out)
}

/// Proto `WifiAuthMode` is only 0..=7. Modes the proto does not name (WAPI, OWE,
/// DPP, enterprise variants outside that range) use Open.
#[cfg(any(feature = "softap", feature = "ble"))]
pub(crate) fn wifi_auth(method: Option<esp_radio::wifi::AuthenticationMethod>) -> WifiAuthMode {
    use esp_radio::wifi::AuthenticationMethod;

    match method {
        None | Some(AuthenticationMethod::None) => WifiAuthMode::Open,
        Some(AuthenticationMethod::Wep) => WifiAuthMode::Wep,
        Some(AuthenticationMethod::Wpa) => WifiAuthMode::WpaPsk,
        Some(AuthenticationMethod::Wpa2Personal) => WifiAuthMode::Wpa2Psk,
        Some(AuthenticationMethod::WpaWpa2Personal) => WifiAuthMode::WpaWpa2Psk,
        Some(AuthenticationMethod::Wpa2Enterprise) => WifiAuthMode::Wpa2Enterprise,
        Some(AuthenticationMethod::Wpa3Personal) => WifiAuthMode::Wpa3Psk,
        Some(AuthenticationMethod::Wpa2Wpa3Personal) => WifiAuthMode::Wpa2Wpa3Psk,
        Some(_) => WifiAuthMode::Open,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::CmdCtrlReset;

    #[test]
    fn oversize_request_is_not_copied() {
        let mut buf = [0u8; 4];
        assert!(copy_request(&[1, 2, 3, 4, 5], &mut buf).is_err());
        assert_eq!(buf, [0, 0, 0, 0]);
        assert_eq!(
            copy_request(&[1, 2, 3, 4], &mut buf).unwrap(),
            &[1, 2, 3, 4]
        );
    }

    #[test]
    fn get_status_disconnected_roundtrip() {
        let mut buf = [0u8; 64];
        let n =
            encode_config_status(&mut buf, Status::Success, ConfigStatus::Disconnected).unwrap();
        let decoded: WiFiConfigPayload = proto::decode(&buf[..n]).unwrap();
        assert_eq!(decoded.msg, WiFiConfigMsgType::TypeRespGetStatus);
        match decoded.payload {
            Some(WiFiConfigPayload_::Payload::RespGetStatus(status)) => {
                assert_eq!(status.sta_state, WifiStationState::Disconnected);
                assert_eq!(status.status, Status::Success);
                assert!(status.state.is_none());
            }
            other => panic!("unexpected payload: {other:?}"),
        }
    }

    #[test]
    fn get_status_connected_populates_connected_state() {
        let mut buf = [0u8; 160];
        let bssid = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let n = encode_config_status(
            &mut buf,
            Status::Success,
            ConfigStatus::Connected(ConnectedWire {
                ipv4: Ipv4Addr::new(192, 168, 1, 42),
                auth_mode: WifiAuthMode::Wpa2Psk,
                ssid: b"Sttars",
                bssid,
                channel: 6,
            }),
        )
        .unwrap();
        let decoded: WiFiConfigPayload = proto::decode(&buf[..n]).unwrap();
        assert_eq!(decoded.msg, WiFiConfigMsgType::TypeRespGetStatus);
        match decoded.payload {
            Some(WiFiConfigPayload_::Payload::RespGetStatus(status)) => {
                assert_eq!(status.status, Status::Success);
                assert_eq!(status.sta_state, WifiStationState::Connected);
                match status.state {
                    Some(RespGetStatus_::State::Connected(conn)) => {
                        assert_eq!(conn.ip4_addr.as_str(), "192.168.1.42");
                        assert_eq!(conn.auth_mode, WifiAuthMode::Wpa2Psk);
                        assert_eq!(conn.ssid.as_slice(), b"Sttars");
                        assert_eq!(conn.bssid.as_slice(), &bssid);
                        assert_eq!(conn.channel, 6);
                    }
                    other => panic!("expected Connected state: {other:?}"),
                }
            }
            other => panic!("unexpected payload: {other:?}"),
        }
    }

    #[test]
    fn get_status_failed_encodes_auth_and_not_found() {
        for (reason, expected) in [
            (StaFailReason::AuthError, WifiConnectFailedReason::AuthError),
            (
                StaFailReason::NetworkNotFound,
                WifiConnectFailedReason::NetworkNotFound,
            ),
        ] {
            let mut buf = [0u8; 64];
            let n = encode_config_status(&mut buf, Status::Success, ConfigStatus::Failed(reason))
                .unwrap();
            let decoded: WiFiConfigPayload = proto::decode(&buf[..n]).unwrap();
            match decoded.payload {
                Some(WiFiConfigPayload_::Payload::RespGetStatus(status)) => {
                    assert_eq!(status.status, Status::Success);
                    assert_eq!(status.sta_state, WifiStationState::ConnectionFailed);
                    assert_eq!(
                        status.state,
                        Some(RespGetStatus_::State::FailReason(expected))
                    );
                }
                other => panic!("unexpected payload: {other:?}"),
            }
        }
    }

    #[test]
    fn unsecured_status_is_invalid_session() {
        let mut buf = [0u8; 64];
        let n = encode_config_status(&mut buf, Status::InvalidSession, ConfigStatus::Disconnected)
            .unwrap();
        let decoded: WiFiConfigPayload = proto::decode(&buf[..n]).unwrap();
        match decoded.payload {
            Some(WiFiConfigPayload_::Payload::RespGetStatus(status)) => {
                assert_eq!(status.status, Status::InvalidSession);
            }
            other => panic!("unexpected payload: {other:?}"),
        }
    }

    #[test]
    fn ctrl_reset_raw_wire_roundtrip() {
        // ESP-IDF Android/iOS wire bytes for TypeCmdCtrlReset (msg=1, cmd_ctrl_reset=empty):
        // field 1 varint 1 = 0x08 0x01; field 11 len 0 = 0x5A 0x00.
        let wire_reset = [0x08u8, 0x01, 0x5A, 0x00];
        assert_eq!(decode_ctrl(&wire_reset), Ok(CtrlCommand::Reset));

        // Verify encoding CmdCtrlReset produces identical wire bytes.
        let req = WiFiCtrlPayload {
            msg: WiFiCtrlMsgType::TypeCmdCtrlReset,
            status: Status::Success,
            payload: Some(WiFiCtrlPayload_::Payload::CmdCtrlReset(CmdCtrlReset {})),
        };
        let mut req_buf = [0u8; MAX_CTRL_LEN];
        let req_len = proto::encode(&req, &mut req_buf).unwrap();
        assert_eq!(&req_buf[..req_len], &wire_reset);

        // RespCtrlReset Success: field 1 varint 2 (0x08 0x02), status=0 omitted, field 12 len 0
        // (0x62 0x00).
        let mut resp_buf = [0u8; MAX_CTRL_LEN];
        let resp_len =
            encode_ctrl_response(CtrlCommand::Reset, Status::Success, &mut resp_buf).unwrap();
        assert_eq!(&resp_buf[..resp_len], &[0x08, 0x02, 0x62, 0x00]);
        let decoded: WiFiCtrlPayload = proto::decode(&resp_buf[..resp_len]).unwrap();
        assert_eq!(decoded.msg, WiFiCtrlMsgType::TypeRespCtrlReset);
        assert_eq!(decoded.status, Status::Success);
        assert_eq!(
            decoded.payload,
            Some(WiFiCtrlPayload_::Payload::RespCtrlReset(RespCtrlReset {}))
        );

        // RespCtrlReset InternalError.
        let resp_len =
            encode_ctrl_response(CtrlCommand::Reset, Status::InternalError, &mut resp_buf).unwrap();
        let decoded: WiFiCtrlPayload = proto::decode(&resp_buf[..resp_len]).unwrap();
        assert_eq!(decoded.msg, WiFiCtrlMsgType::TypeRespCtrlReset);
        assert_eq!(decoded.status, Status::InternalError);
    }

    #[test]
    fn ctrl_reprov_unsecured_and_unknown() {
        // TypeCmdCtrlReprov: field 1 varint 3 (0x08 0x03), field 13 len 0 (0x6A 0x00).
        let wire_reprov = [0x08u8, 0x03, 0x6A, 0x00];
        assert_eq!(decode_ctrl(&wire_reprov), Ok(CtrlCommand::Reprov));

        let mut resp_buf = [0u8; MAX_CTRL_LEN];
        let resp_len =
            encode_ctrl_response(CtrlCommand::Reprov, Status::InternalError, &mut resp_buf)
                .unwrap();
        let decoded: WiFiCtrlPayload = proto::decode(&resp_buf[..resp_len]).unwrap();
        assert_eq!(decoded.msg, WiFiCtrlMsgType::TypeRespCtrlReprov);
        assert_eq!(decoded.status, Status::InternalError);
        assert_eq!(
            decoded.payload,
            Some(WiFiCtrlPayload_::Payload::RespCtrlReprov(RespCtrlReprov {}))
        );

        // Unsecured reply returns TypeRespCtrlReset with Status::InvalidSession.
        let unsecured_len = encode_ctrl_unsecured(&mut resp_buf).unwrap();
        let decoded: WiFiCtrlPayload = proto::decode(&resp_buf[..unsecured_len]).unwrap();
        assert_eq!(decoded.msg, WiFiCtrlMsgType::TypeRespCtrlReset);
        assert_eq!(decoded.status, Status::InvalidSession);
        assert!(decoded.payload.is_none());

        // Unknown / reserved msg (0) is rejected as protocol error.
        assert!(decode_ctrl(&[]).is_err());
        assert!(decode_ctrl(&[0x08, 0x00]).is_err());
        assert!(decode_ctrl(&[0x08, 0x09]).is_err());
    }

    #[test]
    fn apply_config_does_not_invent_credentials() {
        let mut ssid = String::<32>::new();
        let mut pass = String::<64>::new();
        let mut resp = [0u8; 64];
        let mut req = [0u8; 32];
        let request = WiFiConfigPayload {
            msg: WiFiConfigMsgType::TypeCmdApplyConfig,
            payload: Some(WiFiConfigPayload_::Payload::CmdApplyConfig(
                Default::default(),
            )),
        };
        let n = proto::encode(&request, &mut req).unwrap();
        let (event, _) = handle_config(
            &req[..n],
            &mut ssid,
            &mut pass,
            &mut resp,
            ConfigStatus::Connecting,
        )
        .unwrap();
        assert_eq!(event, ConfigEvent::Apply);
        assert!(ssid.is_empty());
        assert!(pass.is_empty());
    }

    fn set_request(ssid: &[u8], pass: &[u8]) -> WiFiConfigPayload {
        let mut ssid_buf = heapless::Vec::new();
        ssid_buf.extend_from_slice(ssid).unwrap();
        let mut pass_buf = heapless::Vec::new();
        pass_buf.extend_from_slice(pass).unwrap();
        WiFiConfigPayload {
            msg: WiFiConfigMsgType::TypeCmdSetConfig,
            payload: Some(WiFiConfigPayload_::Payload::CmdSetConfig(
                crate::proto::CmdSetConfig {
                    ssid: ssid_buf,
                    passphrase: pass_buf,
                    bssid: heapless::Vec::new(),
                    channel: 0,
                },
            )),
        }
    }

    fn pending() -> (String<32>, String<64>) {
        let mut ssid = String::new();
        ssid.push_str("home").unwrap();
        let mut pass = String::new();
        pass.push_str("secret").unwrap();
        (ssid, pass)
    }

    fn assert_rejected(event: ConfigEvent, resp: &[u8]) {
        assert_eq!(event, ConfigEvent::Rejected);
        let decoded: WiFiConfigPayload = proto::decode(resp).unwrap();
        match decoded.payload {
            Some(WiFiConfigPayload_::Payload::RespSetConfig(status)) => {
                assert_eq!(status.status, Status::InvalidArgument);
            }
            other => panic!("unexpected payload: {other:?}"),
        }
    }

    #[test]
    fn overlong_ssid_is_rejected() {
        let (mut ssid, mut pass) = pending();
        let mut resp = [0u8; 64];
        let mut req = [0u8; 128];
        let request = set_request(&[b'a'; 33], b"secret");
        let n = proto::encode(&request, &mut req).unwrap();
        let (event, len) = handle_config(
            &req[..n],
            &mut ssid,
            &mut pass,
            &mut resp,
            ConfigStatus::Disconnected,
        )
        .unwrap();
        assert_rejected(event, &resp[..len]);
        assert_eq!(ssid.as_str(), "home");
        assert_eq!(pass.as_str(), "secret");
    }

    #[test]
    fn invalid_passphrase_is_rejected() {
        let (mut ssid, mut pass) = pending();
        let mut resp = [0u8; 64];
        let mut req = [0u8; 128];
        let request = set_request(b"home", &[0xff, 0xfe]);
        let n = proto::encode(&request, &mut req).unwrap();
        let (event, len) = handle_config(
            &req[..n],
            &mut ssid,
            &mut pass,
            &mut resp,
            ConfigStatus::Disconnected,
        )
        .unwrap();
        assert_rejected(event, &resp[..len]);
        assert_eq!(ssid.as_str(), "home");
        assert_eq!(pass.as_str(), "secret");
    }

    #[test]
    fn overlong_passphrase_is_not_success() {
        let (mut ssid, mut pass) = pending();
        // TypeCmdSetConfig, then a CmdSetConfig whose passphrase is 65 bytes.
        // The generated field holds 64, so this must not become Success.
        let mut req = [0u8; 80];
        req[0] = 0x08;
        req[1] = 0x02;
        req[2] = 0x62;
        req[3] = 67;
        req[4] = 0x12;
        req[5] = 65;
        req[6..71].fill(b'p');
        let mut resp = [0u8; 64];
        let result = handle_config(
            &req[..71],
            &mut ssid,
            &mut pass,
            &mut resp,
            ConfigStatus::Disconnected,
        );
        assert!(result.is_err());
        assert_eq!(ssid.as_str(), "home");
        assert_eq!(pass.as_str(), "secret");
    }
}
