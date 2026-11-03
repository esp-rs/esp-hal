//! Minimal protocomm handlers for Wi-Fi provisioning endpoints.
//!
//! Covers two endpoints after the sec1 handshake:
//! - `prov-config` — WiFiConfigPayload (`wifi_config.proto`)
//! - `prov-scan` — WiFiScanPayload (`wifi_scan.proto`)
//!
//! Both endpoints exchange AES-256-CTR-encrypted protobuf over the wire. The
//! caller decrypts incoming Write data and encrypts the returned response
//! before sending. This module works only with plaintext bytes.

use crate::{
    CachedAp,
    pb::{read_varint, write_varint},
};

/// Max SSID length we accept (NVS and typical Wi-Fi).
const MAX_SSID: usize = 32;
/// Max passphrase length we accept.
const MAX_PASS: usize = 64;

/// WiFiConfigMsgType enum values (`wifi_config.proto`).
const CFG_CMD_GET_STATUS: u64 = 0;
const CFG_CMD_SET_CONFIG: u64 = 2;
const CFG_CMD_APPLY_CONFIG: u64 = 4;

/// Result of handling a prov-config write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum ProvConfigResult {
    /// Nothing actionable yet (set_config stored, unrecognised msg, etc.).
    Ok,
    /// CmdApplyConfig received; pending SSID / passphrase hold the credentials.
    ApplyConfig,
    /// CmdGetStatus received; caller should build and return GetStatus response.
    GetStatus,
}

/// Handle a **plaintext** write to the prov-config characteristic / HTTP body.
#[instability::unstable]
pub fn handle_prov_config_write(
    data: &[u8],
    pending_ssid: &mut heapless::String<MAX_SSID>,
    pending_pass: &mut heapless::String<MAX_PASS>,
    resp_buf: &mut [u8],
) -> (ProvConfigResult, usize) {
    let mut i = 0;
    let msg = parse_msg_type(data, &mut i);

    match msg {
        CFG_CMD_GET_STATUS => {
            let n = encode_resp_get_status(resp_buf, false);
            (ProvConfigResult::GetStatus, n)
        }
        CFG_CMD_SET_CONFIG => {
            parse_cmd_set_config(data, pending_ssid, pending_pass);
            if cfg!(feature = "debug") {
                info!(
                    "esp-prov config: SetConfig ssid=\"{}\" pass_len={}",
                    pending_ssid.as_str(),
                    pending_pass.len()
                );
            }
            let n = encode_resp_set_config(resp_buf);
            (ProvConfigResult::Ok, n)
        }
        CFG_CMD_APPLY_CONFIG => {
            let n = encode_resp_apply_config(resp_buf);
            (ProvConfigResult::ApplyConfig, n)
        }
        other => {
            warn!("esp-prov config: unknown msg_type={}", other);
            (ProvConfigResult::Ok, 0)
        }
    }
}

/// Encode RespGetStatus.
#[instability::unstable]
pub fn encode_resp_get_status(buf: &mut [u8], connected: bool) -> usize {
    let sta = if connected { 0u8 } else { 2u8 };
    // msg=1 (TypeRespGetStatus), status=0 (Success outer),
    // resp_get_status=field11{status=0,sta_state}
    const HEADER: [u8; 4] = [0x08, 0x01, 0x10, 0x00]; // msg + outer status
    let payload: [u8; 4] = [0x08, 0x00, 0x10, sta];
    let total = HEADER.len() + 2 + payload.len(); // header + tag+len + payload
    if buf.len() < total {
        return 0;
    }
    let mut pos = 0;
    buf[pos..pos + HEADER.len()].copy_from_slice(&HEADER);
    pos += HEADER.len();
    buf[pos] = 0x5a; // field 11, wire 2
    pos += 1;
    buf[pos] = payload.len() as u8;
    pos += 1;
    buf[pos..pos + payload.len()].copy_from_slice(&payload);
    pos += payload.len();
    pos
}

fn encode_resp_set_config(buf: &mut [u8]) -> usize {
    // msg=3 (TypeRespSetConfig), status=0 (Success), resp_set_config=field13{status=0}
    const RESP: [u8; 8] = [0x08, 0x03, 0x10, 0x00, 0x6a, 0x02, 0x08, 0x00];
    let n = RESP.len().min(buf.len());
    buf[..n].copy_from_slice(&RESP[..n]);
    n
}

fn encode_resp_apply_config(buf: &mut [u8]) -> usize {
    // msg=5 (TypeRespApplyConfig), status=0 (Success), resp_apply_config=field15{status=0}
    const RESP: [u8; 8] = [0x08, 0x05, 0x10, 0x00, 0x7a, 0x02, 0x08, 0x00];
    let n = RESP.len().min(buf.len());
    buf[..n].copy_from_slice(&RESP[..n]);
    n
}

/// WiFiScanMsgType enum values (`wifi_scan.proto`).
const SCAN_CMD_START: u64 = 0;
const SCAN_CMD_STATUS: u64 = 2;
const SCAN_CMD_RESULT: u64 = 4;

/// Result of handling a prov-scan write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum ProvScanResult {
    /// Recognised scan command handled; all requested entries encoded.
    Ok,
    /// ScanResult response encoded, but the encode buffer filled before every
    /// requested AP could be included (caller should page with a smaller `count`).
    Truncated,
    /// Unrecognised message type.
    Unknown,
}

/// Handle a **plaintext** write to the prov-scan characteristic / HTTP body.
///
/// `aps[..ap_count]` holds the cached scan results (see [`crate::set_cached_aps`]).
#[instability::unstable]
pub fn handle_prov_scan_write(
    data: &[u8],
    resp_buf: &mut [u8],
    aps: &[CachedAp],
    ap_count: usize,
) -> (ProvScanResult, usize) {
    let mut i = 0;
    let msg = parse_msg_type(data, &mut i);

    match msg {
        SCAN_CMD_START => {
            info!("esp-prov scan: ScanStart (cached {})", ap_count);
            let n = encode_resp_scan_start(resp_buf);
            (ProvScanResult::Ok, n)
        }
        SCAN_CMD_STATUS => {
            info!("esp-prov scan: ScanStatus count={}", ap_count);
            let n = encode_resp_scan_status(resp_buf, ap_count);
            (ProvScanResult::Ok, n)
        }
        SCAN_CMD_RESULT => {
            let (start_index, count) = parse_cmd_scan_result(data);
            info!(
                "esp-prov scan: ScanResult start={} count={} (have {})",
                start_index, count, ap_count
            );
            let (n, truncated) =
                encode_resp_scan_result(resp_buf, aps, ap_count, start_index, count);
            if truncated {
                (ProvScanResult::Truncated, n)
            } else {
                (ProvScanResult::Ok, n)
            }
        }
        other => {
            warn!("esp-prov scan: unknown msg_type={}", other);
            (ProvScanResult::Unknown, 0)
        }
    }
}

fn encode_resp_scan_start(buf: &mut [u8]) -> usize {
    // msg=1 (TypeRespScanStart), status=0 (Success), resp_scan_start=field11{} (empty)
    const RESP: [u8; 6] = [0x08, 0x01, 0x10, 0x00, 0x5a, 0x00];
    let n = RESP.len().min(buf.len());
    buf[..n].copy_from_slice(&RESP[..n]);
    n
}

fn encode_resp_scan_status(buf: &mut [u8], ap_count: usize) -> usize {
    // msg=3 (TypeRespScanStatus), status=0 (Success)
    // resp_scan_status = field 13 { scan_finished=true (field1 varint 1), result_count (field2
    // varint) }
    let mut pos = 0;
    if buf.len() < 16 {
        return 0;
    }
    buf[pos] = 0x08;
    pos += 1; // field 1 (msg)
    buf[pos] = 0x03;
    pos += 1; // varint 3
    buf[pos] = 0x10;
    pos += 1; // field 2 (status)
    buf[pos] = 0x00;
    pos += 1; // varint 0 (Success)

    let mut inner = [0u8; 12];
    let mut ip = 0;
    inner[ip] = 0x08;
    ip += 1; // field 1 (scan_finished)
    inner[ip] = 0x01;
    ip += 1; // varint 1 (true)
    inner[ip] = 0x10;
    ip += 1; // field 2 (result_count)
    ip += write_varint(ap_count as u64, &mut inner[ip..]);

    buf[pos] = 0x6a;
    pos += 1; // field 13, wire 2
    pos += write_varint(ip as u64, &mut buf[pos..]);
    buf[pos..pos + ip].copy_from_slice(&inner[..ip]);
    pos += ip;
    pos
}

/// Max bytes for one WiFiScanResult entry (SSID ≤ 32 + channel/rssi/bssid/auth).
const SCAN_ENTRY_MAX: usize = 64;
/// Inner RespScanResult payload sized for [`crate::MAX_CACHED_APS`] full SSIDs.
const SCAN_RESULT_INNER_CAP: usize = crate::MAX_CACHED_APS * (SCAN_ENTRY_MAX + 3);

/// Encode RespScanResult with entries from `aps[start_index..start_index+count]`.
///
/// Returns `(encoded_len, truncated)`. `truncated` is true when at least one
/// requested AP was omitted because the encode buffer filled.
fn encode_resp_scan_result(
    buf: &mut [u8],
    aps: &[CachedAp],
    ap_count: usize,
    start_index: usize,
    count: usize,
) -> (usize, bool) {
    let end = (start_index + count).min(ap_count).min(aps.len());
    let start = start_index.min(end);
    let requested = end.saturating_sub(start);

    let mut inner = [0u8; SCAN_RESULT_INNER_CAP];
    let mut ip = 0;
    let mut encoded = 0usize;
    let mut truncated = false;

    for ap in &aps[start..end] {
        let mut entry = [0u8; SCAN_ENTRY_MAX];
        let mut ep = 0;

        // Cap SSID so a corrupt `ssid_len` cannot overflow `entry`.
        let sl = (ap.ssid_len as usize).min(32).min(ap.ssid.len());

        // ssid (field 1, bytes)
        entry[ep] = 0x0a;
        ep += 1;
        ep += write_varint(sl as u64, &mut entry[ep..]);
        entry[ep..ep + sl].copy_from_slice(&ap.ssid[..sl]);
        ep += sl;

        // channel (field 2, varint)
        entry[ep] = 0x10;
        ep += 1;
        ep += write_varint(ap.channel as u64, &mut entry[ep..]);

        // rssi (field 3, int32 = varint; negative → sign-extend to u64)
        entry[ep] = 0x18;
        ep += 1;
        let rssi_u64 = ap.rssi as i64 as u64;
        ep += write_varint(rssi_u64, &mut entry[ep..]);

        // bssid (field 4, bytes, 6 bytes)
        entry[ep] = 0x22;
        ep += 1;
        entry[ep] = 0x06;
        ep += 1;
        entry[ep..ep + 6].copy_from_slice(&ap.bssid);
        ep += 6;

        // auth (field 5, varint)
        if ap.auth != 0 {
            entry[ep] = 0x28;
            ep += 1;
            ep += write_varint(ap.auth as u64, &mut entry[ep..]);
        }

        // tag (1) + varint length (≤2 for ep < 128) + entry
        let need = 1 + 2 + ep;
        if ip + need > inner.len() {
            truncated = true;
            break;
        }
        inner[ip] = 0x0a;
        ip += 1; // field 1, wire 2
        ip += write_varint(ep as u64, &mut inner[ip..]);
        inner[ip..ip + ep].copy_from_slice(&entry[..ep]);
        ip += ep;
        encoded += 1;
    }

    if encoded < requested {
        truncated = true;
    }

    let mut pos = 0;
    if buf.len() < 6 + ip {
        return (0, truncated || requested > 0);
    }
    buf[pos] = 0x08;
    pos += 1; // field 1 (msg)
    buf[pos] = 0x05;
    pos += 1; // varint 5
    buf[pos] = 0x10;
    pos += 1; // field 2 (status)
    buf[pos] = 0x00;
    pos += 1; // varint 0 (Success)

    buf[pos] = 0x7a;
    pos += 1; // field 15, wire 2
    pos += write_varint(ip as u64, &mut buf[pos..]);
    buf[pos..pos + ip].copy_from_slice(&inner[..ip]);
    pos += ip;
    (pos, truncated)
}

/// Parse CmdScanResult → (start_index, count).
fn parse_cmd_scan_result(data: &[u8]) -> (usize, usize) {
    let mut i = 0;
    let mut start_index: usize = 0;
    let mut count: usize = 4; // default
    while i < data.len() {
        let tag = match read_varint(data, &mut i) {
            Some(t) => t,
            None => break,
        };
        if tag == 0 {
            break;
        }
        let field = tag >> 3;
        let wire = tag & 7;
        match (field, wire) {
            (1, 0) => {
                let _ = read_varint(data, &mut i);
            }
            (14, 2) => {
                let len = read_varint(data, &mut i).unwrap_or(0) as usize;
                let end = (i + len).min(data.len());
                let inner = &data[i..end];
                i = end;
                let mut j = 0;
                while j < inner.len() {
                    let itag = match read_varint(inner, &mut j) {
                        Some(t) => t,
                        None => break,
                    };
                    if itag == 0 {
                        break;
                    }
                    let ifield = itag >> 3;
                    let iwire = itag & 7;
                    if iwire == 0 {
                        let val = read_varint(inner, &mut j).unwrap_or(0) as usize;
                        if ifield == 1 {
                            start_index = val;
                        }
                        if ifield == 2 {
                            count = val;
                        }
                    } else {
                        break;
                    }
                }
            }
            (_, 0) => {
                let _ = read_varint(data, &mut i);
            }
            (_, 2) => {
                let len = read_varint(data, &mut i).unwrap_or(0) as usize;
                i = (i + len).min(data.len());
            }
            _ => break,
        }
    }
    (start_index, count)
}

fn parse_msg_type(data: &[u8], i: &mut usize) -> u64 {
    while *i < data.len() {
        let tag = match read_varint(data, i) {
            Some(t) => t,
            None => break,
        };
        if tag == 0 {
            break;
        }
        let field = tag >> 3;
        let wire = tag & 7;
        match (field, wire) {
            (1, 0) => return read_varint(data, i).unwrap_or(0xFF),
            (_, 0) => {
                let _ = read_varint(data, i);
            }
            (_, 2) => {
                let len = read_varint(data, i).unwrap_or(0) as usize;
                *i = (*i + len).min(data.len());
            }
            _ => break,
        }
    }
    // Field 1 absent → protobuf default = 0 (CmdScanStart / CmdGetStatus).
    0
}

fn parse_cmd_set_config(
    data: &[u8],
    ssid: &mut heapless::String<MAX_SSID>,
    pass: &mut heapless::String<MAX_PASS>,
) {
    let mut i = 0;
    while i < data.len() {
        let tag = match read_varint(data, &mut i) {
            Some(t) => t,
            None => break,
        };
        if tag == 0 {
            break;
        }
        let field = tag >> 3;
        let wire = tag & 7;
        match (field, wire) {
            (12, 2) => {
                let len = read_varint(data, &mut i).unwrap_or(0) as usize;
                let end = (i + len).min(data.len());
                parse_ssid_pass(&data[i..end], ssid, pass);
                i = end;
            }
            (_, 0) => {
                let _ = read_varint(data, &mut i);
            }
            (_, 2) => {
                let len = read_varint(data, &mut i).unwrap_or(0) as usize;
                i = (i + len).min(data.len());
            }
            _ => break,
        }
    }
}

fn parse_ssid_pass(
    payload: &[u8],
    ssid: &mut heapless::String<MAX_SSID>,
    pass: &mut heapless::String<MAX_PASS>,
) {
    let mut i = 0;
    while i < payload.len() {
        let tag = match read_varint(payload, &mut i) {
            Some(t) => t,
            None => break,
        };
        if tag == 0 {
            break;
        }
        let field = tag >> 3;
        let wire = tag & 7;
        if wire != 2 {
            break;
        }
        let len = match read_varint(payload, &mut i) {
            Some(l) => l as usize,
            None => break,
        };
        if i + len > payload.len() {
            break;
        }
        let slice = &payload[i..i + len];
        i += len;
        match field {
            1 => set_str_from_utf8(ssid, slice),
            2 => set_str_from_utf8(pass, slice),
            _ => {}
        }
    }
}

fn set_str_from_utf8<const N: usize>(s: &mut heapless::String<N>, slice: &[u8]) {
    s.clear();
    let valid = core::str::from_utf8(slice).unwrap_or("");
    for c in valid.chars() {
        if s.push(c).is_err() {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_get_status_disconnected() {
        let mut buf = [0u8; 32];
        let n = encode_resp_get_status(&mut buf, false);
        assert_eq!(n, 10);
        assert_eq!(
            &buf[..n],
            &[0x08, 0x01, 0x10, 0x00, 0x5a, 0x04, 0x08, 0x00, 0x10, 0x02]
        );
    }

    #[test]
    fn apply_config_returns_apply() {
        let mut ssid = heapless::String::<MAX_SSID>::new();
        let mut pass = heapless::String::<MAX_PASS>::new();
        let mut resp = [0u8; 16];
        // msg_type = 4 (CmdApplyConfig): field1 varint → 0x08 0x04
        let data = [0x08u8, 0x04];
        let (result, n) = handle_prov_config_write(&data, &mut ssid, &mut pass, &mut resp);
        assert_eq!(result, ProvConfigResult::ApplyConfig);
        assert_eq!(n, 8);
    }

    #[test]
    fn set_and_read_cached_aps() {
        let mut ap = CachedAp::EMPTY;
        ap.ssid_len = 4;
        ap.ssid[..4].copy_from_slice(b"test");
        ap.channel = 6;
        ap.rssi = -50;
        crate::set_cached_aps(&[ap]);
        let (slice, count) = crate::cached_aps();
        assert_eq!(count, 1);
        assert_eq!(slice.len(), 1);
        assert_eq!(slice[0].ssid_len, 4);
        assert_eq!(&slice[0].ssid[..4], b"test");
    }

    #[test]
    fn set_cached_aps_slice_matches_count_after_shrink() {
        let mut first = [CachedAp::EMPTY; 2];
        first[0].ssid_len = 1;
        first[0].ssid[0] = b'a';
        first[1].ssid_len = 1;
        first[1].ssid[0] = b'b';
        crate::set_cached_aps(&first);
        assert_eq!(crate::cached_aps().0.len(), 2);
        crate::set_cached_aps(&first[..1]);
        let (slice, count) = crate::cached_aps();
        assert_eq!(count, 1);
        assert_eq!(slice.len(), 1);
        assert_eq!(slice[0].ssid[0], b'a');
    }

    #[test]
    fn encode_scan_result_caps_oversized_ssid_len() {
        let mut ap = CachedAp::EMPTY;
        ap.ssid_len = 200; // corrupt; must not panic
        ap.ssid[..32].fill(b'x');
        ap.channel = 1;
        let mut buf = [0u8; 256];
        let (n, truncated) = encode_resp_scan_result(&mut buf, &[ap], 1, 0, 1);
        assert!(n > 0);
        assert!(!truncated);
    }

    #[test]
    fn encode_scan_result_full_cache_not_truncated() {
        let mut aps = [CachedAp::EMPTY; crate::MAX_CACHED_APS];
        for (i, ap) in aps.iter_mut().enumerate() {
            ap.ssid_len = 32;
            ap.ssid[..32].fill(b'A' + (i as u8 % 26));
            ap.channel = (i as u8) + 1;
            ap.rssi = -40;
            ap.bssid = [i as u8; 6];
            ap.auth = 3;
        }
        let mut buf = [0u8; 2048];
        let (n, truncated) = encode_resp_scan_result(
            &mut buf,
            &aps,
            crate::MAX_CACHED_APS,
            0,
            crate::MAX_CACHED_APS,
        );
        assert!(n > 0);
        assert!(!truncated);
    }
}
