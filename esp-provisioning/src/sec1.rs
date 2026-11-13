//! ESP-IDF protocomm security 1 (sec1): Curve25519 + PoP + AES-256-CTR.
//!
//! Handles Session Command0/Response0 and Command1/Response1. The application
//! supplies the proof-of-possession (PoP) bytes.

use aes::cipher::{KeyIvInit, StreamCipher};
use rand_core::RngCore;
use sha2::{Digest, Sha256};
use x25519_dalek::{EphemeralSecret, PublicKey};

const PUBLIC_KEY_LEN: usize = 32;
const SZ_RANDOM: usize = 16;

/// Protobuf Sec1MsgType
#[allow(dead_code)]
const SEC1_MSG_SESSION_COMMAND0: u32 = 0;
#[allow(dead_code)]
const SEC1_MSG_SESSION_RESPONSE0: u32 = 1;
const SEC1_MSG_SESSION_COMMAND1: u32 = 2;
#[allow(dead_code)]
const SEC1_MSG_SESSION_RESPONSE1: u32 = 3;

/// Session state for sec1 handshake (one per connection).
#[instability::unstable]
pub struct Sec1State {
    state: Sec1StateInner,
    device_pubkey: [u8; PUBLIC_KEY_LEN],
    client_pubkey: [u8; PUBLIC_KEY_LEN],
    sym_key: [u8; PUBLIC_KEY_LEN],
    rand: [u8; SZ_RANDOM],
}

enum Sec1StateInner {
    /// Waiting for Session Command0 (client pubkey).
    Cmd0,
    /// Sent Resp0; waiting for Session Command1 (client verify).
    Cmd1,
    /// Sec1 handshake complete; cipher not yet taken.
    Done,
    /// Handshake complete and [`Sec1State::post_handshake_cipher`] already returned.
    CipherTaken,
}

impl Sec1State {
    /// Create a new sec1 state waiting for Command0.
    pub fn new() -> Self {
        Self {
            state: Sec1StateInner::Cmd0,
            device_pubkey: [0u8; PUBLIC_KEY_LEN],
            client_pubkey: [0u8; PUBLIC_KEY_LEN],
            sym_key: [0u8; PUBLIC_KEY_LEN],
            rand: [0u8; SZ_RANDOM],
        }
    }

    /// Reset to waiting for Command0.
    pub fn reset(&mut self) {
        self.state = Sec1StateInner::Cmd0;
    }

    /// Returns the post-handshake cipher once when the handshake is Done.
    ///
    /// Callers must keep the returned [`PostHandshakeCipher`] for the connection.
    /// A second call returns `None` so the AES-CTR keystream is never restarted at
    /// offset 64.
    pub fn post_handshake_cipher(&mut self) -> Option<PostHandshakeCipher> {
        if matches!(self.state, Sec1StateInner::Done) {
            self.state = Sec1StateInner::CipherTaken;
            Some(PostHandshakeCipher::new(&self.sym_key, &self.rand))
        } else {
            None
        }
    }
}

impl Default for Sec1State {
    fn default() -> Self {
        Self::new()
    }
}

/// Running AES-256-CTR cipher used for all messages after the sec1 handshake.
///
/// IDF's `sec1_encrypt`/`sec1_decrypt` each call `mbedtls_aes_crypt_ctr` on the
/// SAME persistent context (nc_off advances with every byte). The post-handshake
/// CTR starts fresh at block 0 (key = sym_key, nonce = device_random), independent
/// of the temporary contexts used for Cmd1/Resp1 during the handshake.
///
/// Both sides (device and app) advance the counter in lockstep:
///   App: encrypt Write → decrypt Read
///   Device: decrypt Write → encrypt Read response
#[instability::unstable]
pub struct PostHandshakeCipher {
    cipher: ctr::Ctr128BE<aes::Aes256>,
}

impl PostHandshakeCipher {
    fn new(sym_key: &[u8; PUBLIC_KEY_LEN], rand: &[u8; SZ_RANDOM]) -> Self {
        use aes::cipher::StreamCipherSeek;
        let mut cipher = ctr::Ctr128BE::<aes::Aes256>::new(sym_key.into(), rand.into());
        // The session uses ONE continuous AES-256-CTR stream (key=sym_key, nonce=device_random):
        //   bytes   0-31: Cmd1 — app encrypts device_pubkey → client_verify_data
        //   bytes  32-63: Resp1 — device encrypts client_pubkey → device_verify_data
        //   bytes 64+   : post-handshake (prov-scan, prov-config messages)
        // We skip the first 64 bytes already consumed by the handshake.
        cipher.seek(64u64);
        Self { cipher }
    }

    /// Apply keystream in-place — works for both encrypt and decrypt.
    ///
    /// Advances the internal counter; subsequent calls continue where this left off.
    pub fn apply(&mut self, data: &mut [u8]) {
        self.cipher.apply_keystream(data);
    }
}

/// Result of handling session data: either a response to write into the session
/// buffer, or use sec0.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum SessionResponse<'a> {
    /// Use these bytes as the session response (sec1 Resp0 or Resp1).
    Sec1(&'a [u8]),
    /// Use [`crate::SESSION_RESPONSE_SEC0`] (sec0).
    Sec0,
    /// Proof-of-possession check failed. The wire response is still sec0.
    OwnershipFailed,
    /// `response_buf` is shorter than [`crate::SESSION_RESPONSE_MIN_LEN`].
    BufferTooSmall,
}

use crate::pb::read_varint;

fn skip_field(data: &[u8], i: &mut usize, wire: u8) -> bool {
    match wire {
        0 => read_varint(data, i).is_some(),
        2 => {
            if let Some(len) = read_varint(data, i).map(|l| l as usize) {
                *i = (*i).saturating_add(len).min(data.len());
                true
            } else {
                false
            }
        }
        _ => false,
    }
}

fn parse_session_data(data: &[u8]) -> Option<(u32, Option<&[u8]>)> {
    let mut i = 0;
    let mut sec_ver: Option<u32> = None;
    let mut sec1_payload: Option<&[u8]> = None;
    while i < data.len() {
        let tag = read_varint(data, &mut i)?;
        let field = (tag >> 3) as u32;
        let wire = (tag & 7) as u8;
        if field == 0 {
            // Field 0 is invalid in protobuf; trailing zero bytes — stop.
            break;
        }
        match field {
            2 => sec_ver = read_varint(data, &mut i).map(|v| v as u32),
            11 => {
                let len = read_varint(data, &mut i).map(|l| l as usize)?;
                if i + len <= data.len() {
                    sec1_payload = Some(&data[i..i + len]);
                    i += len;
                } else {
                    return None;
                }
            }
            _ => {
                if !skip_field(data, &mut i, wire) {
                    return None;
                }
            }
        }
    }
    sec_ver.map(|v| (v, sec1_payload))
}

fn parse_sec1_payload(data: &[u8]) -> Option<(u32, Option<&[u8]>, Option<&[u8]>)> {
    let mut i = 0;
    let mut msg: Option<u32> = None;
    let mut sc0: Option<&[u8]> = None;
    let mut sc1: Option<&[u8]> = None;
    while i < data.len() {
        let tag = read_varint(data, &mut i)?;
        let field = (tag >> 3) as u32;
        let wire = (tag & 7) as u8;
        if field == 0 {
            break;
        }
        match field {
            1 => msg = read_varint(data, &mut i).map(|v| v as u32),
            20 => {
                let len = read_varint(data, &mut i).map(|l| l as usize)?;
                if i + len <= data.len() {
                    sc0 = Some(&data[i..i + len]);
                    i += len;
                } else {
                    return None;
                }
            }
            22 => {
                let len = read_varint(data, &mut i).map(|l| l as usize)?;
                if i + len <= data.len() {
                    sc1 = Some(&data[i..i + len]);
                    i += len;
                } else {
                    return None;
                }
            }
            _ => {
                if !skip_field(data, &mut i, wire) {
                    return None;
                }
            }
        }
    }
    // Protobuf default for an enum field is 0 (SESSION_CMD0). The ESP app omits `msg` when
    // sending Cmd0, so treat a missing field 1 as msg=0 rather than failing the parse.
    Some((msg.unwrap_or(0), sc0, sc1))
}

fn parse_sc0(data: &[u8]) -> Option<[u8; PUBLIC_KEY_LEN]> {
    let mut i = 0;
    while i < data.len() {
        let tag = read_varint(data, &mut i)?;
        let field = (tag >> 3) as u32;
        let wire = (tag & 7) as u8;
        if field == 1 && wire == 2 {
            let len = read_varint(data, &mut i).map(|l| l as usize)?;
            if i + len == data.len() && len == PUBLIC_KEY_LEN {
                let mut out = [0u8; PUBLIC_KEY_LEN];
                out.copy_from_slice(&data[i..i + len]);
                return Some(out);
            }
            return None;
        }
        if !skip_field(data, &mut i, wire) {
            return None;
        }
    }
    None
}

fn parse_sc1(data: &[u8]) -> Option<&[u8]> {
    let mut i = 0;
    while i < data.len() {
        let tag = read_varint(data, &mut i)?;
        let field = (tag >> 3) as u32;
        let wire = (tag & 7) as u8;
        if field == 2 && wire == 2 {
            let len = read_varint(data, &mut i).map(|l| l as usize)?;
            if i + len <= data.len() && len == PUBLIC_KEY_LEN {
                return Some(&data[i..i + len]);
            }
            return None;
        }
        if !skip_field(data, &mut i, wire) {
            return None;
        }
    }
    None
}

fn encode_session_sec1_resp0(
    out: &mut [u8],
    device_pubkey: &[u8; PUBLIC_KEY_LEN],
    device_random: &[u8; SZ_RANDOM],
) -> Option<usize> {
    let mut sr0 = [0u8; 64];
    let mut n = 0;
    sr0[n] = 0x08;
    n += 1;
    sr0[n] = 0x00;
    n += 1;
    sr0[n] = 0x12;
    n += 1;
    sr0[n] = 32;
    n += 1;
    sr0[n..n + 32].copy_from_slice(device_pubkey);
    n += 32;
    sr0[n] = 0x1a;
    n += 1;
    sr0[n] = 16;
    n += 1;
    sr0[n..n + 16].copy_from_slice(device_random);
    n += 16;
    let sr0_len = n;

    // Sec1Payload wrapper.
    // Field 21 (sr0), wire 2: tag = (21<<3)|2 = 170 = 0xaa. 170 > 127 → 2-byte varint [0xaa, 0x01].
    let mut sec1 = [0u8; 82];
    n = 0;
    sec1[n] = 0x08;
    n += 1;
    sec1[n] = 0x01; // Session_Response0
    n += 1;
    sec1[n] = 0xaa; // field 21 tag byte 1/2 (varint 170)
    n += 1;
    sec1[n] = 0x01; // field 21 tag byte 2/2
    n += 1;
    sec1[n] = sr0_len as u8;
    n += 1;
    sec1[n..n + sr0_len].copy_from_slice(&sr0[..sr0_len]);
    n += sr0_len;
    let sec1_len = n;

    n = 0;
    if out.len() < 2 + 2 + sec1_len {
        return None;
    }
    out[n] = 0x10;
    n += 1;
    out[n] = 0x01;
    n += 1;
    out[n] = 0x5a;
    n += 1;
    out[n] = sec1_len as u8;
    n += 1;
    out[n..n + sec1_len].copy_from_slice(&sec1[..sec1_len]);
    n += sec1_len;
    Some(n)
}

fn encode_session_sec1_resp1(
    out: &mut [u8],
    device_verify_data: &[u8; PUBLIC_KEY_LEN],
) -> Option<usize> {
    let mut sr1 = [0u8; 48];
    let mut n = 0;
    sr1[n] = 0x08;
    n += 1;
    sr1[n] = 0x00;
    n += 1;
    sr1[n] = 0x1a;
    n += 1;
    sr1[n] = 32;
    n += 1;
    sr1[n..n + 32].copy_from_slice(device_verify_data);
    n += 32;
    let sr1_len = n;

    // Sec1Payload wrapper.
    // Field 23 (sr1), wire 2: tag = (23<<3)|2 = 186 = 0xba. 186 > 127 → 2-byte varint [0xba, 0x01].
    let mut sec1 = [0u8; 50];
    n = 0;
    sec1[n] = 0x08;
    n += 1;
    sec1[n] = 0x03; // Session_Response1
    n += 1;
    sec1[n] = 0xba;
    n += 1;
    sec1[n] = 0x01;
    n += 1;
    sec1[n] = sr1_len as u8;
    n += 1;
    sec1[n..n + sr1_len].copy_from_slice(&sr1[..sr1_len]);
    n += sr1_len;
    let sec1_len = n;

    n = 0;
    if out.len() < 2 + 2 + sec1_len {
        return None;
    }
    out[n] = 0x10;
    n += 1;
    out[n] = 0x01;
    n += 1;
    out[n] = 0x5a;
    n += 1;
    out[n] = sec1_len as u8;
    n += 1;
    out[n..n + sec1_len].copy_from_slice(&sec1[..sec1_len]);
    n += sec1_len;
    Some(n)
}

/// Handle session write data.
///
/// If sec1 Cmd0 or Cmd1, compute the response and return [`SessionResponse::Sec1`].
/// If the response would not fit in `response_buf`, return
/// [`SessionResponse::BufferTooSmall`] (use at least [`crate::SESSION_RESPONSE_MIN_LEN`]).
/// Otherwise return [`SessionResponse::Sec0`], or [`SessionResponse::OwnershipFailed`]
/// when the proof-of-possession check does not match.
///
/// `pop` is the proof-of-possession (PIN) as raw bytes; empty means no PoP XOR.
#[instability::unstable]
pub fn handle_session<'a, R: RngCore + rand_core::CryptoRng>(
    data: &[u8],
    state: &mut Sec1State,
    rng: &mut R,
    pop: &[u8],
    response_buf: &'a mut [u8],
) -> SessionResponse<'a> {
    let (sec_ver, sec1_payload) = match parse_session_data(data) {
        Some(p) => p,
        None => {
            trace!("esp-prov sec1: parse_session_data failed");
            return SessionResponse::Sec0;
        }
    };

    if cfg!(feature = "debug") {
        info!(
            "esp-prov sec1: handle_session sec_ver={} payload={}",
            sec_ver,
            if sec1_payload.is_some() {
                "yes"
            } else {
                "none"
            }
        );
    }

    if sec_ver != 1 {
        warn!("esp-prov sec1: sec_ver={} (not 1), using sec0", sec_ver);
        return SessionResponse::Sec0;
    }

    let sec1 = match sec1_payload {
        Some(s) => s,
        None => {
            warn!("esp-prov sec1: no sec1 payload in session data");
            return SessionResponse::Sec0;
        }
    };

    let (msg, sc0, sc1) = match parse_sec1_payload(sec1) {
        Some(p) => p,
        None => {
            warn!(
                "esp-prov sec1: parse_sec1_payload failed (sec1 len={})",
                sec1.len()
            );
            return SessionResponse::Sec0;
        }
    };

    if cfg!(feature = "debug") {
        info!(
            "esp-prov sec1: msg={} sc0={} sc1={}",
            msg,
            sc0.is_some(),
            sc1.is_some()
        );
    }

    match msg {
        SEC1_MSG_SESSION_COMMAND0 => {
            let client_pubkey = match sc0.and_then(parse_sc0) {
                Some(k) => k,
                None => {
                    warn!("esp-prov sec1: Cmd0 parse sc0 failed");
                    return SessionResponse::Sec0;
                }
            };
            if !matches!(state.state, Sec1StateInner::Cmd0) {
                state.reset();
            }
            let device_secret = EphemeralSecret::random_from_rng(&mut *rng);
            let device_pubkey = PublicKey::from(&device_secret);
            state.device_pubkey = device_pubkey.to_bytes();
            state.client_pubkey = client_pubkey;
            let client_pk = PublicKey::from(client_pubkey);
            let shared = device_secret.diffie_hellman(&client_pk);
            state.sym_key = shared.to_bytes();
            if !pop.is_empty() {
                let mut hasher = Sha256::new();
                hasher.update(pop);
                let hash = hasher.finalize();
                for i in 0..PUBLIC_KEY_LEN {
                    state.sym_key[i] ^= hash[i];
                }
            }
            rng.fill_bytes(&mut state.rand);
            state.state = Sec1StateInner::Cmd1;
            if cfg!(feature = "debug") {
                info!("esp-prov sec1: Cmd0 → Resp0");
            }
            match encode_session_sec1_resp0(response_buf, &state.device_pubkey, &state.rand) {
                Some(n) => return SessionResponse::Sec1(&response_buf[..n]),
                None => return SessionResponse::BufferTooSmall,
            }
        }
        SEC1_MSG_SESSION_COMMAND1 => {
            let client_verify = match sc1.and_then(parse_sc1) {
                Some(s) if s.len() == PUBLIC_KEY_LEN => s,
                _ => {
                    warn!("esp-prov sec1: Cmd1 parse sc1 failed");
                    return SessionResponse::Sec0;
                }
            };
            if !matches!(state.state, Sec1StateInner::Cmd1) {
                warn!("esp-prov sec1: Cmd1 in wrong state");
                return SessionResponse::Sec0;
            }
            // mbedtls_aes_crypt_ctr increments the 16-byte nonce as a 128-bit big-endian integer
            // (last byte first). Ctr128BE matches this exactly.
            type Aes256Ctr = ctr::Ctr128BE<aes::Aes256>;
            let mut cipher =
                Aes256Ctr::new(state.sym_key.as_ref().into(), state.rand.as_ref().into());
            let mut decrypted = [0u8; PUBLIC_KEY_LEN];
            decrypted.copy_from_slice(client_verify);
            cipher.apply_keystream(&mut decrypted);
            info!("Checking ownership.");
            if cfg!(feature = "debug") {
                info!("esp-prov sec1: Cmd1 decrypted={:02x?}", &decrypted[..8]);
                info!(
                    "esp-prov sec1: Cmd1 device_pk={:02x?}",
                    &state.device_pubkey[..8]
                );
            }
            if decrypted != state.device_pubkey {
                warn!("Ownership check failed.");
                if cfg!(feature = "debug") {
                    warn!(
                        "esp-prov sec1: Cmd1 verify failed — decrypted != device_pubkey (cipher or PIN mismatch)"
                    );
                }
                state.reset();
                return SessionResponse::OwnershipFailed;
            }
            info!("Ownership check succeeded.");
            let mut to_encrypt = state.client_pubkey;
            cipher.apply_keystream(&mut to_encrypt);
            state.state = Sec1StateInner::Done;
            if cfg!(feature = "debug") {
                info!("esp-prov sec1: Cmd1 → Resp1");
            }
            match encode_session_sec1_resp1(response_buf, &to_encrypt) {
                Some(n) => return SessionResponse::Sec1(&response_buf[..n]),
                None => return SessionResponse::BufferTooSmall,
            }
        }
        _ => {}
    }
    SessionResponse::Sec0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_session_rejects_empty() {
        assert!(parse_session_data(&[]).is_none());
    }

    #[test]
    fn parse_session_sec_ver_only() {
        // field 2 = sec_ver = 1 → tag 0x10, value 0x01
        let data = [0x10u8, 0x01];
        let (ver, payload) = parse_session_data(&data).expect("parse");
        assert_eq!(ver, 1);
        assert!(payload.is_none());
    }

    #[test]
    fn encode_resp0_rejects_short_buffer() {
        let pubkey = [0x11u8; PUBLIC_KEY_LEN];
        let rand = [0x22u8; SZ_RANDOM];
        let mut tiny = [0u8; 8];
        assert!(encode_session_sec1_resp0(&mut tiny, &pubkey, &rand).is_none());
        let mut ok = [0u8; crate::SESSION_RESPONSE_MIN_LEN];
        assert!(encode_session_sec1_resp0(&mut ok, &pubkey, &rand).is_some());
    }

    #[test]
    fn post_handshake_cipher_is_one_shot() {
        let mut state = Sec1State::new();
        state.state = Sec1StateInner::Done;
        assert!(state.post_handshake_cipher().is_some());
        assert!(state.post_handshake_cipher().is_none());
    }
}
