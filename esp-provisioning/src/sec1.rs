//! ESP-IDF protocomm security 1 (Curve25519 + PoP + AES-256-CTR).
//!
//! Wire messages are the micropb types generated from the ESP-IDF v5.4.2
//! `protocomm` schema (not `network_provisioning`). sec1 is the only
//! [`Security`] implementation in this crate.

use aes::cipher::{KeyIvInit, StreamCipher, StreamCipherSeek};
use rand_core::{CryptoRng, RngCore};
use sha2::{Digest, Sha256};
use x25519_dalek::{EphemeralSecret, PublicKey};

use crate::proto::{
    self,
    Sec1MsgType,
    Sec1Payload,
    Sec1Payload_,
    SecSchemeVersion,
    SessionCmd0,
    SessionCmd1,
    SessionData,
    SessionData_,
    SessionResp0,
    SessionResp1,
    Status,
};

const PUBLIC_KEY_LEN: usize = 32;
const SZ_RANDOM: usize = 16;

/// Why a sec1 handshake or a later encrypted message was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SessionError {
    /// Protobuf did not decode, or a key/verify blob had the wrong length.
    Parse,
    /// The response buffer cannot hold the encoded reply.
    BufferTooSmall,
    /// `prov-config` / `prov-scan` arrived before the handshake finished.
    NotEstablished,
}

/// Handshake plus the post-handshake AES-256-CTR stream.
///
/// [`crate::handlers::ConnState`] is generic over this trait. sec1 is the only
/// implementation; sec2 can be added without rewriting the transports.
///
/// `handshake` is generic over the RNG, so this trait is not object-safe.
pub(crate) trait Security: Default {
    fn established(&self) -> bool;

    fn handshake<R: RngCore + CryptoRng>(
        &mut self,
        request: &[u8],
        rng: &mut R,
        pop: &[u8],
        response: &mut [u8],
    ) -> Result<usize, SessionError>;

    /// Name printed on the shared session and GetStatus lines. Empty in tests.
    #[cfg_attr(not(any(feature = "softap", feature = "ble")), allow(dead_code))]
    fn set_transport(&mut self, _transport: &'static str) {}

    fn encrypt(&mut self, data: &mut [u8]) -> Result<(), SessionError>;

    fn decrypt(&mut self, data: &mut [u8]) -> Result<(), SessionError>;
}

/// Session state for sec1 (one per connection).
pub(crate) struct Sec1 {
    state: Sec1State,
    post: Option<PostHandshakeCipher>,
    transport: &'static str,
}

/// One line both transports share. `event` is `session` or `GetStatus sent`.
#[allow(unused_variables)]
pub(crate) fn log_report(transport: &str, event: &str) {
    info!("[{}] {}", transport, event);
}

enum Sec1State {
    Cmd0,
    Cmd1 {
        device_pubkey: [u8; PUBLIC_KEY_LEN],
        client_pubkey: [u8; PUBLIC_KEY_LEN],
        sym_key: [u8; PUBLIC_KEY_LEN],
        rand: [u8; SZ_RANDOM],
    },
}

impl Sec1 {
    pub(crate) fn new() -> Self {
        Self {
            state: Sec1State::Cmd0,
            post: None,
            transport: "",
        }
    }
}

impl Default for Sec1 {
    fn default() -> Self {
        Self::new()
    }
}

impl Security for Sec1 {
    fn established(&self) -> bool {
        self.post.is_some()
    }

    fn handshake<R: RngCore + CryptoRng>(
        &mut self,
        request: &[u8],
        rng: &mut R,
        pop: &[u8],
        response: &mut [u8],
    ) -> Result<usize, SessionError> {
        if request.is_empty() {
            return Err(SessionError::Parse);
        }
        let session: SessionData = proto::decode(request).map_err(|_| SessionError::Parse)?;
        if session.sec_ver != SecSchemeVersion::SecScheme1 {
            warn!("sec1 rejected version");
            return Ok(session_failure(Status::InvalidSecScheme, response)?);
        }
        let SessionData_::Proto::Sec1(sec1) = session.proto.ok_or(SessionError::Parse)? else {
            warn!("sec1 rejected version");
            return Ok(session_failure(Status::InvalidSecScheme, response)?);
        };

        match sec1.payload {
            Some(Sec1Payload_::Payload::Sc0(cmd)) => self.command0(&cmd, rng, pop, response),
            Some(Sec1Payload_::Payload::Sc1(cmd)) => self.command1(&cmd, response),
            _ => Err(SessionError::Parse),
        }
    }

    fn encrypt(&mut self, data: &mut [u8]) -> Result<(), SessionError> {
        self.post
            .as_mut()
            .ok_or(SessionError::NotEstablished)?
            .apply(data);
        Ok(())
    }

    fn decrypt(&mut self, data: &mut [u8]) -> Result<(), SessionError> {
        self.encrypt(data)
    }

    fn set_transport(&mut self, transport: &'static str) {
        self.transport = transport;
    }
}

impl Sec1 {
    fn command0<R: RngCore + CryptoRng>(
        &mut self,
        cmd: &SessionCmd0,
        rng: &mut R,
        pop: &[u8],
        response: &mut [u8],
    ) -> Result<usize, SessionError> {
        let client_pubkey: [u8; PUBLIC_KEY_LEN] = cmd
            .client_pubkey
            .as_slice()
            .try_into()
            .map_err(|_| SessionError::Parse)?;

        let device_secret = EphemeralSecret::random_from_rng(&mut *rng);
        let device_pubkey = PublicKey::from(&device_secret);
        let device_pubkey = device_pubkey.to_bytes();
        let shared = device_secret.diffie_hellman(&PublicKey::from(client_pubkey));
        let mut sym_key = shared.to_bytes();
        if !pop.is_empty() {
            let mut hasher = Sha256::new();
            hasher.update(pop);
            let hash = hasher.finalize();
            for (byte, mix) in sym_key.iter_mut().zip(hash.iter()) {
                *byte ^= mix;
            }
        }
        let mut rand = [0u8; SZ_RANDOM];
        rng.fill_bytes(&mut rand);

        let mut device_pubkey_buf = heapless::Vec::new();
        device_pubkey_buf
            .extend_from_slice(&device_pubkey)
            .map_err(|_| SessionError::BufferTooSmall)?;
        let mut device_random = heapless::Vec::new();
        device_random
            .extend_from_slice(&rand)
            .map_err(|_| SessionError::BufferTooSmall)?;

        let msg = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionResponse0,
                payload: Some(Sec1Payload_::Payload::Sr0(SessionResp0 {
                    status: Status::Success,
                    device_pubkey: device_pubkey_buf,
                    device_random,
                })),
            })),
        };
        // Commit only after the reply fits. A short buffer must not leave Cmd1
        // or a cipher the phone never received.
        let len = proto::encode(&msg, response).map_err(|_| SessionError::BufferTooSmall)?;
        self.state = Sec1State::Cmd1 {
            device_pubkey,
            client_pubkey,
            sym_key,
            rand,
        };
        self.post = None;
        Ok(len)
    }

    fn command1(&mut self, cmd: &SessionCmd1, response: &mut [u8]) -> Result<usize, SessionError> {
        let Sec1State::Cmd1 {
            device_pubkey,
            client_pubkey,
            sym_key,
            rand,
        } = &self.state
        else {
            warn!("sec1 command1 before command0");
            return Ok(session_failure(Status::CryptoError, response)?);
        };
        let device_pubkey = *device_pubkey;
        let client_pubkey = *client_pubkey;
        let sym_key = *sym_key;
        let rand = *rand;
        let client_verify: [u8; PUBLIC_KEY_LEN] = cmd
            .client_verify_data
            .as_slice()
            .try_into()
            .map_err(|_| SessionError::Parse)?;

        let mut cipher =
            ctr::Ctr128BE::<aes::Aes256>::new(sym_key.as_ref().into(), rand.as_ref().into());
        let mut decrypted = client_verify;
        cipher.apply_keystream(&mut decrypted);
        if decrypted != device_pubkey {
            warn!("Ownership check failed.");
            self.state = Sec1State::Cmd0;
            self.post = None;
            return Ok(session_failure(Status::CryptoError, response)?);
        }
        if !self.transport.is_empty() {
            log_report(self.transport, "session");
        }
        info!("Ownership check succeeded.");

        let mut device_verify = client_pubkey;
        cipher.apply_keystream(&mut device_verify);
        let mut verify = heapless::Vec::new();
        verify
            .extend_from_slice(&device_verify)
            .map_err(|_| SessionError::BufferTooSmall)?;
        let msg = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionResponse1,
                payload: Some(Sec1Payload_::Payload::Sr1(SessionResp1 {
                    status: Status::Success,
                    device_verify_data: verify,
                })),
            })),
        };
        let len = proto::encode(&msg, response).map_err(|_| SessionError::BufferTooSmall)?;
        self.post = Some(PostHandshakeCipher::new(&sym_key, &rand));
        self.state = Sec1State::Cmd0;
        Ok(len)
    }
}

/// sec1 `SessionResp1` with a non-success status and no verify blob.
///
/// The phone sees the failure. The handshake stays unestablished, so a later
/// `prov-config` is still rejected.
fn session_failure(status: Status, response: &mut [u8]) -> Result<usize, SessionError> {
    let msg = SessionData {
        sec_ver: SecSchemeVersion::SecScheme1,
        proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
            msg: Sec1MsgType::SessionResponse1,
            payload: Some(Sec1Payload_::Payload::Sr1(SessionResp1 {
                status,
                device_verify_data: heapless::Vec::new(),
            })),
        })),
    };
    proto::encode(&msg, response).map_err(|_| SessionError::BufferTooSmall)
}

/// Running AES-256-CTR cipher used for every byte after the handshake.
///
/// The session uses one continuous keystream (key = shared secret, nonce =
/// device random). Command1 consumes the first 32 bytes and Response1 the next
/// 32, so post-handshake messages start at offset 64.
struct PostHandshakeCipher {
    cipher: ctr::Ctr128BE<aes::Aes256>,
}

impl PostHandshakeCipher {
    fn new(sym_key: &[u8; PUBLIC_KEY_LEN], rand: &[u8; SZ_RANDOM]) -> Self {
        let mut cipher = ctr::Ctr128BE::<aes::Aes256>::new(sym_key.into(), rand.into());
        cipher.seek(64u64);
        Self { cipher }
    }

    fn apply(&mut self, data: &mut [u8]) {
        self.cipher.apply_keystream(data);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestRng(u64);

    impl RngCore for TestRng {
        fn next_u32(&mut self) -> u32 {
            self.next_u64() as u32
        }

        fn next_u64(&mut self) -> u64 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1);
            self.0
        }

        fn fill_bytes(&mut self, dest: &mut [u8]) {
            for chunk in dest.chunks_mut(8) {
                let bytes = self.next_u64().to_le_bytes();
                chunk.copy_from_slice(&bytes[..chunk.len()]);
            }
        }

        fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
            self.fill_bytes(dest);
            Ok(())
        }
    }

    impl CryptoRng for TestRng {}

    #[test]
    fn handshake_rejects_empty() {
        let mut sec = Sec1::new();
        let mut rng = TestRng(1);
        let mut out = [0u8; 256];
        assert_eq!(
            sec.handshake(&[], &mut rng, b"abcd1234", &mut out),
            Err(SessionError::Parse)
        );
        assert!(!sec.established());
    }

    #[test]
    fn handshake_rejects_version_without_sec1_payload() {
        let msg = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: None,
        };
        let mut raw = [0u8; 32];
        let n = proto::encode(&msg, &mut raw).unwrap();
        let mut sec = Sec1::new();
        let mut rng = TestRng(1);
        let mut out = [0u8; 8];
        assert_eq!(
            sec.handshake(&raw[..n], &mut rng, b"", &mut out),
            Err(SessionError::Parse)
        );
    }

    #[test]
    fn command0_rejects_short_buffer() {
        let mut pubkey = heapless::Vec::new();
        pubkey.extend_from_slice(&[0x11; 32]).unwrap();
        let msg = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionCommand0,
                payload: Some(Sec1Payload_::Payload::Sc0(SessionCmd0 {
                    client_pubkey: pubkey,
                })),
            })),
        };
        let mut raw = [0u8; 128];
        let n = proto::encode(&msg, &mut raw).unwrap();
        let mut sec = Sec1::new();
        let mut rng = TestRng(7);
        let mut tiny = [0u8; 8];
        assert_eq!(
            sec.handshake(&raw[..n], &mut rng, b"pop", &mut tiny),
            Err(SessionError::BufferTooSmall)
        );
        assert!(!sec.established());
        // The failed Response0 must not leave the session in Cmd1.
        let mut verify = heapless::Vec::new();
        verify.extend_from_slice(&[0x22; 32]).unwrap();
        let cmd1 = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionCommand1,
                payload: Some(Sec1Payload_::Payload::Sc1(SessionCmd1 {
                    client_verify_data: verify,
                })),
            })),
        };
        let n = proto::encode(&cmd1, &mut raw).unwrap();
        let mut out = [0u8; 128];
        let len = sec
            .handshake(&raw[..n], &mut rng, b"pop", &mut out)
            .unwrap();
        let decoded: SessionData = proto::decode(&out[..len]).unwrap();
        let SessionData_::Proto::Sec1(payload) = decoded.proto.unwrap() else {
            panic!("expected sec1");
        };
        let Some(Sec1Payload_::Payload::Sr1(resp)) = payload.payload else {
            panic!("expected session response 1");
        };
        assert_eq!(resp.status, Status::CryptoError);
        assert!(!sec.established());
    }

    #[test]
    fn decrypt_before_handshake_is_not_established() {
        let mut sec = Sec1::new();
        let mut buf = [1u8, 2, 3];
        assert_eq!(sec.decrypt(&mut buf), Err(SessionError::NotEstablished));
        assert_eq!(buf, [1, 2, 3]);
    }

    fn session_cmd0(pubkey: [u8; 32]) -> SessionData {
        let mut client_pubkey = heapless::Vec::new();
        client_pubkey.extend_from_slice(&pubkey).unwrap();
        SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionCommand0,
                payload: Some(Sec1Payload_::Payload::Sc0(SessionCmd0 { client_pubkey })),
            })),
        }
    }

    #[test]
    fn wrong_version_replies_with_invalid_sec_scheme() {
        // SecScheme0 is the zero value, so micropb omits it and the body is empty.
        let msg = SessionData {
            sec_ver: SecSchemeVersion::SecScheme2,
            proto: None,
        };
        let mut raw = [0u8; 32];
        let n = proto::encode(&msg, &mut raw).unwrap();
        let mut sec = Sec1::new();
        let mut rng = TestRng(1);
        let mut out = [0u8; 128];
        let len = sec
            .handshake(&raw[..n], &mut rng, b"abcd1234", &mut out)
            .unwrap();
        let decoded: SessionData = proto::decode(&out[..len]).unwrap();
        let SessionData_::Proto::Sec1(sec1) = decoded.proto.unwrap() else {
            panic!("expected sec1");
        };
        let Some(Sec1Payload_::Payload::Sr1(resp)) = sec1.payload else {
            panic!("expected session response 1");
        };
        assert_eq!(resp.status, Status::InvalidSecScheme);
        assert!(!sec.established());
    }

    #[test]
    fn pop_mismatch_replies_with_crypto_error() {
        let mut raw = [0u8; 128];
        let n = proto::encode(&session_cmd0([0x11; 32]), &mut raw).unwrap();
        let mut sec = Sec1::new();
        let mut rng = TestRng(3);
        let mut out = [0u8; 256];
        sec.handshake(&raw[..n], &mut rng, b"abcd1234", &mut out)
            .unwrap();

        let mut verify = heapless::Vec::new();
        verify.extend_from_slice(&[0x22; 32]).unwrap();
        let cmd1 = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionCommand1,
                payload: Some(Sec1Payload_::Payload::Sc1(SessionCmd1 {
                    client_verify_data: verify,
                })),
            })),
        };
        let n = proto::encode(&cmd1, &mut raw).unwrap();
        let len = sec
            .handshake(&raw[..n], &mut rng, b"abcd1234", &mut out)
            .unwrap();
        let decoded: SessionData = proto::decode(&out[..len]).unwrap();
        let SessionData_::Proto::Sec1(sec1) = decoded.proto.unwrap() else {
            panic!("expected sec1");
        };
        let Some(Sec1Payload_::Payload::Sr1(resp)) = sec1.payload else {
            panic!("expected session response 1");
        };
        assert_eq!(resp.status, Status::CryptoError);
        assert!(resp.device_verify_data.is_empty());
        assert!(!sec.established());
        let mut buf = [9, 8, 7];
        assert_eq!(sec.decrypt(&mut buf), Err(SessionError::NotEstablished));
        assert_eq!(buf, [9, 8, 7]);
    }

    #[test]
    fn command1_before_command0_is_crypto_error() {
        let mut verify = heapless::Vec::new();
        verify.extend_from_slice(&[0x22; 32]).unwrap();
        let cmd1 = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionCommand1,
                payload: Some(Sec1Payload_::Payload::Sc1(SessionCmd1 {
                    client_verify_data: verify,
                })),
            })),
        };
        let mut raw = [0u8; 128];
        let n = proto::encode(&cmd1, &mut raw).unwrap();
        let mut sec = Sec1::new();
        let mut rng = TestRng(1);
        let mut out = [0u8; 128];
        let len = sec
            .handshake(&raw[..n], &mut rng, b"abcd1234", &mut out)
            .unwrap();
        let decoded: SessionData = proto::decode(&out[..len]).unwrap();
        let SessionData_::Proto::Sec1(sec1) = decoded.proto.unwrap() else {
            panic!("expected sec1");
        };
        let Some(Sec1Payload_::Payload::Sr1(resp)) = sec1.payload else {
            panic!("expected session response 1");
        };
        assert_eq!(resp.status, Status::CryptoError);
        assert!(resp.device_verify_data.is_empty());
        assert!(!sec.established());
    }

    #[test]
    fn command0_rejects_short_public_key() {
        let mut pubkey = heapless::Vec::new();
        pubkey.extend_from_slice(&[0x11; 31]).unwrap();
        let msg = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionCommand0,
                payload: Some(Sec1Payload_::Payload::Sc0(SessionCmd0 {
                    client_pubkey: pubkey,
                })),
            })),
        };
        let mut raw = [0u8; 128];
        let n = proto::encode(&msg, &mut raw).unwrap();
        let mut sec = Sec1::new();
        let mut rng = TestRng(1);
        let mut out = [0u8; 256];
        assert_eq!(
            sec.handshake(&raw[..n], &mut rng, b"abcd1234", &mut out),
            Err(SessionError::Parse)
        );
        assert!(!sec.established());
    }

    fn mix_pop(mut secret: [u8; 32], pop: &[u8]) -> [u8; 32] {
        if !pop.is_empty() {
            let mut hasher = Sha256::new();
            hasher.update(pop);
            let hash = hasher.finalize();
            for (byte, mix) in secret.iter_mut().zip(hash.iter()) {
                *byte ^= mix;
            }
        }
        secret
    }

    struct Command0 {
        device_pubkey: [u8; 32],
        device_random: [u8; 16],
    }

    fn exchange_command0(sec: &mut Sec1, pop: &[u8], client_public: [u8; 32]) -> Command0 {
        let mut raw = [0u8; 128];
        let n = proto::encode(&session_cmd0(client_public), &mut raw).unwrap();
        let mut rng = TestRng(3);
        let mut out = [0u8; 256];
        let len = sec.handshake(&raw[..n], &mut rng, pop, &mut out).unwrap();
        let decoded: SessionData = proto::decode(&out[..len]).unwrap();
        let SessionData_::Proto::Sec1(payload) = decoded.proto.unwrap() else {
            panic!("expected sec1");
        };
        let Some(Sec1Payload_::Payload::Sr0(resp)) = payload.payload else {
            panic!("expected session response 0");
        };
        assert_eq!(resp.status, Status::Success);
        Command0 {
            device_pubkey: resp.device_pubkey.as_slice().try_into().unwrap(),
            device_random: resp.device_random.as_slice().try_into().unwrap(),
        }
    }

    fn client_verify(sym_key: &[u8; 32], iv: &[u8; 16], device_pubkey: &[u8; 32]) -> [u8; 32] {
        let mut cipher = ctr::Ctr128BE::<aes::Aes256>::new(sym_key.into(), iv.into());
        let mut verify = *device_pubkey;
        cipher.apply_keystream(&mut verify);
        verify
    }

    #[test]
    fn handshake_roundtrip_uses_keystream_offset_64() {
        let client_secret = EphemeralSecret::random_from_rng(&mut TestRng(99));
        let client_public = PublicKey::from(&client_secret).to_bytes();
        let mut sec = Sec1::new();
        let response0 = exchange_command0(&mut sec, b"abcd1234", client_public);
        let shared = client_secret.diffie_hellman(&PublicKey::from(response0.device_pubkey));
        let sym_key = mix_pop(shared.to_bytes(), b"abcd1234");
        let verify = client_verify(&sym_key, &response0.device_random, &response0.device_pubkey);

        let mut verify_buf = heapless::Vec::new();
        verify_buf.extend_from_slice(&verify).unwrap();
        let cmd1 = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionCommand1,
                payload: Some(Sec1Payload_::Payload::Sc1(SessionCmd1 {
                    client_verify_data: verify_buf,
                })),
            })),
        };
        let mut raw = [0u8; 128];
        let n = proto::encode(&cmd1, &mut raw).unwrap();
        let mut rng = TestRng(3);
        let mut out = [0u8; 256];
        let len = sec
            .handshake(&raw[..n], &mut rng, b"abcd1234", &mut out)
            .unwrap();
        let decoded: SessionData = proto::decode(&out[..len]).unwrap();
        let SessionData_::Proto::Sec1(payload) = decoded.proto.unwrap() else {
            panic!("expected sec1");
        };
        let Some(Sec1Payload_::Payload::Sr1(resp)) = payload.payload else {
            panic!("expected session response 1");
        };
        assert_eq!(resp.status, Status::Success);
        let mut device_verify: [u8; 32] = resp.device_verify_data.as_slice().try_into().unwrap();
        let mut continued =
            ctr::Ctr128BE::<aes::Aes256>::new((&sym_key).into(), (&response0.device_random).into());
        continued.seek(32u64);
        continued.apply_keystream(&mut device_verify);
        assert_eq!(device_verify, client_public);
        assert!(sec.established());

        let plain = *b"prov-config-payload!!";
        let mut phone =
            ctr::Ctr128BE::<aes::Aes256>::new((&sym_key).into(), (&response0.device_random).into());
        phone.seek(64u64);
        let mut body = plain;
        phone.apply_keystream(&mut body);
        sec.decrypt(&mut body).unwrap();
        assert_eq!(body, plain);

        let mut at_zero = plain;
        let mut rewind =
            ctr::Ctr128BE::<aes::Aes256>::new((&sym_key).into(), (&response0.device_random).into());
        rewind.apply_keystream(&mut at_zero);
        let mut at_sixty_four = plain;
        let mut offset =
            ctr::Ctr128BE::<aes::Aes256>::new((&sym_key).into(), (&response0.device_random).into());
        offset.seek(64u64);
        offset.apply_keystream(&mut at_sixty_four);
        assert_ne!(at_zero, at_sixty_four);
    }

    #[test]
    fn wrong_pop_does_not_establish() {
        let client_secret = EphemeralSecret::random_from_rng(&mut TestRng(99));
        let client_public = PublicKey::from(&client_secret).to_bytes();
        let mut with_pop = Sec1::new();
        let mut without_pop = Sec1::new();
        let pop_response = exchange_command0(&mut with_pop, b"abcd1234", client_public);
        let empty_response = exchange_command0(&mut without_pop, b"", client_public);
        assert_eq!(pop_response.device_pubkey, empty_response.device_pubkey);
        assert_eq!(pop_response.device_random, empty_response.device_random);

        let shared = client_secret.diffie_hellman(&PublicKey::from(pop_response.device_pubkey));
        let with_secret = mix_pop(shared.to_bytes(), b"abcd1234");
        let without_secret = mix_pop(shared.to_bytes(), b"");
        assert_ne!(with_secret, without_secret);

        let verify = client_verify(
            &without_secret,
            &pop_response.device_random,
            &pop_response.device_pubkey,
        );
        let mut verify_buf = heapless::Vec::new();
        verify_buf.extend_from_slice(&verify).unwrap();
        let cmd1 = SessionData {
            sec_ver: SecSchemeVersion::SecScheme1,
            proto: Some(SessionData_::Proto::Sec1(Sec1Payload {
                msg: Sec1MsgType::SessionCommand1,
                payload: Some(Sec1Payload_::Payload::Sc1(SessionCmd1 {
                    client_verify_data: verify_buf,
                })),
            })),
        };
        let mut raw = [0u8; 128];
        let n = proto::encode(&cmd1, &mut raw).unwrap();
        let mut rng = TestRng(1);
        let mut out = [0u8; 256];
        let len = with_pop
            .handshake(&raw[..n], &mut rng, b"abcd1234", &mut out)
            .unwrap();
        let decoded: SessionData = proto::decode(&out[..len]).unwrap();
        let SessionData_::Proto::Sec1(payload) = decoded.proto.unwrap() else {
            panic!("expected sec1");
        };
        let Some(Sec1Payload_::Payload::Sr1(resp)) = payload.payload else {
            panic!("expected session response 1");
        };
        assert_eq!(resp.status, Status::CryptoError);
        assert!(resp.device_verify_data.is_empty());
        assert!(!with_pop.established());
        let mut buf = [1, 2, 3, 4];
        assert_eq!(
            with_pop.decrypt(&mut buf),
            Err(SessionError::NotEstablished)
        );
        assert_eq!(buf, [1, 2, 3, 4]);
    }
}
