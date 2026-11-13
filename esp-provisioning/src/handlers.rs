//! Shared sec1 session and encrypted prov-config / prov-scan handling.

use heapless::String;
use rand_core::{CryptoRng, RngCore};

use crate::{
    SESSION_RESPONSE_MIN_LEN,
    SESSION_RESPONSE_SEC0,
    cached_aps,
    protocomm,
    sec1,
    sec1::SessionResponse,
};

/// Per-connection provisioning protocol state (sec1 + pending Wi-Fi credentials).
pub struct ProvCommState {
    /// Security 1 handshake state.
    pub sec1_state: sec1::Sec1State,
    /// AES-CTR cipher after handshake completes.
    pub post_cipher: Option<sec1::PostHandshakeCipher>,
    /// SSID from the last `SetConfig` command.
    pub pending_ssid: String<32>,
    /// Passphrase from the last `SetConfig` command.
    pub pending_pass: String<64>,
    /// Whether `ApplyConfig` was received.
    pub config_applied: bool,
}

impl ProvCommState {
    /// Creates fresh provisioning state for a new client session.
    pub fn new() -> Self {
        Self {
            sec1_state: sec1::Sec1State::new(),
            post_cipher: None,
            pending_ssid: String::new(),
            pending_pass: String::new(),
            config_applied: false,
        }
    }

    /// Handles a prov-session request; returns the sec1 response bytes to send.
    pub fn handle_session<'buf>(
        &mut self,
        body: &[u8],
        pop: &[u8],
        rng: &mut (impl RngCore + CryptoRng),
        response_buf: &'buf mut [u8],
        log_prefix: &str,
    ) -> SessionWireResponse<'buf> {
        let n = body.len().min(response_buf.len());
        match sec1::handle_session(&body[..n], &mut self.sec1_state, rng, pop, response_buf) {
            SessionResponse::Sec1(resp) => {
                if let Some(c) = self.sec1_state.post_handshake_cipher() {
                    self.post_cipher = Some(c);
                    info!("{log_prefix} sec1 handshake complete");
                }
                SessionWireResponse::Sec1(resp)
            }
            SessionResponse::Sec0 => {
                warn!("{log_prefix} session parse failed — Sec0 fallback");
                SessionWireResponse::Sec0
            }
            SessionResponse::OwnershipFailed => SessionWireResponse::Sec0,
            SessionResponse::BufferTooSmall => {
                warn!(
                    "{log_prefix} session response buffer too small (need {SESSION_RESPONSE_MIN_LEN} bytes)"
                );
                SessionWireResponse::Sec0
            }
        }
    }

    /// Decrypts an encrypted request body in place when a post-handshake cipher is active.
    pub fn decrypt_body(&mut self, buf: &mut [u8], len: usize) {
        if let Some(c) = self.post_cipher.as_mut() {
            c.apply(&mut buf[..len]);
        }
    }

    /// Encrypts a response body in place when a post-handshake cipher is active.
    pub fn encrypt_response(&mut self, buf: &mut [u8], len: usize) {
        if let Some(c) = self.post_cipher.as_mut() {
            c.apply(&mut buf[..len]);
        }
    }

    /// Handles a decrypted prov-config payload; returns `true` when provisioning is complete.
    pub fn handle_config(
        &mut self,
        plain: &[u8],
        resp_buf: &mut [u8],
        log_prefix: &str,
    ) -> (bool, usize) {
        let (result, mut resp_len) = protocomm::handle_prov_config_write(
            plain,
            &mut self.pending_ssid,
            &mut self.pending_pass,
            resp_buf,
        );

        use protocomm::ProvConfigResult;
        let done = match result {
            ProvConfigResult::ApplyConfig => {
                info!(
                    "{log_prefix} ApplyConfig ssid='{}' pass_len={}",
                    self.pending_ssid.as_str(),
                    self.pending_pass.len()
                );
                self.config_applied = true;
                false
            }
            ProvConfigResult::GetStatus if self.config_applied => {
                info!("{log_prefix} GetStatus → connected");
                resp_len = protocomm::encode_resp_get_status(resp_buf, true);
                true
            }
            ProvConfigResult::GetStatus => {
                warn!("{log_prefix} GetStatus before ApplyConfig");
                false
            }
            _ => false,
        };

        (done, resp_len)
    }

    /// Handles a decrypted prov-scan payload.
    pub fn handle_scan(&mut self, plain: &[u8], resp_buf: &mut [u8]) -> usize {
        let (aps, ap_count) = cached_aps();
        let (_r, sr_len) = protocomm::handle_prov_scan_write(plain, resp_buf, aps, ap_count);
        sr_len
    }
}

/// Bytes to return on the prov-session endpoint.
pub enum SessionWireResponse<'a> {
    /// Sec1 response from the handshake handler.
    Sec1(&'a [u8]),
    /// Fall back to sec0 (no encryption).
    Sec0,
}

impl SessionWireResponse<'_> {
    /// Resolves to the wire bytes for this response.
    pub fn as_bytes(&self) -> &[u8] {
        match self {
            Self::Sec1(b) => b,
            Self::Sec0 => &SESSION_RESPONSE_SEC0,
        }
    }
}
