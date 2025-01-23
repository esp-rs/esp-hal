//! A Rustls [CryptoProvider] and useful utilities

#![no_std]

extern crate alloc;

use alloc::sync::Arc;

use rand_core::RngCore;
use rustls::{crypto::CryptoProvider, pki_types::PrivateKeyDer};

mod aead;
mod hash;
mod hmac;
mod kx;
mod sign;
mod verify;

pub mod adapter;

// some re-exports to make the user's life easier
pub use rustls;
pub use webpki;
pub use webpki_roots;

/// Creates a ready-to-use Rustls [CryptoProvider]
pub fn provider() -> CryptoProvider {
    CryptoProvider {
        cipher_suites: ALL_CIPHER_SUITES.to_vec(),
        kx_groups: kx::ALL_KX_GROUPS.to_vec(),
        signature_verification_algorithms: verify::ALGORITHMS,
        secure_random: &Provider,
        key_provider: &Provider,
    }
}

/// Assume the RNG is actually producing true random numbers - which is the case
/// when radio peripherals are enabled
struct ProbablyTrng;

impl rand_core::RngCore for ProbablyTrng {
    fn next_u32(&mut self) -> u32 {
        esp_hal::rng::Rng::new(unsafe { esp_hal::peripherals::RNG::steal() }).next_u32()
    }

    fn next_u64(&mut self) -> u64 {
        esp_hal::rng::Rng::new(unsafe { esp_hal::peripherals::RNG::steal() }).next_u64()
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        esp_hal::rng::Rng::new(unsafe { esp_hal::peripherals::RNG::steal() }).fill_bytes(dest);
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        esp_hal::rng::Rng::new(unsafe { esp_hal::peripherals::RNG::steal() }).try_fill_bytes(dest)
    }
}

impl rand_core::CryptoRng for ProbablyTrng {}

/// A time provider to be used by Rustls
#[derive(Debug)]
pub struct EspTimeProvider {
    offset: u32,
}

impl EspTimeProvider {
    pub fn new(offset: u32) -> Self {
        Self { offset }
    }
}

impl rustls::time_provider::TimeProvider for EspTimeProvider {
    fn current_time(&self) -> Option<rustls::pki_types::UnixTime> {
        let now = esp_hal::time::now().duration_since_epoch().to_secs();
        Some(rustls::pki_types::UnixTime::since_unix_epoch(
            core::time::Duration::from_secs(self.offset as u64 + now),
        ))
    }
}

#[derive(Debug)]
struct Provider;

impl rustls::crypto::SecureRandom for Provider {
    fn fill(&self, bytes: &mut [u8]) -> Result<(), rustls::crypto::GetRandomFailed> {
        let mut rng = ProbablyTrng;
        rng.fill_bytes(bytes);
        Ok(())
    }
}

impl rustls::crypto::KeyProvider for Provider {
    fn load_private_key(
        &self,
        key_der: PrivateKeyDer<'static>,
    ) -> Result<Arc<dyn rustls::sign::SigningKey>, rustls::Error> {
        Ok(Arc::new(
            sign::EcdsaSigningKeyP256::try_from(key_der)
                .map_err(|err| rustls::Error::General(alloc::format!("{}", err)))?,
        ))
    }
}

static ALL_CIPHER_SUITES: &[rustls::SupportedCipherSuite] = &[
    TLS13_CHACHA20_POLY1305_SHA256,
    TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305_SHA256,
];

#[doc(hidden)]
pub static TLS13_CHACHA20_POLY1305_SHA256: rustls::SupportedCipherSuite =
    rustls::SupportedCipherSuite::Tls13(&rustls::Tls13CipherSuite {
        common: rustls::crypto::CipherSuiteCommon {
            suite: rustls::CipherSuite::TLS13_CHACHA20_POLY1305_SHA256,
            hash_provider: &hash::Sha256,
            confidentiality_limit: u64::MAX,
        },
        hkdf_provider: &rustls::crypto::tls13::HkdfUsingHmac(&hmac::Sha256Hmac),
        aead_alg: &aead::Chacha20Poly1305,
        quic: None,
    });

#[doc(hidden)]
pub static TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305_SHA256: rustls::SupportedCipherSuite =
    rustls::SupportedCipherSuite::Tls12(&rustls::Tls12CipherSuite {
        common: rustls::crypto::CipherSuiteCommon {
            suite: rustls::CipherSuite::TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305_SHA256,
            hash_provider: &hash::Sha256,
            confidentiality_limit: u64::MAX,
        },
        kx: rustls::crypto::KeyExchangeAlgorithm::ECDHE,
        sign: &[
            rustls::SignatureScheme::RSA_PSS_SHA256,
            rustls::SignatureScheme::RSA_PKCS1_SHA256,
        ],
        prf_provider: &rustls::crypto::tls12::PrfUsingHmac(&hmac::Sha256Hmac),
        aead_alg: &aead::Chacha20Poly1305,
    });
