//! AES Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{
    aes::{Aes, Mode},
    peripherals::Peripherals,
};

struct Context<'a> {
    aes: Aes<'a>,
}

impl Context<'_> {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let aes = Aes::new(peripherals.AES);

        Context { aes }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context<'static> {
        Context::init()
    }

    #[test]
    fn test_aes_128_encryption(mut ctx: Context<'static>) {
        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let plaintext = "message".as_bytes();
        let encrypted_message = [
            0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a,
            0x31, 0x96,
        ];

        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let mut block_buf = [0_u8; 16];
        block_buf[..plaintext.len()].copy_from_slice(plaintext);

        let mut block = block_buf.clone();
        ctx.aes.process(&mut block, Mode::Encryption128, keybuf);
        assert_eq!(block, encrypted_message);
    }

    #[test]
    fn test_aes_128_decryption(mut ctx: Context<'static>) {
        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let plaintext = "message".as_bytes();
        let mut encrypted_message = [
            0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a,
            0x31, 0x96,
        ];

        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        ctx.aes
            .process(&mut encrypted_message, Mode::Decryption128, keybuf);
        assert_eq!(&encrypted_message[..plaintext.len()], plaintext);
    }

    #[test]
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    fn test_aes_192_encryption(mut ctx: Context<'static>) {
        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let plaintext = "message".as_bytes();
        let encrypted_message = [
            0x79, 0x88, 0x3f, 0x9d, 0x67, 0x27, 0xf4, 0x18, 0x3, 0xe3, 0xc6, 0x6a, 0x2e, 0x76,
            0xb6, 0xf7,
        ];

        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let mut block_buf = [0_u8; 16];
        block_buf[..plaintext.len()].copy_from_slice(plaintext);

        let mut block = block_buf.clone();
        ctx.aes.process(&mut block, Mode::Encryption192, keybuf);
        assert_eq!(block, encrypted_message);
    }

    #[test]
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    fn test_aes_192_decryption(mut ctx: Context<'static>) {
        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let plaintext = "message".as_bytes();
        let mut encrypted_message = [
            0x79, 0x88, 0x3f, 0x9d, 0x67, 0x27, 0xf4, 0x18, 0x3, 0xe3, 0xc6, 0x6a, 0x2e, 0x76,
            0xb6, 0xf7,
        ];

        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        ctx.aes
            .process(&mut encrypted_message, Mode::Decryption192, keybuf);
        assert_eq!(&encrypted_message[..plaintext.len()], plaintext);
    }

    #[test]
    fn test_aes_256_encryption(mut ctx: Context<'static>) {
        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let plaintext = "message".as_bytes();
        let encrypted_message = [
            0x0, 0x63, 0x3f, 0x2, 0xa4, 0x53, 0x9, 0x72, 0x20, 0x6d, 0xc9, 0x8, 0x7c, 0xe5, 0xfd,
            0xc,
        ];

        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let mut block_buf = [0_u8; 16];
        block_buf[..plaintext.len()].copy_from_slice(plaintext);

        let mut block = block_buf.clone();
        ctx.aes.process(&mut block, Mode::Encryption256, keybuf);
        assert_eq!(block, encrypted_message);
    }

    #[test]
    fn test_aes_256_decryption(mut ctx: Context<'static>) {
        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let plaintext = "message".as_bytes();
        let mut encrypted_message = [
            0x0, 0x63, 0x3f, 0x2, 0xa4, 0x53, 0x9, 0x72, 0x20, 0x6d, 0xc9, 0x8, 0x7c, 0xe5, 0xfd,
            0xc,
        ];

        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        ctx.aes
            .process(&mut encrypted_message, Mode::Decryption256, keybuf);
        assert_eq!(&encrypted_message[..plaintext.len()], plaintext);
    }
}
