//! AES DMA Test

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    aes::{
        dma::{CipherMode, WithDmaAes},
        Aes,
        Mode,
    },
    dma::{Dma, DmaPriority},
    dma_buffers,
    peripherals::Peripherals,
};
use hil_test as _;

const DMA_BUFFER_SIZE: usize = 16;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn test_aes_128_dma_encryption() {
        let peripherals = Peripherals::take();

        let dma = Dma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        let (input, tx_descriptors, mut output, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);

        let mut aes = Aes::new(peripherals.AES).with_dma(
            dma_channel.configure(false, DmaPriority::Priority0),
            tx_descriptors,
            rx_descriptors,
        );

        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let plaintext = "message".as_bytes();
        input[..plaintext.len()].copy_from_slice(plaintext);

        let encrypted_message = [
            0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a,
            0x31, 0x96,
        ];

        let transfer = aes
            .process(
                &input,
                &mut output,
                Mode::Encryption128,
                CipherMode::Ecb,
                keybuf,
            )
            .unwrap();
        transfer.wait().unwrap();

        let mut encrypted_output = [0u8; 16];
        (&mut encrypted_output[..]).copy_from_slice(output);

        assert_eq!(encrypted_output, encrypted_message);
    }

    #[test]
    fn test_aes_128_dma_decryption() {
        let peripherals = Peripherals::take();

        let dma = Dma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        let (input, tx_descriptors, mut output, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);

        let mut aes = Aes::new(peripherals.AES).with_dma(
            dma_channel.configure(false, DmaPriority::Priority0),
            tx_descriptors,
            rx_descriptors,
        );

        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let plaintext = "message".as_bytes();
        let encrypted_message = [
            0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a,
            0x31, 0x96,
        ];
        input.copy_from_slice(&encrypted_message);

        let transfer = aes
            .process(
                &input,
                &mut output,
                Mode::Decryption128,
                CipherMode::Ecb,
                keybuf,
            )
            .unwrap();
        transfer.wait().unwrap();

        let mut decrypted_output = [0u8; 16];
        (&mut decrypted_output[..]).copy_from_slice(output);

        assert_eq!(&decrypted_output[..plaintext.len()], plaintext);
    }

    #[test]
    fn test_aes_256_dma_encryption() {
        let peripherals = Peripherals::take();

        let dma = Dma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        let (input, tx_descriptors, mut output, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);

        let mut aes = Aes::new(peripherals.AES).with_dma(
            dma_channel.configure(false, DmaPriority::Priority0),
            tx_descriptors,
            rx_descriptors,
        );

        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let plaintext = "message".as_bytes();
        input[..plaintext.len()].copy_from_slice(plaintext);

        let encrypted_message = [
            0x0, 0x63, 0x3f, 0x2, 0xa4, 0x53, 0x9, 0x72, 0x20, 0x6d, 0xc9, 0x8, 0x7c, 0xe5, 0xfd,
            0xc,
        ];

        let transfer = aes
            .process(
                &input,
                &mut output,
                Mode::Encryption256,
                CipherMode::Ecb,
                keybuf,
            )
            .unwrap();
        transfer.wait().unwrap();

        let mut encrypted_output = [0u8; 16];
        (&mut encrypted_output[..]).copy_from_slice(output);

        assert_eq!(encrypted_output, encrypted_message);
    }

    #[test]
    fn test_aes_256_dma_decryption() {
        let peripherals = Peripherals::take();

        let dma = Dma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        let (input, tx_descriptors, mut output, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);

        let mut aes = Aes::new(peripherals.AES).with_dma(
            dma_channel.configure(false, DmaPriority::Priority0),
            tx_descriptors,
            rx_descriptors,
        );

        let keytext = "SUp4SeCp@sSw0rd".as_bytes();
        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let plaintext = "message".as_bytes();
        let encrypted_message = [
            0x0, 0x63, 0x3f, 0x2, 0xa4, 0x53, 0x9, 0x72, 0x20, 0x6d, 0xc9, 0x8, 0x7c, 0xe5, 0xfd,
            0xc,
        ];
        input.copy_from_slice(&encrypted_message);

        let transfer = aes
            .process(
                &input,
                &mut output,
                Mode::Decryption256,
                CipherMode::Ecb,
                keybuf,
            )
            .unwrap();
        transfer.wait().unwrap();

        let mut decrypted_output = [0u8; 16];
        (&mut decrypted_output[..]).copy_from_slice(output);

        assert_eq!(&decrypted_output[..plaintext.len()], plaintext);
    }
}
