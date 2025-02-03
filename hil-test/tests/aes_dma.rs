//! AES DMA Test

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    aes::{dma::CipherMode, Aes, Mode},
    dma_buffers,
    peripherals::Peripherals,
};
use hil_test as _;

const DMA_BUFFER_SIZE: usize = 16;

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::dma::{DmaRxBuf, DmaTxBuf};

    use super::*;

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn test_aes_128_dma_encryption(peripherals: Peripherals) {
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                let dma_channel = peripherals.DMA_CRYPTO;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let (output, rx_descriptors, input, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut output = DmaRxBuf::new(rx_descriptors, output).unwrap();
        let mut input = DmaTxBuf::new(tx_descriptors, input).unwrap();

        let aes = Aes::new(peripherals.AES).with_dma(dma_channel);

        let keytext = b"SUp4SeCp@sSw0rd";
        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let plaintext = b"message";
        input.as_mut_slice()[..plaintext.len()].copy_from_slice(plaintext);

        let encrypted_message = [
            0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a,
            0x31, 0x96,
        ];

        let transfer = aes
            .process(
                1,
                output,
                input,
                Mode::Encryption128,
                CipherMode::Ecb,
                keybuf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (_, output, _) = transfer.wait();

        let mut encrypted_output = [0u8; 16];
        (&mut encrypted_output[..]).copy_from_slice(output.as_slice());

        assert_eq!(encrypted_output, encrypted_message);
    }

    #[test]
    fn test_aes_128_dma_decryption(peripherals: Peripherals) {
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                let dma_channel = peripherals.DMA_CRYPTO;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let (output, rx_descriptors, input, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut output = DmaRxBuf::new(rx_descriptors, output).unwrap();
        let mut input = DmaTxBuf::new(tx_descriptors, input).unwrap();

        let aes = Aes::new(peripherals.AES).with_dma(dma_channel);

        let keytext = b"SUp4SeCp@sSw0rd";
        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let plaintext = b"message";
        let encrypted_message = [
            0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a,
            0x31, 0x96,
        ];
        input.as_mut_slice().copy_from_slice(&encrypted_message);

        let transfer = aes
            .process(
                1,
                output,
                input,
                Mode::Decryption128,
                CipherMode::Ecb,
                keybuf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (_, output, _) = transfer.wait();

        let mut decrypted_output = [0u8; 16];
        (&mut decrypted_output[..]).copy_from_slice(output.as_slice());

        assert_eq!(&decrypted_output[..plaintext.len()], plaintext);
    }

    #[test]
    fn test_aes_256_dma_encryption(peripherals: Peripherals) {
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                let dma_channel = peripherals.DMA_CRYPTO;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let (output, rx_descriptors, input, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut output = DmaRxBuf::new(rx_descriptors, output).unwrap();
        let mut input = DmaTxBuf::new(tx_descriptors, input).unwrap();

        let aes = Aes::new(peripherals.AES).with_dma(dma_channel);

        let keytext = b"SUp4SeCp@sSw0rd";
        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let plaintext = b"message";
        input.as_mut_slice()[..plaintext.len()].copy_from_slice(plaintext);

        let encrypted_message = [
            0x0, 0x63, 0x3f, 0x2, 0xa4, 0x53, 0x9, 0x72, 0x20, 0x6d, 0xc9, 0x8, 0x7c, 0xe5, 0xfd,
            0xc,
        ];

        let transfer = aes
            .process(
                1,
                output,
                input,
                Mode::Encryption256,
                CipherMode::Ecb,
                keybuf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (_, output, _) = transfer.wait();

        let mut encrypted_output = [0u8; 16];
        (&mut encrypted_output[..]).copy_from_slice(output.as_slice());

        assert_eq!(encrypted_output, encrypted_message);
    }

    #[test]
    fn test_aes_256_dma_decryption(peripherals: Peripherals) {
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                let dma_channel = peripherals.DMA_CRYPTO;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let (output, rx_descriptors, input, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut output = DmaRxBuf::new(rx_descriptors, output).unwrap();
        let mut input = DmaTxBuf::new(tx_descriptors, input).unwrap();

        let aes = Aes::new(peripherals.AES).with_dma(dma_channel);

        let keytext = b"SUp4SeCp@sSw0rd";
        let mut keybuf = [0_u8; 16];
        keybuf[..keytext.len()].copy_from_slice(keytext);

        let plaintext = b"message";
        let encrypted_message = [
            0x0, 0x63, 0x3f, 0x2, 0xa4, 0x53, 0x9, 0x72, 0x20, 0x6d, 0xc9, 0x8, 0x7c, 0xe5, 0xfd,
            0xc,
        ];
        input.as_mut_slice().copy_from_slice(&encrypted_message);

        let transfer = aes
            .process(
                1,
                output,
                input,
                Mode::Decryption256,
                CipherMode::Ecb,
                keybuf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (_, output, _) = transfer.wait();

        let mut decrypted_output = [0u8; 16];
        (&mut decrypted_output[..]).copy_from_slice(output.as_slice());

        assert_eq!(&decrypted_output[..plaintext.len()], plaintext);
    }
}
