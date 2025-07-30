//! AES DMA Test

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    aes::{Aes, Key, Operation, dma::CipherMode},
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
};
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

const fn pad_to<const K: usize>(input: &[u8]) -> [u8; K] {
    let mut out = [0; K];

    let in_bytes = input.len();
    assert!(in_bytes <= K);

    let mut i = 0;
    while i < in_bytes {
        out[i] = input[i];
        i += 1;
    }

    out
}

const DMA_BUFFER_SIZE: usize = 16;

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::aes::dma::AesDma;

    use super::*;

    const KEY: &[u8] = b"SUp4SeCp@sSw0rd";

    const PLAINTEXT: &[u8] = b"message";
    const PLAINTEXT_BUF: [u8; 16] = pad_to::<16>(PLAINTEXT);

    const CIPHERTEXT_ECB_128: [u8; 16] = [
        0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a, 0x31,
        0x96,
    ];
    const CIPHERTEXT_ECB_256: [u8; 16] = [
        0x0, 0x63, 0x3f, 0x02, 0xa4, 0x53, 0x09, 0x72, 0x20, 0x6d, 0xc9, 0x08, 0x7c, 0xe5, 0xfd,
        0xc,
    ];

    #[test]
    fn test_aes_dma_ecb() {
        fn test_aes_ecb<const K: usize>(
            mut aes: AesDma<'_>,
            plaintext: [u8; 16],
            ciphertext: [u8; 16],
        ) -> AesDma<'_>
        where
            Key: From<[u8; K]>,
        {
            let (output, rx_descriptors, input, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
            let mut output = DmaRxBuf::new(rx_descriptors, output).unwrap();
            let mut input = DmaTxBuf::new(tx_descriptors, input).unwrap();

            // Encrypt
            input.as_mut_slice().copy_from_slice(&plaintext);
            let transfer = aes
                .process(
                    1,
                    output,
                    input,
                    Operation::Encrypt,
                    CipherMode::Ecb,
                    pad_to::<K>(KEY),
                )
                .map_err(|e| e.0)
                .unwrap();
            (aes, output, input) = transfer.wait();
            assert_eq!(output.as_slice(), ciphertext);

            // Decrypt
            input.as_mut_slice().copy_from_slice(&ciphertext);
            let transfer = aes
                .process(
                    1,
                    output,
                    input,
                    Operation::Decrypt,
                    CipherMode::Ecb,
                    pad_to::<K>(KEY),
                )
                .map_err(|e| e.0)
                .unwrap();
            (aes, output, _) = transfer.wait();
            assert_eq!(output.as_slice(), plaintext);

            aes
        }
        let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                let dma_channel = peripherals.DMA_CRYPTO;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let aes = Aes::new(peripherals.AES).with_dma(dma_channel);

        let aes = test_aes_ecb::<16>(aes, PLAINTEXT_BUF, CIPHERTEXT_ECB_128);
        let _ = test_aes_ecb::<32>(aes, PLAINTEXT_BUF, CIPHERTEXT_ECB_256);
    }
}
