//! AES Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

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

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        Config,
        aes::{Aes, AesBackend, AesContext, Key, Operation},
        clock::CpuClock,
    };
    use hil_test as _;

    use super::*;

    const KEY: &[u8] = b"SUp4SeCp@sSw0rd";
    const KEY_128: [u8; 16] = pad_to::<16>(KEY);
    const KEY_256: [u8; 32] = pad_to::<32>(KEY);

    const PLAINTEXT: &[u8] = b"message";
    const PLAINTEXT_BUF: [u8; 16] = pad_to::<16>(PLAINTEXT);

    const CIPHERTEXT_ECB_128: [u8; 16] = [
        0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a, 0x31,
        0x96,
    ];
    #[cfg(any(esp32, esp32s2))]
    const CIPHERTEXT_ECB_192: [u8; 16] = [
        0x79, 0x88, 0x3f, 0x9d, 0x67, 0x27, 0xf4, 0x18, 0x3, 0xe3, 0xc6, 0x6a, 0x2e, 0x76, 0xb6,
        0xf7,
    ];
    const CIPHERTEXT_ECB_256: [u8; 16] = [
        0x0, 0x63, 0x3f, 0x02, 0xa4, 0x53, 0x09, 0x72, 0x20, 0x6d, 0xc9, 0x08, 0x7c, 0xe5, 0xfd,
        0xc,
    ];

    #[test]
    fn test_aes_block_operation() {
        fn test_aes_block<const K: usize>(
            aes: &mut Aes<'_>,
            plaintext: [u8; 16],
            ciphertext: [u8; 16],
        ) where
            Key: From<[u8; K]>,
        {
            let mut block_buf = plaintext;
            aes.encrypt(&mut block_buf, pad_to::<K>(KEY));
            assert_eq!(&block_buf[..ciphertext.len()], ciphertext);

            let mut block_buf = ciphertext;
            aes.decrypt(&mut block_buf, pad_to::<K>(KEY));
            assert_eq!(&block_buf[..plaintext.len()], plaintext);
        }

        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));
        let mut aes = Aes::new(p.AES);

        test_aes_block::<16>(&mut aes, PLAINTEXT_BUF, CIPHERTEXT_ECB_128);

        #[cfg(any(esp32, esp32s2))]
        test_aes_block::<24>(&mut aes, PLAINTEXT_BUF, CIPHERTEXT_ECB_192);

        test_aes_block::<32>(&mut aes, PLAINTEXT_BUF, CIPHERTEXT_ECB_256);
    }

    #[test]
    #[cfg(not(esp32))]
    fn test_aes_dma_ecb() {
        use esp_hal::{
            aes::dma::{AesDma, CipherMode},
            dma::{DmaRxBuf, DmaTxBuf},
            dma_buffers,
        };

        fn test_aes_ecb<const K: usize>(
            mut aes: AesDma<'_>,
            plaintext: [u8; 16],
            ciphertext: [u8; 16],
        ) -> AesDma<'_>
        where
            Key: From<[u8; K]>,
        {
            const DMA_BUFFER_SIZE: usize = 16;

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
        let peripherals = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

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

    #[test]
    fn test_aes_work_queue() {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

        let mut aes = AesBackend::new(p.AES);
        let _backend = aes.start();

        let mut output = [0; 16];
        let mut ecb_encrypt = AesContext::ecb(Operation::Encrypt, KEY_128);
        let handle = ecb_encrypt.process(&PLAINTEXT_BUF, &mut output).unwrap();

        handle.wait_blocking();

        assert_eq!(output, CIPHERTEXT_ECB_128);
    }

    #[test]
    fn test_aes_work_queue_work_posted_before_queue_started() {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

        let mut output = [0; 16];
        let mut ecb_encrypt = AesContext::ecb(Operation::Encrypt, KEY_128);
        let handle = ecb_encrypt.process(&PLAINTEXT_BUF, &mut output).unwrap();

        let mut aes = AesBackend::new(p.AES);
        let _backend = aes.start();

        handle.wait_blocking();

        assert_eq!(output, CIPHERTEXT_ECB_128);
    }

    #[test]
    fn test_aes_work_queue_in_place() {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

        let mut buffer = PLAINTEXT_BUF;
        let mut ecb_encrypt = AesContext::ecb(Operation::Encrypt, KEY_128);
        let handle = ecb_encrypt.process_in_place(&mut buffer).unwrap();

        let mut aes = AesBackend::new(p.AES);
        let _backend = aes.start();

        handle.wait_blocking();

        assert_eq!(buffer, CIPHERTEXT_ECB_128);
    }

    #[test]
    fn test_aes_cancelling_work() {
        // In this test, we post two work items, and cancel the first one before starting the
        // backend. We will assert that, when the second item has finished correctly, the first did
        // not modify its output buffer.
        // Note that this result is not guaranteed. The cancellation can come later than the work
        // item has finished processing. We can only reliably test it because we start the backend
        // after cancelling the operation.
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

        let mut buffer1 = PLAINTEXT_BUF;
        let mut ecb_encrypt = AesContext::ecb(Operation::Encrypt, KEY_128);
        let handle1 = ecb_encrypt.process_in_place(&mut buffer1).unwrap();

        let mut buffer2 = PLAINTEXT_BUF;
        let mut ecb_encrypt = AesContext::ecb(Operation::Encrypt, KEY_256);
        let handle2 = ecb_encrypt.process_in_place(&mut buffer2).unwrap();

        let mut buffer3 = PLAINTEXT_BUF;
        let mut ecb_encrypt = AesContext::ecb(Operation::Encrypt, KEY_128);
        let handle3 = ecb_encrypt.process_in_place(&mut buffer3).unwrap();

        core::mem::drop(handle1);

        let mut aes = AesBackend::new(p.AES);
        let _backend = aes.start();

        handle3.wait_blocking();
        handle2.wait_blocking();

        assert_eq!(buffer1, PLAINTEXT_BUF);
        assert_eq!(buffer2, CIPHERTEXT_ECB_256);
        assert_eq!(buffer3, CIPHERTEXT_ECB_128);
    }
}
