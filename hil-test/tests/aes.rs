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
        aes::{Aes, Key},
        clock::CpuClock,
    };
    use hil_test as _;

    use super::*;

    const KEY: &[u8] = b"SUp4SeCp@sSw0rd";

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
}
