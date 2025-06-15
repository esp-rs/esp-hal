//! CRC and MD5 Tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use core::ops::Deref;

use esp_hal::rom::{crc, md5};
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn test_crc() {
        let data = "123456789";

        let crc_hdlc = crc::crc32_le(!0xffffffff, data.as_ref());
        let crc_bzip2 = crc::crc32_be(!0xffffffff, data.as_ref());
        let crc_mpeg2 = !crc::crc32_be(!0xffffffff, data.as_ref());
        let crc_cksum = crc::crc32_be(!0, data.as_ref());
        let crc_kermit = !crc::crc16_le(!0, data.as_ref());
        let crc_genibus = crc::crc16_be(!0xffff, data.as_ref());
        let crc_rohc = !crc::crc8_le(!0xff, data.as_ref());
        let crc_smbus = !crc::crc8_be(!0, data.as_ref());

        assert_eq!(crc_hdlc, 0xcbf43926);
        assert_eq!(crc_bzip2, 0xfc891918);
        assert_eq!(crc_mpeg2, 0x0376e6e7);
        assert_eq!(crc_cksum, 0x765e7680);
        assert_eq!(crc_kermit, 0x2189);
        assert_eq!(crc_genibus, 0xd64e);
        assert_eq!(crc_rohc, 0xd0);
        assert_eq!(crc_smbus, 0xf4);
    }

    #[test]
    fn test_md5() {
        let sentence = "The quick brown fox jumps over a lazy dog";

        let mut md5_ctx = md5::Context::new();
        let mut it = sentence.split_whitespace().peekable();
        while let Some(word) = it.next() {
            md5_ctx.consume(word);
            if it.peek().is_some() {
                md5_ctx.consume(" ");
            }
        }
        let md5_digest = md5_ctx.compute();

        let expected_md5_digest = md5::Digest([
            0x30, 0xde, 0xd8, 0x07, 0xd6, 0x5e, 0xe0, 0x37, 0x0f, 0xc6, 0xd7, 0x3d, 0x6a, 0xb5,
            0x5a, 0x95,
        ])
        .deref();

        assert_eq!(expected_md5_digest, md5_digest.deref());
    }
}
