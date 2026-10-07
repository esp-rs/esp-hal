use super::{Error, FlashStorage, PartitionEntry, PartitionType};
use crate::flash::FlashAccess;

impl PartitionEntry {
    /// Calculates the SHA-256 digest of this partition.
    ///
    /// - App / bootloader with appended hash: return that digest after verifying it
    /// - App / bootloader without appended hash: hash the image (not the whole partition)
    /// - Other types: hash the entire partition
    ///
    /// For app images this is the **validation hash** (shown by
    /// `esptool.py image-info`), not the ELF file SHA-256 stored in
    /// [`crate::EspAppDesc`].
    pub fn sha256(&self, flash: &mut FlashStorage<'_>) -> Result<[u8; 32], Error> {
        if self.is_empty() {
            return Err(Error::InvalidArgument);
        }

        let address = self.offset();
        let encrypted = self.is_effectively_encrypted();
        let mut size = self.len();

        if matches!(
            self.partition_type(),
            PartitionType::App(_) | PartitionType::Bootloader(_)
        ) {
            let data = get_image_metadata(flash, address, size, encrypted)?;
            if data.hash_appended {
                let calc = sha256_flash_contents(
                    flash,
                    address,
                    data.image_len - PARTITION_HASH_LEN as u32,
                    encrypted,
                )?;
                if calc != data.image_digest {
                    return Err(Error::InvalidImage);
                }
                return Ok(data.image_digest);
            }
            size = data.image_len;
        }

        sha256_flash_contents(flash, address, size, encrypted)
    }
}

const PARTITION_HASH_LEN: usize = 32;
const IMAGE_HEADER_MAGIC: u8 = 0xE9;
const IMAGE_HEADER_LEN: u32 = 24;
const IMAGE_MAX_SEGMENTS: u8 = 16;
const IMAGE_MAX_FLASH_ADDR_SIZE: u32 = 16 * 1024 * 1024;

/// Subset of ESP-IDF `esp_image_metadata_t` for partition SHA-256 convenience.
struct ImageMetadata {
    image_len: u32,
    image_digest: [u8; PARTITION_HASH_LEN],
    hash_appended: bool,
}

/// Parses an app/bootloader image on flash and returns its length and optional
/// appended SHA-256 digest.
///
/// Walks the image header and segment table, accounts for the checksum
/// padding, and — if the image has a simple hash appended — reads that digest.
/// Does not verify the checksum or load any segments.
fn get_image_metadata<F: FlashAccess>(
    flash: &mut F,
    address: u32,
    part_size: u32,
    encrypted: bool,
) -> Result<ImageMetadata, Error> {
    if part_size == 0 || part_size > IMAGE_MAX_FLASH_ADDR_SIZE {
        return Err(Error::InvalidArgument);
    }

    // process_image_header()
    let mut hdr = [0u8; IMAGE_HEADER_LEN as usize];
    flash_read(flash, address, &mut hdr, encrypted)?;
    // `esp_image_get_metadata` skips header verify, but refuse obvious garbage.
    if hdr[0] != IMAGE_HEADER_MAGIC || hdr[1] > IMAGE_MAX_SEGMENTS {
        return Err(Error::InvalidImage);
    }

    let mut image_len = IMAGE_HEADER_LEN;

    // process_segments()
    for _ in 0..hdr[1] {
        let mut seg = [0u8; 8];
        flash_read(flash, address + image_len, &mut seg, encrypted)?;
        // seg[0..4] - load address
        let data_len = u32::from_le_bytes(unwrap!(seg[4..8].try_into()));
        if data_len % 4 != 0 || data_len >= IMAGE_MAX_FLASH_ADDR_SIZE {
            return Err(Error::InvalidImage);
        }
        image_len = image_len
            .checked_add(8 + data_len)
            .ok_or(Error::InvalidImage)?;
    }

    // process_checksum()
    // add a byte for the checksum, pad to next full 16 byte block
    image_len = (image_len + 1 + 15) & !15;

    // process_appended_hash_and_sig()
    let hash_appended = hdr[23] != 0;
    let mut image_digest = [0u8; PARTITION_HASH_LEN];
    if hash_appended {
        flash_read(flash, address + image_len, &mut image_digest, encrypted)?;
        image_len += PARTITION_HASH_LEN as u32;
    }

    if image_len > part_size {
        return Err(Error::InvalidImage);
    }

    Ok(ImageMetadata {
        image_len,
        image_digest,
        hash_appended,
    })
}

fn flash_read<F: FlashAccess>(
    flash: &mut F,
    address: u32,
    bytes: &mut [u8],
    encrypted: bool,
) -> Result<(), Error> {
    if encrypted {
        flash.flash_read_encrypted(address, bytes)
    } else {
        flash.flash_read(address, bytes)
    }
}

/// Hashes `len` bytes of flash starting at `flash_offset`.
///
/// Reads the region in fixed-size chunks so large partitions do not need to be
/// loaded into memory at once.
fn sha256_flash_contents<F: FlashAccess>(
    flash: &mut F,
    mut flash_offset: u32,
    mut len: u32,
    encrypted: bool,
) -> Result<[u8; PARTITION_HASH_LEN], Error> {
    use sha2::{Digest, Sha256};

    let mut hasher = Sha256::new();
    let mut chunk = [0u8; 4096];

    while len > 0 {
        let n = len.min(chunk.len() as u32) as usize;
        flash_read(flash, flash_offset, &mut chunk[..n], encrypted)?;
        hasher.update(&chunk[..n]);
        flash_offset += n as u32;
        len -= n as u32;
    }

    Ok(hasher.finalize().into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partitions::{
        AppPartitionSubType,
        DataPartitionSubType,
        PARTITION_TABLE_MAX_LEN,
        PARTITION_TABLE_OFFSET,
        read_partition_table,
    };

    /// SHA-256 of `0x6000` bytes filled with `0xA5`.
    const NVS_DIGEST: [u8; 32] = [
        0xb0, 0x5b, 0x4f, 0x2c, 0xc2, 0xa7, 0x54, 0x25, 0x54, 0xfa, 0x32, 0x8b, 0xd0, 0x5d, 0x86,
        0x7f, 0x0c, 0x1d, 0xae, 0xed, 0x46, 0x48, 0x8e, 0x31, 0xb0, 0x0c, 0xb0, 0xaa, 0xe5, 0xb5,
        0x49, 0x81,
    ];

    /// SHA-256 of the minimal test image body (32 bytes) with `hash_appended = 1`.
    const IMAGE_DIGEST_WITH_HASH_FLAG: [u8; 32] = [
        0xb2, 0xb7, 0x64, 0x4a, 0x57, 0x62, 0x46, 0x05, 0xf7, 0xe4, 0xb1, 0xc3, 0xbf, 0x96, 0x5a,
        0x20, 0x87, 0x37, 0x3d, 0x7a, 0xc6, 0x2d, 0xf8, 0x6a, 0xcf, 0x2b, 0x1a, 0xcf, 0xe4, 0x8e,
        0xe8, 0xa0,
    ];

    /// SHA-256 of the same minimal image body with `hash_appended = 0`.
    const IMAGE_DIGEST_WITHOUT_HASH_FLAG: [u8; 32] = [
        0x02, 0x50, 0xbb, 0x56, 0xe1, 0x91, 0xf6, 0x6d, 0xde, 0xf1, 0x5e, 0x2d, 0x7c, 0xb4, 0x48,
        0x23, 0x75, 0x36, 0x52, 0x54, 0x7f, 0xc3, 0xd9, 0xd5, 0x83, 0xaa, 0xca, 0x2e, 0xec, 0xfe,
        0x99, 0x00,
    ];

    fn test_flash() -> FlashStorage<'static> {
        let mut flash = FlashStorage::new();
        let mut data = [0xffu8; 0x10000];
        data[PARTITION_TABLE_OFFSET as usize..][..PARTITION_TABLE_MAX_LEN]
            .copy_from_slice(include_bytes!("../../testdata/single_factory_no_ota.bin"));
        flash.write(0, &data).unwrap();
        flash
    }

    /// Header-only ESP image (0 segments); body pads to 32 bytes for the checksum.
    fn write_minimal_app_image(
        flash: &mut FlashStorage<'static>,
        offset: u32,
        hash_appended: bool,
    ) {
        let mut image = [0u8; 64];
        image[0] = IMAGE_HEADER_MAGIC;
        image[23] = u8::from(hash_appended);
        // image[1] = 0 segments; bytes 24..32 are checksum padding
        if hash_appended {
            image[32..64].copy_from_slice(&IMAGE_DIGEST_WITH_HASH_FLAG);
            flash.write(offset, &image).unwrap();
        } else {
            flash.write(offset, &image[..32]).unwrap();
        }
    }

    #[test]
    fn sha256_of_data_partition_matches_known_digest() {
        let mut flash = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut flash, &mut buffer).unwrap();
        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();

        nvs.as_flash_region(&mut flash)
            .unwrap()
            .write(0, &[0xa5u8; 0x6000])
            .unwrap();

        assert_eq!(nvs.sha256(&mut flash).unwrap(), NVS_DIGEST);
    }

    #[test]
    fn sha256_of_app_with_appended_hash_returns_validation_digest() {
        let mut flash = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut flash, &mut buffer).unwrap();
        let factory = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Factory))
            .unwrap()
            .unwrap();

        write_minimal_app_image(&mut flash, factory.offset(), true);

        assert_eq!(
            factory.sha256(&mut flash).unwrap(),
            IMAGE_DIGEST_WITH_HASH_FLAG
        );
    }

    #[test]
    fn sha256_of_app_without_appended_hash_hashes_image() {
        let mut flash = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut flash, &mut buffer).unwrap();
        let factory = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Factory))
            .unwrap()
            .unwrap();

        write_minimal_app_image(&mut flash, factory.offset(), false);

        assert_eq!(
            factory.sha256(&mut flash).unwrap(),
            IMAGE_DIGEST_WITHOUT_HASH_FLAG
        );
    }

    #[test]
    fn sha256_rejects_corrupt_appended_hash() {
        let mut flash = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut flash, &mut buffer).unwrap();
        let factory = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Factory))
            .unwrap()
            .unwrap();

        write_minimal_app_image(&mut flash, factory.offset(), true);

        // Corrupt the appended digest
        flash.write(factory.offset() + 32, &[0u8; 32]).unwrap();

        assert_eq!(factory.sha256(&mut flash), Err(Error::InvalidImage));
    }
}
