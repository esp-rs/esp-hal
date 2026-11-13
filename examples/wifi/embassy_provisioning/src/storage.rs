//! Versioned credential blob in the `nvs` partition (not ESP-IDF NVS key-value format).

use esp_bootloader_esp_idf::partitions::{self, DataPartitionSubType, PartitionType};
use esp_hal::{Blocking, flash::Flash};
use esp_println::println;
use heapless::String;

const MAGIC: [u8; 4] = *b"PRV1";
const BLOB_LEN: usize = 4 + 1 + 32 + 1 + 64;

/// Load credentials from the first `nvs` data partition, if present.
pub fn load(flash: &mut Flash<'_, Blocking>) -> Option<(String<32>, String<64>)> {
    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(flash, &mut pt_mem).ok()?;
    let nvs = pt
        .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
        .ok()??;
    let mut region = nvs.as_flash_region(flash).ok()?;
    let mut buf = [0u8; BLOB_LEN];
    region.read(0, &mut buf).ok()?;
    if buf[..4] != MAGIC {
        return None;
    }
    let ssid_len = buf[4] as usize;
    let pass_len = buf[5 + 32] as usize;
    if ssid_len > 32 || pass_len > 64 {
        return None;
    }
    let mut ssid = String::new();
    let mut pass = String::new();
    let _ = ssid.push_str(core::str::from_utf8(&buf[5..5 + ssid_len]).ok()?);
    let _ = pass.push_str(core::str::from_utf8(&buf[5 + 32 + 1..5 + 32 + 1 + pass_len]).ok()?);
    Some((ssid, pass))
}

/// Persist credentials into the `nvs` partition (erases the first sector first).
pub fn save(flash: &mut Flash<'_, Blocking>, ssid: &str, pass: &str) -> Result<(), ()> {
    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(flash, &mut pt_mem).map_err(|_| ())?;
    let nvs = pt
        .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
        .map_err(|_| ())?
        .ok_or(())?;
    let mut region = nvs.as_flash_region(flash).map_err(|_| ())?;

    let mut buf = [0u8; BLOB_LEN];
    buf[..4].copy_from_slice(&MAGIC);
    let ssid_len = ssid.len().min(32);
    let pass_len = pass.len().min(64);
    buf[4] = ssid_len as u8;
    buf[5..5 + ssid_len].copy_from_slice(ssid.as_bytes());
    buf[5 + 32] = pass_len as u8;
    buf[5 + 32 + 1..5 + 32 + 1 + pass_len].copy_from_slice(pass.as_bytes());

    region.erase(0, 4096).map_err(|_| ())?;
    region.write(0, &buf).map_err(|_| ())?;
    println!("Credentials saved to nvs partition");
    Ok(())
}
