//! Test we place the app descriptor at the right position in the image and we
//! can read it

//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6 esp32h2
//% FEATURES: unstable esp-storage

#![no_std]
#![no_main]

use embedded_storage::ReadStorage;
use esp_storage::FlashStorage;
use hil_test as _;
#[repr(C)]
pub struct EspAppDesc {
    pub magic_word: u32,                       // Magic word ESP_APP_DESC_MAGIC_WORD
    pub secure_version: u32,                   // Secure version
    pub reserv1: [u32; 2],                     // reserv1
    pub version: [core::ffi::c_char; 32],      // Application version
    pub project_name: [core::ffi::c_char; 32], // Project name
    pub time: [core::ffi::c_char; 16],         // Compile time
    pub date: [core::ffi::c_char; 16],         // Compile date
    pub idf_ver: [core::ffi::c_char; 32],      // Version IDF
    pub app_elf_sha256: [u8; 32],              // sha256 of elf file
    pub min_efuse_blk_rev_full: u16,           /* Minimal eFuse block revision supported by
                                                * image, in format: major * 100 + minor */
    pub max_efuse_blk_rev_full: u16, /* Maximal eFuse block revision supported by image, in
                                      * format: major * 100 + minor */
    pub mmu_page_size: u8,  // MMU page size in log base 2 format
    pub reserv3: [u8; 3],   // reserv3
    pub reserv2: [u32; 18], // reserv2
}

const ESP_APP_DESC_MAGIC_WORD: u32 = 0xABCD5432;

const fn str_to_cstr_array<const C: usize>(s: &str) -> [::core::ffi::c_char; C] {
    let bytes = s.as_bytes();
    if bytes.len() >= C {
        assert!(true, "String is too long for the C-string field");
    }

    let mut ret: [::core::ffi::c_char; C] = [0; C];
    let mut i = 0;
    loop {
        ret[i] = bytes[i] as _;
        i += 1;
        if i >= bytes.len() {
            break;
        }
    }
    ret
}

#[unsafe(no_mangle)]
#[used]
#[unsafe(link_section = ".rodata_desc")]
#[allow(non_upper_case_globals)]
pub static esp_app_desc: EspAppDesc = EspAppDesc {
    magic_word: ESP_APP_DESC_MAGIC_WORD,
    secure_version: 0,
    reserv1: [0; 2],
    version: str_to_cstr_array(env!("CARGO_PKG_VERSION")),
    project_name: str_to_cstr_array(env!("CARGO_PKG_NAME")),
    time: str_to_cstr_array("15:15:15"),
    date: str_to_cstr_array("2025-03-05"),
    // just pretending some esp-idf version here
    idf_ver: str_to_cstr_array("5.3.1"),
    app_elf_sha256: [0; 32],
    min_efuse_blk_rev_full: 0,
    max_efuse_blk_rev_full: u16::MAX,
    mmu_page_size: 0,
    reserv3: [0; 3],
    reserv2: [0; 18],
};

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[test]
    fn test_can_read_app_desc() {
        let _ = esp_hal::init(esp_hal::Config::default());

        let mut bytes = [0u8; 256];

        let mut flash = FlashStorage::new();

        // esp-idf 2nd stage bootloader would expect the app-descriptor at the start of
        // DROM it also expects DROM segment to the the first page of the
        // app-image and we need to account for the image header - so we end up
        // with flash-address 0x10_000 + 0x20
        flash.read(0x10_020, &mut bytes).unwrap();

        assert_eq!(&bytes, unsafe {
            core::mem::transmute::<&EspAppDesc, &[u8; 256]>(&esp_app_desc)
        });
    }
}
