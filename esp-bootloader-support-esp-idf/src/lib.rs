//! # Bootloader Support Library supplementing esp-hal
//!
//! ## Overview
//!
//! This crate contains functionality related to the ESP-IDF 2nd stage
//! bootloader.
//!
//! - populating the application-descriptor
//!
//! ## Additional configuration
//!
//! We've exposed some configuration options that don't fit into cargo
//! features. These can be set via environment variables, or via cargo's `[env]`
//! section inside `.cargo/config.toml`. Below is a table of tunable parameters
//! for this crate:
#![doc = ""]
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_bootloader_support_esp_idf_config_table.md"))]
#![doc = ""]
#![no_std]

/// ESP-IDF compatible application descriptor
///
/// This gets populated by the [esp_app_desc] macro.
#[repr(C)]
pub struct EspAppDesc {
    /// Magic word ESP_APP_DESC_MAGIC_WORD
    magic_word: u32,
    /// Secure version               
    secure_version: u32,
    /// Reserved
    reserv1: [u32; 2],
    /// Application version
    version: [core::ffi::c_char; 32],
    /// Project name
    project_name: [core::ffi::c_char; 32],
    /// Compile time
    time: [core::ffi::c_char; 16],
    /// Compile date
    date: [core::ffi::c_char; 16],
    /// Version IDF
    idf_ver: [core::ffi::c_char; 32],
    /// sha256 of elf file
    app_elf_sha256: [u8; 32],
    /// Minimal eFuse block revision supported by image, in format: major * 100
    /// + minor
    min_efuse_blk_rev_full: u16,
    /// Maximal eFuse block revision supported by image, in format: major * 100
    /// + minor
    max_efuse_blk_rev_full: u16,
    /// MMU page size in log base 2 format
    mmu_page_size: u8,
    /// Reserved
    reserv3: [u8; 3],
    /// Reserved
    reserv2: [u32; 18],
}

impl EspAppDesc {
    /// The magic word - should be `0xABCD5432`
    pub fn magic_word(&self) -> u32 {
        self.magic_word
    }

    /// Secure version
    pub fn secure_version(&self) -> u32 {
        self.secure_version
    }

    /// Application version
    pub fn version(&self) -> &str {
        array_to_str(&self.version)
    }

    /// Application name
    pub fn project_name(&self) -> &str {
        array_to_str(&self.project_name)
    }

    /// Compile time
    pub fn time(&self) -> &str {
        array_to_str(&self.time)
    }

    /// Compile data
    pub fn date(&self) -> &str {
        array_to_str(&self.date)
    }

    /// IDF version
    pub fn idf_ver(&self) -> &str {
        array_to_str(&self.idf_ver)
    }

    /// SHA256
    ///
    /// The default tooling won't populate this
    pub fn app_elf_sha256(&self) -> &[u8; 32] {
        &self.app_elf_sha256
    }

    /// Minimal eFuse block revision supported by image
    pub fn min_efuse_blk_rev_full(&self) -> u16 {
        self.min_efuse_blk_rev_full
    }

    /// Maximal eFuse block revision supported by image
    pub fn max_efuse_blk_rev_full(&self) -> u16 {
        self.max_efuse_blk_rev_full
    }

    /// MMU page size
    pub fn mmu_page_size(&self) -> u8 {
        self.mmu_page_size
    }
}

impl core::fmt::Debug for EspAppDesc {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EspAppDesc")
            .field("magic_word", &self.magic_word)
            .field("secure_version", &self.secure_version)
            .field("version", &self.version())
            .field("project_name", &self.project_name())
            .field("time", &self.time())
            .field("date", &self.date())
            .field("idf_ver", &self.idf_ver())
            .field("app_elf_sha256", &self.app_elf_sha256)
            .field("min_efuse_blk_rev_full", &self.min_efuse_blk_rev_full)
            .field("max_efuse_blk_rev_full", &self.max_efuse_blk_rev_full)
            .field("mmu_page_size", &self.mmu_page_size)
            .finish()
    }
}

fn array_to_str(array: &[core::ffi::c_char]) -> &str {
    let len = array.iter().position(|b| *b == 0).unwrap_or(array.len());
    unsafe {
        core::str::from_utf8_unchecked(core::slice::from_raw_parts(array.as_ptr().cast(), len))
    }
}

/// Build time
pub const BUILD_TIME: &str = env!("ESP_BOOTLOADER_SUPPORT_BUILD_TIME");

/// Build date
pub const BUILD_DATE: &str = env!("ESP_BOOTLOADER_SUPPORT_BUILD_DATE");

/// MMU page size in bytes
pub const MMU_PAGE_SIZE: u32 = {
    let mmu_page_size =
        esp_config::esp_config_str!("ESP_BOOTLOADER_SUPPORT_ESP_IDF_CONFIG_MMU_PAGE_SIZE")
            .as_bytes();
    match mmu_page_size {
        b"8k" => 8 * 1024,
        b"16k" => 16 * 1024,
        b"32k" => 32 * 1024,
        b"64k" => 64 * 1024,
        _ => 64 * 1024,
    }
};

/// This macro populates the application descriptor (see [EspAppDesc]) which is available as a static named `ESP_APP_DESC`
///
/// In most cases you can just use the no-arguments version of this macro.
#[macro_export]
macro_rules! esp_app_desc {
    () => {
        $crate::esp_app_desc!(
            env!("CARGO_PKG_VERSION"),
            env!("CARGO_PKG_NAME"),
            $crate::BUILD_TIME,
            $crate::BUILD_DATE,
            "5.3.1",
            (31 - u32::leading_zeros($crate::MMU_PAGE_SIZE)) as u8,
            0,
            u16::MAX
        );
    };

    (
     $version: expr,
     $project_name: expr,
     $build_time: expr,
     $build_date: expr,
     $idf_ver: expr,
     $mmu_page_size: expr,
     $min_efuse_blk_rev_full: expr,
     $max_efuse_blk_rev_full: expr
    ) => {
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

        #[export_name = "esp_app_desc"]
        #[link_section = ".rodata_desc.appdesc"]
        pub static ESP_APP_DESC: $crate::EspAppDesc = $crate::EspAppDesc {
            magic_word: ESP_APP_DESC_MAGIC_WORD,
            secure_version: 0,
            reserv1: [0; 2],
            version: str_to_cstr_array($version),
            project_name: str_to_cstr_array($project_name),
            time: str_to_cstr_array($build_time),
            date: str_to_cstr_array($build_date),
            idf_ver: str_to_cstr_array($idf_ver),
            app_elf_sha256: [0; 32],
            min_efuse_blk_rev_full: $min_efuse_blk_rev_full,
            max_efuse_blk_rev_full: $max_efuse_blk_rev_full,
            mmu_page_size: $mmu_page_size,
            reserv3: [0; 3],
            reserv2: [0; 18],
        };
    };
}
