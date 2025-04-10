//! # Bootloader Support Library supplementing esp-hal
//!
//! ## Overview
//!
//! This crate contains functionality related to the ESP-IDF 2nd stage
//! bootloader.
//!
//! - populating the application-descriptor
//! - read the partition table
//! - conveniently use a partition to read and write flash contents
//!
//! ## Additional configuration
//!
//! We've exposed some configuration options that don't fit into cargo
//! features. These can be set via environment variables, or via cargo's `[env]`
//! section inside `.cargo/config.toml`. Below is a table of tunable parameters
//! for this crate:
#![doc = ""]
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_bootloader_esp_idf_config_table.md"))]
#![doc = ""]
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![no_std]

// MUST be the first module
mod fmt;

#[cfg(not(feature = "pure-rust"))]
mod rom;
#[cfg(not(feature = "pure-rust"))]
pub(crate) use rom as crypto;

#[cfg(feature = "pure-rust")]
mod non_rom;
#[cfg(feature = "pure-rust")]
pub(crate) use non_rom as crypto;

pub mod partitions;

#[cfg(feature = "ota")]
pub mod ota;

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
    /// Needs to be public since it's used by the macro
    #[doc(hidden)]
    #[allow(clippy::too_many_arguments, reason = "For internal use only")]
    pub const fn new_internal(
        version: &str,
        project_name: &str,
        build_time: &str,
        build_date: &str,
        idf_ver: &str,
        min_efuse_blk_rev_full: u16,
        max_efuse_blk_rev_full: u16,
        mmu_page_size: u32,
    ) -> Self {
        Self {
            magic_word: ESP_APP_DESC_MAGIC_WORD,
            secure_version: 0,
            reserv1: [0; 2],
            version: str_to_cstr_array(version),
            project_name: str_to_cstr_array(project_name),
            time: str_to_cstr_array(build_time),
            date: str_to_cstr_array(build_date),
            idf_ver: str_to_cstr_array(idf_ver),
            app_elf_sha256: [0; 32],
            min_efuse_blk_rev_full,
            max_efuse_blk_rev_full,
            mmu_page_size: (mmu_page_size.ilog2()) as u8,
            reserv3: [0; 3],
            reserv2: [0; 18],
        }
    }

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
    ///
    /// Format `major * 100 + minor`
    pub fn min_efuse_blk_rev_full(&self) -> u16 {
        self.min_efuse_blk_rev_full
    }

    /// Maximal eFuse block revision supported by image
    ///
    /// Format `major * 100 + minor`
    pub fn max_efuse_blk_rev_full(&self) -> u16 {
        self.max_efuse_blk_rev_full
    }

    /// MMU page size in bytes
    pub fn mmu_page_size(&self) -> u32 {
        2_u32.pow(self.mmu_page_size as u32)
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

#[cfg(feature = "defmt")]
impl defmt::Format for EspAppDesc {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "EspAppDesc (\
            magic_word = {}, \
            secure_version = {}, \
            version = {}, \
            project_name = {}, \
            time = {}, \
            date = {}, \
            idf_ver = {}, \
            app_elf_sha256 = {}, \
            min_efuse_blk_rev_full = {}, \
            max_efuse_blk_rev_full = {}, \
            mmu_page_size = {}\
            )",
            self.magic_word,
            self.secure_version,
            self.version(),
            self.project_name(),
            self.time(),
            self.date(),
            self.idf_ver(),
            self.app_elf_sha256,
            self.min_efuse_blk_rev_full,
            self.max_efuse_blk_rev_full,
            self.mmu_page_size,
        )
    }
}

fn array_to_str(array: &[core::ffi::c_char]) -> &str {
    let len = array.iter().position(|b| *b == 0).unwrap_or(array.len());
    unsafe {
        core::str::from_utf8_unchecked(core::slice::from_raw_parts(array.as_ptr().cast(), len))
    }
}

const ESP_APP_DESC_MAGIC_WORD: u32 = 0xABCD5432;

const fn str_to_cstr_array<const C: usize>(s: &str) -> [::core::ffi::c_char; C] {
    let bytes = s.as_bytes();
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

/// Build time
pub const BUILD_TIME: &str = env!("ESP_BOOTLOADER_BUILD_TIME");

/// Build date
pub const BUILD_DATE: &str = env!("ESP_BOOTLOADER_BUILD_DATE");

/// MMU page size in bytes
pub const MMU_PAGE_SIZE: u32 = {
    let mmu_page_size =
        esp_config::esp_config_str!("ESP_BOOTLOADER_ESP_IDF_CONFIG_MMU_PAGE_SIZE").as_bytes();
    match mmu_page_size {
        b"8k" => 8 * 1024,
        b"16k" => 16 * 1024,
        b"32k" => 32 * 1024,
        b"64k" => 64 * 1024,
        _ => 64 * 1024,
    }
};

/// The (pretended) ESP-IDF version
pub const ESP_IDF_COMPATIBLE_VERSION: &str =
    esp_config::esp_config_str!("ESP_BOOTLOADER_ESP_IDF_CONFIG_MMU_PAGE_SIZE");

/// This macro populates the application descriptor (see [EspAppDesc]) which is
/// available as a static named `ESP_APP_DESC`
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
            $crate::ESP_IDF_COMPATIBLE_VERSION,
            $crate::MMU_PAGE_SIZE,
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
        #[export_name = "esp_app_desc"]
        #[link_section = ".rodata_desc.appdesc"]
        pub static ESP_APP_DESC: $crate::EspAppDesc = $crate::EspAppDesc::new_internal(
            $version,
            $project_name,
            $build_time,
            $build_date,
            $idf_ver,
            $min_efuse_blk_rev_full,
            $max_efuse_blk_rev_full,
            $mmu_page_size,
        );
    };
}
