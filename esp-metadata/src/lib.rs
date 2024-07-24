//! Metadata for Espressif devices, primarily intended for use in build scripts.

use anyhow::{bail, Result};

const ESP32_TOML: &str = include_str!("../devices/esp32.toml");
const ESP32C2_TOML: &str = include_str!("../devices/esp32c2.toml");
const ESP32C3_TOML: &str = include_str!("../devices/esp32c3.toml");
const ESP32C6_TOML: &str = include_str!("../devices/esp32c6.toml");
const ESP32H2_TOML: &str = include_str!("../devices/esp32h2.toml");
const ESP32S2_TOML: &str = include_str!("../devices/esp32s2.toml");
const ESP32S3_TOML: &str = include_str!("../devices/esp32s3.toml");

lazy_static::lazy_static! {
    static ref ESP32_CFG: Config = basic_toml::from_str(ESP32_TOML).unwrap();
    static ref ESP32C2_CFG: Config = basic_toml::from_str(ESP32C2_TOML).unwrap();
    static ref ESP32C3_CFG: Config = basic_toml::from_str(ESP32C3_TOML).unwrap();
    static ref ESP32C6_CFG: Config = basic_toml::from_str(ESP32C6_TOML).unwrap();
    static ref ESP32H2_CFG: Config = basic_toml::from_str(ESP32H2_TOML).unwrap();
    static ref ESP32S2_CFG: Config = basic_toml::from_str(ESP32S2_TOML).unwrap();
    static ref ESP32S3_CFG: Config = basic_toml::from_str(ESP32S3_TOML).unwrap();
}

/// Supported device architectures.
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    serde::Deserialize,
    serde::Serialize,
    strum::Display,
    strum::EnumIter,
    strum::EnumString,
)]
#[serde(rename_all = "lowercase")]
#[strum(serialize_all = "lowercase")]
pub enum Arch {
    /// RISC-V architecture
    RiscV,
    /// Xtensa architecture
    Xtensa,
}

/// Device core count.
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    serde::Deserialize,
    serde::Serialize,
    strum::Display,
    strum::EnumIter,
    strum::EnumString,
)]
pub enum Cores {
    /// Single CPU core
    #[serde(rename = "single_core")]
    #[strum(serialize = "single_core")]
    Single,
    /// Two or more CPU cores
    #[serde(rename = "multi_core")]
    #[strum(serialize = "multi_core")]
    Multi,
}

/// Supported devices.
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    serde::Deserialize,
    serde::Serialize,
    strum::Display,
    strum::EnumIter,
    strum::EnumString,
    clap::ValueEnum,
)]
#[serde(rename_all = "kebab-case")]
#[strum(serialize_all = "kebab-case")]
pub enum Chip {
    /// ESP32
    Esp32,
    /// ESP32-C2, ESP8684
    Esp32c2,
    /// ESP32-C3, ESP8685
    Esp32c3,
    /// ESP32-C6
    Esp32c6,
    /// ESP32-H2
    Esp32h2,
    /// ESP32-S2
    Esp32s2,
    /// ESP32-S3
    Esp32s3,
}

impl Chip {
    pub fn target(&self) -> &str {
        use Chip::*;

        match self {
            Esp32 => "xtensa-esp32-none-elf",
            Esp32c2 | Esp32c3 => "riscv32imc-unknown-none-elf",
            Esp32c6 | Esp32h2 => "riscv32imac-unknown-none-elf",
            Esp32s2 => "xtensa-esp32s2-none-elf",
            Esp32s3 => "xtensa-esp32s3-none-elf",
        }
    }

    pub fn has_lp_core(&self) -> bool {
        use Chip::*;

        matches!(self, Esp32c6 | Esp32s2 | Esp32s3)
    }

    pub fn lp_target(&self) -> Result<&str> {
        use Chip::*;

        match self {
            Esp32c6 => Ok("riscv32imac-unknown-none-elf"),
            Esp32s2 | Esp32s3 => Ok("riscv32imc-unknown-none-elf"),
            _ => bail!("Chip does not contain an LP core: '{}'", self),
        }
    }

    pub fn pretty_name(&self) -> &str {
        match self {
            Chip::Esp32 => "ESP32",
            Chip::Esp32c2 => "ESP32-C2",
            Chip::Esp32c3 => "ESP32-C3",
            Chip::Esp32c6 => "ESP32-C6",
            Chip::Esp32h2 => "ESP32-H2",
            Chip::Esp32s2 => "ESP32-S2",
            Chip::Esp32s3 => "ESP32-S3",
        }
    }

    pub fn is_xtensa(&self) -> bool {
        matches!(self, Chip::Esp32 | Chip::Esp32s2 | Chip::Esp32s3)
    }

    pub fn is_riscv(&self) -> bool {
        !self.is_xtensa()
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
struct Device {
    pub name: String,
    pub arch: Arch,
    pub cores: Cores,
    pub peripherals: Vec<String>,
    pub symbols: Vec<String>,
}

/// Device configuration file format.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct Config {
    device: Device,
}

impl Config {
    /// The configuration for the specified chip.
    pub fn for_chip(chip: &Chip) -> Self {
        match chip {
            Chip::Esp32 => ESP32_CFG.clone(),
            Chip::Esp32c2 => ESP32C2_CFG.clone(),
            Chip::Esp32c3 => ESP32C3_CFG.clone(),
            Chip::Esp32c6 => ESP32C6_CFG.clone(),
            Chip::Esp32h2 => ESP32H2_CFG.clone(),
            Chip::Esp32s2 => ESP32S2_CFG.clone(),
            Chip::Esp32s3 => ESP32S3_CFG.clone(),
        }
    }

    /// The name of the device.
    pub fn name(&self) -> String {
        self.device.name.clone()
    }

    /// The CPU architecture of the device.
    pub fn arch(&self) -> Arch {
        self.device.arch
    }

    /// The core count of the device.
    pub fn cores(&self) -> Cores {
        self.device.cores
    }

    /// The peripherals of the device.
    pub fn peripherals(&self) -> &[String] {
        &self.device.peripherals
    }

    /// User-defined symbols for the device.
    pub fn symbols(&self) -> &[String] {
        &self.device.symbols
    }

    /// All configuration values for the device.
    pub fn all(&self) -> Vec<String> {
        [
            vec![
                self.device.name.clone(),
                self.device.arch.to_string(),
                self.device.cores.to_string(),
            ],
            self.device.peripherals.clone(),
            self.device.symbols.clone(),
        ]
        .concat()
    }

    /// Does the configuration contain `item`?
    pub fn contains(&self, item: &String) -> bool {
        self.all().contains(item)
    }

    /// Define all symbols for a given configuration.
    pub fn define_symbols(&self) {
        // Define all necessary configuration symbols for the configured device:
        println!("cargo:rustc-cfg={}", self.name());
        println!("cargo:rustc-cfg={}", self.arch());
        println!("cargo:rustc-cfg={}", self.cores());

        for peripheral in self.peripherals() {
            println!("cargo:rustc-cfg={peripheral}");
        }

        for symbol in self.symbols() {
            println!("cargo:rustc-cfg={symbol}");
        }
    }
}
