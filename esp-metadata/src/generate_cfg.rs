use core::str::FromStr;
use std::sync::OnceLock;

use anyhow::{Result, bail};
use proc_macro2::TokenStream;
use strum::IntoEnumIterator;

macro_rules! include_toml {
    ($type:ty, $file:expr) => {{
        static LOADED_TOML: OnceLock<$type> = OnceLock::new();
        LOADED_TOML.get_or_init(|| basic_toml::from_str(include_str!($file)).unwrap())
    }};
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
    strum::AsRefStr,
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
    strum::AsRefStr,
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
    Hash,
    serde::Deserialize,
    serde::Serialize,
    strum::Display,
    strum::EnumIter,
    strum::EnumString,
    strum::AsRefStr,
)]
#[cfg_attr(feature = "clap", derive(clap::ValueEnum))]
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
    pub fn from_cargo_feature() -> Result<Self> {
        let all_chips = Chip::iter().map(|c| c.to_string()).collect::<Vec<_>>();

        let mut chip = None;
        for c in all_chips.iter() {
            if std::env::var(format!("CARGO_FEATURE_{}", c.to_uppercase())).is_ok() {
                if chip.is_some() {
                    bail!(
                        "Expected exactly one of the following features to be enabled: {}",
                        all_chips.join(", ")
                    );
                }
                chip = Some(c);
            }
        }

        let Some(chip) = chip else {
            bail!(
                "Expected exactly one of the following features to be enabled: {}",
                all_chips.join(", ")
            );
        };

        Ok(Self::from_str(chip.as_str()).unwrap())
    }

    pub fn target(&self) -> &'static str {
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

    pub fn lp_target(&self) -> Result<&'static str> {
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
pub struct MemoryRegion {
    name: String,
    start: u32,
    end: u32,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
struct Device {
    pub name: String,
    pub arch: Arch,
    pub cores: usize,
    pub trm: String,
    pub peripherals: Vec<String>,
    pub symbols: Vec<String>,
    pub memory: Vec<MemoryRegion>,
}

/// Device configuration file format.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct Config {
    device: Device,
}

impl Config {
    /// The configuration for the specified chip.
    pub fn for_chip(chip: &Chip) -> &Self {
        match chip {
            Chip::Esp32 => include_toml!(Config, "../devices/esp32.toml"),
            Chip::Esp32c2 => include_toml!(Config, "../devices/esp32c2.toml"),
            Chip::Esp32c3 => include_toml!(Config, "../devices/esp32c3.toml"),
            Chip::Esp32c6 => include_toml!(Config, "../devices/esp32c6.toml"),
            Chip::Esp32h2 => include_toml!(Config, "../devices/esp32h2.toml"),
            Chip::Esp32s2 => include_toml!(Config, "../devices/esp32s2.toml"),
            Chip::Esp32s3 => include_toml!(Config, "../devices/esp32s3.toml"),
        }
    }

    /// Create an empty configuration
    pub fn empty() -> Self {
        Self {
            device: Device {
                name: "".to_owned(),
                arch: Arch::RiscV,
                cores: 1,
                trm: "".to_owned(),
                peripherals: Vec::new(),
                symbols: Vec::new(),
                memory: Vec::new(),
            },
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
        if self.device.cores > 1 {
            Cores::Multi
        } else {
            Cores::Single
        }
    }

    /// The peripherals of the device.
    pub fn peripherals(&self) -> &[String] {
        &self.device.peripherals
    }

    /// User-defined symbols for the device.
    pub fn symbols(&self) -> &[String] {
        &self.device.symbols
    }

    /// Memory regions.
    ///
    /// Will be available as env-variables `REGION-<NAME>-START` /
    /// `REGION-<NAME>-END`
    pub fn memory(&self) -> &[MemoryRegion] {
        &self.device.memory
    }

    /// All configuration values for the device.
    pub fn all(&self) -> impl Iterator<Item = &str> + '_ {
        [
            self.device.name.as_str(),
            self.device.arch.as_ref(),
            match self.cores() {
                Cores::Single => "single_core",
                Cores::Multi => "multi_core",
            },
        ]
        .into_iter()
        .chain(self.device.peripherals.iter().map(|s| s.as_str()))
        .chain(self.device.symbols.iter().map(|s| s.as_str()))
    }

    /// Does the configuration contain `item`?
    pub fn contains(&self, item: &str) -> bool {
        self.all().any(|i| i == item)
    }

    /// Define all symbols for a given configuration.
    pub fn define_symbols(&self) {
        define_all_possible_symbols();
        // Define all necessary configuration symbols for the configured device:
        for symbol in self.all() {
            println!("cargo:rustc-cfg={symbol}");
        }

        // Define env-vars for all memory regions
        for memory in self.memory() {
            println!("cargo:rustc-cfg=has_{}_region", memory.name.to_lowercase());
        }
    }

    pub fn generate_metadata(&self) {
        // Output a number as a TokenStream, without the type suffix.
        fn number(n: usize) -> TokenStream {
            TokenStream::from_str(&format!("{n}")).unwrap()
        }

        let out_dir = std::env::var_os("OUT_DIR").unwrap();
        let out_dir = std::path::Path::new(&out_dir);
        let out_file = out_dir.join("_generated.rs").to_string_lossy().to_string();

        let mut g = TokenStream::new();

        let chip_name = self.name();
        // Public API, can't use a private macro:
        g.extend(quote::quote! {
            /// The name of the chip as `&str`
            #[macro_export]
            macro_rules! chip {
                () => { #chip_name };
            }
        });

        // Translate the chip properties into a macro that can be used in esp-hal:
        let arch = self.device.arch.as_ref();
        let cores = number(self.device.cores);
        let trm = &self.device.trm;

        // Not public API, can use a private macro:
        g.extend(quote::quote! {
            /// A link to the Technical Reference Manual (TRM) for the chip.
            #[doc(hidden)]
            #[macro_export]
            macro_rules! property {
                ("chip") => { #chip_name };
                ("arch") => { #arch };
                ("cores") => { #cores };
                ("cores", str) => { stringify!(#cores) };
                ("trm") => { #trm };
            }
        });

        let region_branches = self.memory().iter().map(|region| {
            let name = region.name.to_uppercase();
            let start = number(region.start as usize);
            let end = number(region.end as usize);

            quote::quote! {
                ( #name ) => {
                    #start .. #end
                };
            }
        });

        g.extend(quote::quote! {
            /// Macro to get the address range of the given memory region.
            #[macro_export]
            macro_rules! memory_range {
                #(#region_branches)*
            }
        });

        std::fs::write(&out_file, g.to_string()).unwrap();
    }
}

/// Defines all possible symbols that _could_ be output from this crate
/// regardless of the chosen configuration.
///
/// This is required to avoid triggering the unexpected-cfgs lint.
fn define_all_possible_symbols() {
    // Used by our documentation builds to prevent the huge red warning banner.
    println!("cargo:rustc-check-cfg=cfg(not_really_docsrs)");

    for chip in Chip::iter() {
        let config = Config::for_chip(&chip);
        for symbol in config.all() {
            // https://doc.rust-lang.org/cargo/reference/build-scripts.html#rustc-check-cfg
            println!("cargo:rustc-check-cfg=cfg({})", symbol);
        }
    }
}
