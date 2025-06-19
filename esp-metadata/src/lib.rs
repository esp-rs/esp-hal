//! Metadata for Espressif devices, primarily intended for use in build scripts.
mod cfg;

use core::str::FromStr;
use std::{collections::HashMap, fmt::Write, path::Path, sync::OnceLock};

use anyhow::{Result, bail, ensure};
use cfg::PeriConfig;
use proc_macro2::TokenStream;
use strum::IntoEnumIterator;

use crate::cfg::{SupportItem, SupportStatus, Value};

macro_rules! include_toml {
    (Config, $file:expr) => {{
        static LOADED_TOML: OnceLock<Config> = OnceLock::new();
        LOADED_TOML.get_or_init(|| {
            let config: Config = basic_toml::from_str(include_str!($file)).unwrap();

            config.validate().expect("Invalid device configuration");

            config
        })
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
    name: String,
    arch: Arch,
    cores: usize,
    trm: String,

    peripherals: Vec<String>,
    // For now, this is only used to double-check the configuration.
    virtual_peripherals: Vec<String>,
    symbols: Vec<String>,
    memory: Vec<MemoryRegion>,

    // Peripheral driver configuration:
    #[serde(flatten)]
    peri_config: PeriConfig,
}

// Output a Display-able value as a TokenStream, intended to generate numbers
// without the type suffix.
fn number(n: impl std::fmt::Display) -> TokenStream {
    TokenStream::from_str(&format!("{n}")).unwrap()
}

/// Device configuration file format.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct Config {
    device: Device,
    #[serde(skip)]
    all_symbols: OnceLock<Vec<String>>,
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
                virtual_peripherals: Vec::new(),
                symbols: Vec::new(),
                memory: Vec::new(),
                peri_config: PeriConfig::default(),
            },
            all_symbols: OnceLock::new(),
        }
    }

    fn validate(&self) -> Result<()> {
        for instance in self.device.peri_config.driver_instances() {
            let (driver, peri) = instance.split_once('.').unwrap();
            ensure!(
                self.device.peripherals.iter().any(|p| p == peri)
                    || self.device.virtual_peripherals.iter().any(|p| p == peri),
                "Driver {driver} marks an implementation for '{peri}' but this peripheral is not defined for '{}'",
                self.device.name
            );
        }

        Ok(())
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
    pub fn all(&self) -> &[String] {
        self.all_symbols.get_or_init(|| {
            let mut all = vec![
                self.device.name.clone(),
                self.device.arch.to_string(),
                match self.cores() {
                    Cores::Single => String::from("single_core"),
                    Cores::Multi => String::from("multi_core"),
                },
            ];
            all.extend(
                self.device
                    .peripherals
                    .iter()
                    .map(|p| format!("soc_has_{p}")),
            );
            all.extend_from_slice(&self.device.symbols);
            all.extend(
                self.device
                    .peri_config
                    .driver_names()
                    .map(|name| name.to_string()),
            );
            all.extend(self.device.peri_config.driver_instances());

            all.extend(self.device.peri_config.properties().filter_map(
                |(name, value)| match value {
                    Value::Boolean(true) => Some(name.to_string()),
                    Value::Number(value) => Some(format!("{name}=\"{value}\"")),
                    _ => None,
                },
            ));
            all
        })
    }

    /// Does the configuration contain `item`?
    pub fn contains(&self, item: &str) -> bool {
        self.all().iter().any(|i| i == item)
    }

    /// Define all symbols for a given configuration.
    pub fn define_symbols(&self) {
        define_all_possible_symbols();
        // Define all necessary configuration symbols for the configured device:
        for symbol in self.all() {
            println!("cargo:rustc-cfg={}", symbol.replace('.', "_"));
        }

        // Define env-vars for all memory regions
        for memory in self.memory() {
            println!("cargo:rustc-cfg=has_{}_region", memory.name.to_lowercase());
        }
    }

    pub fn generate_metadata(&self) {
        let out_dir = std::env::var_os("OUT_DIR").unwrap();
        let out_dir = Path::new(&out_dir);

        self.generate_properties(out_dir, "_generated.rs");
        self.generate_gpios(out_dir, "_generated_gpio.rs");
    }

    fn generate_properties(&self, out_dir: &Path, file_name: &str) {
        let out_file = out_dir.join(file_name).to_string_lossy().to_string();

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

        let peripheral_properties =
            self.device
                .peri_config
                .properties()
                .flat_map(|(name, value)| match value {
                    Value::Unset => quote::quote! {},
                    Value::Number(value) => {
                        let value = number(value); // ensure no numeric suffix is added
                        quote::quote! {
                            (#name) => { #value };
                            (#name, str) => { stringify!(#value) };
                        }
                    }
                    Value::Boolean(value) => quote::quote! {
                        (#name) => { #value };
                    },
                });

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
                #(#peripheral_properties)*
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
            #[doc(hidden)]
            macro_rules! memory_range {
                #(#region_branches)*
            }
        });

        std::fs::write(&out_file, g.to_string()).unwrap();
    }

    fn generate_gpios(&self, out_dir: &Path, file_name: &str) {
        let out_file = out_dir.join(file_name).to_string_lossy().to_string();

        let pins = self.device.peri_config.gpio.as_ref().unwrap().instances[0]
            .instance_config
            .pins
            .iter()
            .map(|pin| {
                let pin_number = number(pin.pin);

                struct PinAttrs {
                    input: bool,
                    output: bool,
                    analog: bool,
                    rtc_io: bool,
                    touch: bool,
                    usb_dm: bool,
                    usb_dp: bool,
                }

                let mut pin_attrs = PinAttrs {
                    input: false,
                    output: false,
                    analog: false,
                    rtc_io: false,
                    touch: false,
                    usb_dm: false,
                    usb_dp: false,
                };
                pin.kind.iter().for_each(|kind| match kind {
                    cfg::PinCapability::Input => pin_attrs.input = true,
                    cfg::PinCapability::Output => pin_attrs.output = true,
                    cfg::PinCapability::Analog => pin_attrs.analog = true,
                    cfg::PinCapability::Rtc => pin_attrs.rtc_io = true,
                    cfg::PinCapability::Touch => pin_attrs.touch = true,
                    cfg::PinCapability::UsbDm => pin_attrs.usb_dm = true,
                    cfg::PinCapability::UsbDp => pin_attrs.usb_dp = true,
                });

                let mut attrs = vec![];

                if pin_attrs.input {
                    attrs.push(quote::quote! { Input });
                }
                if pin_attrs.output {
                    attrs.push(quote::quote! { Output });
                }
                if pin_attrs.analog {
                    attrs.push(quote::quote! { Analog });
                }
                if pin_attrs.rtc_io {
                    if !pin_attrs.output {
                        attrs.push(quote::quote! { RtcIo });
                    } else {
                        attrs.push(quote::quote! { RtcIoInput });
                    }
                }
                if pin_attrs.touch {
                    attrs.push(quote::quote! { Touch });
                }
                if pin_attrs.usb_dm {
                    attrs.push(quote::quote! { UsbDm });
                }
                if pin_attrs.usb_dp {
                    attrs.push(quote::quote! { UsbDp });
                }

                let mut input_afs = vec![];
                let mut output_afs = vec![];

                for af in 0..6 {
                    if let Some(signal) = pin.af_input.get(af) {
                        let af = quote::format_ident!("_{af}");
                        let signal = TokenStream::from_str(signal).unwrap();
                        input_afs.push(quote::quote! { #af => #signal });
                    }
                    if let Some(signal) = pin.af_output.get(af) {
                        let af = quote::format_ident!("_{af}");
                        let signal = TokenStream::from_str(signal).unwrap();
                        output_afs.push(quote::quote! { #af => #signal });
                    }
                }

                quote::quote! {
                    ( #pin_number, [#(#attrs),*] ( #(#input_afs)* ) ( #(#output_afs)* ) )
                }
            });

        let g = quote::quote! {
            crate::gpio! {
                #(#pins)*
            }
        };

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

    let mut cfg_values: HashMap<String, Vec<String>> = HashMap::new();

    for chip in Chip::iter() {
        let config = Config::for_chip(&chip);
        for symbol in config.all() {
            if let Some((symbol_name, symbol_value)) = symbol.split_once('=') {
                // cfg's with values need special syntax, so let's collect all
                // of them separately.
                let symbol_name = symbol_name.replace('.', "_");
                let entry = cfg_values.entry(symbol_name).or_default();
                // Avoid duplicates in the same cfg.
                if !entry.contains(&symbol_value.to_string()) {
                    entry.push(symbol_value.to_string());
                }
            } else {
                // https://doc.rust-lang.org/cargo/reference/build-scripts.html#rustc-check-cfg
                println!("cargo:rustc-check-cfg=cfg({})", symbol.replace('.', "_"));
            }
        }
    }

    // Now output all cfgs with values.
    for (symbol_name, symbol_values) in cfg_values {
        println!(
            "cargo:rustc-check-cfg=cfg({symbol_name}, values({}))",
            symbol_values.join(",")
        );
    }
}

pub fn generate_chip_support_status(output: &mut impl Write) -> std::fmt::Result {
    let nothing = "";

    // Calculate the width of the first column.
    let driver_col_width = std::iter::once("Driver")
        .chain(PeriConfig::drivers().iter().map(|i| i.name))
        .map(|c| c.len())
        .max()
        .unwrap();

    // Header
    write!(output, "| {:driver_col_width$} |", "Driver")?;
    for chip in Chip::iter() {
        write!(output, " {} |", chip.pretty_name())?;
    }
    writeln!(output)?;

    // Header separator
    write!(output, "| {nothing:-<driver_col_width$} |")?;
    for chip in Chip::iter() {
        write!(
            output,
            ":{nothing:-<width$}:|",
            width = chip.pretty_name().len()
        )?;
    }
    writeln!(output)?;

    // Driver support status
    for SupportItem {
        name,
        symbols,
        config_group,
    } in PeriConfig::drivers()
    {
        write!(output, "| {name:driver_col_width$} |")?;
        for chip in Chip::iter() {
            let config = Config::for_chip(&chip);

            let status = config
                .device
                .peri_config
                .support_status(config_group)
                .inspect(|status| {
                    // TODO: this is good for double-checking, but it should probably go the
                    // other way around. Driver config should define what peripheral symbols exist.
                    assert!(
                        matches!(status, SupportStatus::NotSupported)
                            || symbols.is_empty()
                            || symbols.iter().any(|p| config.contains(p)),
                        "{} has configuration for {} but no compatible symbols have been defined",
                        chip.pretty_name(),
                        config_group
                    );
                })
                .or_else(|| {
                    // If the driver is not supported by the chip, we return None.
                    if symbols.iter().any(|p| config.contains(p)) {
                        Some(SupportStatus::NotSupported)
                    } else {
                        None
                    }
                });
            let status_icon = match status {
                None => " ",
                Some(status) => status.icon(),
            };
            // VSCode displays emojis just a bit wider than 2 characters, making this
            // approximation a bit too wide but good enough.
            let support_cell_width = chip.pretty_name().len() - status.is_some() as usize;
            write!(output, " {status_icon:support_cell_width$} |")?;
        }
        writeln!(output)?;
    }

    writeln!(output)?;

    // Print legend
    writeln!(output, " * Empty cell: not available")?;
    for s in [
        SupportStatus::NotSupported,
        SupportStatus::Partial,
        SupportStatus::Supported,
    ] {
        writeln!(output, " * {}: {}", s.icon(), s.status())?;
    }

    Ok(())
}
