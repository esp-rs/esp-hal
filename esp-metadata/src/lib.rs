//! Metadata for Espressif devices, primarily intended for use in build scripts.
mod cfg;

use core::str::FromStr;
use std::{fmt::Write, sync::OnceLock};

use anyhow::{Result, bail, ensure};
use cfg::PeriConfig;
use indexmap::IndexMap;
pub use proc_macro2::TokenStream;
use quote::{format_ident, quote};
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

    pub fn target(&self) -> String {
        Config::for_chip(self).device.target.clone()
    }

    pub fn has_lp_core(&self) -> bool {
        use Chip::*;

        matches!(self, Esp32c6 | Esp32s2 | Esp32s3)
    }

    pub fn lp_target(&self) -> Result<&'static str> {
        match self {
            Chip::Esp32c6 => Ok("riscv32imac-unknown-none-elf"),
            Chip::Esp32s2 | Chip::Esp32s3 => Ok("riscv32imc-unknown-none-elf"),
            _ => bail!("Chip does not contain an LP core: '{self}'"),
        }
    }

    pub fn name(&self) -> &str {
        match self {
            Chip::Esp32 => "Esp32",
            Chip::Esp32c2 => "Esp32c2",
            Chip::Esp32c3 => "Esp32c3",
            Chip::Esp32c6 => "Esp32c6",
            Chip::Esp32h2 => "Esp32h2",
            Chip::Esp32s2 => "Esp32s2",
            Chip::Esp32s3 => "Esp32s3",
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

    pub fn list_of_check_cfgs() -> Vec<String> {
        let mut cfgs = vec![];

        // Used by our documentation builds to prevent the huge red warning banner.
        cfgs.push(String::from("cargo:rustc-check-cfg=cfg(not_really_docsrs)"));
        cfgs.push(String::from("cargo:rustc-check-cfg=cfg(semver_checks)"));

        let mut cfg_values: IndexMap<String, Vec<String>> = IndexMap::new();

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
                    let cfg = format!("cargo:rustc-check-cfg=cfg({})", symbol.replace('.', "_"));

                    if !cfgs.contains(&cfg) {
                        cfgs.push(cfg);
                    }
                }
            }
        }

        // Now output all cfgs with values.
        for (symbol_name, symbol_values) in cfg_values {
            cfgs.push(format!(
                "cargo:rustc-check-cfg=cfg({symbol_name}, values({}))",
                symbol_values.join(",")
            ));
        }

        cfgs
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct MemoryRegion {
    name: String,
    start: u32,
    end: u32,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct PeripheralDef {
    /// The name of the esp-hal peripheral singleton
    name: String,
    /// When omitted, same as `name`
    #[serde(default, rename = "pac")]
    pac_name: Option<String>,
    /// Whether or not the peripheral has a PAC counterpart
    #[serde(default, rename = "virtual")]
    is_virtual: bool,
    /// List of related interrupt signals
    #[serde(default)]
    interrupts: IndexMap<String, String>,
}

impl PeripheralDef {
    fn symbol_name(&self) -> String {
        format!("soc_has_{}", self.name.to_lowercase())
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
struct Device {
    name: String,
    arch: Arch,
    target: String,
    cores: usize,
    trm: String,

    peripherals: Vec<PeripheralDef>,
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
                name: String::new(),
                arch: Arch::RiscV,
                target: String::new(),
                cores: 1,
                trm: String::new(),
                peripherals: Vec::new(),
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
                self.device
                    .peripherals
                    .iter()
                    .any(|p| p.name.eq_ignore_ascii_case(peri)),
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
    pub fn peripherals(&self) -> &[PeripheralDef] {
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
            all.extend(self.device.peripherals.iter().map(|p| p.symbol_name()));
            all.extend_from_slice(&self.device.symbols);
            all.extend(
                self.device
                    .peri_config
                    .driver_names()
                    .map(|name| name.to_string()),
            );
            all.extend(self.device.peri_config.driver_instances());

            all.extend(
                self.device
                    .peri_config
                    .properties()
                    .filter_map(|(name, optional, value)| {
                        let is_unset = matches!(value, Value::Unset);
                        let mut syms = match value {
                            Value::Boolean(true) => Some(vec![name.to_string()]),
                            Value::NumberList(_) => None,
                            Value::String(value) => Some(vec![format!("{name}=\"{value}\"")]),
                            Value::Generic(v) => v.cfgs(),
                            Value::StringList(values) => Some(
                                values
                                    .iter()
                                    .map(|val| {
                                        format!(
                                            "{name}_{}",
                                            val.to_lowercase().replace("-", "_").replace("/", "_")
                                        )
                                    })
                                    .collect(),
                            ),
                            Value::Number(value) => Some(vec![format!("{name}=\"{value}\"")]),
                            _ => None,
                        };

                        if optional && !is_unset {
                            syms.get_or_insert_default().push(format!("{name}_is_set"));
                        }

                        syms
                    })
                    .flatten(),
            );
            all
        })
    }

    /// Does the configuration contain `item`?
    pub fn contains(&self, item: &str) -> bool {
        self.all().iter().any(|i| i == item)
    }

    pub fn generate_metadata(&self) -> TokenStream {
        let properties = self.generate_properties();
        let peris = self.generate_peripherals();
        let gpios = self.generate_gpios();

        quote! {
            #properties
            #peris
            #gpios
        }
    }

    fn generate_properties(&self) -> TokenStream {
        let mut tokens = TokenStream::new();

        let chip_name = self.name();
        // Public API, can't use a private macro:
        tokens.extend(quote! {
            /// The name of the chip as `&str`
            ///
            /// # Example
            ///
            /// ```rust, no_run
            /// use esp_hal::chip;
            /// let chip_name = chip!();
            #[doc = concat!("assert_eq!(chip_name, ", chip!(), ")")]
            /// ```
            #[macro_export]
            #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
            macro_rules! chip {
                () => { #chip_name };
            }
        });

        // Translate the chip properties into a macro that can be used in esp-hal:
        let arch = self.device.arch.as_ref();
        let cores = number(self.device.cores);
        let trm = &self.device.trm;

        let mut for_each_macros = vec![];

        let peripheral_properties =
            self.device
                .peri_config
                .properties()
                .flat_map(|(name, _optional, value)| match value {
                    Value::Number(value) => {
                        let value = number(value); // ensure no numeric suffix is added
                        quote! {
                            (#name) => { #value };
                            (#name, str) => { stringify!(#value) };
                        }
                    }
                    Value::Boolean(value) => quote! {
                        (#name) => { #value };
                    },
                    Value::String(value) => quote! {
                        (#name) => { #value };
                    },
                    Value::NumberList(numbers) => {
                        let numbers = numbers.into_iter().map(number).collect::<Vec<_>>();
                        for_each_macros.push(generate_for_each_macro(
                            &name.replace(".", "_"),
                            &[("all", &numbers)],
                        ));
                        quote! {}
                    }
                    Value::Generic(v) => {
                        if let Some(for_each) = v.for_each_macro() {
                            for_each_macros.push(for_each);
                        }
                        v.property_macro_branches()
                    }
                    Value::Unset | Value::StringList(_) => {
                        quote! {}
                    }
                });

        // Not public API, can use a private macro:
        tokens.extend(quote! {
            /// The properties of this chip and its drivers.
            #[macro_export]
            #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
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

            quote! {
                ( #name ) => {
                    #start .. #end
                };
            }
        });

        tokens.extend(quote! {
            /// Macro to get the address range of the given memory region.
            #[macro_export]
            #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
            macro_rules! memory_range {
                #(#region_branches)*
            }

            #(#for_each_macros)*
        });

        tokens
    }

    fn generate_gpios(&self) -> TokenStream {
        let Some(gpio) = self.device.peri_config.gpio.as_ref() else {
            // No GPIOs defined, nothing to do.
            return quote! {};
        };

        cfg::generate_gpios(gpio)
    }

    fn generate_peripherals(&self) -> TokenStream {
        let mut tokens = TokenStream::new();

        // TODO: repeat for all drivers that have Instance traits
        if let Some(peri) = self.device.peri_config.i2c_master.as_ref() {
            tokens.extend(cfg::generate_i2c_master_peripherals(peri));
        };
        if let Some(peri) = self.device.peri_config.uart.as_ref() {
            tokens.extend(cfg::generate_uart_peripherals(peri));
        }
        if let Some(peri) = self.device.peri_config.spi_master.as_ref() {
            tokens.extend(cfg::generate_spi_master_peripherals(peri));
        };
        if let Some(peri) = self.device.peri_config.spi_slave.as_ref() {
            tokens.extend(cfg::generate_spi_slave_peripherals(peri));
        };

        tokens.extend(self.generate_peripherals_macro());

        tokens
    }

    fn generate_peripherals_macro(&self) -> TokenStream {
        let mut stable = vec![];
        let mut unstable = vec![];
        let mut all_peripherals = vec![];

        let mut stable_peris = vec![];

        for item in PeriConfig::drivers() {
            if self.device.peri_config.support_status(item.config_group)
                == Some(SupportStatus::Supported)
            {
                for p in self.device.peri_config.driver_peris(item.config_group) {
                    if !stable_peris.contains(&p) {
                        stable_peris.push(p);
                    }
                }
            }
        }

        if let Some(gpio) = self.device.peri_config.gpio.as_ref() {
            for pin in gpio.pins_and_signals.pins.iter() {
                let pin = format_ident!("GPIO{}", pin.pin);
                let tokens = quote! {
                    #pin <= virtual ()
                };
                all_peripherals.push(quote! { #tokens });
                stable.push(tokens);
            }
        }

        for peri in self.device.peripherals.iter() {
            let hal = format_ident!("{}", peri.name);
            let pac = if peri.is_virtual {
                format_ident!("virtual")
            } else {
                format_ident!("{}", peri.pac_name.as_deref().unwrap_or(peri.name.as_str()))
            };
            // Make sure we have a stable order
            let mut interrupts = peri.interrupts.iter().collect::<Vec<_>>();
            interrupts.sort_by_key(|(k, _)| k.as_str());
            let interrupts = interrupts.iter().map(|(k, v)| {
                let pac_interrupt_name = format_ident!("{v}");
                let bind = format_ident!("bind_{k}_interrupt");
                let enable = format_ident!("enable_{k}_interrupt");
                let disable = format_ident!("disable_{k}_interrupt");
                quote! { #pac_interrupt_name: { #bind, #enable, #disable } }
            });
            let tokens = quote! {
                #hal <= #pac ( #(#interrupts),* )
            };
            if stable_peris
                .iter()
                .any(|p| peri.name.eq_ignore_ascii_case(p))
            {
                all_peripherals.push(quote! { #tokens });
                stable.push(tokens);
            } else {
                all_peripherals.push(quote! { #tokens (unstable) });
                unstable.push(tokens);
            }
        }

        generate_for_each_macro("peripheral", &[("all", &all_peripherals)])
    }

    pub fn active_cfgs(&self) -> Vec<String> {
        let mut cfgs = vec![];

        // Define all necessary configuration symbols for the configured device:
        for symbol in self.all() {
            cfgs.push(symbol.replace('.', "_"));
        }

        // Define env-vars for all memory regions
        for memory in self.memory() {
            cfgs.push(format!("has_{}_region", memory.name.to_lowercase()));
        }

        cfgs
    }

    /// For each symbol generates a cargo directive that activates it.
    pub fn list_of_cfgs(&self) -> Vec<String> {
        self.active_cfgs()
            .iter()
            .map(|cfg| format!("cargo:rustc-cfg={cfg}"))
            .collect()
    }
}

type Branch<'a> = (&'a str, &'a [TokenStream]);

fn generate_for_each_macro(name: &str, branches: &[Branch<'_>]) -> TokenStream {
    let macro_name = format_ident!("for_each_{name}");

    let flat_branches = branches.iter().flat_map(|b| b.1.iter());
    let repeat_names = branches.iter().map(|b| TokenStream::from_str(b.0).unwrap());
    let repeat_branches = branches.iter().map(|b| b.1);

    quote! {
        // This macro is called in esp-hal to implement a driver's
        // Instance trait for available peripherals. It works by defining, then calling an inner
        // macro that substitutes the properties into the template provided by the call in esp-hal.
        #[macro_export]
        #[cfg_attr(docsrs, doc(cfg(feature = "_device-selected")))]
        macro_rules! #macro_name {
            (
                $($pattern:tt => $code:tt;)*
            ) => {
                macro_rules! _for_each_inner {
                    $(($pattern) => $code;)*
                    ($other:tt) => {}
                }

                // Generate single macro calls with each branch
                // Usage:
                // ```
                // for_each_x! {
                //     ( $pattern ) => {
                //         $code
                //     }
                // }
                // ```
                #(_for_each_inner!(( #flat_branches ));)*

                // Generate a single macro call with all branches.
                // Usage:
                // ```
                // for_each_x! {
                //     (all $( ($pattern) ),*) => {
                //         $( $code )*
                //     }
                // }
                // ```
                #( _for_each_inner!( (#repeat_names #( (#repeat_branches) ),*) ); )*
            };
        }
    }
}

pub fn generate_build_script_utils() -> TokenStream {
    let check_cfgs = Chip::list_of_check_cfgs();

    let chip = Chip::iter()
        .map(|c| format_ident!("{}", c.name()))
        .collect::<Vec<_>>();
    let feature_env = Chip::iter().map(|c| format!("CARGO_FEATURE_{}", c.as_ref().to_uppercase()));
    let name = Chip::iter()
        .map(|c| c.as_ref().to_string())
        .collect::<Vec<_>>();
    let all_chip_features = name.join(", ");
    let config = Chip::iter().map(|chip| {
        let config = Config::for_chip(&chip);
        let symbols = config.active_cfgs();
        let arch = config.device.arch.to_string();
        let target = config.device.target.as_str();
        let cfgs = config.list_of_cfgs();
        quote! {
            Config {
                architecture: #arch,
                target: #target,
                symbols: &[
                    #(#symbols,)*
                ],
                cfgs: &[
                    #(#cfgs,)*
                ],
            }
        }
    });

    let bail_message = format!(
        "Expected exactly one of the following features to be enabled: {all_chip_features}"
    );

    quote! {
        // make it possible to build documentation without `std`.
        #[cfg(docsrs)]
        macro_rules! println {
            ($($any:tt)*) => {};
        }

        #[derive(Clone, Copy, PartialEq, Eq, Hash)]
        #[cfg_attr(docsrs, doc(cfg(feature = "build-script")))]
        pub enum Chip {
            #(#chip),*
        }

        impl core::str::FromStr for Chip {
            type Err = ();

            fn from_str(s: &str) -> Result<Self, Self::Err> {
                match s {
                    #( #name => Ok(Self::#chip),)*
                    _ => Err(()),
                }
            }
        }

        impl Chip {
            /// Tries to extract the active chip from the active cargo features.
            ///
            /// Exactly one device feature must be enabled for this function to succeed.
            pub fn from_cargo_feature() -> Result<Self, &'static str> {
                let all_chips = [
                    #(( #feature_env, Self::#chip )),*
                ];

                let mut chip = None;
                for (env, c) in all_chips {
                    if std::env::var(env).is_ok() {
                        if chip.is_some() {
                            return Err(#bail_message);
                        }
                        chip = Some(c);
                    }
                }

                match chip {
                    Some(chip) => Ok(chip),
                    None => Err(#bail_message)
                }
            }

            /// Returns whether the current chip uses the Tensilica Xtensa ISA.
            pub fn is_xtensa(self) -> bool {
                self.config().architecture == "xtensa"
            }

            /// The target triple of the current chip.
            pub fn target(self) -> &'static str {
                self.config().target
            }

            /// The simple name of the current chip.
            ///
            /// ## Example
            ///
            /// ```rust
            /// assert_eq!(Chip::Esp32s3.name(), "esp32s3");
            /// ```
            pub fn name(self) -> &'static str {
                match self {
                    #( Self::#chip => #name ),*
                }
            }

            /// Returns whether the chip configuration contains the given symbol.
            ///
            /// This function is a short-hand for `self.all_symbols().contains(&symbol)`.
            ///
            /// ## Example
            ///
            /// ```rust
            /// assert!(Chip::Esp32s3.contains("soc_has_pcnt"));
            /// ```
            pub fn contains(self, symbol: &str) -> bool {
                self.all_symbols().contains(&symbol)
            }

            /// Calling this function will define all cfg symbols for the firmware crate to use.
            pub fn define_cfgs(self) {
                self.config().define_cfgs()
            }

            /// Returns all symbols as a big slice.
            ///
            /// ## Example
            ///
            /// ```rust
            /// assert!(Chip::Esp32s3.all_symbols().contains("soc_has_pcnt"));
            /// ```
            pub fn all_symbols(&self) -> &'static [&'static str] {
                self.config().symbols
            }

            /// Returns an iterator over all chips.
            ///
            /// ## Example
            ///
            /// ```rust
            /// assert!(Chip::iter().any(|c| c == Chip::Esp32));
            /// ```
            pub fn iter() -> impl Iterator<Item = Chip> {
                [
                    #( Self::#chip ),*
                ].into_iter()
            }

            fn config(self) -> Config {
                match self {
                    #(Self::#chip => #config),*
                }
            }
        }

        struct Config {
            architecture: &'static str,
            target: &'static str,
            symbols: &'static [&'static str],
            cfgs: &'static [&'static str],
        }

        impl Config {
            fn define_cfgs(&self) {
                emit_check_cfg_directives();
                for cfg in self.cfgs {
                    println!("{cfg}");
                }
            }
        }

        /// Prints `cargo:rustc-check-cfg` lines.
        pub fn emit_check_cfg_directives() {
            #(println!(#check_cfgs);)*
        }
    }
}

pub fn generate_lib_rs() -> TokenStream {
    let chips = Chip::iter().map(|c| {
        let feature = format!("{c}");
        let file = format!("_generated_{c}.rs");
        quote! {
            #[cfg(feature = #feature)]
            include!(#file);
        }
    });

    quote! {
        //! # (Generated) metadata for Espressif MCUs.
        //!
        //! This crate provides properties that are specific to various Espressif microcontrollers,
        //! and provides macros to work with peripherals, pins, and various other parts of the chips.
        //!
        //! This crate can be used both in firmware, as well as in build scripts, but the usage is different.
        //!
        //! ## Usage in build scripts
        //!
        //! To use the `Chip` enum, add the crate to your `Cargo.toml` build
        //! dependencies, with the `build-script` feature:
        //!
        //! ```toml
        //! [build-dependencies]
        //! esp-metadata-generated = { version = "...", features = ["build-script"] }
        //! ```
        //!
        //! ## Usage in firmware
        //!
        //! To use the various macros, add the crate to your `Cargo.toml` dependencies.
        //! A device-specific feature needs to be enabled in order to use the crate, usually
        //! picked by the user:
        //!
        //! ```toml
        //! [dependencies]
        //! esp-metadata-generated = { version = "..." }
        //! # ...
        //!
        //! [features]
        //! esp32 = ["esp-metadata-generated/esp32"]
        //! esp32c2 = ["esp-metadata-generated/esp32c2"]
        //! # ...
        //! ```
        //!
        //! ## `for_each` macros
        //!
        //! The basic syntax of this macro looks like a macro definition with two distinct syntax options:
        //!
        //! ```rust, no_run
        //! for_each_peripherals! {
        //!     // Individual matcher, invoked separately for each peripheral instance
        //!     ( <individual match syntax> ) => { /* some code */ };
        //!
        //!     // Repeated matcher, invoked once with all peripheral instances
        //!     ( all $( (<individual match syntax>) ),* ) => { /* some code */ };
        //! }
        //! ```
        //!
        //! You can specify any number of matchers in the same invocation.
        //!
        //! > The way code is generated, you will need to use the full `return` syntax to return any
        //! > values from code generated with these macros.
        //!
        //! ### Using the individual matcher
        //!
        //! In this use case, each item's data is individually passed through the macro. This can be used to
        //! generate code for each item separately, allowing specializing the implementation where needed.
        //!
        //! ```rust,no_run
        //! for_each_gpio! {
        //!   // Example data: `(0, GPIO0 (_5 => EMAC_TX_CLK) (_1 => CLK_OUT1 _5 => EMAC_TX_CLK) ([Input] [Output]))`
        //!   ($n:literal, $gpio:ident ($($digital_input_function:ident => $digital_input_signal:ident)*) ($($digital_output_function:ident => $digital_output_signal:ident)*) ($($pin_attribute:ident)*)) => { /* some code */ };
        //!
        //!   // You can create matchers with data filled in. This example will specifically match GPIO2
        //!   ($n:literal, GPIO2 $input_af:tt $output_af:tt $attributes:tt) => { /* Additional case only for GPIO2 */ };
        //! }
        //! ```
        //!
        //! Different macros can have multiple different syntax options for their individual matchers, usually
        //! to provide more detailed information, while preserving simpler syntax for more basic use cases.
        //! Consult each macro's documentation for available options.
        //!
        //! ### Repeated matcher
        //!
        //! With this option, all data is passed through the macro all at once. This form can be used to,
        //! for example, generate struct fields. If the macro has multiple individual matcher options,
        //! there are separate repeated matchers for each of the options.
        //!
        //! To use this option, start the match pattern with the name of the individual matcher option. When
        //! there is only a single individual matcher option, its repeated matcher is named `all` unless
        //! otherwise specified by the macro.
        //!
        //! ```rust,no_run
        //! // Example usage to create a struct containing all GPIOs:
        //! for_each_gpio! {
        //!     (all $( ($n:literal, $gpio:ident $_af_ins:tt $_af_outs:tt $_attrs:tt) ),*) => {
        //!         struct Gpios {
        //!             $(
        //!                 #[doc = concat!(" The ", stringify!($n), "th GPIO pin")]
        //!                 pub $gpio: Gpio<$n>,
        //!             )*
        //!         }
        //!     };
        //! }
        //! ```
        #![cfg_attr(docsrs, feature(doc_cfg))]
        #![cfg_attr(not(feature = "build-script"), no_std)]

        #(#chips)*

        #[cfg(any(feature = "build-script", docsrs))]
        include!( "_build_script_utils.rs");
    }
}

pub fn generate_chip_support_status(output: &mut impl Write) -> std::fmt::Result {
    let nothing = "";

    // Calculate the width of the first column.
    let driver_col_width = std::iter::once("Driver")
        .chain(
            PeriConfig::drivers()
                .iter()
                .filter(|i| !i.hide_from_peri_table)
                .map(|i| i.name),
        )
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
        config_group,
        hide_from_peri_table,
    } in PeriConfig::drivers()
    {
        if *hide_from_peri_table {
            continue;
        }
        write!(output, "| {name:driver_col_width$} |")?;
        for chip in Chip::iter() {
            let config = Config::for_chip(&chip);

            let status = config.device.peri_config.support_status(config_group);
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
