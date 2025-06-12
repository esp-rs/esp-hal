use core::str::FromStr;
use std::{fmt::Write, sync::OnceLock};

use anyhow::{Result, bail, ensure};
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
    name: String,
    arch: Arch,
    cores: usize,
    trm: String,

    peripherals: Vec<String>,
    symbols: Vec<String>,
    memory: Vec<MemoryRegion>,

    // Peripheral driver configuration:
    #[serde(flatten)]
    peri_config: PeriConfig,
}

/// Represents a value in the driver configuration.
enum Value {
    Unset,
    /// A numeric value. The generated macro will not include a type suffix
    /// (i.e. will not be generated as `0u32`).
    // TODO: may add a (`name`, str) macro variant in the future if strings are needed.
    Number(u32),
    /// A boolean value. If true, the value is included in the cfg symbols.
    Boolean(bool),
}

impl From<u32> for Value {
    fn from(value: u32) -> Self {
        Value::Number(value)
    }
}
impl From<Option<u32>> for Value {
    fn from(value: Option<u32>) -> Self {
        match value {
            Some(v) => Value::Number(v),
            None => Value::Unset,
        }
    }
}
impl From<bool> for Value {
    fn from(value: bool) -> Self {
        Value::Boolean(value)
    }
}

#[derive(Debug, Default, Clone, Copy, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
enum SupportStatus {
    NotSupported,
    #[default] // Just the common option to reduce visual noise of "declare only" drivers.
    Partial,
    Supported,
}

impl SupportStatus {
    fn icon(self) -> &'static str {
        match self {
            SupportStatus::NotSupported => "❌",
            SupportStatus::Partial => "⚒️",
            SupportStatus::Supported => "✔️",
        }
    }

    fn status(self) -> &'static str {
        match self {
            SupportStatus::NotSupported => "Not supported",
            SupportStatus::Partial => "Partial support",
            SupportStatus::Supported => "Supported",
        }
    }
}

/// An empty configuration, used when a driver just wants to declare that
/// it supports a peripheral, but does not have any configuration options.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
struct EmptyInstanceConfig {}

/// A peripheral instance for which a driver is implemented.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
struct PeriInstance<I> {
    /// The name of the instance
    name: String,
    #[serde(flatten)]
    instance_config: I,
}

struct SupportItem {
    name: &'static str,
    config_group: &'static str,
    symbols: &'static [&'static str],
}

/// Define driver configuration structs, and a PeriConfig struct
/// that contains all of them.
macro_rules! driver_configs {
    // Creates a single struct
    (@one
        $struct:tt($group:ident) {
            $($(#[$meta:meta])? $config:ident: $ty:ty),* $(,)?
        }
    ) => {
        #[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
        struct $struct {
            #[serde(default)]
            status: SupportStatus,
            /// The list of peripherals for which this driver is implemented.
            /// If empty, the driver supports a single instance only.
            #[serde(default)]
            instances: Vec<PeriInstance<EmptyInstanceConfig>>,
            $(
                $(#[$meta])?
                $config: $ty,
            )*
        }

        impl $struct {
            fn support_status(&self) -> SupportStatus {
                self.status
            }

            fn properties(&self) -> impl Iterator<Item = (&str, Value)> {
                [$( // for each property, generate a tuple
                    (
                        /* name: */ concat!(stringify!($group), ".", stringify!($config)),
                        /* value: */ Value::from(self.$config),
                    ),
                )*].into_iter()
            }
        }
    };

    // Repeat pattern for multiple structs
    ($(
        $struct:tt {
            // This name will be emitted as a cfg symbol, to activate a driver.
            driver: $driver:ident,
            // Driver name, used in the generated documentation.
            name: $name:literal,
            // The list of peripheral symbols that this driver supports. For now this is used to
            // double-check the configuration.
            // TODO: remove once the metadata encodes which instances are supported.
            peripherals: $symbols:expr,
            properties: {
                $($tokens:tt)*
            }
        }
    ),+ $(,)?) => {
        // Implement the config driver and DriverConfig trait for each driver
        $(
            driver_configs!(@one $struct($driver) {
                $($tokens)*
            });
        )+

        // Generate a single PeriConfig struct that contains all the drivers. Each of the
        // drivers is optional to support devices that may not have all peripherals.
        #[derive(Default, Debug, Clone, serde::Deserialize, serde::Serialize)]
        struct PeriConfig {
            $(
                // Each driver is an optional struct.
                #[serde(default)]
                $driver: Option<$struct>,
            )+
        }

        impl PeriConfig {
            fn drivers() -> &'static [SupportItem] {
                &[
                    $(
                        SupportItem {
                            name: $name,
                            config_group: stringify!($driver),
                            symbols: $symbols,
                        },
                    )+
                ]
            }

            /// Returns an iterator over all driver names, that are
            /// available on the selected device.
            fn driver_names(&self) -> impl Iterator<Item = &str> {
                // Chain all driver names from each driver.
                std::iter::empty()
                    $(.chain(self.$driver.as_ref().and_then(|d| {
                        if !matches!(d.status, SupportStatus::NotSupported) {
                            Some(stringify!($driver))
                        } else {
                            None
                        }
                    } )))*
            }

            fn driver_instances(&self) -> impl Iterator<Item = String> {
                // Chain all driver instances from each driver.
                std::iter::empty()
                    $(.chain(self.$driver.iter().flat_map(|d| {
                        d.instances.iter().map(|i| format!("{}.{}", stringify!($driver), i.name.as_str()))
                    })))*
            }

            /// Returns an iterator over all properties of all peripherals.
            fn properties(&self) -> impl Iterator<Item = (&str, Value)> {
                // Chain all properties from each driver.
                std::iter::empty()
                    $(.chain(self.$driver.iter().flat_map(|p| p.properties())))*
            }

            /// Returns the support status of a peripheral by its name.
            fn support_status(&self, peripheral: &str) -> Option<SupportStatus> {
                // Find the driver by name and return its support status.
                match peripheral {
                    $(stringify!($driver) => self.$driver.as_ref().map(|p| p.support_status()),)*
                    _ => None, // If the peripheral is not found, return None.
                }
            }
        }
    };
}

// TODO: sort this similar to how the product portfolio is organized
driver_configs![
    AdcProperties {
        driver: adc,
        name: "ADC",
        peripherals: &["adc1", "adc2"],
        properties: {}
    },
    AesProperties {
        driver: aes,
        name: "AES",
        peripherals: &["aes"],
        properties: {}
    },
    AssistDebugProperties {
        driver: assist_debug,
        name: "ASSIST_DEBUG",
        peripherals: &["assist_debug"],
        properties: {}
    },
    DacProperties {
        driver: dac,
        name: "DAC",
        peripherals: &["dac"],
        properties: {}
    },
    DmaProperties {
        driver: dma,
        name: "DMA",
        peripherals: &["pdma", "gdma"],
        properties: {}
    },
    DsProperties {
        driver: ds,
        name: "DS",
        peripherals: &["ds"],
        properties: {}
    },
    EccProperties {
        driver: ecc,
        name: "ECC",
        peripherals: &["ecc"],
        properties: {}
    },
    EthernetProperties {
        driver: ethernet,
        name: "Ethernet",
        peripherals: &["emac"],
        properties: {}
    },
    EtmProperties {
        driver: etm,
        name: "ETM",
        peripherals: &["soc_etm"],
        properties: {}
    },
    GpioProperties {
        driver: gpio,
        name: "GPIO",
        peripherals: &["gpio"],
        properties: {}
    },
    HmacProperties {
        driver: hmac,
        name: "HMAC",
        peripherals: &["hmac"],
        properties: {}
    },
    I2cMasterProperties {
        driver: i2c_master,
        name: "I2C master",
        peripherals: &["i2c0", "i2c1"],
        properties: {
            #[serde(default)]
            has_fsm_timeouts: bool,
            #[serde(default)]
            has_hw_bus_clear: bool,
            #[serde(default)]
            has_bus_timeout_enable: bool,
            #[serde(default)]
            separate_filter_config_registers: bool,
            #[serde(default)]
            can_estimate_nack_reason: bool,
            #[serde(default)]
            has_conf_update: bool,
            #[serde(default)]
            has_reliable_fsm_reset: bool,
            #[serde(default)]
            has_arbitration_en: bool,
            #[serde(default)]
            has_tx_fifo_watermark: bool,
            #[serde(default)]
            bus_timeout_is_exponential: bool,
            #[serde(default)]
            i2c0_data_register_ahb_address: Option<u32>,
            max_bus_timeout: u32,
            ll_intr_mask: u32,
            fifo_size: u32,
        }
    },
    I2cSlaveProperties {
        driver: i2c_slave,
        name: "I2C slave",
        peripherals: &["i2c0", "i2c1"],
        properties: {}
    },
    I2sProperties {
        driver: i2s,
        name: "I2S",
        peripherals: &["i2s0", "i2s1"],
        properties: {}
    },
    InterruptProperties {
        driver: interrupts,
        name: "Interrupts",
        peripherals: &[],
        properties: {}
    },
    IoMuxProperties {
        driver: io_mux,
        name: "IOMUX",
        peripherals: &["io_mux"],
        properties: {}
    },
    CameraProperties {
        driver: camera,
        name: "Camera interface", // LCD_CAM, ESP32 I2S, S2 SPI
        peripherals: &[],
        properties: {}
    },
    RgbProperties {
        driver: rgb_display,
        name: "RGB display", // LCD_CAM, ESP32 I2S, S2 SPI
        peripherals: &[],
        properties: {}
    },
    LedcProperties {
        driver: ledc,
        name: "LEDC",
        peripherals: &["ledc"],
        properties: {}
    },
    McpwmProperties {
        driver: mcpwm,
        name: "MCPWM",
        peripherals: &["mcpwm0", "mcpwm1"],
        properties: {}
    },
    ParlIoProperties {
        driver: parl_io,
        name: "PARL_IO",
        peripherals: &["parl_io"],
        properties: {}
    },
    PcntProperties {
        driver: pcnt,
        name: "PCNT",
        peripherals: &["pcnt"],
        properties: {}
    },
    PsramProperties {
        driver: psram,
        name: "PSRAM",
        peripherals: &["psram"],
        properties: {}
    },
    RmtProperties {
        driver: rmt,
        name: "RMT",
        peripherals: &["rmt"],
        properties: {
            ram_start: u32,
            channel_ram_size: u32,
        }
    },
    RngProperties {
        driver: rng,
        name: "RNG",
        peripherals: &["rng"],
        properties: {}
    },
    RsaProperties {
        driver: rsa,
        name: "RSA",
        peripherals: &["rsa"],
        properties: {}
    },
    SdHostProperties {
        driver: sd_host,
        name: "SDIO host",
        peripherals: &["sdhost"],
        properties: {}
    },
    SdSlaveProperties {
        driver: sd_slave,
        name: "SDIO slave",
        peripherals: &["slchost"],
        properties: {}
    },
    SleepProperties {
        driver: sleep,
        name: "Light/deep sleep",
        peripherals: &[],
        properties: {}
    },
    ShaProperties {
        driver: sha,
        name: "SHA",
        peripherals: &["sha"],
        properties: {}
    },
    SpiMasterProperties {
        driver: spi_master,
        name: "SPI master",
        peripherals: &["spi2", "spi3"],
        properties: {}
    },
    SpiSlaveProperties {
        driver: spi_slave,
        name: "SPI slave",
        peripherals: &["spi2", "spi3"],
        properties: {}
    },
    SysTimerProperties {
        driver: systimer,
        name: "SYSTIMER",
        peripherals: &["systimer"],
        properties: {}
    },
    TempProperties {
        driver: temp_sensor,
        name: "Temperature sensor",
        peripherals: &[],
        properties: {}
    },
    TimersProperties {
        driver: timergroup,
        name: "Timers",
        peripherals: &[],
        properties: {
            #[serde(default)]
            timg_has_timer1: bool,
        }
    },
    TouchProperties {
        driver: touch,
        name: "Touch",
        peripherals: &["touch"],
        properties: {}
    },
    TwaiProperties {
        driver: twai,
        name: "TWAI",
        peripherals: &["twai0", "twai1"],
        properties: {}
    },
    UartProperties {
        driver: uart,
        name: "UART",
        peripherals: &["uart0", "uart1", "uart2"],
        properties: {}
    },
    UlpFsmProperties {
        driver: ulp_fsm,
        name: "ULP (FSM)",
        peripherals: &["ulp_supported"],
        properties: {}
    },
    UlpRiscvProperties {
        driver: ulp_riscv,
        name: "ULP (RISC-V)",
        peripherals: &["ulp_riscv_core", "lp_core"],
        properties: {}
    },
    UsbOtgProperties {
        driver: usb_otg,
        name: "USB OTG FS",
        peripherals: &["usb0"],
        properties: {}
    },
    UsbSerialJtagProperties {
        driver: usb_serial_jtag,
        name: "USB Serial/JTAG",
        peripherals: &["usb_device"],
        properties: {}
    },
    WifiProperties {
        driver: wifi,
        name: "WIFI",
        peripherals: &["wifi"],
        properties: {
            #[serde(default)]
            has_wifi6: bool,
        }
    },
    BluetoothProperties {
        driver: bt,
        name: "Bluetooth",
        peripherals: &["bt"],
        properties: {}
    },
    IeeeProperties {
        driver: ieee802154,
        name: "IEEE 802.15.4",
        peripherals: &["ieee802154"],
        properties: {}
    },
];

// Output a Display-able value as a TokenStream, intended to generate numbers
// without the type suffix.
fn number(n: impl std::fmt::Display) -> TokenStream {
    TokenStream::from_str(&format!("{n}")).unwrap()
}

/// Device configuration file format.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct Config {
    device: Device,
}

impl Config {
    /// The configuration for the specified chip.
    pub fn for_chip(chip: &Chip) -> &Self {
        let config = match chip {
            Chip::Esp32 => include_toml!(Config, "../devices/esp32.toml"),
            Chip::Esp32c2 => include_toml!(Config, "../devices/esp32c2.toml"),
            Chip::Esp32c3 => include_toml!(Config, "../devices/esp32c3.toml"),
            Chip::Esp32c6 => include_toml!(Config, "../devices/esp32c6.toml"),
            Chip::Esp32h2 => include_toml!(Config, "../devices/esp32h2.toml"),
            Chip::Esp32s2 => include_toml!(Config, "../devices/esp32s2.toml"),
            Chip::Esp32s3 => include_toml!(Config, "../devices/esp32s3.toml"),
        };

        config.validate().expect("Invalid device configuration");

        config
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
                peri_config: PeriConfig::default(),
            },
        }
    }

    fn validate(&self) -> Result<()> {
        for instance in self.device.peri_config.driver_instances() {
            let (driver, peri) = instance.split_once('.').unwrap();
            ensure!(
                self.device.peripherals.iter().any(|p| p == peri),
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
    pub fn all(&self) -> impl Iterator<Item = String> + '_ {
        [
            self.device.name.clone(),
            self.device.arch.to_string(),
            match self.cores() {
                Cores::Single => String::from("single_core"),
                Cores::Multi => String::from("multi_core"),
            },
        ]
        .into_iter()
        .chain(self.device.peripherals.iter().cloned())
        .chain(self.device.symbols.iter().cloned())
        .chain(
            self.device
                .peri_config
                .driver_names()
                .map(|name| name.to_string()),
        )
        .chain(self.device.peri_config.driver_instances())
        .chain(
            self.device
                .peri_config
                .properties()
                .filter_map(|(name, value)| match value {
                    Value::Boolean(true) => Some(name.to_string()),
                    _ => None,
                }),
        )
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
            println!("cargo:rustc-cfg={}", symbol.replace('.', "_"));
        }

        // Define env-vars for all memory regions
        for memory in self.memory() {
            println!("cargo:rustc-cfg=has_{}_region", memory.name.to_lowercase());
        }
    }

    pub fn generate_metadata(&self) {
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
            println!("cargo:rustc-check-cfg=cfg({})", symbol.replace('.', "_"));
        }
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
