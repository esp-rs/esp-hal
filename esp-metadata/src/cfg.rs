pub(crate) mod aes;
pub(crate) mod gpio;
pub(crate) mod i2c_master;
pub(crate) mod rmt;
pub(crate) mod rsa;
pub(crate) mod sha;
pub(crate) mod soc;
pub(crate) mod spi_master;
pub(crate) mod spi_slave;
pub(crate) mod uart;

pub(crate) use aes::*;
pub(crate) use gpio::*;
pub(crate) use i2c_master::*;
pub(crate) use rmt::*;
pub(crate) use sha::*;
pub(crate) use spi_master::*;
pub(crate) use spi_slave::*;
pub(crate) use uart::*;

pub(crate) trait GenericProperty {
    fn cfgs(&self) -> Option<Vec<String>> {
        None
    }

    fn for_each_macro(&self) -> Option<proc_macro2::TokenStream> {
        None
    }

    fn property_macro_branches(&self) -> proc_macro2::TokenStream {
        quote::quote! {}
    }
}

/// Represents a value in the driver configuration.
pub(crate) enum Value {
    Unset,
    /// A numeric value. The generated macro will not include a type suffix
    /// (i.e. will not be generated as `0u32`).
    Number(u32),
    /// A boolean value. If true, the value is included in the cfg symbols.
    Boolean(bool),
    /// A string.
    String(String),
    /// A list of numeric values. A for-each macro is generated for the list.
    NumberList(Vec<u32>),
    /// A list of strings. A separate symbol is generated for each string.
    StringList(Vec<String>),
    /// A generic object.
    Generic(Box<dyn GenericProperty>),
}

impl From<Option<u32>> for Value {
    fn from(value: Option<u32>) -> Self {
        match value {
            Some(v) => Value::Number(v),
            None => Value::Unset,
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum SupportStatus {
    NotSupported,
    #[default] // Just the common option to reduce visual noise of "declare only" drivers.
    Partial,
    Supported,
}

impl SupportStatus {
    pub fn icon(self) -> &'static str {
        match self {
            SupportStatus::NotSupported => "❌",
            SupportStatus::Partial => "⚒️",
            SupportStatus::Supported => "✔️",
        }
    }

    pub fn status(self) -> &'static str {
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
pub(crate) struct EmptyInstanceConfig {}

/// A peripheral instance for which a driver is implemented.
#[derive(Debug, Default, Clone, serde::Deserialize, serde::Serialize)]
pub(crate) struct PeriInstance<I = EmptyInstanceConfig> {
    /// The name of the instance
    pub name: String,
    #[serde(flatten)]
    pub instance_config: I,
}

/// A single cell in the peripheral support table.
#[derive(Default)]
pub(crate) struct SupportItem {
    /// The human-readable name of the driver in the table (leftmost cell.)
    pub name: &'static str,
    /// The ID of the driver ([device.<config_group>]) in the TOML, that this
    /// item corresponds to.
    pub config_group: &'static str,
    /// If true, this driver is not shown in the peripheral support table.
    pub hide_from_peri_table: bool,
}

/// Define driver configuration structs, and a PeriConfig struct
/// that contains all of them.
macro_rules! driver_configs {
    (@ignore $t:tt) => {};
    (@property (u32)           $self:ident, $config:ident) => { Value::Number($self.$config) };
    (@property (bool)          $self:ident, $config:ident) => { Value::Boolean($self.$config) };
    (@property (String)        $self:ident, $config:ident) => { Value::String($self.$config.clone()) };
    (@property (Vec<u32>)      $self:ident, $config:ident) => { Value::NumberList($self.$config.clone()) };
    (@property (Vec<String>)   $self:ident, $config:ident) => { Value::StringList($self.$config.clone()) };
    (@property (Option<u32>)   $self:ident, $config:ident) => { Value::from($self.$config) };
    (@property ($($other:ty)*) $self:ident, $config:ident) => { Value::Generic(Box::new($self.$config.clone())) };
    (@is_optional Option<$t:ty>) => { true };
    (@is_optional $t:ty) => { false };

    (@default $default:literal) => { $default };
    (@default $default:literal $opt:literal) => { $opt };

    // Creates a single struct
    (@one
        $struct:ident $(<$instance_config:ident>)? ($group:ident) {
            $(
                $(#[$meta:meta])? $config:ident: $ty:tt $(<$generic:tt>)?,
            )*
        }
    ) => {
        #[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
        pub(crate) struct $struct {
            #[serde(default)]
            pub support_status: SupportStatus,
            // The list of peripherals for which this driver is implemented.
            // If empty, the driver supports a single instance only.
            #[serde(default)]
            pub instances: Vec<PeriInstance $(<$instance_config>)?>,
            $(
                $(#[$meta])?
                pub $config: $ty $(<$generic>)?
            ),*
        }

        impl $struct {
            fn properties(&self) -> impl Iterator<Item = (&str, bool, Value)> {
                [$( // for each property, generate a tuple
                    (
                        /* name: */  concat!(stringify!($group), ".", stringify!($config)),
                        /* is_optional: */ driver_configs!(@is_optional $ty $(<$generic>)?),
                        /* value: */ driver_configs!(@property ($ty $(<$generic>)?) self, $config),
                    ),
                )*].into_iter()
            }
        }
    };

    // Repeat pattern for multiple structs
    ($(
        $struct:ident $(<$instance_config:ident>)? {
            // This name will be emitted as a cfg symbol, to activate a driver.
            driver: $driver:ident,

            // Driver name, used in the generated documentation.
            name: $name:literal,

            $(hide_from_peri_table: $hide:literal,)?

            // When set, the type must provide `fn computed_properties(&self) -> impl Iterator<Item = (&str, bool, Value)>`.
            // The iterator yields `(name_with_prefix, optional, value)`.
            $(has_computed_properties: $computed:literal,)?

            properties: $tokens:tt
        },
    )+) => {
        // Implement the config driver and DriverConfig trait for each driver
        $(
            driver_configs!(@one $struct $(<$instance_config>)? ($driver) $tokens);
        )+

        // Generate a single PeriConfig struct that contains all the drivers. Each of the
        // drivers is optional to support devices that may not have all peripherals.
        #[derive(Default, Debug, Clone, serde::Deserialize, serde::Serialize)]
        pub(crate) struct PeriConfig {
            $(
                // Each driver is an optional struct.
                #[serde(default)]
                pub(crate) $driver: Option<$struct>,
            )+
        }

        impl PeriConfig {
            pub fn drivers() -> &'static [SupportItem] {
                &[
                    $(
                        SupportItem {
                            name: $name,
                            config_group: stringify!($driver),
                            hide_from_peri_table: driver_configs!(@default false $($hide)?),
                        },
                    )+
                ]
            }

            /// Returns an iterator over all driver names, that are
            /// available on the selected device.
            pub fn driver_names(&self) -> impl Iterator<Item = &str> {
                [$(
                    self.$driver.as_ref().and_then(|d| {
                        match d.support_status {
                            SupportStatus::NotSupported => None,
                            _ => Some(stringify!($driver)),
                        }
                    }),
                )*].into_iter().flatten()
            }

            pub fn driver_instances(&self) -> impl Iterator<Item = String> {
                // Collect into a vector. This compiles faster than chaining iterators.
                let mut instances = vec![];
                $(
                    if let Some(driver) = &self.$driver {
                        instances.extend(driver.instances.iter().map(|i| {
                            format!("{}.{}", stringify!($driver), i.name)
                        }));
                    }
                )*
                instances.into_iter()
            }

            /// Returns an iterator over all properties of all peripherals.
            ///
            /// (property name, optional?, value)
            pub fn properties(&self) -> impl Iterator<Item = (&str, bool, Value)> {
                // Collect into a vector. This compiles faster than chaining iterators.
                let mut properties = vec![];
                $(
                    if let Some(driver) = &self.$driver {
                        properties.extend(driver.properties());
                        $(
                            driver_configs!(@ignore $computed);
                            properties.extend(driver.computed_properties());
                        )?
                    }
                )*
                properties.into_iter()
            }

            /// Returns the support status of a peripheral by its name.
            pub fn support_status(&self, driver: &str) -> Option<SupportStatus> {
                match driver {
                    $(stringify!($driver) => self.$driver.as_ref().map(|p| p.support_status),)*
                    _ => None, // If the peripheral is not found, return None.
                }
            }

            /// Returns the peripheral instances used by the given driver.
            pub fn driver_peris<'a>(&'a self, driver: &str) -> Vec<&'a str> {
                match driver {
                    $(stringify!($driver) => self.$driver.iter().flat_map(|p| p.instances.iter().map(|i| i.name.as_str())).collect::<Vec<_>>(),)*
                    _ => vec![],
                }
            }
        }
    };
}

// TODO: sort this similar to how the product portfolio is organized
driver_configs![
    SocProperties {
        driver: soc,
        name: "SOC",
        hide_from_peri_table: true,
        has_computed_properties: true,
        properties: {
            #[serde(default)]
            cpu_has_csr_pc: bool,
            #[serde(default)]
            cpu_has_prv_mode: bool,
            #[serde(default)]
            ref_tick_hz: Option<u32>,
            #[serde(default)]
            rc_fast_clk_default: Option<u32>,
            #[serde(default)]
            rc_slow_clock: Option<u32>,
            xtal_options: Vec<u32>,
        }
    },

    AdcProperties {
        driver: adc,
        name: "ADC",
        properties: {}
    },
    AesProperties {
        driver: aes,
        name: "AES",
        properties: {
            key_length: AesKeyLength,
            #[serde(default)]
            dma: bool,
            #[serde(default)]
            dma_mode: Vec<String>,
            has_split_text_registers: bool,
            endianness_configurable: bool,
        }
    },
    AssistDebugProperties {
        driver: assist_debug,
        name: "ASSIST_DEBUG",
        properties: {
            #[serde(default)]
            has_sp_monitor: bool,
            #[serde(default)]
            has_region_monitor: bool,
        }
    },
    DacProperties {
        driver: dac,
        name: "DAC",
        properties: {}
    },
    DmaProperties {
        driver: dma,
        name: "DMA",
        properties: {}
    },
    DsProperties {
        driver: ds,
        name: "DS",
        properties: {}
    },
    EccProperties {
        driver: ecc,
        name: "ECC",
        properties: {}
    },
    EthernetProperties {
        driver: ethernet,
        name: "Ethernet",
        properties: {}
    },
    EtmProperties {
        driver: etm,
        name: "ETM",
        properties: {}
    },
    GpioProperties {
        driver: gpio,
        name: "GPIO",
        has_computed_properties: true,
        properties: {
            #[serde(default)]
            has_bank_1: bool,
            gpio_function: u32,
            constant_0_input: u32,
            constant_1_input: u32,
            #[serde(default)]
            remap_iomux_pin_registers: bool,
            #[serde(default)] // currently 0 in all devices
            func_in_sel_offset: u32,

            #[serde(flatten)]
            pins_and_signals: GpioPinsAndSignals,
        }
    },
    HmacProperties {
        driver: hmac,
        name: "HMAC",
        properties: {}
    },
    I2cMasterProperties<I2cMasterInstanceConfig> {
        driver: i2c_master,
        name: "I2C master",
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
    LpI2cMasterProperties {
        driver: lp_i2c_master,
        name: "LP I2C master",
        properties: {
            fifo_size: u32,
        }
    },
    I2cSlaveProperties {
        driver: i2c_slave,
        name: "I2C slave",
        properties: {}
    },
    I2sProperties {
        driver: i2s,
        name: "I2S",
        properties: {}
    },
    InterruptProperties {
        driver: interrupts,
        name: "Interrupts",
        properties: {
            status_registers: u32,
        }
    },
    IoMuxProperties {
        driver: io_mux,
        name: "IOMUX",
        properties: {}
    },
    CameraProperties {
        driver: camera,
        name: "Camera interface", // LCD_CAM, ESP32 I2S, S2 SPI
        properties: {}
    },
    RgbProperties {
        driver: rgb_display,
        name: "RGB display", // LCD_CAM, ESP32 I2S, S2 SPI
        properties: {}
    },
    LedcProperties {
        driver: ledc,
        name: "LEDC",
        properties: {}
    },
    McpwmProperties {
        driver: mcpwm,
        name: "MCPWM",
        properties: {}
    },
    ParlIoProperties {
        driver: parl_io,
        name: "PARL_IO",
        properties: {}
    },
    PcntProperties {
        driver: pcnt,
        name: "PCNT",
        properties: {}
    },
    PsramProperties {
        driver: psram,
        name: "PSRAM",
        properties: {}
    },
    RmtProperties {
        driver: rmt,
        name: "RMT",
        properties: {
            ram_start: u32,
            channel_ram_size: u32,
            channels: RmtChannelConfig,
            #[serde(default)]
            has_tx_immediate_stop: bool,
            #[serde(default)]
            has_tx_loop_count: bool,
            #[serde(default)]
            has_tx_loop_auto_stop: bool,
            #[serde(default)]
            has_tx_carrier_data_only: bool,
            #[serde(default)]
            has_tx_sync: bool,
            #[serde(default)]
            has_rx_wrap: bool,
            #[serde(default)]
            has_rx_demodulation: bool,
            #[serde(default)]
            has_dma: bool,
            #[serde(default)]
            has_per_channel_clock: bool,
            clock_sources: RmtClockSourcesConfig,
            max_idle_threshold: u32,
        }
    },
    RngProperties {
        driver: rng,
        name: "RNG",
        properties: {
            apb_cycle_wait_num: u32,
        }
    },
    RsaProperties {
        driver: rsa,
        name: "RSA",
        has_computed_properties: true,
        properties: {
            size_increment: u32,
            memory_size_bytes: u32,
        }
    },
    SdHostProperties {
        driver: sd_host,
        name: "SDIO host",
        properties: {}
    },
    SdSlaveProperties {
        driver: sd_slave,
        name: "SDIO slave",
        properties: {}
    },
    SleepProperties {
        driver: sleep,
        name: "Light/deep sleep",
        properties: {}
    },
    ShaProperties {
        driver: sha,
        name: "SHA",
        properties: {
            #[serde(default)]
            dma: bool,
            #[serde(default)]
            algo: ShaAlgoMap,
        }
    },
    SpiMasterProperties<SpiMasterInstanceConfig> {
        driver: spi_master,
        name: "SPI master",
        properties: {
            #[serde(default)]
            has_octal: bool,
        }
    },
    SpiSlaveProperties<SpiSlaveInstanceConfig> {
        driver: spi_slave,
        name: "SPI slave",
        properties: {}
    },
    SysTimerProperties {
        driver: systimer,
        name: "SYSTIMER",
        properties: {}
    },
    TempProperties {
        driver: temp_sensor,
        name: "Temperature sensor",
        properties: {}
    },
    TimersProperties {
        driver: timergroup,
        name: "Timers",
        properties: {
            #[serde(default)]
            timg_has_timer1: bool,
            #[serde(default)]
            timg_has_divcnt_rst: bool,
            #[serde(default)]
            default_clock_source: Option<u32>,
            #[serde(default)]
            default_wdt_clock_source: Option<u32>,
        }
    },
    TouchProperties {
        driver: touch,
        name: "Touch",
        properties: {}
    },
    TwaiProperties {
        driver: twai,
        name: "TWAI",
        properties: {}
    },
    UartProperties<UartInstanceConfig> {
        driver: uart,
        name: "UART",
        properties: {
            ram_size: u32,
        }
    },
    LpUartProperties {
        driver: lp_uart,
        name: "LP UART",
        properties: {
            ram_size: u32,
        }
    },
    UlpFsmProperties {
        driver: ulp_fsm,
        name: "ULP (FSM)",
        properties: {}
    },
    UlpRiscvProperties {
        driver: ulp_riscv,
        name: "ULP (RISC-V)",
        properties: {}
    },
    UsbOtgProperties {
        driver: usb_otg,
        name: "USB OTG FS",
        properties: {}
    },
    UsbSerialJtagProperties {
        driver: usb_serial_jtag,
        name: "USB Serial/JTAG",
        properties: {}
    },
    WifiProperties {
        driver: wifi,
        name: "WIFI",
        properties: {
            #[serde(default)]
            has_wifi6: bool,
        }
    },
    BluetoothProperties {
        driver: bt,
        name: "Bluetooth",
        properties: {
            controller: String,
        }
    },
    IeeeProperties {
        driver: ieee802154,
        name: "IEEE 802.15.4",
        properties: {}
    },
    PhyProperties {
        driver: phy,
        name: "PHY",
        properties: {
            #[serde(default)]
            combo_module: bool,
            #[serde(default)]
            backed_up_digital_register_count: Option<u32>,
        }
    },
];
