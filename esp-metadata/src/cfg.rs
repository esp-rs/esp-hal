pub(crate) mod gpio;
pub(crate) mod i2c_master;
pub(crate) mod spi_master;
pub(crate) mod spi_slave;
pub(crate) mod uart;

pub(crate) use gpio::*;
pub(crate) use i2c_master::*;
pub(crate) use spi_master::*;
pub(crate) use spi_slave::*;
pub(crate) use uart::*;

/// Represents a value in the driver configuration.
pub(crate) enum Value {
    Unset,
    /// A numeric value. The generated macro will not include a type suffix
    /// (i.e. will not be generated as `0u32`).
    Number(u32),
    /// A boolean value. If true, the value is included in the cfg symbols.
    Boolean(bool),
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
pub(crate) struct SupportItem {
    /// The human-readable name of the driver in the table (leftmost cell.)
    pub name: &'static str,
    /// The ID of the driver ([device.<config_group>]) in the TOML, that this
    /// item corresponds to.
    pub config_group: &'static str,
    /// When the driver's configuration is not present in the device's TOML,
    /// these symbols decide whether to generate a Not Available or a Not
    /// Supported cell. If the device has one of these symbols, the support
    /// status will be Not Supported (i.e. yet to be implemented).
    pub symbols: &'static [&'static str],
}

/// Define driver configuration structs, and a PeriConfig struct
/// that contains all of them.
macro_rules! driver_configs {
    (@reify $t:tt) => { $t };
    (@property (u32)           $self:ident, $config:ident) => { Value::Number($self.$config) };
    (@property (bool)          $self:ident, $config:ident) => { Value::Boolean($self.$config) };
    (@property (Option<u32>)   $self:ident, $config:ident) => { Value::from($self.$config) };
    (@property ($($other:ty)*) $self:ident, $config:ident) => { Value::Unset };  // Not a property

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
            fn properties(&self) -> impl Iterator<Item = (&str, Value)> {
                [$( // for each property, generate a tuple
                    (
                        /* name: */  concat!(stringify!($group), ".", stringify!($config)),
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
            // The list of peripheral symbols that this driver supports. For now this is used to
            // double-check the configuration.
            // TODO: remove once the metadata encodes which instances are supported.
            peripherals: $symbols:expr,
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
                            symbols: $symbols,
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
            pub fn properties(&self) -> impl Iterator<Item = (&str, Value)> {
                // Collect into a vector. This compiles faster than chaining iterators.
                let mut properties = vec![];
                $(
                    if let Some(driver) = &self.$driver {
                        properties.extend(driver.properties());
                    }
                )*
                properties.into_iter()
            }

            /// Returns the support status of a peripheral by its name.
            pub fn support_status(&self, peripheral: &str) -> Option<SupportStatus> {
                // Find the driver by name and return its support status.
                match peripheral {
                    $(stringify!($driver) => self.$driver.as_ref().map(|p| p.support_status),)*
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
        peripherals: &[],
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
        peripherals: &["etm"],
        properties: {}
    },
    GpioProperties {
        driver: gpio,
        name: "GPIO",
        peripherals: &[], // gpio omitted to prevent stabilizing the singleton
        properties: {
            #[serde(default)]
            has_bank_1: bool,
            gpio_function: u32,
            input_signal_max: u32,
            output_signal_max: u32,
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
        peripherals: &["hmac"],
        properties: {}
    },
    I2cMasterProperties<I2cMasterInstanceConfig> {
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
        properties: {
            status_registers: u32,
        }
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
    SpiMasterProperties<SpiMasterInstanceConfig> {
        driver: spi_master,
        name: "SPI master",
        peripherals: &["spi2", "spi3"],
        properties: {
            #[serde(default)]
            has_octal: bool,
        }
    },
    SpiSlaveProperties<SpiSlaveInstanceConfig> {
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
    UartProperties<UartInstanceConfig> {
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
