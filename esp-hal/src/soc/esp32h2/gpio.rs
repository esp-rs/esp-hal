//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32-H2` chip. It allows you to
//! configure pins as inputs or outputs, set their state and read their state.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
//!   - `io_mux_reg(gpio_num: u8) -> &'static
//!     crate::peripherals::io_mux::GPIO0:`:
//!       * Returns the IO_MUX register for the specified GPIO pin number.
//!   - `gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8`:
//!       * This function enables or disables GPIO interrupts and Non-Maskable
//!         Interrupts (NMI). It takes two boolean arguments int_enable and
//!         nmi_enable to control the interrupt and NMI enable settings. The
//!         function returns an u8 value representing the interrupt enable
//!         settings.
//!   - `gpio` block:
//!       * Defines the pin configurations for various GPIO pins. Each line
//!         represents a pin and its associated options such as input/output
//!         mode, analog capability, and corresponding functions.
//!   - `analog` block:
//!       * Block defines the analog capabilities of various GPIO pins. Each
//!         line represents a pin and its associated options such as mux
//!         selection, function selection, and input enable.
//!   - `enum InputSignal`:
//!       * This enumeration defines input signals for the GPIO mux. Each input
//!         signal is assigned a specific value.
//!   - `enum OutputSignal`:
//!       * This enumeration defines output signals for the GPIO mux. Each
//!         output signal is assigned a specific value.
//!
//! This trait provides functions to read the interrupt status and NMI status
//! registers for both the `PRO CPU` and `APP CPU`. The implementation uses the
//! `gpio` peripheral to access the appropriate registers.

use crate::{gpio::AlternateFunction, peripherals::GPIO};

// https://github.com/espressif/esp-idf/blob/df9310a/components/soc/esp32h2/gpio_periph.c#L42
/// The total number of GPIO pins available.
pub const NUM_PINS: usize = 28;

pub(crate) const FUNC_IN_SEL_OFFSET: usize = 0;

pub(crate) type InputSignalType = u8;
pub(crate) type OutputSignalType = u8;
pub(crate) const OUTPUT_SIGNAL_MAX: u8 = 128;
pub(crate) const INPUT_SIGNAL_MAX: u8 = 124;

pub(crate) const ONE_INPUT: u8 = 0x38;
pub(crate) const ZERO_INPUT: u8 = 0x3c;

pub(crate) const GPIO_FUNCTION: AlternateFunction = AlternateFunction::Function1;

pub(crate) const fn io_mux_reg(gpio_num: u8) -> &'static crate::peripherals::io_mux::GPIO {
    unsafe { (*crate::peripherals::IO_MUX::PTR).gpio(gpio_num as usize) }
}

pub(crate) fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    int_enable as u8 | ((nmi_enable as u8) << 1)
}

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(hidden)]
pub enum InputSignal {
    EXT_ADC_START       = 0,
    U0RXD               = 6,
    U0CTS               = 7,
    U0DSR               = 8,
    U1RXD               = 9,
    U1CTS               = 10,
    U1DSR               = 11,
    I2S_MCLK            = 12,
    I2SO_BCK            = 13,
    I2SO_WS             = 14,
    I2SI_SD             = 15,
    I2SI_BCK            = 16,
    I2SI_WS             = 17,
    USB_JTAG_TDO_BRIDGE = 19,
    CPU_GPIO0           = 28,
    CPU_GPIO1           = 29,
    CPU_GPIO2           = 30,
    CPU_GPIO3           = 31,
    CPU_GPIO4           = 32,
    CPU_GPIO5           = 33,
    CPU_GPIO6           = 34,
    CPU_GPIO7           = 35,
    I2CEXT0_SCL         = 45,
    I2CEXT0_SDA         = 46,
    PARL_RX_DATA0       = 47,
    PARL_RX_DATA1       = 48,
    PARL_RX_DATA2       = 49,
    PARL_RX_DATA3       = 50,
    PARL_RX_DATA4       = 51,
    PARL_RX_DATA5       = 52,
    PARL_RX_DATA6       = 53,
    PARL_RX_DATA7       = 54,
    I2CEXT1_SCL         = 55,
    I2CEXT1_SDA         = 56,
    FSPICLK             = 63,
    FSPIQ               = 64,
    FSPID               = 65,
    FSPIHD              = 66,
    FSPIWP              = 67,
    FSPICS0             = 68,
    PARL_RX_CLK         = 69,
    PARL_TX_CLK         = 70,
    RMT_SIG_0           = 71,
    RMT_SIG_1           = 72,
    TWAI0_RX            = 73,
    PWM0_SYNC0          = 87,
    PWM0_SYNC1          = 88,
    PWM0_SYNC2          = 89,
    PWM0_F0             = 90,
    PWM0_F1             = 91,
    PWM0_F2             = 92,
    PWM0_CAP0           = 93,
    PWM0_CAP1           = 94,
    PWM0_CAP2           = 95,
    SIG_FUNC_97         = 97,
    SIG_FUNC_98         = 98,
    SIG_FUNC_99         = 99,
    SIG_FUNC_100        = 100,
    PCNT0_SIG_CH0       = 101,
    PCNT0_SIG_CH1       = 102,
    PCNT0_CTRL_CH0      = 103,
    PCNT0_CTRL_CH1      = 104,
    PCNT1_SIG_CH0       = 105,
    PCNT1_SIG_CH1       = 106,
    PCNT1_CTRL_CH0      = 107,
    PCNT1_CTRL_CH1      = 108,
    PCNT2_SIG_CH0       = 109,
    PCNT2_SIG_CH1       = 110,
    PCNT2_CTRL_CH0      = 111,
    PCNT2_CTRL_CH1      = 112,
    PCNT3_SIG_CH0       = 113,
    PCNT3_SIG_CH1       = 114,
    PCNT3_CTRL_CH0      = 115,
    PCNT3_CTRL_CH1      = 116,
    SPIQ                = 121,
    SPID                = 122,
    SPIHD               = 123,
    SPIWP               = 124,
}

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(hidden)]
pub enum OutputSignal {
    LEDC_LS_SIG0     = 0,
    LEDC_LS_SIG1     = 1,
    LEDC_LS_SIG2     = 2,
    LEDC_LS_SIG3     = 3,
    LEDC_LS_SIG4     = 4,
    LEDC_LS_SIG5     = 5,
    U0TXD            = 6,
    U0RTS            = 7,
    U0DTR            = 8,
    U1TXD            = 9,
    U1RTS            = 10,
    U1DTR            = 11,
    I2S_MCLK         = 12,
    I2SO_BCK         = 13,
    I2SO_WS          = 14,
    I2SO_SD          = 15,
    I2SI_BCK         = 16,
    I2SI_WS          = 17,
    I2SO_SD1         = 18,
    USB_JTAG_TRST    = 19,
    CPU_GPIO_OUT0    = 28,
    CPU_GPIO_OUT1    = 29,
    CPU_GPIO_OUT2    = 30,
    CPU_GPIO_OUT3    = 31,
    CPU_GPIO_OUT4    = 32,
    CPU_GPIO_OUT5    = 33,
    CPU_GPIO_OUT6    = 34,
    CPU_GPIO_OUT7    = 35,
    I2CEXT0_SCL      = 45,
    I2CEXT0_SDA      = 46,
    PARL_TX_DATA0    = 47,
    PARL_TX_DATA1    = 48,
    PARL_TX_DATA2    = 49,
    PARL_TX_DATA3    = 50,
    PARL_TX_DATA4    = 51,
    PARL_TX_DATA5    = 52,
    PARL_TX_DATA6    = 53,
    PARL_TX_DATA7    = 54,
    I2CEXT1_SCL      = 55,
    I2CEXT1_SDA      = 56,
    FSPICLK_MUX      = 63,
    FSPIQ            = 64,
    FSPID            = 65,
    FSPIHD           = 66,
    FSPIWP           = 67,
    FSPICS0          = 68,
    PARL_RX_CLK      = 69,
    PARL_TX_CLK      = 70,
    RMT_SIG_0        = 71,
    RMT_SIG_1        = 72,
    TWAI0_TX         = 73,
    TWAI0_BUS_OFF_ON = 74,
    TWAI0_CLKOUT     = 75,
    TWAI0_STANDBY    = 76,
    CTE_ANT7         = 78,
    CTE_ANT8         = 79,
    CTE_ANT9         = 80,
    GPIO_SD0         = 83,
    GPIO_SD1         = 84,
    GPIO_SD2         = 85,
    GPIO_SD3         = 86,
    PWM0_0A          = 87,
    PWM0_0B          = 88,
    PWM0_1A          = 89,
    PWM0_1B          = 90,
    PWM0_2A          = 91,
    PWM0_2B          = 92,
    SIG_IN_FUNC97    = 97,
    SIG_IN_FUNC98    = 98,
    SIG_IN_FUNC99    = 99,
    SIG_IN_FUNC100   = 100,
    FSPICS1          = 101,
    FSPICS2          = 102,
    FSPICS3          = 103,
    FSPICS4          = 104,
    FSPICS5          = 105,
    CTE_ANT10        = 106,
    CTE_ANT11        = 107,
    CTE_ANT12        = 108,
    CTE_ANT13        = 109,
    CTE_ANT14        = 110,
    CTE_ANT15        = 111,
    SPICLK           = 114,
    SPICS0           = 115,
    SPICS1           = 116,
    SPIQ             = 121,
    SPID             = 122,
    SPIHD            = 123,
    SPIWP            = 124,
    CLK_OUT_OUT1     = 125,
    CLK_OUT_OUT2     = 126,
    CLK_OUT_OUT3     = 127,
    GPIO             = 128,
}

#[derive(Clone, Copy)]
pub(crate) enum InterruptStatusRegisterAccess {
    Bank0,
}

impl InterruptStatusRegisterAccess {
    pub(crate) fn interrupt_status_read(self) -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_int().read().bits()
    }
}
