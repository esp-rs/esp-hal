//! # GPIO configuration module (ESP32-C2)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32-C2` chip. It allows you to
//! configure pins as inputs or outputs, set their state and read their state.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
//!   - `get_io_mux_reg(gpio_num: u8) -> &'static
//!     crate::peripherals::io_mux::GPIO`:
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
use crate::{
    gpio::{AlternateFunction, GpioPin},
    peripherals::GPIO,
};

/// The total number of GPIO pins available.
pub const NUM_PINS: usize = 21;

pub(crate) const FUNC_IN_SEL_OFFSET: usize = 0;

pub(crate) type OutputSignalType = u8;
pub(crate) const OUTPUT_SIGNAL_MAX: u8 = 128;
pub(crate) const INPUT_SIGNAL_MAX: u8 = 100;

pub(crate) const ONE_INPUT: u8 = 0x1e;
pub(crate) const ZERO_INPUT: u8 = 0x1f;

pub(crate) const GPIO_FUNCTION: AlternateFunction = AlternateFunction::Function1;

pub(crate) const fn get_io_mux_reg(gpio_num: u8) -> &'static crate::peripherals::io_mux::GPIO {
    unsafe { (*crate::peripherals::IO_MUX::PTR).gpio(gpio_num as usize) }
}

pub(crate) fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    int_enable as u8 | ((nmi_enable as u8) << 1)
}

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
#[doc(hidden)]
pub enum InputSignal {
    SPIQ          = 0,
    SPID          = 1,
    SPIHD         = 2,
    SPIWP         = 3,
    U0RXD         = 6,
    U0CTS         = 7,
    U0DSR         = 8,
    U1RXD         = 9,
    U1CTS         = 10,
    U1DSR         = 11,
    CPU_GPIO_0    = 28,
    CPU_GPIO_1    = 29,
    CPU_GPIO_2    = 30,
    CPU_GPIO_3    = 31,
    CPU_GPIO_4    = 32,
    CPU_GPIO_5    = 33,
    CPU_GPIO_6    = 34,
    CPU_GPIO_7    = 35,
    EXT_ADC_START = 45,
    RMT_SIG_0     = 51,
    RMT_SIG_1     = 52,
    I2CEXT0_SCL   = 53,
    I2CEXT0_SDA   = 54,
    FSPICLK       = 63,
    FSPIQ         = 64,
    FSPID         = 65,
    FSPIHD        = 66,
    FSPIWP        = 67,
    FSPICS0       = 68,
    SIG_FUNC_97   = 97,
    SIG_FUNC_98   = 98,
    SIG_FUNC_99   = 99,
    SIG_FUNC_100  = 100,
}

/// Peripheral output signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
#[doc(hidden)]
pub enum OutputSignal {
    SPIQ          = 0,
    SPID          = 1,
    SPIHD         = 2,
    SPIWP         = 3,
    SPICLK_MUX    = 4,
    SPICS0        = 5,
    U0TXD         = 6,
    U0RTS         = 7,
    U0DTR         = 8,
    U1TXD         = 9,
    U1RTS         = 10,
    U1DTR         = 11,
    SPIQ_MONITOR  = 15,
    SPID_MONITOR  = 16,
    SPIHD_MONITOR = 17,
    SPIWP_MONITOR = 18,
    SPICS1        = 19,
    CPU_GPIO_0    = 28,
    CPU_GPIO_1    = 29,
    CPU_GPIO_2    = 30,
    CPU_GPIO_3    = 31,
    CPU_GPIO_4    = 32,
    CPU_GPIO_5    = 33,
    CPU_GPIO_6    = 34,
    CPU_GPIO_7    = 35,
    LEDC_LS_SIG0  = 45,
    LEDC_LS_SIG1  = 46,
    LEDC_LS_SIG2  = 47,
    LEDC_LS_SIG3  = 48,
    LEDC_LS_SIG4  = 49,
    LEDC_LS_SIG5  = 50,
    RMT_SIG_0     = 51,
    RMT_SIG_1     = 52,
    I2CEXT0_SCL   = 53,
    I2CEXT0_SDA   = 54,
    FSPICLK_MUX   = 63,
    FSPIQ         = 64,
    FSPID         = 65,
    FSPIHD        = 66,
    FSPIWP        = 67,
    FSPICS0       = 68,
    FSPICS1       = 69,
    FSPICS3       = 70,
    FSPICS2       = 71,
    FSPICS4       = 72,
    FSPICS5       = 73,
    ANT_SEL0      = 89,
    ANT_SEL1      = 90,
    ANT_SEL2      = 91,
    ANT_SEL3      = 92,
    ANT_SEL4      = 93,
    ANT_SEL5      = 94,
    ANT_SEL6      = 95,
    ANT_SEL7      = 96,
    SIG_FUNC_97   = 97,
    SIG_FUNC_98   = 98,
    SIG_FUNC_99   = 99,
    SIG_FUNC_100  = 100,
    CLK_OUT1      = 123,
    CLK_OUT2      = 124,
    CLK_OUT3      = 125,
    GPIO          = 128,
}

crate::gpio! {
    (0, 0, InputOutputAnalog)
    (1, 0, InputOutputAnalog)
    (2, 0, InputOutputAnalog (2 => FSPIQ) (2 => FSPIQ))
    (3, 0, InputOutputAnalog)
    (4, 0, InputOutputAnalog (2 => FSPIHD) (2 => FSPIHD))
    (5, 0, InputOutput (2 => FSPIWP) (2 => FSPIWP))
    (6, 0, InputOutput (2 => FSPICLK) (2 => FSPICLK_MUX))
    (7, 0, InputOutput (2 => FSPID) (2 => FSPID))
    (8, 0, InputOutput)
    (9, 0, InputOutput)
    (10, 0, InputOutput (2 => FSPICS0) (2 => FSPICS0))
    (18, 0, InputOutput)
    (19, 0, InputOutput)
    (20, 0, InputOutput (0 => U0RXD) ())
}

crate::rtc_pins! {
    0
    1
    2
    3
    4
    5
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
