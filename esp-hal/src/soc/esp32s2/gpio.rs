//! # GPIO configuration module (ESP32-S2)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32-S2` chip. It allows you to
//! configure pins as inputs or outputs, set their state and read their state.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
//!   - `get_io_mux_reg(gpio_num: u8) -> &'static
//!     crate::peripherals::io_mux::GPIO0:`:
//!       * This function returns a reference to the GPIO register associated
//!         with the given GPIO number. It uses unsafe code and transmutation to
//!         access the GPIO registers based on the provided GPIO number.
//!   - `gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8`:
//!       * This function enables or disables GPIO interrupts and Non-Maskable
//!         Interrupts (NMI). It takes two boolean arguments int_enable and
//!         nmi_enable to control the interrupt and NMI enable settings. The
//!         function returns an u8 value representing the interrupt enable
//!         settings.
//!   - `impl_get_rtc_pad`:
//!       * This macro_rule generates a function to get a specific RTC pad. It
//!         takes a single argument `$pad_name`, which is an identifier
//!         representing the name of the pad. Returns a reference to the
//!         corresponding RTC pad.
//!   - `impl_get_rtc_pad_indexed`:
//!       * This macro_rule generates a function similar to the previous one but
//!         for indexed RTC pads. It takes two arguments: `$pad_name`, which
//!         represents the name of the pad, and `$idx`, which is the index of
//!         the specific pad. Returns a reference to the indexed RTC pad.
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
//! This module also implements the `InterruptStatusRegisterAccess` trait for
//! two different banks:
//!   * `InterruptStatusRegisterAccessBank0`
//!   * `InterruptStatusRegisterAccessBank1`.
//!
//! This trait provides functions to read the interrupt status and NMI status
//! registers for both the `PRO CPU` and `APP CPU`. The implementation uses the
//! `gpio` peripheral to access the appropriate registers.

use crate::{
    gpio::{
        AlternateFunction,
        GpioPin,
        InterruptStatusRegisterAccess,
        InterruptStatusRegisterAccessBank0,
        InterruptStatusRegisterAccessBank1,
    },
    peripherals::GPIO,
};

pub const NUM_PINS: usize = 47;

pub(crate) const FUNC_IN_SEL_OFFSET: usize = 0;

pub(crate) type OutputSignalType = u16;
pub(crate) const OUTPUT_SIGNAL_MAX: u16 = 256;
pub(crate) const INPUT_SIGNAL_MAX: u16 = 204;

pub(crate) const ONE_INPUT: u8 = 0x38;
pub(crate) const ZERO_INPUT: u8 = 0x3c;

pub(crate) const GPIO_FUNCTION: AlternateFunction = AlternateFunction::Function1;

pub(crate) const fn get_io_mux_reg(gpio_num: u8) -> &'static crate::peripherals::io_mux::GPIO0 {
    unsafe {
        let iomux = &*crate::peripherals::IO_MUX::PTR;

        match gpio_num {
            0 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO0,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio0()),
            1 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO1,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio1()),
            2 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO2,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio2()),
            3 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO3,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio3()),
            4 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO4,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio4()),
            5 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO5,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio5()),
            6 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO6,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio6()),
            7 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO7,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio7()),
            8 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO8,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio8()),
            9 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO9,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio9()),
            10 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO10,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio10()),
            11 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO11,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio11()),
            12 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO12,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio12()),
            13 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO13,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio13()),
            14 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO14,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio14()),
            15 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO15,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio15()),
            16 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO16,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio16()),
            17 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO17,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio17()),
            18 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO18,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio18()),
            19 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO19,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio19()),
            20 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO20,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio20()),
            21 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO21,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio21()),
            26 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO26,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio26()),
            27 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO27,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio27()),
            32 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO32,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio32()),
            33 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO33,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio33()),
            34 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO34,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio34()),
            35 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO35,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio35()),
            36 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO36,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio36()),
            37 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO37,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio37()),
            38 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO38,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio38()),
            39 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO39,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio39()),
            40 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO40,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio40()),
            41 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO41,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio41()),
            42 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO42,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio42()),
            43 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO43,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio43()),
            44 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO44,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio44()),
            45 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO45,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio45()),
            46 => core::mem::transmute::<
                &'static crate::peripherals::io_mux::GPIO46,
                &'static crate::peripherals::io_mux::GPIO0,
            >(iomux.gpio46()),
            _ => ::core::unreachable!(),
        }
    }
}

pub(crate) fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    int_enable as u8
        | ((nmi_enable as u8) << 1)
        | (int_enable as u8) << 2
        | ((nmi_enable as u8) << 3)
}

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
#[doc(hidden)]
pub enum InputSignal {
    SPIQ              = 0,
    SPID              = 1,
    SPIHD             = 2,
    SPIWP             = 3,
    SPID4             = 7,
    SPID5             = 8,
    SPID6             = 9,
    SPID7             = 10,
    SPIDQS            = 11,
    U0RXD             = 14,
    U0CTS             = 15,
    U0DSR             = 16,
    U1RXD             = 17,
    U1CTS             = 18,
    U1DSR             = 21,
    I2S0O_BCK         = 23,
    I2S0O_WS          = 25,
    I2S0I_BCK         = 27,
    I2S0I_WS          = 28,
    I2CEXT0_SCL       = 29,
    I2CEXT0_SDA       = 30,
    PCNT0_SIG_CH0     = 39,
    PCNT0_SIG_CH1     = 40,
    PCNT0_CTRL_CH0    = 41,
    PCNT0_CTRL_CH1    = 42,
    PCNT1_SIG_CH0     = 43,
    PCNT1_SIG_CH1     = 44,
    PCNT1_CTRL_CH0    = 45,
    PCNT1_CTRL_CH1    = 46,
    PCNT2_SIG_CH0     = 47,
    PCNT2_SIG_CH1     = 48,
    PCNT2_CTRL_CH0    = 49,
    PCNT2_CTRL_CH1    = 50,
    PCNT3_SIG_CH0     = 51,
    PCNT3_SIG_CH1     = 52,
    PCNT3_CTRL_CH0    = 53,
    PCNT3_CTRL_CH1    = 54,
    USB_EXTPHY_VP     = 61,
    USB_EXTPHY_VM     = 62,
    USB_EXTPHY_RCV    = 63,
    USB_OTG_IDDIG     = 64,
    USB_OTG_AVALID    = 65,
    USB_SRP_BVALID    = 66,
    USB_OTG_VBUSVALID = 67,
    USB_SRP_SESSEND   = 68,
    SPI3_CLK          = 72,
    SPI3_Q            = 73,
    SPI3_D            = 74,
    SPI3_HD           = 75,
    SPI3_CS0          = 76,
    RMT_SIG_0         = 83,
    RMT_SIG_1         = 84,
    RMT_SIG_2         = 85,
    RMT_SIG_3         = 86,
    I2CEXT1_SCL       = 95,
    I2CEXT1_SDA       = 96,
    FSPICLK           = 108,
    FSPIQ             = 109,
    FSPID             = 110,
    FSPIHD            = 111,
    FSPIWP            = 112,
    FSPIIO4           = 113,
    FSPIIO5           = 114,
    FSPIIO6           = 115,
    FSPIIO7           = 116,
    FSPICS0           = 117,
    TWAI_RX           = 123,
    SUBSPIQ           = 127,
    SUBSPID           = 128,
    SUBSPIHD          = 129,
    SUBSPIWP          = 130,
    I2S0I_DATA_IN15   = 158,
    SUBSPID4          = 167,
    SUBSPID5          = 168,
    SUBSPID6          = 169,
    SUBSPID7          = 170,
    SUBSPIDQS         = 171,
    PCMFSYNC          = 203,
    PCMCLK            = 204,
}

/// Peripheral output signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
#[doc(hidden)]
pub enum OutputSignal {
    SPIQ             = 0,
    SPID             = 1,
    SPIHD            = 2,
    SPIWP            = 3,
    SPICLK           = 4,
    SPICS0           = 5,
    SPICS1           = 6,
    SPID4            = 7,
    SPID5            = 8,
    SPID6            = 9,
    SPID7            = 10,
    SPIDQS           = 11,
    U0TXD            = 14,
    U0RTS            = 15,
    U0DTR            = 16,
    U1TXD            = 17,
    U1RTS            = 18,
    U1DTR            = 21,
    I2S0O_BCK        = 23,
    I2S0O_WS         = 25,
    I2S0I_BCK        = 27,
    I2S0I_WS         = 28,
    I2CEXT0_SCL      = 29,
    I2CEXT0_SDA      = 30,
    SDIO_TOHOST_INT  = 31,
    USB_EXTPHY_OEN   = 61,
    USB_EXTPHY_VPO   = 63,
    USB_EXTPHY_VMO   = 64,
    SPI3_CLK         = 72,
    SPI3_Q           = 73,
    SPI3_D           = 74,
    SPI3_HD          = 75,
    SPI3_CS0         = 76,
    SPI3_CS1         = 77,
    SPI3_CS2         = 78,
    LEDC_LS_SIG0     = 79,
    LEDC_LS_SIG1     = 80,
    LEDC_LS_SIG2     = 81,
    LEDC_LS_SIG3     = 82,
    LEDC_LS_SIG4     = 83,
    LEDC_LS_SIG5     = 84,
    LEDC_LS_SIG6     = 85,
    LEDC_LS_SIG7     = 86,
    RMT_SIG_0        = 87,
    RMT_SIG_1        = 88,
    RMT_SIG_2        = 89,
    RMT_SIG_3        = 90,
    I2CEXT1_SCL      = 95,
    I2CEXT1_SDA      = 96,
    GPIO_SD0         = 100,
    GPIO_SD1         = 101,
    GPIO_SD2         = 102,
    GPIO_SD3         = 103,
    GPIO_SD4         = 104,
    GPIO_SD5         = 105,
    GPIO_SD6         = 106,
    GPIO_SD7         = 107,
    FSPICLK          = 108,
    FSPIQ            = 109,
    FSPID            = 110,
    FSPIHD           = 111,
    FSPIWP           = 112,
    FSPIIO4          = 113,
    FSPIIO5          = 114,
    FSPIIO6          = 115,
    FSPIIO7          = 116,
    FSPICS0          = 117,
    FSPICS1          = 118,
    FSPICS2          = 119,
    FSPICS3          = 120,
    FSPICS4          = 121,
    FSPICS5          = 122,
    TWAI_TX          = 123,
    SUBSPICLK        = 126,
    SUBSPIQ          = 127,
    SUBSPID          = 128,
    SUBSPIHD         = 129,
    SUBSPIWP         = 130,
    SUBSPICS0        = 131,
    SUBSPICS1        = 132,
    FSPIDQS          = 133,
    FSPI_HSYNC       = 134,
    FSPI_VSYNC       = 135,
    FSPI_DE          = 136,
    FSPICD           = 137,
    SPI3_CD          = 139,
    SPI3_DQS         = 140,
    I2S0O_DATA_OUT23 = 166,
    SUBSPID4         = 167,
    SUBSPID5         = 168,
    SUBSPID6         = 169,
    SUBSPID7         = 170,
    SUBSPIDQS        = 171,
    PCMFSYNC         = 209,
    PCMCLK           = 210,
    CLK_I2S          = 251,
    GPIO             = 256,
}

crate::gpio::gpio! {
    (0, 0, InputOutputAnalog)
    (1, 0, InputOutputAnalog)
    (2, 0, InputOutputAnalog)
    (3, 0, InputOutputAnalog)
    (4, 0, InputOutputAnalog)
    (5, 0, InputOutputAnalog)
    (6, 0, InputOutputAnalog)
    (7, 0, InputOutputAnalog)
    (8, 0, InputOutputAnalog)
    (9, 0, InputOutputAnalog)
    (10, 0, InputOutputAnalog)
    (11, 0, InputOutputAnalog)
    (12, 0, InputOutputAnalog)
    (13, 0, InputOutputAnalog)
    (14, 0, InputOutputAnalog)
    (15, 0, InputOutputAnalog)
    (16, 0, InputOutputAnalog)
    (17, 0, InputOutputAnalog)
    (18, 0, InputOutputAnalog)
    (19, 0, InputOutputAnalog)
    (20, 0, InputOutputAnalog)
    (21, 0, InputOutputAnalog)

    (26, 0, InputOutput)
    (27, 0, InputOutput)
    (28, 0, InputOutput)
    (29, 0, InputOutput)
    (30, 0, InputOutput)
    (31, 0, InputOutput)
    (32, 1, InputOutput)
    (33, 1, InputOutput)
    (34, 1, InputOutput)
    (35, 1, InputOutput)
    (36, 1, InputOutput)
    (37, 1, InputOutput)
    (38, 1, InputOutput)
    (39, 1, InputOutput)
    (40, 1, InputOutput)
    (41, 1, InputOutput)
    (42, 1, InputOutput)
    (43, 1, InputOutput)
    (44, 1, InputOutput)
    (45, 1, InputOutput)
    (46, 1, InputOutput)
}

crate::gpio::analog! {
    ( 0,  0,  touch_pad(0),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    ( 1,  1,  touch_pad(1),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    ( 2,  2,  touch_pad(2),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    ( 3,  3,  touch_pad(3),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    ( 4,  4,  touch_pad(4),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    ( 5,  5,  touch_pad(5),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    ( 6,  6,  touch_pad(6),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    ( 7,  7,  touch_pad(7),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    ( 8,  8,  touch_pad(8),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    ( 9,  9,  touch_pad(9),    mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (10, 10,  touch_pad(10),   mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (11, 11,  touch_pad(11),   mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (12, 12,  touch_pad(12),   mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (13, 13,  touch_pad(13),   mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (14, 14,  touch_pad(14),   mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (15, 15,  xtal_32p_pad(),  x32p_mux_sel,  x32p_fun_sel,  x32p_fun_ie,  x32p_rue,  x32p_rde)
    (16, 16,  xtal_32n_pad(),  x32n_mux_sel,  x32n_fun_sel,  x32n_fun_ie,  x32n_rue,  x32n_rde)
    (17, 17,  pad_dac1(),      mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (18, 18,  pad_dac2(),      mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (19, 19,  rtc_pad19(),     mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (20, 20,  rtc_pad20(),     mux_sel,       fun_sel,       fun_ie,       rue,       rde)
    (21, 21,  rtc_pad21(),     mux_sel,       fun_sel,       fun_ie,       rue,       rde)
}

crate::gpio::rtc_pins! {
    ( 0,  0,  touch_pad(0),   "",     touch_pad0_hold,  rue,       rde)
    ( 1,  1,  touch_pad(1),   "",     touch_pad1_hold,  rue,       rde)
    ( 2,  2,  touch_pad(2),   "",     touch_pad2_hold,  rue,       rde)
    ( 3,  3,  touch_pad(3),   "",     touch_pad3_hold,  rue,       rde)
    ( 4,  4,  touch_pad(4),   "",     touch_pad4_hold,  rue,       rde)
    ( 5,  5,  touch_pad(5),   "",     touch_pad5_hold,  rue,       rde)
    ( 6,  6,  touch_pad(6),   "",     touch_pad6_hold,  rue,       rde)
    ( 7,  7,  touch_pad(7),   "",     touch_pad7_hold,  rue,       rde)
    ( 8,  8,  touch_pad(8),   "",     touch_pad8_hold,  rue,       rde)
    ( 9,  9,  touch_pad(9),   "",     touch_pad9_hold,  rue,       rde)
    (10, 10,  touch_pad(10),  "",     touch_pad10_hold, rue,       rde)
    (11, 11,  touch_pad(11),  "",     touch_pad11_hold, rue,       rde)
    (12, 12,  touch_pad(12),  "",     touch_pad12_hold, rue,       rde)
    (13, 13,  touch_pad(13),  "",     touch_pad13_hold, rue,       rde)
    (14, 14,  touch_pad(14),  "",     touch_pad14_hold, rue,       rde)
    (15, 15,  xtal_32p_pad(), x32p_,  x32p_hold,        x32p_rue,  x32p_rde)
    (16, 16,  xtal_32n_pad(), x32n_,  x32n_hold,        x32n_rue,  x32n_rde)
    (17, 17,  pad_dac1(),     "",     pdac1_hold,       rue,       rde)
    (18, 18,  pad_dac2(),     "",     pdac2_hold,       rue,       rde)
    (19, 19,  rtc_pad19(),    "",     pad19_hold,       rue,       rde)
    (20, 20,  rtc_pad20(),    "",     pad20_hold,       rue,       rde)
    (21, 21,  rtc_pad21(),    "",     pad21_hold,       rue,       rde)
}

impl InterruptStatusRegisterAccess for InterruptStatusRegisterAccessBank0 {
    fn pro_cpu_interrupt_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_int().read().bits()
    }

    fn pro_cpu_nmi_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_nmi_int().read().bits()
    }
}

impl InterruptStatusRegisterAccess for InterruptStatusRegisterAccessBank1 {
    fn pro_cpu_interrupt_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_int1().read().bits()
    }

    fn pro_cpu_nmi_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_nmi_int1().read().bits()
    }
}

// implement marker traits on USB pins
impl crate::otg_fs::UsbDm for Gpio19 {}
impl crate::otg_fs::UsbDp for Gpio20 {}
