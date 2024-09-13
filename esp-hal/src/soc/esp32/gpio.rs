//! # GPIO configuration module (ESP32)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32` chip. It allows you to
//! configure pins as inputs or outputs, set their state and read their state.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
//!   - `get_io_mux_reg(gpio_num: u8) -> &'static io_mux::GPIO0:`:
//!       * This function returns a reference to the GPIO register associated
//!         with the given GPIO number. It uses unsafe code and transmutation to
//!         access the GPIO registers based on the provided GPIO number.
//!   - `gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8`:
//!       * This function enables or disables GPIO interrupts and Non-Maskable
//!         Interrupts (NMI). It takes two boolean arguments int_enable and
//!         nmi_enable to control the interrupt and NMI enable settings. The
//!         function returns an u8 value representing the interrupt enable
//!         settings.
//!   - `errata36(pin_num: u8, pull_up: bool, pull_down: bool)`:
//!       * Handles the configuration of pull-up and pull-down resistors for
//!         specific GPIO pins
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

use core::mem::transmute;

use crate::{
    gpio::{AlternateFunction, GpioPin},
    peripherals::{io_mux, GPIO, IO_MUX},
    Cpu,
};

/// The total number of GPIO pins available.
pub const NUM_PINS: usize = 40;

pub(crate) const FUNC_IN_SEL_OFFSET: usize = 0;

pub(crate) type OutputSignalType = u16;
pub(crate) const OUTPUT_SIGNAL_MAX: u16 = 548;
pub(crate) const INPUT_SIGNAL_MAX: u16 = 539;

pub(crate) const ONE_INPUT: u8 = 0x38;
pub(crate) const ZERO_INPUT: u8 = 0x30;

pub(crate) const GPIO_FUNCTION: AlternateFunction = AlternateFunction::Function2;

pub(crate) fn get_io_mux_reg(gpio_num: u8) -> &'static io_mux::GPIO0 {
    unsafe {
        let iomux = &*IO_MUX::PTR;

        match gpio_num {
            0 => transmute::<&'static io_mux::GPIO0, &'static io_mux::GPIO0>(iomux.gpio0()),
            1 => transmute::<&'static io_mux::GPIO1, &'static io_mux::GPIO0>(iomux.gpio1()),
            2 => transmute::<&'static io_mux::GPIO2, &'static io_mux::GPIO0>(iomux.gpio2()),
            3 => transmute::<&'static io_mux::GPIO3, &'static io_mux::GPIO0>(iomux.gpio3()),
            4 => transmute::<&'static io_mux::GPIO4, &'static io_mux::GPIO0>(iomux.gpio4()),
            5 => transmute::<&'static io_mux::GPIO5, &'static io_mux::GPIO0>(iomux.gpio5()),
            6 => transmute::<&'static io_mux::GPIO6, &'static io_mux::GPIO0>(iomux.gpio6()),
            7 => transmute::<&'static io_mux::GPIO7, &'static io_mux::GPIO0>(iomux.gpio7()),
            8 => transmute::<&'static io_mux::GPIO8, &'static io_mux::GPIO0>(iomux.gpio8()),
            9 => transmute::<&'static io_mux::GPIO9, &'static io_mux::GPIO0>(iomux.gpio9()),
            10 => transmute::<&'static io_mux::GPIO10, &'static io_mux::GPIO0>(iomux.gpio10()),
            11 => transmute::<&'static io_mux::GPIO11, &'static io_mux::GPIO0>(iomux.gpio11()),
            12 => transmute::<&'static io_mux::GPIO12, &'static io_mux::GPIO0>(iomux.gpio12()),
            13 => transmute::<&'static io_mux::GPIO13, &'static io_mux::GPIO0>(iomux.gpio13()),
            14 => transmute::<&'static io_mux::GPIO14, &'static io_mux::GPIO0>(iomux.gpio14()),
            15 => transmute::<&'static io_mux::GPIO15, &'static io_mux::GPIO0>(iomux.gpio15()),
            16 => transmute::<&'static io_mux::GPIO16, &'static io_mux::GPIO0>(iomux.gpio16()),
            17 => transmute::<&'static io_mux::GPIO17, &'static io_mux::GPIO0>(iomux.gpio17()),
            18 => transmute::<&'static io_mux::GPIO18, &'static io_mux::GPIO0>(iomux.gpio18()),
            19 => transmute::<&'static io_mux::GPIO19, &'static io_mux::GPIO0>(iomux.gpio19()),
            20 => transmute::<&'static io_mux::GPIO20, &'static io_mux::GPIO0>(iomux.gpio20()),
            21 => transmute::<&'static io_mux::GPIO21, &'static io_mux::GPIO0>(iomux.gpio21()),
            22 => transmute::<&'static io_mux::GPIO22, &'static io_mux::GPIO0>(iomux.gpio22()),
            23 => transmute::<&'static io_mux::GPIO23, &'static io_mux::GPIO0>(iomux.gpio23()),
            24 => transmute::<&'static io_mux::GPIO24, &'static io_mux::GPIO0>(iomux.gpio24()),
            25 => transmute::<&'static io_mux::GPIO25, &'static io_mux::GPIO0>(iomux.gpio25()),
            26 => transmute::<&'static io_mux::GPIO26, &'static io_mux::GPIO0>(iomux.gpio26()),
            27 => transmute::<&'static io_mux::GPIO27, &'static io_mux::GPIO0>(iomux.gpio27()),
            32 => transmute::<&'static io_mux::GPIO32, &'static io_mux::GPIO0>(iomux.gpio32()),
            33 => transmute::<&'static io_mux::GPIO33, &'static io_mux::GPIO0>(iomux.gpio33()),
            34 => transmute::<&'static io_mux::GPIO34, &'static io_mux::GPIO0>(iomux.gpio34()),
            35 => transmute::<&'static io_mux::GPIO35, &'static io_mux::GPIO0>(iomux.gpio35()),
            36 => transmute::<&'static io_mux::GPIO36, &'static io_mux::GPIO0>(iomux.gpio36()),
            37 => transmute::<&'static io_mux::GPIO37, &'static io_mux::GPIO0>(iomux.gpio37()),
            38 => transmute::<&'static io_mux::GPIO38, &'static io_mux::GPIO0>(iomux.gpio38()),
            39 => transmute::<&'static io_mux::GPIO39, &'static io_mux::GPIO0>(iomux.gpio39()),
            _ => panic!(),
        }
    }
}

pub(crate) fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    match crate::get_core() {
        Cpu::AppCpu => int_enable as u8 | ((nmi_enable as u8) << 1),
        // this should be bits 3 & 4 respectively, according to the TRM, but it doesn't seem to
        // work. This does though.
        Cpu::ProCpu => (int_enable as u8) << 2 | ((nmi_enable as u8) << 3),
    }
}

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
#[doc(hidden)]
pub enum InputSignal {
    SPICLK                = 0,
    SPIQ                  = 1,
    SPID                  = 2,
    SPIHD                 = 3,
    SPIWP                 = 4,
    SPICS0                = 5,
    SPICS1                = 6,
    SPICS2                = 7,
    HSPICLK               = 8,
    HSPIQ                 = 9,
    HSPID                 = 10,
    HSPICS0               = 11,
    HSPIHD                = 12,
    HSPIWP                = 13,
    U0RXD                 = 14,
    U0CTS                 = 15,
    U0DSR                 = 16,
    U1RXD                 = 17,
    U1CTS                 = 18,
    I2CM_SDA              = 20,
    EXT_I2C_SDA           = 22,
    I2S0O_BCK             = 23,
    I2S1O_BCK             = 24,
    I2S0O_WS              = 25,
    I2S1O_WS              = 26,
    I2S0I_BCK             = 27,
    I2S0I_WS              = 28,
    I2CEXT0_SCL           = 29,
    I2CEXT0_SDA           = 30,
    PWM0_SYNC0            = 31,
    PWM0_SYNC1            = 32,
    PWM0_SYNC2            = 33,
    PWM0_F0               = 34,
    PWM0_F1               = 35,
    PWM0_F2               = 36,
    GPIO_BT_ACTIVE        = 37,
    GPIO_BT_PRIORITY      = 38,
    PCNT0_SIG_CH0         = 39,
    PCNT0_SIG_CH1         = 40,
    PCNT0_CTRL_CH0        = 41,
    PCNT0_CTRL_CH1        = 42,
    PCNT1_SIG_CH0         = 43,
    PCNT1_SIG_CH1         = 44,
    PCNT1_CTRL_CH0        = 45,
    PCNT1_CTRL_CH1        = 46,
    PCNT2_SIG_CH0         = 47,
    PCNT2_SIG_CH1         = 48,
    PCNT2_CTRL_CH0        = 49,
    PCNT2_CTRL_CH1        = 50,
    PCNT3_SIG_CH0         = 51,
    PCNT3_SIG_CH1         = 52,
    PCNT3_CTRL_CH0        = 53,
    PCNT3_CTRL_CH1        = 54,
    PCNT4_SIG_CH0         = 55,
    PCNT4_SIG_CH1         = 56,
    PCNT4_CTRL_CH0        = 57,
    PCNT4_CTRL_CH1        = 58,
    HSPICS1               = 61,
    HSPICS2               = 62,
    VSPICLK               = 63,
    VSPIQ                 = 64,
    VSPID                 = 65,
    VSPIHD                = 66,
    VSPIWP                = 67,
    VSPICS0               = 68,
    VSPICS1               = 69,
    VSPICS2               = 70,
    PCNT5_SIG_CH0         = 71,
    PCNT5_SIG_CH1         = 72,
    PCNT5_CTRL_CH0        = 73,
    PCNT5_CTRL_CH1        = 74,
    PCNT6_SIG_CH0         = 75,
    PCNT6_SIG_CH1         = 76,
    PCNT6_CTRL_CH0        = 77,
    PCNT6_CTRL_CH1        = 78,
    PCNT7_SIG_CH0         = 79,
    PCNT7_SIG_CH1         = 80,
    PCNT7_CTRL_CH0        = 81,
    PCNT7_CTRL_CH1        = 82,
    RMT_SIG_0             = 83,
    RMT_SIG_1             = 84,
    RMT_SIG_2             = 85,
    RMT_SIG_3             = 86,
    RMT_SIG_4             = 87,
    RMT_SIG_5             = 88,
    RMT_SIG_6             = 89,
    RMT_SIG_7             = 90,
    EXT_ADC_START         = 93,
    TWAI_RX               = 94,
    I2CEXT1_SCL           = 95,
    I2CEXT1_SDA           = 96,
    HOST_CARD_DETECT_N_1  = 97,
    HOST_CARD_DETECT_N_2  = 98,
    HOST_CARD_WRITE_PRT_1 = 99,
    HOST_CARD_WRITE_PRT_2 = 100,
    HOST_CARD_INT_N_1     = 101,
    HOST_CARD_INT_N_2     = 102,
    PWM1_SYNC0            = 103,
    PWM1_SYNC1            = 104,
    PWM1_SYNC2            = 105,
    PWM1_F0               = 106,
    PWM1_F1               = 107,
    PWM1_F2               = 108,
    PWM0_CAP0             = 109,
    PWM0_CAP1             = 110,
    PWM0_CAP2             = 111,
    PWM1_CAP0             = 112,
    PWM1_CAP1             = 113,
    PWM1_CAP2             = 114,
    PWM2_FLTA             = 115,
    PWM2_FLTB             = 116,
    PWM2_CAP1             = 117,
    PWM2_CAP2             = 118,
    PWM2_CAP3             = 119,
    PWM3_FLTA             = 120,
    PWM3_FLTB             = 121,
    PWM3_CAP1             = 122,
    PWM3_CAP2             = 123,
    PWM3_CAP3             = 124,
    CAN_CLKOUT            = 125,
    SPID4                 = 128,
    SPID5                 = 129,
    SPID6                 = 130,
    SPID7                 = 131,
    HSPID4                = 132,
    HSPID5                = 133,
    HSPID6                = 134,
    HSPID7                = 135,
    VSPID4                = 136,
    VSPID5                = 137,
    VSPID6                = 138,
    VSPID7                = 139,
    I2S0I_DATA_0          = 140,
    I2S0I_DATA_1          = 141,
    I2S0I_DATA_2          = 142,
    I2S0I_DATA_3          = 143,
    I2S0I_DATA_4          = 144,
    I2S0I_DATA_5          = 145,
    I2S0I_DATA_6          = 146,
    I2S0I_DATA_7          = 147,
    I2S0I_DATA_8          = 148,
    I2S0I_DATA_9          = 149,
    I2S0I_DATA_10         = 150,
    I2S0I_DATA_11         = 151,
    I2S0I_DATA_12         = 152,
    I2S0I_DATA_13         = 153,
    I2S0I_DATA_14         = 154,
    I2S0I_DATA_15         = 155,
    I2S1I_BCK             = 164,
    I2S1I_WS              = 165,
    I2S1I_DATA_0          = 166,
    I2S1I_DATA_1          = 167,
    I2S1I_DATA_2          = 168,
    I2S1I_DATA_3          = 169,
    I2S1I_DATA_4          = 170,
    I2S1I_DATA_5          = 171,
    I2S1I_DATA_6          = 172,
    I2S1I_DATA_7          = 173,
    I2S1I_DATA_8          = 174,
    I2S1I_DATA_9          = 175,
    I2S1I_DATA_10         = 176,
    I2S1I_DATA_11         = 177,
    I2S1I_DATA_12         = 178,
    I2S1I_DATA_13         = 179,
    I2S1I_DATA_14         = 180,
    I2S1I_DATA_15         = 181,
    I2S0I_H_SYNC          = 190,
    I2S0I_V_SYNC          = 191,
    I2S0I_H_ENABLE        = 192,
    I2S1I_H_SYNC          = 193,
    I2S1I_V_SYNC          = 194,
    I2S1I_H_ENABLE        = 195,
    U2RXD                 = 198,
    U2CTS                 = 199,
    EMAC_MDC              = 200,
    EMAC_MDI              = 201,
    EMAC_CRS              = 202,
    EMAC_COL              = 203,
    PCMFSYNC              = 204,
    PCMCLK                = 205,
    PCMDIN                = 206,
    SIG_IN_FUNC224        = 224,
    SIG_IN_FUNC225        = 225,
    SIG_IN_FUNC226        = 226,
    SIG_IN_FUNC227        = 227,
    SIG_IN_FUNC228        = 228,

    SD_DATA0              = 512,
    SD_DATA1,
    SD_DATA2,
    SD_DATA3,
    HS1_DATA0,
    HS1_DATA1,
    HS1_DATA2,
    HS1_DATA3,
    HS1_DATA4,
    HS1_DATA5,
    HS1_DATA6,
    HS1_DATA7,
    HS2_DATA0,
    HS2_DATA1,
    HS2_DATA2,
    HS2_DATA3,

    EMAC_TX_CLK,
    EMAC_RXD2,
    EMAC_TX_ER,
    EMAC_RX_CLK,
    EMAC_RX_ER,
    EMAC_RXD3,
    EMAC_RXD0,
    EMAC_RXD1,
    EMAC_RX_DV,

    MTDI,
    MTCK,
    MTMS,
}

/// Peripheral output signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
#[doc(hidden)]
pub enum OutputSignal {
    SPICLK                   = 0,
    SPIQ                     = 1,
    SPID                     = 2,
    SPIHD                    = 3,
    SPIWP                    = 4,
    SPICS0                   = 5,
    SPICS1                   = 6,
    SPICS2                   = 7,
    HSPICLK                  = 8,
    HSPIQ                    = 9,
    HSPID                    = 10,
    HSPICS0                  = 11,
    HSPIHD                   = 12,
    HSPIWP                   = 13,
    U0TXD                    = 14,
    U0RTS                    = 15,
    U0DTR                    = 16,
    U1TXD                    = 17,
    U1RTS                    = 18,
    I2CM_SCL                 = 19,
    I2CM_SDA                 = 20,
    EXT2C_SCL                = 21,
    EXT2C_SDA                = 22,
    I2S0O_BCK                = 23,
    I2S1O_BCK                = 24,
    I2S0O_WS                 = 25,
    I2S1O_WS                 = 26,
    I2S0I_BCK                = 27,
    I2S0I_WS                 = 28,
    I2CEXT0_SCL              = 29,
    I2CEXT0_SDA              = 30,
    SDIO_TOHOSTT             = 31,
    PWM0_0A                  = 32,
    PWM0_0B                  = 33,
    PWM0_1A                  = 34,
    PWM0_1B                  = 35,
    PWM0_2A                  = 36,
    PWM0_2B                  = 37,
    GPIO_WLAN_ACTIVE         = 40,
    BB_DIAG0                 = 41,
    BB_DIAG1                 = 42,
    BB_DIAG2                 = 43,
    BB_DIAG3                 = 44,
    BB_DIAG4                 = 45,
    BB_DIAG5                 = 46,
    BB_DIAG6                 = 47,
    BB_DIAG7                 = 48,
    BB_DIAG8                 = 49,
    BB_DIAG9                 = 50,
    BB_DIAG10                = 51,
    BB_DIAG11                = 52,
    BB_DIAG12                = 53,
    BB_DIAG13                = 54,
    BB_DIAG14                = 55,
    BB_DIAG15                = 56,
    BB_DIAG16                = 57,
    BB_DIAG17                = 58,
    BB_DIAG18                = 59,
    BB_DIAG19                = 60,
    HSPICS1                  = 61,
    HSPICS2                  = 62,
    VSPICLK                  = 63,
    VSPIQ                    = 64,
    VSPID                    = 65,
    VSPIHD                   = 66,
    VSPIWP                   = 67,
    VSPICS0                  = 68,
    VSPICS1                  = 69,
    VSPICS2                  = 70,
    LEDC_HS_SIG0             = 71,
    LEDC_HS_SIG1             = 72,
    LEDC_HS_SIG2             = 73,
    LEDC_HS_SIG3             = 74,
    LEDC_HS_SIG4             = 75,
    LEDC_HS_SIG5             = 76,
    LEDC_HS_SIG6             = 77,
    LEDC_HS_SIG7             = 78,
    LEDC_LS_SIG0             = 79,
    LEDC_LS_SIG1             = 80,
    LEDC_LS_SIG2             = 81,
    LEDC_LS_SIG3             = 82,
    LEDC_LS_SIG4             = 83,
    LEDC_LS_SIG5             = 84,
    LEDC_LS_SIG6             = 85,
    LEDC_LS_SIG7             = 86,
    RMT_SIG_0                = 87,
    RMT_SIG_1                = 88,
    RMT_SIG_2                = 89,
    RMT_SIG_3                = 90,
    RMT_SIG_4                = 91,
    RMT_SIG_5                = 92,
    RMT_SIG_6                = 93,
    RMT_SIG_7                = 94,
    I2CEXT1_SCL              = 95,
    I2CEXT1_SDA              = 96,
    HOST_CCMD_OD_PULLUP_EN_N = 97,
    HOST_RST_N_1             = 98,
    HOST_RST_N_2             = 99,
    GPIO_SD0                 = 100,
    GPIO_SD1                 = 101,
    GPIO_SD2                 = 102,
    GPIO_SD3                 = 103,
    GPIO_SD4                 = 104,
    GPIO_SD5                 = 105,
    GPIO_SD6                 = 106,
    GPIO_SD7                 = 107,
    PWM1_0A                  = 108,
    PWM1_0B                  = 109,
    PWM1_1A                  = 110,
    PWM1_1B                  = 111,
    PWM1_2A                  = 112,
    PWM1_2B                  = 113,
    PWM2_1H                  = 114,
    PWM2_1L                  = 115,
    PWM2_2H                  = 116,
    PWM2_2L                  = 117,
    PWM2_3H                  = 118,
    PWM2_3L                  = 119,
    PWM2_4H                  = 120,
    PWM2_4L                  = 121,
    TWAI_TX                  = 123,
    CAN_BUS_OFF_ON           = 124,
    SPID4                    = 128,
    SPID5                    = 129,
    SPID6                    = 130,
    SPID7                    = 131,
    HSPID4                   = 132,
    HSPID5                   = 133,
    HSPID6                   = 134,
    HSPID7                   = 135,
    VSPID4                   = 136,
    VSPID5                   = 137,
    VSPID6                   = 138,
    VSPID7                   = 139,
    I2S0O_DATA_0             = 140,
    I2S0O_DATA_1             = 141,
    I2S0O_DATA_2             = 142,
    I2S0O_DATA_3             = 143,
    I2S0O_DATA_4             = 144,
    I2S0O_DATA_5             = 145,
    I2S0O_DATA_6             = 146,
    I2S0O_DATA_7             = 147,
    I2S0O_DATA_8             = 148,
    I2S0O_DATA_9             = 149,
    I2S0O_DATA_10            = 150,
    I2S0O_DATA_11            = 151,
    I2S0O_DATA_12            = 152,
    I2S0O_DATA_13            = 153,
    I2S0O_DATA_14            = 154,
    I2S0O_DATA_15            = 155,
    I2S0O_DATA_16            = 156,
    I2S0O_DATA_17            = 157,
    I2S0O_DATA_18            = 158,
    I2S0O_DATA_19            = 159,
    I2S0O_DATA_20            = 160,
    I2S0O_DATA_21            = 161,
    I2S0O_DATA_22            = 162,
    I2S0O_DATA_23            = 163,
    I2S1I_BCK                = 164,
    I2S1I_WS                 = 165,
    I2S1O_DATA_0             = 166,
    I2S1O_DATA_1             = 167,
    I2S1O_DATA_2             = 168,
    I2S1O_DATA_3             = 169,
    I2S1O_DATA_4             = 170,
    I2S1O_DATA_5             = 171,
    I2S1O_DATA_6             = 172,
    I2S1O_DATA_7             = 173,
    I2S1O_DATA_8             = 174,
    I2S1O_DATA_9             = 175,
    I2S1O_DATA_10            = 176,
    I2S1O_DATA_11            = 177,
    I2S1O_DATA_12            = 178,
    I2S1O_DATA_13            = 179,
    I2S1O_DATA_14            = 180,
    I2S1O_DATA_15            = 181,
    I2S1O_DATA_16            = 182,
    I2S1O_DATA_17            = 183,
    I2S1O_DATA_18            = 184,
    I2S1O_DATA_19            = 185,
    I2S1O_DATA_20            = 186,
    I2S1O_DATA_21            = 187,
    I2S1O_DATA_22            = 188,
    I2S1O_DATA_23            = 189,
    PWM3_1H                  = 190,
    PWM3_1L                  = 191,
    PWM3_2H                  = 192,
    PWM3_2L                  = 193,
    PWM3_3H                  = 194,
    PWM3_3L                  = 195,
    PWM3_4H                  = 196,
    PWM3_4L                  = 197,
    U2TXD                    = 198,
    U2RTS                    = 199,
    EMAC_MDC                 = 200,
    EMAC_MDO                 = 201,
    EMAC_CRS                 = 202,
    EMAC_COL                 = 203,
    BT_AUDIO0RQ              = 204,
    BT_AUDIO1RQ              = 205,
    BT_AUDIO2RQ              = 206,
    BLE_AUDIO0RQ             = 207,
    BLE_AUDIO1RQ             = 208,
    BLE_AUDIO2RQ             = 209,
    PCMFSYNC                 = 210,
    PCMCLK                   = 211,
    PCMDOUT                  = 212,
    BLE_AUDIO_SYNC0_P        = 213,
    BLE_AUDIO_SYNC1_P        = 214,
    BLE_AUDIO_SYNC2_P        = 215,
    ANT_SEL0                 = 216,
    ANT_SEL1                 = 217,
    ANT_SEL2                 = 218,
    ANT_SEL3                 = 219,
    ANT_SEL4                 = 220,
    ANT_SEL5                 = 221,
    ANT_SEL6                 = 222,
    ANT_SEL7                 = 223,
    SIGNAL_224               = 224,
    SIGNAL_225               = 225,
    SIGNAL_226               = 226,
    SIGNAL_227               = 227,
    SIGNAL_228               = 228,
    GPIO                     = 256,

    CLK_OUT1                 = 512,
    CLK_OUT2,
    CLK_OUT3,
    SD_CLK,
    SD_CMD,
    SD_DATA0,
    SD_DATA1,
    SD_DATA2,
    SD_DATA3,
    HS1_CLK,
    HS1_CMD,
    HS1_DATA0,
    HS1_DATA1,
    HS1_DATA2,
    HS1_DATA3,
    HS1_DATA4,
    HS1_DATA5,
    HS1_DATA6,
    HS1_DATA7,
    HS1_STROBE,
    HS2_CLK,
    HS2_CMD,
    HS2_DATA0,
    HS2_DATA1,
    HS2_DATA2,
    HS2_DATA3,

    EMAC_TX_CLK,
    EMAC_TX_ER,
    EMAC_TXD3,
    EMAC_RX_ER,
    EMAC_TXD2,
    EMAC_CLK_OUT,
    EMAC_CLK_180,
    EMAC_TXD0,
    EMAC_TX_EN,
    EMAC_TXD1,

    MTDO,
}

pub(crate) fn errata36(pin_num: u8, pull_up: Option<bool>, pull_down: Option<bool>) {
    use crate::peripherals::RTC_IO;
    let rtcio = unsafe { &*RTC_IO::PTR };

    match pin_num {
        0 => {
            rtcio.touch_pad1().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        2 => {
            rtcio.touch_pad2().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        4 => {
            rtcio.touch_pad0().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        12 => {
            rtcio.touch_pad5().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        13 => {
            rtcio.touch_pad4().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        14 => {
            rtcio.touch_pad6().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        15 => {
            rtcio.touch_pad3().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        25 => {
            rtcio.pad_dac1().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        26 => {
            rtcio.pad_dac2().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        27 => {
            rtcio.touch_pad7().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.rde().bit(pull_down);
                }
                w
            });
        }
        32 => {
            rtcio.xtal_32k_pad().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.x32p_rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.x32p_rde().bit(pull_down);
                }
                w
            });
        }
        33 => {
            rtcio.xtal_32k_pad().modify(|_, w| {
                if let Some(pull_up) = pull_up {
                    w.x32n_rue().bit(pull_up);
                }
                if let Some(pull_down) = pull_down {
                    w.x32n_rde().bit(pull_down);
                }
                w
            });
        }
        _ => (),
    }
}

crate::gpio::gpio! {
    (0, 0, InputOutputAnalogTouch (5 => EMAC_TX_CLK) (1 => CLK_OUT1))
    (1, 0, InputOutput (5 => EMAC_RXD2) (0 => U0TXD 1 => CLK_OUT3))
    (2, 0, InputOutputAnalogTouch (1 => HSPIWP 3 => HS2_DATA0 4 => SD_DATA0) (3 => HS2_DATA0 4 => SD_DATA0))
    (3, 0, InputOutput (0 => U0RXD) (1 => CLK_OUT2))
    (4, 0, InputOutputAnalogTouch (1 => HSPIHD 3 => HS2_DATA1 4 => SD_DATA1 5 => EMAC_TX_ER) (3 => HS2_DATA1 4 => SD_DATA1))
    (5, 0, InputOutput (1 => VSPICS0 3 => HS1_DATA6 5 => EMAC_RX_CLK) (3 => HS1_DATA6))
    (6, 0, InputOutput (4 => U1CTS) (0 => SD_CLK 1 => SPICLK 3 => HS1_CLK))
    (7, 0, InputOutput (0 => SD_DATA0 1 => SPIQ 3 => HS1_DATA0) (0 => SD_DATA0 1 => SPIQ 3 => HS1_DATA0 4 => U2RTS))
    (8, 0, InputOutput (0 => SD_DATA1 1 => SPID 3 => HS1_DATA1 4 => U2CTS) (0 => SD_DATA1 1 => SPID 3 => HS1_DATA1))
    (9, 0, InputOutput (0 => SD_DATA2 1 => SPIHD 3 => HS1_DATA2 4 => U1RXD) (0 => SD_DATA2 1 => SPIHD 3 => HS1_DATA2))
    (10, 0, InputOutput ( 0 => SD_DATA3 1 => SPIWP 3 => HS1_DATA3) (0 => SD_DATA3 1 => SPIWP 3 => HS1_DATA3 4 => U1TXD))
    (11, 0, InputOutput ( 1 => SPICS0) (0 => SD_CMD 1 => SPICS0 3 => HS1_CMD 4 => U1RTS))
    (12, 0, InputOutputAnalogTouch (0 => MTDI 1 => HSPIQ 3 => HS2_DATA2 4 => SD_DATA2) (1 => HSPIQ 3 => HS2_DATA2 4 => SD_DATA2 5 => EMAC_TXD3))
    (13, 0, InputOutputAnalogTouch (0 => MTCK 1 => HSPID 3 => HS2_DATA3 4 => SD_DATA3) (1 => HSPID 3 => HS2_DATA3 4 => SD_DATA3 5 => EMAC_RX_ER))
    (14, 0, InputOutputAnalogTouch (0 => MTMS 1 => HSPICLK) (1 => HSPICLK 3 => HS2_CLK 4 => SD_CLK 5 => EMAC_TXD2))
    (15, 0, InputOutputAnalogTouch (1 => HSPICS0 5 => EMAC_RXD3) (0 => MTDO 1 => HSPICS0 3 => HS2_CMD 4 => SD_CMD))
    (16, 0, InputOutput (3 => HS1_DATA4 4 => U2RXD) (3 => HS1_DATA4 5 => EMAC_CLK_OUT))
    (17, 0, InputOutput (3 => HS1_DATA5) (3 => HS1_DATA5 4 => U2TXD 5 => EMAC_CLK_180))
    (18, 0, InputOutput (1 => VSPICLK 3 => HS1_DATA7) (1 => VSPICLK 3 => HS1_DATA7))
    (19, 0, InputOutput (1 => VSPIQ 3 => U0CTS) (1 => VSPIQ 5 => EMAC_TXD0))
    (20, 0, InputOutput)
    (21, 0, InputOutput (1 => VSPIHD) (1 => VSPIHD 5 => EMAC_TX_EN))
    (22, 0, InputOutput (1 => VSPIWP) (1 => VSPIWP 3 => U0RTS 5 => EMAC_TXD1))
    (23, 0, InputOutput  (1 => VSPID) (1 => VSPID 3 => HS1_STROBE))
    (24, 0, InputOutput)
    (25, 0, InputOutputAnalog (5 => EMAC_RXD0) ())
    (26, 0, InputOutputAnalog (5 => EMAC_RXD1) ())
    (27, 0, InputOutputAnalogTouch (5 => EMAC_RX_DV) ())
    (32, 1, InputOutputAnalogTouch)
    (33, 1, InputOutputAnalogTouch)
    (34, 1, InputOnlyAnalog)
    (35, 1, InputOnlyAnalog)
    (36, 1, InputOnlyAnalog)
    (37, 1, InputOnlyAnalog)
    (38, 1, InputOnlyAnalog)
    (39, 1, InputOnlyAnalog)
}

crate::gpio::analog! {
    (36, 0,  sensor_pads(),   sense1_mux_sel, sense1_fun_sel, sense1_fun_ie)
    (37, 1,  sensor_pads(),   sense2_mux_sel, sense2_fun_sel, sense2_fun_ie)
    (38, 2,  sensor_pads(),   sense3_mux_sel, sense3_fun_sel, sense3_fun_ie)
    (39, 3,  sensor_pads(),   sense4_mux_sel, sense4_fun_sel, sense4_fun_ie)
    (34, 4,  adc_pad(),         adc1_mux_sel,   adc1_fun_sel,   adc1_fun_ie)
    (35, 5,  adc_pad(),         adc2_mux_sel,   adc2_fun_sel,   adc1_fun_ie)
    (25, 6,  pad_dac1(),             mux_sel,        fun_sel,        fun_ie, rue,       rde)
    (26, 7,  pad_dac2(),             mux_sel,        fun_sel,        fun_ie, rue,       rde)
    (33, 8,  xtal_32k_pad(),    x32n_mux_sel,   x32n_fun_sel,   x32n_fun_ie, x32n_rue,  x32n_rde )
    (32, 9,  xtal_32k_pad(),    x32p_mux_sel,   x32p_fun_sel,   x32p_fun_ie, x32p_rue,  x32p_rde )
    (4,  10, touch_pad0(),           mux_sel,        fun_sel,        fun_ie, rue,       rde      )
    (0,  11, touch_pad1(),           mux_sel,        fun_sel,        fun_ie, rue,       rde      )
    (2,  12, touch_pad2(),           mux_sel,        fun_sel,        fun_ie, rue,       rde      )
    (15, 13, touch_pad3(),           mux_sel,        fun_sel,        fun_ie, rue,       rde      )
    (13, 14, touch_pad4(),           mux_sel,        fun_sel,        fun_ie, rue,       rde      )
    (12, 15, touch_pad5(),           mux_sel,        fun_sel,        fun_ie, rue,       rde      )
    (14, 16, touch_pad6(),           mux_sel,        fun_sel,        fun_ie, rue,       rde      )
    (27, 17, touch_pad7(),           mux_sel,        fun_sel,        fun_ie, rue,       rde      )
}

crate::gpio::rtc_pins! {
    (36, 0,  sensor_pads(),    sense1_, sense1_hold_force )
    (37, 1,  sensor_pads(),    sense2_, sense2_hold_force )
    (38, 2,  sensor_pads(),    sense3_, sense3_hold_force )
    (39, 3,  sensor_pads(),    sense4_, sense4_hold_force )
    (34, 4,  adc_pad(),        adc1_,   adc1_hold_force )
    (35, 5,  adc_pad(),        adc2_,   adc2_hold_force )
    (25, 6,  pad_dac1(),       "",      pdac1_hold_force,      rue,       rde )
    (26, 7,  pad_dac2(),       "",      pdac2_hold_force,      rue,       rde )
    (33, 8,  xtal_32k_pad(),   x32n_,   x32n_hold_force,       x32n_rue,  x32n_rde  )
    (32, 9,  xtal_32k_pad(),   x32p_,   x32p_hold_force,       x32p_rue,  x32p_rde  )
    (4,  10, touch_pad0(),     "",      touch_pad0_hold_force, rue,       rde       )
    (0,  11, touch_pad1(),     "",      touch_pad1_hold_force, rue,       rde       )
    (2,  12, touch_pad2(),     "",      touch_pad2_hold_force, rue,       rde       )
    (15, 13, touch_pad3(),     "",      touch_pad3_hold_force, rue,       rde       )
    (13, 14, touch_pad4(),     "",      touch_pad4_hold_force, rue,       rde       )
    (12, 15, touch_pad5(),     "",      touch_pad5_hold_force, rue,       rde       )
    (14, 16, touch_pad6(),     "",      touch_pad6_hold_force, rue,       rde       )
    (27, 17, touch_pad7(),     "",      touch_pad7_hold_force, rue,       rde       )
}

crate::gpio::touch! {
    // touch_nr, pin_nr, rtc_pin, touch_out_reg, meas_field, touch_thres_reg, touch_thres_field, normal_pin
    (0, 4,  10, sar_touch_out1, touch_meas_out0, sar_touch_thres1, touch_out_th0, true)
    (1, 0,  11, sar_touch_out1, touch_meas_out1, sar_touch_thres1, touch_out_th1, true)
    (2, 2,  12, sar_touch_out2, touch_meas_out2, sar_touch_thres2, touch_out_th2, true)
    (3, 15, 13, sar_touch_out2, touch_meas_out3, sar_touch_thres2, touch_out_th3, true)
    (4, 13, 14, sar_touch_out3, touch_meas_out4, sar_touch_thres3, touch_out_th4, true)
    (5, 12, 15, sar_touch_out3, touch_meas_out5, sar_touch_thres3, touch_out_th5, true)
    (6, 14, 16, sar_touch_out4, touch_meas_out6, sar_touch_thres4, touch_out_th6, true)
    (7, 27, 17, sar_touch_out4, touch_meas_out7, sar_touch_thres4, touch_out_th7, true)
    // ---
    (8, 33, 8, sar_touch_out5, touch_meas_out8, sar_touch_thres5, touch_out_th8, false)
    (9, 32, 9, sar_touch_out5, touch_meas_out9, sar_touch_thres5, touch_out_th9, false)
}

#[doc(hidden)]
#[derive(Clone, Copy)]
pub enum InterruptStatusRegisterAccess {
    Bank0,
    Bank1,
}

impl InterruptStatusRegisterAccess {
    pub(crate) fn interrupt_status_read(self) -> u32 {
        match self {
            Self::Bank0 => match crate::get_core() {
                crate::Core::ProCpu => unsafe { &*GPIO::PTR }.pcpu_int().read().bits(),
                crate::Core::AppCpu => unsafe { &*GPIO::PTR }.acpu_int().read().bits(),
            },
            Self::Ban1 => match crate::get_core() {
                crate::Core::ProCpu => unsafe { &*GPIO::PTR }.pcpu_int1().read().bits(),
                crate::Core::AppCpu => unsafe { &*GPIO::PTR }.acpu_int1().read().bits(),
            },
        }
    }
}
