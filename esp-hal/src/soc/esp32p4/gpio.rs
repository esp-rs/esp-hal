//! # GPIO configuration module (ESP32-P4)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32-P4` chip. It allows you to
//! configure pins as inputs or outputs, set their state and read their state.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
//!   - `get_io_mux_reg(gpio_num: u8) -> &'static
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
//! This module also implements the `InterruptStatusRegisterAccess` trait for
//! two different banks:
//!   * `InterruptStatusRegisterAccessBank0`
//!   * `InterruptStatusRegisterAccessBank1`.
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
        Unknown,
    },
    peripherals::GPIO,
};

pub const NUM_PINS: usize = 55;

pub(crate) const FUNC_IN_SEL_OFFSET: usize = 1;

pub type OutputSignalType = u16;
pub const OUTPUT_SIGNAL_MAX: u16 = 256;
pub const INPUT_SIGNAL_MAX: u16 = 203;

pub const ONE_INPUT: u8 = 0x3f;
pub const ZERO_INPUT: u8 = 0x3e;

pub(crate) const GPIO_FUNCTION: AlternateFunction = AlternateFunction::Function1;

pub(crate) const fn get_io_mux_reg(gpio_num: u8) -> &'static crate::peripherals::io_mux::GPIO {
    unsafe { &(&*crate::peripherals::IO_MUX::PTR).gpio(gpio_num as usize) }
}

pub(crate) fn gpio_intr_enable(int_enable: bool, _nmi_enable: bool) -> u8 {
    // there is no nmi_enable
    int_enable as u8
}

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
pub enum InputSignal {
    SD_CARD_CCMD_2_PAD      = 1,
    SD_CARD_CDATA0_2_PAD    = 2,
    SD_CARD_CDATA1_2_PAD    = 3,
    SD_CARD_CDATA2_2_PAD    = 4,
    SD_CARD_CDATA3_2_PAD    = 5,
    SD_CARD_CDATA4_2_PAD    = 6,
    SD_CARD_CDATA5_2_PAD    = 7,
    SD_CARD_CDATA6_2_PAD    = 8,
    SD_CARD_CDATA7_2_PAD    = 9,
    UART0_RXD_PAD           = 10,
    UART0_CTS_PAD           = 11,
    UART0_DSR_PAD           = 12,
    UART1_RXD_PAD           = 13,
    UART1_CTS_PAD           = 14,
    UART1_DSR_PAD           = 15,
    UART2_RXD_PAD           = 16,
    UART2_CTS_PAD           = 17,
    UART2_DSR_PAD           = 18,
    UART3_RXD_PAD           = 19,
    UART3_CTS_PAD           = 20,
    UART3_DSR_PAD           = 21,
    UART4_RXD_PAD           = 22,
    UART4_CTS_PAD           = 23,
    UART4_DSR_PAD           = 24,
    I2S0_O_BCK_PAD          = 25,
    I2S0_MCLK_PAD           = 26,
    I2S0_O_WS_PAD           = 27,
    I2S0_I_SD_PAD           = 28,
    I2S0_I_BCK_PAD          = 29,
    I2S0_I_WS_PAD           = 30,
    I2S1_O_BCK_PAD          = 31,
    I2S1_MCLK_PAD           = 32,
    I2S1_O_WS_PAD           = 33,
    I2S1_I_SD_PAD           = 34,
    I2S1_I_BCK_PAD          = 35,
    I2S1_I_WS_PAD           = 36,
    I2S2_O_BCK_PAD          = 37,
    I2S2_MCLK_PAD           = 38,
    I2S2_O_WS_PAD           = 39,
    I2S2_I_SD_PAD           = 40,
    I2S2_I_BCK_PAD          = 41,
    I2S2_I_WS_PAD           = 42,
    I2S0_I_SD1_PAD          = 43,
    I2S0_I_SD2_PAD          = 44,
    I2S0_I_SD3_PAD          = 45,
    SPI3_CK_PAD             = 47,
    SPI3_Q_PAD              = 48,
    SPI3_D_PAD              = 49,
    SPI3_HOLD_PAD           = 50,
    SPI3_WP_PAD             = 51,
    SPI3_CS_PAD             = 52,
    SPI2_CK_PAD             = 53,
    SPI2_Q_PAD              = 54,
    SPI2_D_PAD              = 55,
    SPI2_HOLD_PAD           = 56,
    SPI2_WP_PAD             = 57,
    SPI2_IO4_PAD            = 58,
    SPI2_IO5_PAD            = 59,
    SPI2_IO6_PAD            = 60,
    SPI2_IO7_PAD            = 61,
    SPI2_CS_PAD             = 62,
    I2C0_SCL_PAD            = 68,
    I2C0_SDA_PAD            = 69,
    I2C1_SCL_PAD            = 70,
    I2C1_SDA_PAD            = 71,
    UART0_SLP_CLK_PAD       = 74,
    UART1_SLP_CLK_PAD       = 75,
    UART2_SLP_CLK_PAD       = 76,
    UART3_SLP_CLK_PAD       = 77,
    UART4_SLP_CLK_PAD       = 78,
    TWAI0_RX_PAD            = 80,
    TWAI1_RX_PAD            = 83,
    TWAI2_RX_PAD            = 86,
    PWM0_SYNC0_PAD          = 89,
    PWM0_SYNC1_PAD          = 90,
    PWM0_SYNC2_PAD          = 91,
    PWM0_F0_PAD             = 92,
    PWM0_F1_PAD             = 93,
    PWM0_F2_PAD             = 94,
    PWM0_CAP0_PAD           = 95,
    PWM0_CAP1_PAD           = 96,
    PWM0_CAP2_PAD           = 97,
    PWM1_SYNC0_PAD          = 98,
    PWM1_SYNC1_PAD          = 99,
    PWM1_SYNC2_PAD          = 100,
    PWM1_F0_PAD             = 101,
    PWM1_F1_PAD             = 102,
    PWM1_F2_PAD             = 103,
    PWM1_CAP0_PAD           = 104,
    PWM1_CAP1_PAD           = 105,
    PWM1_CAP2_PAD           = 106,
    GMII_MDI_PAD            = 107,
    GMAC_PHY_COL_PAD        = 108,
    GMAC_PHY_CRS_PAD        = 109,
    USB_OTG11_IDDIG_PAD     = 110,
    USB_OTG11_AVALID_PAD    = 111,
    USB_SRP_BVALID_PAD      = 112,
    USB_OTG11_VBUSVALID_PAD = 113,
    USB_SRP_SESSEND_PAD     = 114,
    ULPI_CLK_PAD            = 117,
    USB_HSPHY_REFCLK        = 118,
    SD_CARD_DETECT_N_1_PAD  = 126,
    SD_CARD_DETECT_N_2_PAD  = 127,
    SD_CARD_INT_N_1_PAD     = 128,
    SD_CARD_INT_N_2_PAD     = 129,
    SD_CARD_WRITE_PRT_1_PAD = 130,
    SD_CARD_WRITE_PRT_2_PAD = 131,
    SD_DATA_STROBE_1_PAD    = 132,
    SD_DATA_STROBE_2_PAD    = 133,
    I3C_MST_SCL_PAD         = 134,
    I3C_MST_SDA_PAD         = 135,
    I3C_SLV_SCL_PAD         = 136,
    I3C_SLV_SDA_PAD         = 137,
    ADP_PRB_PAD             = 138,
    ADP_SNS_PAD             = 139,
    USB_JTAG_TDO_BRIDGE_PAD = 140,
    CAM_PCLK_PAD            = 158,
    CAM_H_ENABLE_PAD        = 159,
    CAM_H_SYNC_PAD          = 160,
    CAM_V_SYNC_PAD          = 161,
    GMAC_PHY_RXDV_PAD       = 178,
    GMAC_PHY_RXD0_PAD       = 179,
    GMAC_PHY_RXD1_PAD       = 180,
    GMAC_PHY_RXD2_PAD       = 181,
    GMAC_PHY_RXD3_PAD       = 182,
    GMAC_PHY_RXER_PAD       = 183,
    GMAC_RX_CLK_PAD         = 184,
    GMAC_TX_CLK_PAD         = 185,
    PARLIO_RX_CLK_PAD       = 186,
    PARLIO_TX_CLK_PAD       = 187,
    PARLIO_RX_DATA0_PAD     = 188,
    PARLIO_RX_DATA1_PAD     = 189,
    PARLIO_RX_DATA2_PAD     = 190,
    PARLIO_RX_DATA3_PAD     = 191,
    PARLIO_RX_DATA4_PAD     = 192,
    PARLIO_RX_DATA5_PAD     = 193,
    PARLIO_RX_DATA6_PAD     = 194,
    PARLIO_RX_DATA7_PAD     = 195,
    PARLIO_RX_DATA8_PAD     = 196,
    PARLIO_RX_DATA9_PAD     = 197,
    PARLIO_RX_DATA10_PAD    = 198,
    PARLIO_RX_DATA11_PAD    = 199,
    PARLIO_RX_DATA12_PAD    = 200,
    PARLIO_RX_DATA13_PAD    = 201,
    PARLIO_RX_DATA14_PAD    = 202,
    PARLIO_RX_DATA15_PAD    = 203,
}

/// Peripheral output signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
pub enum OutputSignal {
    SD_CARD_CCLK_2_PAD         = 0,
    SD_CARD_CCMD_2_PAD         = 1,
    SD_CARD_CDATA0_2_PAD       = 2,
    SD_CARD_CDATA1_2_PAD       = 3,
    SD_CARD_CDATA2_2_PAD       = 4,
    SD_CARD_CDATA3_2_PAD       = 5,
    SD_CARD_CDATA4_2_PAD       = 6,
    SD_CARD_CDATA5_2_PAD       = 7,
    SD_CARD_CDATA6_2_PAD       = 8,
    SD_CARD_CDATA7_2_PAD       = 9,
    UART0_TXD_PAD              = 10,
    UART0_RTS_PAD              = 11,
    UART0_DTR_PAD              = 12,
    UART1_TXD_PAD              = 13,
    UART1_RTS_PAD              = 14,
    UART1_DTR_PAD              = 15,
    UART2_TXD_PAD              = 16,
    UART2_RTS_PAD              = 17,
    UART2_DTR_PAD              = 18,
    UART3_TXD_PAD              = 19,
    UART3_RTS_PAD              = 20,
    UART3_DTR_PAD              = 21,
    UART4_TXD_PAD              = 22,
    UART4_RTS_PAD              = 23,
    UART4_DTR_PAD              = 24,
    I2S0_O_BCK_PAD             = 25,
    I2S0_MCLK_PAD              = 26,
    I2S0_O_WS_PAD              = 27,
    I2S0_O_SD_PAD              = 28,
    I2S0_I_BCK_PAD             = 29,
    I2S0_I_WS_PAD              = 30,
    I2S1_O_BCK_PAD             = 31,
    I2S1_MCLK_PAD              = 32,
    I2S1_O_WS_PAD              = 33,
    I2S1_O_SD_PAD              = 34,
    I2S1_I_BCK_PAD             = 35,
    I2S1_I_WS_PAD              = 36,
    I2S2_O_BCK_PAD             = 37,
    I2S2_MCLK_PAD              = 38,
    I2S2_O_WS_PAD              = 39,
    I2S2_O_SD_PAD              = 40,
    I2S2_I_BCK_PAD             = 41,
    I2S2_I_WS_PAD              = 42,
    I2S0_O_SD1_PAD             = 43,
    SPI2_DQS_PAD               = 44,
    SPI3_CS2_PAD               = 45,
    SPI3_CS1_PAD               = 46,
    SPI3_CK_PAD                = 47,
    SPI3_QO_PAD                = 48,
    SPI3_D_PAD                 = 49,
    SPI3_HOLD_PAD              = 50,
    SPI3_WP_PAD                = 51,
    SPI3_CS_PAD                = 52,
    SPI2_CK_PAD                = 53,
    SPI2_Q_PAD                 = 54,
    SPI2_D_PAD                 = 55,
    SPI2_HOLD_PAD              = 56,
    SPI2_WP_PAD                = 57,
    SPI2_IO4_PAD               = 58,
    SPI2_IO5_PAD               = 59,
    SPI2_IO6_PAD               = 60,
    SPI2_IO7_PAD               = 61,
    SPI2_CS_PAD                = 62,
    SPI2_CS1_PAD               = 63,
    SPI2_CS2_PAD               = 64,
    SPI2_CS3_PAD               = 65,
    SPI2_CS4_PAD               = 66,
    SPI2_CS5_PAD               = 67,
    I2C0_SCL_PAD               = 68,
    I2C0_SDA_PAD               = 69,
    I2C1_SCL_PAD               = 70,
    I2C1_SDA_PAD               = 71,
    GPIO_SD0                   = 72,
    GPIO_SD1                   = 73,
    GPIO_SD2                   = 74,
    GPIO_SD3                   = 75,
    GPIO_SD4                   = 76,
    GPIO_SD5                   = 77,
    GPIO_SD6                   = 78,
    GPIO_SD7                   = 79,
    TWAI0_TX_PAD               = 80,
    TWAI0_BUS_OFF_ON_PAD       = 81,
    TWAI0_CLKOUT_PAD           = 82,
    TWAI1_TX_PAD               = 83,
    TWAI1_BUS_OFF_ON_PAD       = 84,
    TWAI1_CLKOUT_PAD           = 85,
    TWAI2_TX_PAD               = 86,
    TWAI2_BUS_OFF_ON_PAD       = 87,
    TWAI2_CLKOUT_PAD           = 88,
    PWM0_CH0_A_PAD             = 89,
    PWM0_CH0_B_PAD             = 90,
    PWM0_CH1_A_PAD             = 91,
    PWM0_CH1_B_PAD             = 92,
    PWM0_CH2_A_PAD             = 93,
    PWM0_CH2_B_PAD             = 94,
    PWM1_CH0_A_PAD             = 95,
    PWM1_CH0_B_PAD             = 96,
    PWM1_CH1_A_PAD             = 97,
    PWM1_CH1_B_PAD             = 98,
    PWM1_CH2_A_PAD             = 99,
    PWM1_CH2_B_PAD             = 100,
    ADP_CHRG_PAD               = 101,
    ADP_DISCHRG_PAD            = 102,
    ADP_PRB_EN_PAD             = 103,
    ADP_SNS_EN_PAD             = 104,
    TWAI0_STANDBY_PAD          = 105,
    TWAI1_STANDBY_PAD          = 106,
    TWAI2_STANDBY_PAD          = 107,
    GMII_MDC_PAD               = 108,
    GMII_MDO_PAD               = 109,
    USB_SRP_DISCHRGVBUS_PAD    = 110,
    USB_OTG11_IDPULLUP_PAD     = 111,
    USB_OTG11_DPPULLDOWN_PAD   = 112,
    USB_OTG11_DMPULLDOWN_PAD   = 113,
    USB_OTG11_DRVVBUS_PAD      = 114,
    USB_SRP_CHRGVBUS_PAD       = 115,
    OTG_DRVVBUS_PAD            = 116,
    RNG_CHAIN_CLK_PAD          = 117,
    I3C_MST_SCL_PAD            = 134,
    I3C_MST_SDA_PAD            = 135,
    I3C_SLV_SCL_PAD            = 136,
    I3C_SLV_SDA_PAD            = 137,
    I3C_MST_SCL_PULLUP_EN_PAD  = 138,
    I3C_MST_SDA_PULLUP_EN_PAD  = 139,
    USB_JTAG_TDI_BRIDGE_PAD    = 140,
    USB_JTAG_TMS_BRIDGE_PAD    = 141,
    USB_JTAG_TCK_BRIDGE_PAD    = 142,
    USB_JTAG_TRST_BRIDGE_PAD   = 143,
    LCD_CS_PAD                 = 144,
    LCD_DC_PAD                 = 145,
    SD_RST_N_1_PAD             = 146,
    SD_RST_N_2_PAD             = 147,
    SD_CCMD_OD_PULLUP_EN_N_PAD = 148,
    LCD_PCLK_PAD               = 149,
    CAM_CLK_PAD                = 150,
    LCD_H_ENABLE_PAD           = 151,
    LCD_H_SYNC_PAD             = 152,
    LCD_V_SYNC_PAD             = 153,
    GMAC_PHY_TXEN_PAD          = 178,
    GMAC_PHY_TXD0_PAD          = 179,
    GMAC_PHY_TXD1_PAD          = 180,
    GMAC_PHY_TXD2_PAD          = 181,
    GMAC_PHY_TXD3_PAD          = 182,
    GMAC_PHY_TXER_PAD          = 183,
    PARLIO_RX_CLK_PAD          = 186,
    PARLIO_TX_CLK_PAD          = 187,
    PARLIO_TX_DATA0_PAD        = 188,
    PARLIO_TX_DATA1_PAD        = 189,
    PARLIO_TX_DATA2_PAD        = 190,
    PARLIO_TX_DATA3_PAD        = 191,
    PARLIO_TX_DATA4_PAD        = 192,
    PARLIO_TX_DATA5_PAD        = 193,
    PARLIO_TX_DATA6_PAD        = 194,
    PARLIO_TX_DATA7_PAD        = 195,
    PARLIO_TX_DATA8_PAD        = 196,
    PARLIO_TX_DATA9_PAD        = 197,
    PARLIO_TX_DATA10_PAD       = 198,
    PARLIO_TX_DATA11_PAD       = 199,
    PARLIO_TX_DATA12_PAD       = 200,
    PARLIO_TX_DATA13_PAD       = 201,
    PARLIO_TX_DATA14_PAD       = 202,
    PARLIO_TX_DATA15_PAD       = 203,
    CONSTANT0_PAD              = 212,
    CONSTANT1_PAD              = 213,
    GPIO                       = 256,
}

crate::gpio::gpio! {
    (0, 0, InputOutput)
    (1, 0, InputOutput)
    (2, 0, InputOutput)
    (3, 0, InputOutput)
    (4, 0, InputOutput)
    (5, 0, InputOutput)
    (6, 0, InputOutput (3 => SPI2_HOLD_PAD) (3 => SPI2_HOLD_PAD))
    (7, 0, InputOutput (3 => SPI2_CS_PAD) (3 => SPI2_CS_PAD))
    (8, 0, InputOutput (3 => SPI2_D_PAD) (2 => UART0_RTS_PAD 3 => SPI2_D_PAD))
    (9, 0, InputOutput (2 => UART0_CTS_PAD) (3 => SPI2_CK_PAD))
    (10, 0, InputOutput (3 => SPI2_Q_PAD) (2 => UART1_TXD_PAD))
    (11, 0, InputOutput (2 => UART1_RXD_PAD 3 => SPI2_WP_PAD) (3 => SPI2_WP_PAD))
    (12, 0, InputOutput () (2 => UART1_RTS_PAD))
    (13, 0, InputOutput (2 => UART1_CTS_PAD) ())
    (14, 0, InputOutput)
    (15, 0, InputOutput)
    (16, 0, InputOutput)
    (17, 0, InputOutput)
    (18, 0, InputOutput)
    (19, 0, InputOutput)
    (20, 0, InputOutput)
    (21, 0, InputOutput)
    (22, 0, InputOutput)
    (23, 0, InputOutput)
    (24, 0, InputOutput)
    (25, 0, InputOutput)
    (26, 0, InputOutput)
    (27, 0, InputOutput)
    (28, 0, InputOutput (2 => SPI2_CS_PAD 3 => GMAC_PHY_RXDV_PAD) ())
    (29, 0, InputOutput (2 => SPI2_D_PAD 3 => GMAC_PHY_RXD0_PAD) ())
    (30, 0, InputOutput (3 => GMAC_PHY_RXD1_PAD) (2 => SPI2_CK_PAD))
    (31, 0, InputOutput (2 => SPI2_Q_PAD 3 => GMAC_PHY_RXER_PAD) ())
    (32, 1, InputOutput (2 => SPI2_HOLD_PAD 3 => GMAC_RX_CLK_PAD) (2 => SPI2_HOLD_PAD))
    (33, 1, InputOutput (2 => SPI2_WP_PAD) (2 => SPI2_WP_PAD 3 => GMAC_PHY_TXEN_PAD))
    (34, 1, InputOutput (2 => SPI2_IO4_PAD) (2 => SPI2_IO4_PAD 3 => GMAC_PHY_TXD0_PAD))
    (35, 1, InputOutput (2 => SPI2_IO5_PAD) (2 => SPI2_IO5_PAD 3 => GMAC_PHY_TXD1_PAD))
    (36, 1, InputOutput (2 => SPI2_IO6_PAD) (2 => SPI2_IO6_PAD 3 => GMAC_PHY_TXER_PAD))
    (37, 1, InputOutput (2 => SPI2_IO7_PAD) (2 => SPI2_IO7_PAD))
    (38, 1, InputOutput (2 => SPI2_Q_PAD) (2 => SPI2_Q_PAD))
    (39, 1, InputOutput)
    (40, 1, InputOutput () (3 => GMAC_PHY_TXEN_PAD))
    (41, 1, InputOutput () (3 => GMAC_PHY_TXD0_PAD))
    (42, 1, InputOutput () (3 => GMAC_PHY_TXD1_PAD))
    (43, 1, InputOutput () (3 => GMAC_PHY_TXER_PAD))
    (44, 1, InputOutput (3 => GMAC_RX_CLK_PAD) ())
    (45, 1, InputOutput (3 => GMAC_PHY_RXDV_PAD) ())
    (46, 1, InputOutput (3 => GMAC_PHY_RXD0_PAD) ())
    (47, 1, InputOutput (3 => GMAC_PHY_RXD1_PAD) ())
    (48, 1, InputOutput (3 => GMAC_PHY_RXER_PAD) ())
    (49, 1, InputOutput () (3 => GMAC_PHY_TXEN_PAD))
    (50, 1, InputOutput (3 => GMAC_RX_CLK_PAD) ())
    (51, 1, InputOutput (3 => GMAC_PHY_RXDV_PAD) ())
    (52, 1, InputOutput (3 => GMAC_PHY_RXD0_PAD) ())
    (53, 1, InputOutput (3 => GMAC_PHY_RXD1_PAD) ())
    (54, 1, InputOutput (3 => GMAC_PHY_RXER_PAD) ())
}

// crate::gpio::analog! {
// }

// crate::gpio::lp_gpio::lp_gpio! {
//     0
//     1
//     2
//     3
//     4
//     5
//     6
//     7
//     9
//     10
//     11
//     12
//     13
//     13
//     15
// }

// ESP32-P4 supports four interrupts for GPIO - for now just using INTR_0
impl InterruptStatusRegisterAccess for InterruptStatusRegisterAccessBank0 {
    fn pro_cpu_interrupt_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.intr_0().read().bits()
    }

    fn pro_cpu_nmi_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.intr_0().read().bits()
    }
}

impl InterruptStatusRegisterAccess for InterruptStatusRegisterAccessBank1 {
    fn pro_cpu_interrupt_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.intr1_0().read().bits()
    }

    fn pro_cpu_nmi_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.intr1_0().read().bits()
    }
}

// TODO USB pins
// implement marker traits on USB pins
