use paste::paste;

use crate::{
    gpio::{
        AlternateFunction,
        Bank0GpioRegisterAccess,
        GpioPin,
        InputOutputAnalogPinType,
        InputOutputPinType,
        Unknown,
    },
    peripherals::GPIO,
};

// https://github.com/espressif/esp-idf/blob/df9310a/components/soc/esp32h2/gpio_periph.c#L42
pub const NUM_PINS: usize = 27;

pub type OutputSignalType = u8;
pub const OUTPUT_SIGNAL_MAX: u8 = 128;
pub const INPUT_SIGNAL_MAX: u8 = 124;

pub const ONE_INPUT: u8 = 0x1e;
pub const ZERO_INPUT: u8 = 0x1f;

pub(crate) const GPIO_FUNCTION: AlternateFunction = AlternateFunction::Function1;

pub(crate) const fn get_io_mux_reg(gpio_num: u8) -> &'static crate::peripherals::io_mux::GPIO {
    unsafe { &(&*crate::peripherals::IO_MUX::PTR).gpio[gpio_num as usize] }
}

pub(crate) fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    int_enable as u8 | ((nmi_enable as u8) << 1)
}

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
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
    RMT_SIG0            = 71,
    RMT_SIG1            = 72,
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
    PCNT_SIG_CH00       = 101,
    PCNT_SIG_CH10       = 102,
    PCNT_CTRL_CH00      = 103,
    PCNT_CTRL_CH10      = 104,
    PCNT_SIG_CH01       = 105,
    PCNT_SIG_CH11       = 106,
    PCNT_CTRL_CH01      = 107,
    PCNT_CTRL_CH11      = 108,
    PCNT_SIG_CH02       = 109,
    PCNT_SIG_CH12       = 110,
    PCNT_CTRL_CH02      = 111,
    PCNT_CTRL_CH12      = 112,
    PCNT_SIG_CH03       = 113,
    PCNT_SIG_CH13       = 114,
    PCNT_CTRL_CH03      = 115,
    PCNT_CTRL_CH13      = 116,
    SPIQ                = 121,
    SPID                = 122,
    SPIHD               = 123,
    SPIWP               = 124,
}

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
pub enum OutputSignal {
    LEDC_LS_SIG_OUT0 = 0,
    LEDC_LS_SIG_OUT1 = 1,
    LEDC_LS_SIG_OUT2 = 2,
    LEDC_LS_SIG_OUT3 = 3,
    LEDC_LS_SIG_OUT4 = 4,
    LEDC_LS_SIG_OUT5 = 5,
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
    FSPICLK_OUT_MUX  = 63,
    FSPIQ            = 64,
    FSPID            = 65,
    FSPIHD           = 66,
    FSPIWP           = 67,
    FSPICS0          = 68,
    PARL_RX_CLK      = 69,
    PARL_TX_CLK      = 70,
    RMT_SIG_OUT0     = 71,
    RMT_SIG_OUT1     = 72,
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
    SPICLK_OUT_MUX   = 114,
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

// FIXME: add alternate function numbers/signals where necessary
crate::gpio::gpio! {
    Single,
    (0, 0, InputOutput)
    (1, 0, InputOutputAnalog)
    (2, 0, InputOutputAnalog)
    (3, 0, InputOutputAnalog)
    (4, 0, InputOutputAnalog)
    (5, 0, InputOutputAnalog)
    (6, 0, InputOutput)
    (7, 0, InputOutput)
    (8, 0, InputOutput)
    (9, 0, InputOutput)
    (10, 0, InputOutput)
    (11, 0, InputOutput)
    (12, 0, InputOutput)
    (13, 0, InputOutput)
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
}

crate::gpio::analog! {
    1
    2
    3
    4
    5
}

// TODO USB pins
// implement marker traits on USB pins
// impl<T> crate::otg_fs::UsbSel for Gpio??<T> {}
// impl<T> crate::otg_fs::UsbDp for Gpio27<T> {}
// impl<T> crate::otg_fs::UsbDm for Gpio26<T> {}
