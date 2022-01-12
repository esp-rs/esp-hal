use esp_hal_common::gpio::*;

type OutputSignalType = u8;
const OUTPUT_SIGNAL_MAX: u8 = 128;
const INPUT_SIGNAL_MAX: u8 = 100;

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum InputSignal {
    SPIQ             = 0,
    SPID             = 1,
    SPIHD            = 2,
    SPIWP            = 3,
    U0RXD            = 6,
    U0CTS            = 7,
    U0DSR            = 8,
    U1RXD            = 9,
    U1CTS            = 10,
    U1DSR            = 11,
    I2S_MCLK         = 12,
    I2SO_BCK         = 13,
    I2SO_WS          = 14,
    I2SI_SD          = 15,
    I2SI_BCK         = 16,
    I2SI_WS          = 17,
    GPIO_BT_PRIORITY = 18,
    GPIO_BT_ACTIVE   = 19,
    CPU_GPIO_0       = 28,
    CPU_GPIO_1       = 29,
    CPU_GPIO_2       = 30,
    CPU_GPIO_3       = 31,
    CPU_GPIO_4       = 32,
    CPU_GPIO_5       = 33,
    CPU_GPIO_6       = 34,
    CPU_GPIO_7       = 35,
    EXT_ADC_START    = 45,
    RMT_SIG_0        = 51,
    RMT_SIG_1        = 52,
    I2CEXT0_SCL      = 53,
    I2CEXT0_SDA      = 54,
    FSPICLK          = 63,
    FSPIQ            = 64,
    FSPID            = 65,
    FSPIHD           = 66,
    FSPIWP           = 67,
    FSPICS0          = 68,
    TWAI_RX          = 74,
    SIG_FUNC_97      = 97,
    SIG_FUNC_98      = 98,
    SIG_FUNC_99      = 99,
    SIG_FUNC_100     = 100,
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum OutputSignal {
    SPIQ             = 0,
    SPID             = 1,
    SPIHD            = 2,
    SPIWP            = 3,
    SPICLK_MUX       = 4,
    SPICS0           = 5,
    U0TXD            = 6,
    U0RTS            = 7,
    U0DTR            = 8,
    U1TXD            = 9,
    U1RTS            = 10,
    U1DTR            = 11,
    I2S_MCLK         = 12,
    I2SO_BCK         = 13,
    I2SO_WS          = 14,
    I2SI_SD          = 15,
    I2SI_BCK         = 16,
    I2SI_WS          = 17,
    GPIO_WLAN_PRIO   = 18,
    GPIO_WLAN_ACTIVE = 19,
    CPU_GPIO_0       = 28,
    CPU_GPIO_1       = 29,
    CPU_GPIO_2       = 30,
    CPU_GPIO_3       = 31,
    CPU_GPIO_4       = 32,
    CPU_GPIO_5       = 33,
    CPU_GPIO_6       = 34,
    CPU_GPIO_7       = 35,
    USB_JTAG_TCK     = 36,
    USB_JTAG_TMS     = 37,
    USB_JTAG_TDI     = 38,
    USB_JTAG_TDO     = 39,
    LEDC_LS_SIG0     = 45,
    LEDC_LS_SIG1     = 46,
    LEDC_LS_SIG2     = 47,
    LEDC_LS_SIG3     = 48,
    LEDC_LS_SIG4     = 49,
    LEDC_LS_SIG5     = 50,
    RMT_SIG_0        = 51,
    RMT_SIG_1        = 52,
    I2CEXT0_SCL      = 53,
    I2CEXT0_SDA      = 54,
    GPIO_SD0         = 55,
    GPIO_SD1         = 56,
    GPIO_SD2         = 57,
    GPIO_SD3         = 58,
    I2SO_SD1         = 59,
    FSPICLK_MUX      = 63,
    FSPIQ            = 64,
    FSPID            = 65,
    FSPIHD           = 66,
    FSPIWP           = 67,
    FSPICS0          = 68,
    FSPICS1          = 69,
    FSPICS3          = 70,
    FSPICS2          = 71,
    FSPICS4          = 72,
    FSPICS5          = 73,
    TWAI_TX          = 74,
    TWAI_BUS_OFF_ON  = 75,
    TWAI_CLKOUT      = 76,
    ANT_SEL0         = 89,
    ANT_SEL1         = 90,
    ANT_SEL2         = 91,
    ANT_SEL3         = 92,
    ANT_SEL4         = 93,
    ANT_SEL5         = 94,
    ANT_SEL6         = 95,
    ANT_SEL7         = 96,
    SIG_FUNC_97      = 97,
    SIG_FUNC_98      = 98,
    SIG_FUNC_99      = 99,
    SIG_FUNC_100     = 100,
    CLK_OUT1         = 123,
    CLK_OUT2         = 124,
    CLK_OUT3         = 125,
    SPICS1           = 126,
    USB_JTAG_TRST    = 127,
    GPIO             = 128,
}

gpio! {
    Function1,
    SingleCore,

    Gpio0:  ( gpio0,  0,  gpio[0], IO, RTC, Bank0),
    Gpio1:  ( gpio1,  1,  gpio[1], IO, RTC, Bank0),
    Gpio2:  ( gpio2,  2,  gpio[2], IO, RTC, Bank0), (FSPIQ: Function2), (FSPIQ: Function2),
    Gpio3:  ( gpio3,  3,  gpio[3], IO, RTC, Bank0),
    Gpio4:  ( gpio4,  4,  gpio[4], IO, RTC, Bank0), (FSPIHD: Function2), (USB_JTAG_TMS: Function0, FSPIHD: Function2),
    Gpio5:  ( gpio5,  5,  gpio[5], IO, RTC, Bank0), (FSPIWP: Function2), (USB_JTAG_TDI: Function0, FSPIWP: Function2),
    Gpio6:  ( gpio6,  6,  gpio[6], IO,   0, Bank0), (FSPICLK: Function2), (USB_JTAG_TCK: Function0, FSPICLK_MUX: Function2),
    Gpio7:  ( gpio7,  7,  gpio[7], IO,   0, Bank0), (FSPID: Function2), (USB_JTAG_TDO: Function0, FSPID: Function2),
    Gpio8:  ( gpio8,  8,  gpio[8], IO,   0, Bank0),
    Gpio9:  ( gpio9,  9,  gpio[9], IO,   0, Bank0),
    Gpio10: (gpio10, 10, gpio[10], IO,   0, Bank0), (FSPICS0: Function2), (FSPICS0: Function2),
    Gpio11: (gpio11, 11, gpio[11], IO,   0, Bank0),
    Gpio12: (gpio12, 12, gpio[12], IO,   0, Bank0), (SPIHD: Function0), (SPIHD: Function0),
    Gpio13: (gpio13, 13, gpio[13], IO,   0, Bank0), (SPIWP: Function0), (SPIWP: Function0),
    Gpio14: (gpio14, 14, gpio[14], IO,   0, Bank0), (), (SPICS0: Function0),
    Gpio15: (gpio15, 15, gpio[15], IO,   0, Bank0), (), (SPICLK_MUX: Function0),
    Gpio16: (gpio16, 16, gpio[16], IO,   0, Bank0), (SPID: Function0), (SPID: Function0),
    Gpio17: (gpio17, 17, gpio[17], IO,   0, Bank0), (SPIQ: Function0), (SPIQ: Function0),
    Gpio18: (gpio18, 18, gpio[18], IO,   0, Bank0),
    Gpio19: (gpio19, 19, gpio[19], IO,   0, Bank0),
    Gpio20: (gpio20, 20, gpio[20], IO,   0, Bank0), (U0RXD: Function0), (),
    Gpio21: (gpio21, 21, gpio[21], IO,   0, Bank0), (), (U0TXD: Function0),
}
