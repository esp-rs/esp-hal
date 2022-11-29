use paste::paste;

use crate::{
    gpio::PhantomData,
    peripherals::GPIO,
    AlternateFunction,
    Bank0GpioRegisterAccess,
    Bank1GpioRegisterAccess,
    GpioPin,
    InputOutputAnalogPinType,
    InputOutputPinType,
    Unknown,
};

pub type OutputSignalType = u16;
pub const OUTPUT_SIGNAL_MAX: u16 = 256;
pub const INPUT_SIGNAL_MAX: u16 = 189;

pub const ONE_INPUT: u8 = 0x38;
pub const ZERO_INPUT: u8 = 0x3c;

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
    SPIQ              = 0,
    SPID              = 1,
    SPIHD             = 2,
    SPIWP             = 3,
    SPID4             = 7,
    SPID5             = 8,
    SPID6             = 9,
    SPID7             = 10,
    SPIDQS            = 11,
    U0RXD             = 12,
    U0CTS             = 13,
    U0DSR             = 14,
    U1RXD             = 15,
    U1CTS             = 16,
    U1DSR             = 17,
    U2RXD             = 18,
    U2CTS             = 19,
    U2DSR             = 20,
    I2S1_MCLK         = 21,
    I2S0O_BCK         = 22,
    I2S0_MCLK         = 23,
    I2S0O_WS          = 24,
    I2S0I_SD          = 25,
    I2S0I_BCK         = 26,
    I2S0I_WS          = 27,
    I2S1O_BCK         = 28,
    I2S1O_WS          = 29,
    I2S1I_SD          = 30,
    I2S1I_BCK         = 31,
    I2S1I_WS          = 32,
    I2S0I_SD1         = 51,
    I2S0I_SD2         = 52,
    I2S0I_SD3         = 53,
    USB_OTG_IDDIG     = 58,
    USB_OTG_AVALID    = 59,
    USB_SRP_BVALID    = 60,
    USB_OTG_VBUSVALID = 61,
    USB_SRP_SESSEND   = 62,
    SPI3_CLK          = 66,
    SPI3_Q            = 67,
    SPI3_D            = 68,
    SPI3_HD           = 69,
    SPI3_WP           = 70,
    SPI3_CS0          = 71,
    RMT_SIG_IN0       = 81,
    RMT_SIG_IN1       = 82,
    RMT_SIG_IN2       = 83,
    RMT_SIG_IN3       = 84,
    I2CEXT0_SCL       = 89,
    I2CEXT0_SDA       = 90,
    I2CEXT1_SCL       = 91,
    I2CEXT1_SDA       = 92,
    FSPICLK           = 101,
    FSPIQ             = 102,
    FSPID             = 103,
    FSPIHD            = 104,
    FSPIWP            = 105,
    FSPIIO4           = 106,
    FSPIIO5           = 107,
    FSPIIO6           = 108,
    FSPIIO7           = 109,
    FSPICS0           = 110,
    TWAI_RX           = 116,
    SUBSPIQ           = 120,
    SUBSPID           = 121,
    SUBSPIHD          = 122,
    SUBSPIWP          = 123,
    SUBSPID4          = 155,
    SUBSPID5          = 156,
    SUBSPID6          = 157,
    SUBSPID7          = 158,
    SUBSPIDQS         = 159,
    PWM0_SYNC0        = 160,
    PWM0_SYNC1        = 161,
    PWM0_SYNC2        = 162,
    PWM0_F0           = 163,
    PWM0_F1           = 164,
    PWM0_F2           = 165,
    PWM0_CAP0         = 166,
    PWM0_CAP1         = 167,
    PWM0_CAP2         = 168,
    PWM1_SYNC0        = 169,
    PWM1_SYNC1        = 170,
    PWM1_SYNC2        = 171,
    PWM1_F0           = 172,
    PWM1_F1           = 173,
    PWM1_F2           = 174,
    PWM1_CAP0         = 175,
    PWM1_CAP1         = 176,
    PWM1_CAP2         = 177,
    PCMFSYNC          = 188,
    PCMCLK            = 189,
}

/// Peripheral output signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
pub enum OutputSignal {
    SPIQ            = 0,
    SPID            = 1,
    SPIHD           = 2,
    SPIWP           = 3,
    SPICLK          = 4,
    SPICS0          = 5,
    SPICS1          = 6,
    SPID4           = 7,
    SPID5           = 8,
    SPID6           = 9,
    SPID7           = 10,
    SPIDQS          = 11,
    U0TXD           = 12,
    U0RTS           = 13,
    U0DTR           = 14,
    U1TXD           = 15,
    U1RTS           = 16,
    U1DTR           = 17,
    U2TXD           = 18,
    U2RTS           = 19,
    U2DTR           = 20,
    I2S1_MCLK       = 21,
    I2S0O_BCK       = 22,
    I2S0_MCLK       = 23,
    I2S0O_WS        = 24,
    I2S0O_SD        = 25,
    I2S0I_BCK       = 26,
    I2S0I_WS        = 27,
    I2S1O_BCK       = 28,
    I2S1O_WS        = 29,
    I2S1O_SD        = 30,
    I2S1I_BCK       = 31,
    I2S1I_WS        = 32,
    SPI3_CLK        = 66,
    SPI3_Q          = 67,
    SPI3_D          = 68,
    SPI3_HD         = 69,
    SPI3_WP         = 70,
    SPI3_CS0        = 71,
    SPI3_CS1        = 72,
    LEDC_LS_SIG0    = 73,
    LEDC_LS_SIG1    = 74,
    LEDC_LS_SIG2    = 75,
    LEDC_LS_SIG3    = 76,
    LEDC_LS_SIG4    = 77,
    LEDC_LS_SIG5    = 78,
    LEDC_LS_SIG6    = 79,
    LEDC_LS_SIG7    = 80,
    RMT_SIG_OUT0    = 81,
    RMT_SIG_OUT1    = 82,
    RMT_SIG_OUT2    = 83,
    RMT_SIG_OUT3    = 84,
    I2CEXT0_SCL     = 89,
    I2CEXT0_SDA     = 90,
    I2CEXT1_SCL     = 91,
    I2CEXT1_SDA     = 92,
    GPIO_SD0        = 93,
    GPIO_SD1        = 94,
    GPIO_SD2        = 95,
    GPIO_SD3        = 96,
    GPIO_SD4        = 97,
    GPIO_SD5        = 98,
    GPIO_SD6        = 99,
    GPIO_SD7        = 100,
    FSPICLK         = 101,
    FSPIQ           = 102,
    FSPID           = 103,
    FSPIHD          = 104,
    FSPIWP          = 105,
    FSPIIO4         = 106,
    FSPIIO5         = 107,
    FSPIIO6         = 108,
    FSPIIO7         = 109,
    FSPICS0         = 110,
    FSPICS1         = 111,
    FSPICS2         = 112,
    FSPICS3         = 113,
    FSPICS4         = 114,
    FSPICS5         = 115,
    TWAI_TX         = 116,
    SUBSPICLK       = 119,
    SUBSPIQ         = 120,
    SUBSPID         = 121,
    SUBSPIHD        = 122,
    SUBSPIWP        = 123,
    SUBSPICS0       = 124,
    SUBSPICS1       = 125,
    FSPIDQS         = 126,
    SPI3_CS2        = 127,
    I2S0O_SD1       = 128,
    SUBSPID4        = 155,
    SUBSPID5        = 156,
    SUBSPID6        = 157,
    SUBSPID7        = 158,
    SUBSPIDQS       = 159,
    PWM0_0A         = 160,
    PWM0_0B         = 161,
    PWM0_1A         = 162,
    PWM0_1B         = 163,
    PWM0_2A         = 164,
    PWM0_2B         = 165,
    PWM1_0A         = 166,
    PWM1_0B         = 167,
    PWM1_1A         = 168,
    PWM1_1B         = 169,
    PWM1_2A         = 170,
    PWM1_2B         = 171,
    SDIO_TOHOST_INT = 177,
    PCMFSYNC        = 194,
    PCMCLK          = 195,
    GPIO            = 256,
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
    (8, 0, InputOutputAnalog () (3 => SUBSPICS1))
    (9, 0, InputOutputAnalog (3 => SUBSPIHD 4 => FSPIHD) (3 => SUBSPIHD 4 => FSPIHD))
    (10, 0, InputOutputAnalog (2 => FSPIIO4 4 => FSPICS0) (2 => FSPIIO4 3 => SUBSPICS0 4 => FSPICS0))
    (11, 0, InputOutputAnalog (2 => FSPIIO5 3 => SUBSPID 4 => FSPID) (2 => FSPIIO5 3 => SUBSPID 4 => FSPID))
    (12, 0, InputOutputAnalog (2 => FSPIIO6 4 => FSPICLK) (2 => FSPIIO6 3=> SUBSPICLK 4 => FSPICLK))
    (13, 0, InputOutputAnalog (2 => FSPIIO7 3 => SUBSPIQ 4 => FSPIQ) (2 => FSPIIO7 3 => SUBSPIQ 4 => FSPIQ))
    (14, 0, InputOutputAnalog (3 => SUBSPIWP 4 => FSPIWP) (2 => FSPIDQS 3 => SUBSPIWP 4 => FSPIWP))
    (15, 0, InputOutputAnalog () (2 => U0RTS))
    (16, 0, InputOutputAnalog (2 => U0CTS) ())
    (17, 0, InputOutputAnalog () (2 => U1TXD))
    (18, 0, InputOutputAnalog (2 => U1RXD) ())
    (19, 0, InputOutputAnalog () (2 => U1RTS))
    (20, 0, InputOutputAnalog (2 => U1CTS) ())
    (21, 0, InputOutputAnalog)
    (26, 0, InputOutput)
    (27, 0, InputOutput)
    (28, 0, InputOutput)
    (29, 0, InputOutput)
    (30, 0, InputOutput)
    (31, 0, InputOutput)
    (32, 1, InputOutput)
    (33, 1, InputOutput (2 => FSPIHD 3 => SUBSPIHD) (2 => FSPIHD 3 => SUBSPIHD))
    (34, 1, InputOutput (2 => FSPICS0) (2 => FSPICS0 3 => SUBSPICS0))
    (35, 1, InputOutput (2 => FSPID 3 => SUBSPID) (2 => FSPID 3 => SUBSPID))
    (36, 1, InputOutput (2 => FSPICLK) (2 => FSPICLK 3 => SUBSPICLK))
    (37, 1, InputOutput (2 => FSPIQ 3 => SUBSPIQ 4 => SPIDQS) (2 => FSPIQ 3=> SUBSPIQ 4 => SPIDQS))
    (38, 1, InputOutput (2 => FSPIWP 3 => SUBSPIWP) (3 => FSPIWP 3 => SUBSPIWP))
    (39, 1, InputOutput () (4 => SUBSPICS1))
    (40, 1, InputOutput)
    (41, 1, InputOutput)
    (42, 1, InputOutput)
    (43, 1, InputOutput)
    (44, 1, InputOutput)
    (45, 1, InputOutput)
    (46, 1, InputOutput)
    (47, 1, InputOutput)
    (48, 1, InputOutput)
}

crate::gpio::analog! {
     ( 0,  0,  touch_pad0,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     ( 1,  1,  touch_pad1,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     ( 2,  2,  touch_pad2,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     ( 3,  3,  touch_pad3,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     ( 4,  4,  touch_pad4,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     ( 5,  5,  touch_pad5,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     ( 6,  6,  touch_pad6,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     ( 7,  7,  touch_pad7,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     ( 8,  8,  touch_pad8,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     ( 9,  9,  touch_pad9,     mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     (10, 10,  touch_pad10,    mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     (11, 11,  touch_pad11,    mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     (12, 12,  touch_pad12,    mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     (13, 13,  touch_pad13,    mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     (14, 14,  touch_pad14,    mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     (15, 15,  xtal_32p_pad,   x32p_mux_sel, x32p_fun_sel, x32p_fun_ie,    x32p_rue,  x32p_rde)
     (16, 16,  xtal_32n_pad,   x32n_mux_sel, x32n_fun_sel, x32n_fun_ie,    x32n_rue,  x32n_rde)
     (17, 17,  pad_dac1,       pdac1_mux_sel,pdac1_fun_sel,pdac1_fun_ie,  pdac1_rue, pdac1_rde)
     (18, 18,  pad_dac2,       pdac2_mux_sel,pdac2_fun_sel,pdac2_fun_ie,  pdac2_rue, pdac2_rde)
     (19, 19,  rtc_pad19,      mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     (20, 20,  rtc_pad20,      mux_sel,      fun_sel,      fun_ie,              rue,       rde)
     (21, 21,  rtc_pad21,      mux_sel,      fun_sel,      fun_ie,              rue,       rde)
}

// implement marker traits on USB pins
impl<T> crate::otg_fs::UsbSel for Gpio18<T> {}
impl<T> crate::otg_fs::UsbDp for Gpio19<T> {}
impl<T> crate::otg_fs::UsbDm for Gpio20<T> {}
