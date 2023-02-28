use paste::paste;

use crate::{
    gpio::{
        AlternateFunction,
        Bank0GpioRegisterAccess,
        Bank1GpioRegisterAccess,
        GpioPin,
        InputOutputAnalogPinType,
        InputOutputPinType,
        Unknown,
    },
    peripherals::GPIO,
};

pub const NUM_PINS: usize = 46;

pub type OutputSignalType = u16;
pub const OUTPUT_SIGNAL_MAX: u16 = 256;
pub const INPUT_SIGNAL_MAX: u16 = 204;

pub const ONE_INPUT: u8 = 0x38;
pub const ZERO_INPUT: u8 = 0x3c;

pub(crate) const GPIO_FUNCTION: AlternateFunction = AlternateFunction::Function1;

pub(crate) const fn get_io_mux_reg(gpio_num: u8) -> &'static crate::peripherals::io_mux::GPIO0 {
    unsafe {
        let iomux = &*crate::peripherals::IO_MUX::PTR;

        match gpio_num {
            0 => core::mem::transmute(&(iomux.gpio0)),
            1 => core::mem::transmute(&(iomux.gpio1)),
            2 => core::mem::transmute(&(iomux.gpio2)),
            3 => core::mem::transmute(&(iomux.gpio3)),
            4 => core::mem::transmute(&(iomux.gpio4)),
            5 => core::mem::transmute(&(iomux.gpio5)),
            6 => core::mem::transmute(&(iomux.gpio6)),
            7 => core::mem::transmute(&(iomux.gpio7)),
            8 => core::mem::transmute(&(iomux.gpio8)),
            9 => core::mem::transmute(&(iomux.gpio9)),
            10 => core::mem::transmute(&(iomux.gpio10)),
            11 => core::mem::transmute(&(iomux.gpio11)),
            12 => core::mem::transmute(&(iomux.gpio12)),
            13 => core::mem::transmute(&(iomux.gpio13)),
            14 => core::mem::transmute(&(iomux.gpio14)),
            15 => core::mem::transmute(&(iomux.gpio15)),
            16 => core::mem::transmute(&(iomux.gpio16)),
            17 => core::mem::transmute(&(iomux.gpio17)),
            18 => core::mem::transmute(&(iomux.gpio18)),
            19 => core::mem::transmute(&(iomux.gpio19)),
            20 => core::mem::transmute(&(iomux.gpio20)),
            21 => core::mem::transmute(&(iomux.gpio21)),
            26 => core::mem::transmute(&(iomux.gpio26)),
            27 => core::mem::transmute(&(iomux.gpio27)),
            32 => core::mem::transmute(&(iomux.gpio32)),
            33 => core::mem::transmute(&(iomux.gpio33)),
            34 => core::mem::transmute(&(iomux.gpio34)),
            35 => core::mem::transmute(&(iomux.gpio35)),
            36 => core::mem::transmute(&(iomux.gpio36)),
            37 => core::mem::transmute(&(iomux.gpio37)),
            38 => core::mem::transmute(&(iomux.gpio38)),
            39 => core::mem::transmute(&(iomux.gpio39)),
            40 => core::mem::transmute(&(iomux.gpio40)),
            41 => core::mem::transmute(&(iomux.gpio41)),
            42 => core::mem::transmute(&(iomux.gpio42)),
            43 => core::mem::transmute(&(iomux.gpio43)),
            44 => core::mem::transmute(&(iomux.gpio44)),
            45 => core::mem::transmute(&(iomux.gpio45)),
            46 => core::mem::transmute(&(iomux.gpio46)),
            _ => panic!(),
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
    RMT_SIG_IN0       = 83,
    RMT_SIG_IN1       = 84,
    RMT_SIG_IN2       = 85,
    RMT_SIG_IN3       = 86,
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
    RMT_SIG_OUT0     = 87,
    RMT_SIG_OUT1     = 88,
    RMT_SIG_OUT2     = 89,
    RMT_SIG_OUT3     = 90,
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
    Single,
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

// on ESP32-S2 the touch_pad registers are indexed and the fields are weirdly
// named
macro_rules! impl_get_rtc_pad {
    ($pad_name:ident) => {
        paste!{
            pub(crate) fn [<esp32s2_get_rtc_pad_ $pad_name >]() -> &'static crate::peripherals::rtcio::[< $pad_name:upper >] {
                use crate::peripherals::RTCIO;
                let rtcio = unsafe{ &*RTCIO::ptr() };
                &rtcio.$pad_name
            }
        }
    };
}

macro_rules! impl_get_rtc_pad_indexed {
    ($pad_name:ident, $idx:literal) => {
        paste!{
            pub(crate) fn [<esp32s2_get_rtc_pad_ $pad_name $idx>]() -> &'static crate::peripherals::rtcio::[< $pad_name:upper >] {
                use crate::peripherals::RTCIO;
                let rtcio = unsafe{ &*RTCIO::ptr() };
                &rtcio.$pad_name[$idx]
            }
        }
    };
}

impl_get_rtc_pad_indexed!(touch_pad, 0);
impl_get_rtc_pad_indexed!(touch_pad, 1);
impl_get_rtc_pad_indexed!(touch_pad, 2);
impl_get_rtc_pad_indexed!(touch_pad, 3);
impl_get_rtc_pad_indexed!(touch_pad, 4);
impl_get_rtc_pad_indexed!(touch_pad, 5);
impl_get_rtc_pad_indexed!(touch_pad, 6);
impl_get_rtc_pad_indexed!(touch_pad, 7);
impl_get_rtc_pad_indexed!(touch_pad, 8);
impl_get_rtc_pad_indexed!(touch_pad, 9);
impl_get_rtc_pad_indexed!(touch_pad, 10);
impl_get_rtc_pad_indexed!(touch_pad, 11);
impl_get_rtc_pad_indexed!(touch_pad, 12);
impl_get_rtc_pad_indexed!(touch_pad, 13);
impl_get_rtc_pad_indexed!(touch_pad, 14);
impl_get_rtc_pad!(xtal_32p_pad);
impl_get_rtc_pad!(xtal_32n_pad);
impl_get_rtc_pad!(pad_dac1);
impl_get_rtc_pad!(pad_dac2);
impl_get_rtc_pad!(rtc_pad19);
impl_get_rtc_pad!(rtc_pad20);
impl_get_rtc_pad!(rtc_pad21);

crate::gpio::analog! {
    ( 0,  0,  touch_pad0,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    ( 1,  1,  touch_pad1,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    ( 2,  2,  touch_pad2,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    ( 3,  3,  touch_pad3,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    ( 4,  4,  touch_pad4,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    ( 5,  5,  touch_pad5,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    ( 6,  6,  touch_pad6,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    ( 7,  7,  touch_pad7,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    ( 8,  8,  touch_pad8,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    ( 9,  9,  touch_pad9,     touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    (10, 10,  touch_pad10,    touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    (11, 11,  touch_pad11,    touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    (12, 12,  touch_pad12,    touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    (13, 13,  touch_pad13,    touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    (14, 14,  touch_pad14,    touch_pad0_mux_sel,  touch_pad0_fun_sel,  touch_pad0_fun_ie,  touch_pad0_rue,  touch_pad0_rde)
    (15, 15,  xtal_32p_pad,   x32p_mux_sel,        x32p_fun_sel,        x32p_fun_ie,        x32p_rue,        x32p_rde)
    (16, 16,  xtal_32n_pad,   x32n_mux_sel,        x32n_fun_sel,        x32n_fun_ie,        x32n_rue,        x32n_rde)
    (17, 17,  pad_dac1,       pdac1_mux_sel,       pdac1_fun_sel,       pdac1_fun_ie,       pdac1_rue,       pdac1_rde)
    (18, 18,  pad_dac2,       pdac2_mux_sel,       pdac2_fun_sel,       pdac2_fun_ie,       pdac2_rue,       pdac2_rde)
    (19, 19,  rtc_pad19,      mux_sel,             fun_sel,             fun_ie,             rue,             rde)
    (20, 20,  rtc_pad20,      mux_sel,             fun_sel,             fun_ie,             rue,             rde)
    (21, 21,  rtc_pad21,      mux_sel,             fun_sel,             fun_ie,             rue,             rde)
}

// implement marker traits on USB pins
impl<T> crate::otg_fs::UsbSel for Gpio18<T> {}
impl<T> crate::otg_fs::UsbDp for Gpio19<T> {}
impl<T> crate::otg_fs::UsbDm for Gpio20<T> {}
