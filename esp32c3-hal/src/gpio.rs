use esp_hal_common::gpio::{types::*, *};

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
