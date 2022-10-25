//! General Purpose I/Os
//!
//! To get access to the pins, you first need to convert them into a HAL
//! designed struct from the pac struct `GPIO` and `IO_MUX` using `IO::new`.
//!
//! ```no_run
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let mut led = io.pins.gpio5.into_push_pull_output();
//! ```

use esp_hal_common::gpio::{types::*, *};

gpio! {
    Function1,
    SingleCore,

    Gpio0:  ( gpio0,  0,  gpio0, IO,  RTC, Bank0, None),
    Gpio1:  ( gpio1,  1,  gpio1, IO,  RTC, Bank0, None),
    Gpio2:  ( gpio2,  2,  gpio2, IO,  RTC, Bank0, None),
    Gpio3:  ( gpio3,  3,  gpio3, IO,  RTC, Bank0, None),
    Gpio4:  ( gpio4,  4,  gpio4, IO,  RTC, Bank0, None),
    Gpio5:  ( gpio5,  5,  gpio5, IO,  RTC, Bank0, None),
    Gpio6:  ( gpio6,  6,  gpio6, IO,  RTC, Bank0, None),
    Gpio7:  ( gpio7,  7,  gpio7, IO,  RTC, Bank0, None),
    Gpio8:  ( gpio8,  8,  gpio8, IO,  RTC, Bank0, None), (), (SUBSPICS1: Function3),
    Gpio9:  ( gpio9,  9,  gpio9, IO,  RTC, Bank0, None), (SUBSPIHD: Function3, FSPIHD: Function4), (SUBSPIHD: Function3, FSPIHD: Function4),
    Gpio10: (gpio10, 10, gpio10, IO,  RTC, Bank0, None), (FSPIIO4: Function2, FSPICS0: Function4), (FSPIIO4: Function2, SUBSPICS0: Function3, FSPICS0: Function4),
    Gpio11: (gpio11, 11, gpio11, IO,  RTC, Bank0, None), (FSPIIO5: Function2, SUBSPID: Function3, FSPID: Function4), (FSPIIO5: Function2, SUBSPID: Function3, FSPID: Function4),
    Gpio12: (gpio12, 12, gpio12, IO,  RTC, Bank0, None), (FSPICLK: Function4, FSPIIO6: Function2), (FSPIIO6: Function2, SUBSPICLK: Function3, FSPICLK: Function4),
    Gpio13: (gpio13, 13, gpio13, IO,  RTC, Bank0, None), (FSPIIO7: Function2, SUBSPIQ: Function3, FSPIQ: Function4), (FSPIIO7: Function2, SUBSPIQ: Function3, FSPIQ: Function4),
    Gpio14: (gpio14, 14, gpio14, IO,  RTC, Bank0, None), (SUBSPIWP: Function3, FSPIWP: Function4), (FSPIDQS: Function2, SUBSPIWP: Function3, FSPIWP: Function4),
    Gpio15: (gpio15, 15, gpio15, IO,  RTC, Bank0, None), (), (U0RTS: Function2),
    Gpio16: (gpio16, 16, gpio16, IO,  RTC, Bank0, None), (U0CTS: Function2), (),
    Gpio17: (gpio17, 17, gpio17, IO,  RTC, Bank0, None), (), (U1TXD: Function2),
    Gpio18: (gpio18, 18, gpio18, IO,  RTC, Bank0, None), (U1RXD: Function2), (),
    Gpio19: (gpio19, 19, gpio19, IO,  RTC, Bank0, None), (), (U1RTS: Function2),
    Gpio20: (gpio20, 20, gpio20, IO,  RTC, Bank0, None), (U1CTS: Function2), (),
    Gpio21: (gpio21, 21, gpio21, IO,  RTC, Bank0, None),
    Gpio26: (gpio26, 26, gpio26, IO,    0, Bank0, None), (), (SPICS1: Function0),
    Gpio27: (gpio27, 27, gpio27, IO,    0, Bank0, None), (SPIHD: Function0), (SPIHD: Function0),
    Gpio28: (gpio28, 28, gpio28, IO,    0, Bank0, None), (SPIWP: Function0), (SPIWP: Function0),
    Gpio29: (gpio29, 29, gpio29, IO,    0, Bank0, None), (), (SPICS0: Function0),
    Gpio30: (gpio30, 30, gpio30, IO,    0, Bank0, None), (), (SPICLK: Function0),
    Gpio31: (gpio31, 31, gpio31, IO,    0, Bank0, None), (SPIQ: Function0), (SPIQ: Function0),
    Gpio32: (gpio32, 32, gpio32, IO,    0, Bank1, None), (SPID: Function0), (SPID: Function0),
    Gpio33: (gpio33, 33, gpio33, IO,    0, Bank1, None), (FSPIHD: Function2, SUBSPIHD: Function3, FSPIIO4: Function4), (FSPIHD: Function2, SUBSPIHD: Function3),
    Gpio34: (gpio34, 34, gpio34, IO,    0, Bank1, None), (FSPICS0: Function2, FSPIIO5: Function4), (FSPICS0: Function2, SUBSPICS0: Function3),
    Gpio35: (gpio35, 35, gpio35, IO,    0, Bank1, None), (FSPID: Function2, SUBSPID: Function3, FSPIIO6: Function4), (FSPID: Function2, SUBSPID: Function3),
    Gpio36: (gpio36, 36, gpio36, IO,    0, Bank1, None), (FSPICLK: Function2, FSPIIO7: Function4), (FSPICLK: Function2, SUBSPICLK: Function3),
    Gpio37: (gpio37, 37, gpio37, IO,    0, Bank1, None), (FSPIQ: Function2, SUBSPIQ: Function3, SPIDQS: Function4), (FSPIQ: Function2, SUBSPIQ: Function3),
    Gpio38: (gpio38, 38, gpio38, IO,    0, Bank1, None), (FSPIWP: Function2, SUBSPIWP: Function3), (FSPIWP: Function2, SUBSPIWP: Function3),
    Gpio39: (gpio39, 39, gpio39, IO,    0, Bank1, None), (), (SUBSPICS1: Function3),
    Gpio40: (gpio40, 40, gpio40, IO,    0, Bank1, None),
    Gpio41: (gpio41, 41, gpio41, IO,    0, Bank1, None),
    Gpio42: (gpio42, 42, gpio42, IO,    0, Bank1, None),
    Gpio43: (gpio43, 43, gpio43, IO,    0, Bank1, None), (), (U0TXD: Function0),
    Gpio44: (gpio44, 44, gpio44, IO,    0, Bank1, None), (U0RXD: Function0), (),
    Gpio45: (gpio45, 45, gpio45, IO,    0, Bank1, None),
    Gpio46: (gpio46, 46, gpio46, IO,Input, Bank1, None),
}

analog! {
    Gpio0:  ( 0,  touch_pad[0],  touch_pad0_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio1:  ( 1,  touch_pad[1],  touch_pad1_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio2:  ( 2,  touch_pad[2],  touch_pad2_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio3:  ( 3,  touch_pad[3],  touch_pad3_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio4:  ( 4,  touch_pad[4],  touch_pad4_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio5:  ( 5,  touch_pad[5],  touch_pad5_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio6:  ( 6,  touch_pad[6],  touch_pad6_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio7:  ( 7,  touch_pad[7],  touch_pad7_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio8:  ( 8,  touch_pad[8],  touch_pad8_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio9:  ( 9,  touch_pad[9],  touch_pad9_hold,  touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio10: (10,  touch_pad[10], touch_pad10_hold, touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio11: (11,  touch_pad[11], touch_pad11_hold, touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio12: (12,  touch_pad[12], touch_pad12_hold, touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio13: (13,  touch_pad[13], touch_pad13_hold, touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio14: (14,  touch_pad[14], touch_pad14_hold, touch_pad0_mux_sel, touch_pad0_fun_sel, touch_pad0_fun_ie, touch_pad0_slp_ie,  touch_pad0_slp_sel, touch_pad0_rue, touch_pad0_rde, touch_pad0_drv, touch_pad0_slp_oe),
    Gpio15: (15,  xtal_32p_pad,  x32p_hold,        x32p_mux_sel,       x32p_fun_sel,       x32p_fun_ie,       x32p_slp_ie,        x32p_slp_sel,       x32p_rue,       x32p_rde,       x32p_drv,       x32p_slp_oe),
    Gpio16: (16,  xtal_32n_pad,  x32n_hold,        x32n_mux_sel,       x32n_fun_sel,       x32n_fun_ie,       x32n_slp_ie,        x32n_slp_sel,       x32n_rue,       x32n_rde,       x32n_drv,       x32pn_slp_oe),
    Gpio17: (17,  pad_dac1,      pdac1_hold,       pdac1_mux_sel,      pdac1_fun_sel,      pdac1_fun_ie,      pdac1_slp_ie,       pdac1_slp_sel,      pdac1_rue,      pdac1_rde,      pdac1_drv,      pdac1_slp_oe),
    Gpio18: (18,  pad_dac2,      pdac2_hold,       pdac2_mux_sel,      pdac2_fun_sel,      pdac2_fun_ie,      pdac2_slp_ie,       pdac2_slp_sel,      pdac2_rue,      pdac2_rde,      pdac2_drv,      pdac2_slp_oe),
    Gpio19: (19,  rtc_pad19,     rtc_pad19_hold,   mux_sel,            fun_sel,            fun_ie,            slp_ie,             slp_sel,            rue,            rde,            drv,            slp_oe),
    Gpio20: (20,  rtc_pad20,     rtc_pad20_hold,   mux_sel,            fun_sel,            fun_ie,            slp_ie,             slp_sel,            rue,            rde,            drv,            slp_oe),
    Gpio21: (21,  rtc_pad21,     rtc_pad21_hold,   mux_sel,            fun_sel,            fun_ie,            slp_ie,             slp_sel,            rue,            rde,            drv,            slp_oe),
}

// implement marker traits on USB pins
impl<T> esp_hal_common::otg_fs::UsbSel for Gpio18<T> {}
impl<T> esp_hal_common::otg_fs::UsbDp for Gpio19<T> {}
impl<T> esp_hal_common::otg_fs::UsbDm for Gpio20<T> {}
