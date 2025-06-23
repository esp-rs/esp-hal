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
//! This trait provides functions to read the interrupt status and NMI status
//! registers for both the `PRO CPU` and `APP CPU`. The implementation uses the
//! `gpio` peripheral to access the appropriate registers.

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Debug, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Debug, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

macro_rules! rtcio_analog {
    ($pin_num:expr, $pin_reg:expr, $hold:ident) => {
        paste::paste!{
            impl $crate::gpio::RtcPin for $crate::peripherals::[<GPIO $pin_num>]<'_> {
                fn rtc_number(&self) -> u8 {
                    $pin_num
                }

                /// Set the RTC properties of the pin. If `mux` is true then then pin is
                /// routed to RTC, when false it is routed to IO_MUX.
                fn rtc_set_config(&self, input_enable: bool, mux: bool, func: $crate::gpio::RtcFunction) {
                    enable_iomux_clk_gate();

                    // We need `paste` to rewrite something in each function, so that rustc
                    // doesn't trip over trying to substitute a partial expression as `$pin_reg`
                    $crate::peripherals::[<RTC _IO>]::regs()
                        .$pin_reg.modify(|_,w| unsafe {
                            w.fun_ie().bit(input_enable);
                            w.mux_sel().bit(mux);
                            w.fun_sel().bits(func as u8)
                        });
                }

                fn rtcio_pad_hold(&self, enable: bool) {
                    $crate::peripherals::LPWR::regs()
                        .pad_hold()
                        .modify(|_, w| w.$hold().bit(enable));
                }
            }

            impl $crate::gpio::RtcPinWithResistors for $crate::peripherals::[<GPIO $pin_num>]<'_> {
                fn rtcio_pullup(&self, enable: bool) {
                    $crate::peripherals::[<RTC _IO>]::regs()
                        .$pin_reg.modify(|_, w| w.rue().bit(enable));
                }

                fn rtcio_pulldown(&self, enable: bool) {
                    $crate::peripherals::[<RTC _IO>]::regs()
                        .$pin_reg.modify(|_, w| w.rde().bit(enable));
                }
            }

            impl $crate::gpio::AnalogPin for $crate::peripherals::[<GPIO $pin_num>]<'_> {
                /// Configures the pin for analog mode.
                fn set_analog(&self, _: $crate::private::Internal) {
                    use $crate::gpio::RtcPin;
                    enable_iomux_clk_gate();

                    let rtcio = $crate::peripherals::[<RTC _IO>]::regs();

                    // disable output
                    rtcio.enable_w1tc().write(|w| unsafe { w.enable_w1tc().bits(1 << self.rtc_number()) });

                    // disable open drain
                    rtcio.pin(self.rtc_number() as usize).modify(|_,w| w.pad_driver().bit(false));

                    rtcio.$pin_reg.modify(|_,w| {
                        w.fun_ie().clear_bit();

                        // Connect pin to analog / RTC module instead of standard GPIO
                        w.mux_sel().set_bit();

                        // Select function "RTC function 1" (GPIO) for analog use
                        unsafe { w.fun_sel().bits(0b00) };

                        // Disable pull-up and pull-down resistors on the pin
                        w.rue().bit(false);
                        w.rde().bit(false);

                        w
                    });
                }
            }
        }
    };

    (
        $( ( $pin_num:expr, $pin_reg:expr, $hold:ident ) )+
    ) => {
        $(
            rtcio_analog!($pin_num, $pin_reg, $hold);
        )+
    };
}

rtcio_analog! {
    ( 0, touch_pad(0),   touch_pad0 )
    ( 1, touch_pad(1),   touch_pad1 )
    ( 2, touch_pad(2),   touch_pad2 )
    ( 3, touch_pad(3),   touch_pad3 )
    ( 4, touch_pad(4),   touch_pad4 )
    ( 5, touch_pad(5),   touch_pad5 )
    ( 6, touch_pad(6),   touch_pad6 )
    ( 7, touch_pad(7),   touch_pad7 )
    ( 8, touch_pad(8),   touch_pad8 )
    ( 9, touch_pad(9),   touch_pad9 )
    (10, touch_pad(10),  touch_pad10)
    (11, touch_pad(11),  touch_pad11)
    (12, touch_pad(12),  touch_pad12)
    (13, touch_pad(13),  touch_pad13)
    (14, touch_pad(14),  touch_pad14)
    (15, xtal_32p_pad(), x32p       )
    (16, xtal_32n_pad(), x32n       )
    (17, pad_dac1(),     pdac1      )
    (18, pad_dac2(),     pdac2      )
    (19, rtc_pad19(),    pad19      )
    (20, rtc_pad20(),    pad20      )
    (21, rtc_pad21(),    pad21      )
}

fn enable_iomux_clk_gate() {
    crate::peripherals::SENS::regs()
        .sar_io_mux_conf()
        .modify(|_, w| w.iomux_clk_gate_en().set_bit());
}
