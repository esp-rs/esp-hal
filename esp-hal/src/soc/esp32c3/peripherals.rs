//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

use esp32c3 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

// Note that certain are marked with `virtual` in the invocation of the
// `peripherals!` macro below. Basically, this indicates there's no physical
// peripheral (no `PSRAM`, `RADIO`, etc. peripheral in the PACs), so we're
// creating "virtual peripherals" for them.
crate::peripherals! {
    peripherals: [
        ADC1 <= virtual,
        ADC2 <= virtual,
        AES <= AES,
        APB_CTRL <= APB_CTRL,
        ASSIST_DEBUG <= ASSIST_DEBUG,
        BT <= virtual,
        DMA <= DMA (DMA_CH0,DMA_CH1,DMA_CH2),
        DS <= DS,
        EFUSE <= EFUSE,
        EXTMEM <= EXTMEM,
        GPIO_SD <= GPIO_SD,
        HMAC <= HMAC,
        I2C0 <= I2C0,
        I2S0 <= I2S0 (I2S0),
        INTERRUPT_CORE0 <= INTERRUPT_CORE0,
        IO_MUX <= IO_MUX,
        LEDC <= LEDC,
        LPWR <= RTC_CNTL,
        RADIO_CLK <= virtual,
        RMT <= RMT,
        RNG <= RNG,
        RSA <= RSA,
        SENSITIVE <= SENSITIVE,
        SHA <= SHA,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SPI2 <= SPI2 (SPI2),
        SYSTEM <= SYSTEM,
        SYSTIMER <= SYSTIMER,
        SW_INTERRUPT <= virtual,
        TIMG0 <= TIMG0,
        TIMG1 <= TIMG1,
        TWAI0 <= TWAI0,
        UART0 <= UART0,
        UART1 <= UART1,
        UHCI0 <= UHCI0,
        UHCI1 <= UHCI1,
        USB_DEVICE <= USB_DEVICE,
        WIFI <= virtual,
        XTS_AES <= XTS_AES,
    ],
    pins: [
        (0, [Input, Output, Analog, RtcIo])
        (1, [Input, Output, Analog, RtcIo])
        (2, [Input, Output, Analog, RtcIo] (2 => FSPIQ) (2 => FSPIQ))
        (3, [Input, Output, Analog, RtcIo])
        (4, [Input, Output, Analog, RtcIo] (2 => FSPIHD) (0 => USB_JTAG_TMS 2 => FSPIHD))
        (5, [Input, Output, Analog, RtcIo] (2 => FSPIWP) (0 => USB_JTAG_TDI 2 => FSPIWP))
        (6, [Input, Output] (2 => FSPICLK) (0 => USB_JTAG_TCK 2 => FSPICLK_MUX))
        (7, [Input, Output] (2 => FSPID) (0 => USB_JTAG_TDO 2 => FSPID))
        (8, [Input, Output])
        (9, [Input, Output])
        (10, [Input, Output] (2 => FSPICS0) (2 => FSPICS0))
        (11, [Input, Output])
        (12, [Input, Output] (0 => SPIHD) (0 => SPIHD))
        (13, [Input, Output] (0 => SPIWP) (0 => SPIWP))
        (14, [Input, Output] () (0 => SPICS0))
        (15, [Input, Output] () (0 => SPICLK_MUX))
        (16, [Input, Output] (0 => SPID) (0 => SPID))
        (17, [Input, Output] (0 => SPIQ) (0 => SPIQ))
        (18, [Input, Output])
        (19, [Input, Output])
        (20, [Input, Output] (0 => U0RXD) ())
        (21, [Input, Output] () (0 => U0TXD))
    ]
}
