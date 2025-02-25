//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

pub(crate) use esp32c2 as pac;
// We need to export this for users to use
#[doc(hidden)]
pub use pac::Interrupt;

// Note that certain are marked with `virtual` in the invocation of the
// `peripherals!` macro below. Basically, this indicates there's no physical
// peripheral (no `PSRAM`, `RADIO`, etc. peripheral in the PACs), so we're
// creating "virtual peripherals" for them.
crate::peripherals! {
    peripherals: [
        I2C0 <= I2C0,
        SPI2 <= SPI2 (SPI2),
        UART0 <= UART0,
        UART1 <= UART1,
    ],
    unstable_peripherals: [
        ADC1 <= virtual,
        APB_CTRL <= APB_CTRL,
        APB_SARADC <= APB_SARADC,
        ASSIST_DEBUG <= ASSIST_DEBUG,
        BB <= BB,
        BT <= virtual,
        DMA <= DMA,
        ECC <= ECC,
        EFUSE <= EFUSE,
        EXTMEM <= EXTMEM,
        GPIO <= GPIO,
        I2C_ANA_MST <= I2C_ANA_MST,
        INTERRUPT_CORE0 <= INTERRUPT_CORE0,
        IO_MUX <= IO_MUX,
        LEDC <= LEDC,
        LPWR <= RTC_CNTL,
        MODEM_CLKRST <= MODEM_CLKRST,
        RADIO_CLK <= virtual,
        RNG <= RNG,
        SENSITIVE <= SENSITIVE,
        SHA <= SHA,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SYSTEM <= SYSTEM,
        SYSTIMER <= SYSTIMER,
        SW_INTERRUPT <= virtual,
        TIMG0 <= TIMG0,
        WIFI <= virtual,
        XTS_AES <= XTS_AES,
        MEM2MEM1 <= virtual,
        MEM2MEM2 <= virtual,
        MEM2MEM3 <= virtual,
        MEM2MEM4 <= virtual,
        MEM2MEM5 <= virtual,
        MEM2MEM6 <= virtual,
        MEM2MEM8 <= virtual,
    ],
    pins: [
        (0, [Input, Output, Analog, RtcIo])
        (1, [Input, Output, Analog, RtcIo])
        (2, [Input, Output, Analog, RtcIo] (2 => FSPIQ) (2 => FSPIQ))
        (3, [Input, Output, Analog, RtcIo])
        (4, [Input, Output, Analog, RtcIo] (2 => FSPIHD) (2 => FSPIHD))
        (5, [Input, Output, Analog, RtcIo] (2 => FSPIWP) (2 => FSPIWP))
        (6, [Input, Output] (2 => FSPICLK) (2 => FSPICLK_MUX))
        (7, [Input, Output] (2 => FSPID) (2 => FSPID))
        (8, [Input, Output])
        (9, [Input, Output])
        (10, [Input, Output] (2 => FSPICS0) (2 => FSPICS0))
        (18, [Input, Output])
        (19, [Input, Output])
        (20, [Input, Output] (0 => U0RXD) ())
    ],
    dma_channels: [
        DMA_CH0: DmaChannel0,
    ]
}
