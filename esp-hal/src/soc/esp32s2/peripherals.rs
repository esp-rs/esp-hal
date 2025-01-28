//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

pub(crate) use esp32s2 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// Note that certain are marked with `virtual` in the invocation of the
// `peripherals!` macro below. Basically, this indicates there's no physical
// peripheral (no `PSRAM`, `RADIO`, etc. peripheral in the PACs), so we're
// creating "virtual peripherals" for them.
crate::peripherals! {
    peripherals: [
        I2C0 <= I2C0,
        I2C1 <= I2C1,
        SPI2 <= SPI2 (SPI2_DMA, SPI2),
        SPI3 <= SPI3 (SPI3_DMA, SPI3),
        UART0 <= UART0,
        UART1 <= UART1,
    ],
    unstable_peripherals: [
        ADC1 <= virtual,
        ADC2 <= virtual,
        AES <= AES,
        APB_SARADC <= APB_SARADC,
        DAC1 <= virtual,
        DAC2 <= virtual,
        CRYPTO_DMA <= CRYPTO_DMA,
        DEDICATED_GPIO <= DEDICATED_GPIO,
        DS <= DS,
        EFUSE <= EFUSE,
        EXTMEM <= EXTMEM,
        GPIO <= GPIO,
        GPIO_SD <= GPIO_SD,
        HMAC <= HMAC,
        I2S0 <= I2S0 (I2S0),
        INTERRUPT_CORE0 <= INTERRUPT_CORE0,
        IO_MUX <= IO_MUX,
        LEDC <= LEDC,
        LPWR <= RTC_CNTL,
        PCNT <= PCNT,
        PMS <= PMS,
        PSRAM <= virtual,
        RADIO_CLK <= virtual,
        RMT <= RMT,
        RNG <= RNG,
        RSA <= RSA,
        RTC_IO <= RTC_IO,
        RTC_I2C <= RTC_I2C,
        SENS <= SENS,
        SHA <= SHA,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SYSCON <= SYSCON,
        SYSTEM <= SYSTEM,
        SYSTIMER <= SYSTIMER,
        SW_INTERRUPT <= virtual,
        TIMG0 <= TIMG0,
        TIMG1 <= TIMG1,
        TWAI0 <= TWAI0,
        UHCI0 <= UHCI0,
        ULP_RISCV_CORE <= virtual,
        USB0 <= USB0,
        USB_WRAP <= USB_WRAP,
        WIFI <= WIFI,
        XTS_AES <= XTS_AES,
    ],
    pins: [
        (0, [Input, Output, Analog, RtcIo])
        (1, [Input, Output, Analog, RtcIo])
        (2, [Input, Output, Analog, RtcIo])
        (3, [Input, Output, Analog, RtcIo])
        (4, [Input, Output, Analog, RtcIo])
        (5, [Input, Output, Analog, RtcIo])
        (6, [Input, Output, Analog, RtcIo])
        (7, [Input, Output, Analog, RtcIo])
        (8, [Input, Output, Analog, RtcIo]  (                                       ) (              3 => SUBSPICS1             ))
        (9, [Input, Output, Analog, RtcIo]  (             3 => SUBSPIHD 4 => FSPIHD ) (              3 => SUBSPIHD  4 => FSPIHD ))
        (10, [Input, Output, Analog, RtcIo] (2 => FSPIIO4               4 => FSPICS0) (2 => FSPIIO4  3 => SUBSPICS0 4 => FSPICS0))
        (11, [Input, Output, Analog, RtcIo] (2 => FSPIIO5 3 => SUBSPID  4 => FSPID  ) (2 => FSPIIO5  3 => SUBSPID   4 => FSPID  ))
        (12, [Input, Output, Analog, RtcIo] (2 => FSPIIO6               4 => FSPICLK) (2 => FSPIIO6  3 => SUBSPICLK 4 => FSPICLK))
        (13, [Input, Output, Analog, RtcIo] (2 => FSPIIO7 3 => SUBSPIQ  4 => FSPIQ  ) (2 => FSPIIO7  3 => SUBSPIQ   4 => FSPIQ  ))
        (14, [Input, Output, Analog, RtcIo] (             3 => SUBSPIWP 4 => FSPIWP ) (2 => FSPIDQS  3 => SUBSPIWP  4 => FSPIWP ))
        (15, [Input, Output, Analog, RtcIo] (          ) (2 => U0RTS))
        (16, [Input, Output, Analog, RtcIo] (2 => U0CTS) (          ))
        (17, [Input, Output, Analog, RtcIo] (          ) (2 => U1TXD))
        (18, [Input, Output, Analog, RtcIo] (2 => U1RXD) (          ))
        (19, [Input, Output, Analog, RtcIo] (          ) (2 => U1RTS))
        (20, [Input, Output, Analog, RtcIo] (2 => U1CTS) (          ))
        (21, [Input, Output, Analog, RtcIo])

        (26, [Input, Output])
        (27, [Input, Output])
        (28, [Input, Output])
        (29, [Input, Output])
        (30, [Input, Output])
        (31, [Input, Output])
        (32, [Input, Output])
        (33, [Input, Output] (2 => FSPIHD  3 => SUBSPIHD            ) (2 => FSPIHD  3 => SUBSPIHD             ))
        (34, [Input, Output] (2 => FSPICS0                          ) (2 => FSPICS0 3 => SUBSPICS0            ))
        (35, [Input, Output] (2 => FSPID   3 => SUBSPID             ) (2 => FSPID   3 => SUBSPID              ))
        (36, [Input, Output] (2 => FSPICLK                          ) (2 => FSPICLK 3 => SUBSPICLK            ))
        (37, [Input, Output] (2 => FSPIQ   3 => SUBSPIQ  4 => SPIDQS) (2 => FSPIQ   3 => SUBSPIQ   4 => SPIDQS))
        (38, [Input, Output] (2 => FSPIWP  3 => SUBSPIWP            ) (2 => FSPIWP  3 => SUBSPIWP             ))
        (39, [Input, Output] (                                      ) (             3 => SUBSPICS1            ))
        (40, [Input, Output])
        (41, [Input, Output])
        (42, [Input, Output])
        (43, [Input, Output])
        (44, [Input, Output])
        (45, [Input, Output])
        (46, [Input, Output])
    ],
    dma_channels: [
        DMA_SPI2: Spi2DmaChannel,
        DMA_SPI3: Spi3DmaChannel,
        DMA_I2S0: I2s0DmaChannel,
        DMA_CRYPTO: CryptoDmaChannel,
    ]
}
