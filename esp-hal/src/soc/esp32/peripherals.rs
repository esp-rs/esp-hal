//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

use esp32 as pac;
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
        BB <= BB,
        BT <= virtual,
        CPU_CTRL <= virtual,
        DAC1 <= virtual,
        DAC2 <= virtual,
        DMA <= virtual,
        EFUSE <= EFUSE,
        FLASH_ENCRYPTION <= FLASH_ENCRYPTION,
        FRC_TIMER <= FRC_TIMER,
        GPIO_SD <= GPIO_SD,
        HINF <= HINF,
        I2C0 <= I2C0,
        I2C1 <= I2C1,
        I2S0 <= I2S0 (I2S0),
        I2S1 <= I2S1 (I2S1),
        IO_MUX <= IO_MUX,
        LEDC <= LEDC,
        MCPWM0 <= MCPWM0,
        MCPWM1 <= MCPWM1,
        NRX <= NRX,
        PCNT <= PCNT,
        PSRAM <= virtual,
        RMT <= RMT,
        RNG <= RNG,
        RSA <= RSA,
        LPWR <= RTC_CNTL,
        RADIO_CLK <= virtual,
        RTC_IO <= RTC_IO,
        RTC_I2C <= RTC_I2C,
        SDHOST <= SDHOST,
        SHA <= SHA,
        SLC <= SLC,
        SLCHOST <= SLCHOST,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SPI2 <= SPI2 (SPI2_DMA, SPI2),
        SPI3 <= SPI3 (SPI3_DMA, SPI3),
        SYSTEM <= DPORT,
        SW_INTERRUPT <= virtual,
        TIMG0 <= TIMG0,
        TIMG1 <= TIMG1,
        TOUCH <= virtual,
        TWAI0 <= TWAI0,
        UART0 <= UART0,
        UART1 <= UART1,
        UART2 <= UART2,
        UHCI0 <= UHCI0,
        UHCI1 <= UHCI1,
        WIFI <= virtual,
    ],
    pins: [
        (0, [Input, Output, Analog, RtcIo, Touch] (5 => EMAC_TX_CLK) (1 => CLK_OUT1))
        (1, [Input, Output] (5 => EMAC_RXD2) (0 => U0TXD 1 => CLK_OUT3))
        (2, [Input, Output, Analog, RtcIo, Touch] (1 => HSPIWP 3 => HS2_DATA0 4 => SD_DATA0) (3 => HS2_DATA0 4 => SD_DATA0))
        (3, [Input, Output] (0 => U0RXD) (1 => CLK_OUT2))
        (4, [Input, Output, Analog, RtcIo, Touch] (1 => HSPIHD 3 => HS2_DATA1 4 => SD_DATA1 5 => EMAC_TX_ER) (3 => HS2_DATA1 4 => SD_DATA1))
        (5, [Input, Output] (1 => VSPICS0 3 => HS1_DATA6 5 => EMAC_RX_CLK) (3 => HS1_DATA6))
        (6, [Input, Output] (4 => U1CTS) (0 => SD_CLK 1 => SPICLK 3 => HS1_CLK))
        (7, [Input, Output] (0 => SD_DATA0 1 => SPIQ 3 => HS1_DATA0) (0 => SD_DATA0 1 => SPIQ 3 => HS1_DATA0 4 => U2RTS))
        (8, [Input, Output] (0 => SD_DATA1 1 => SPID 3 => HS1_DATA1 4 => U2CTS) (0 => SD_DATA1 1 => SPID 3 => HS1_DATA1))
        (9, [Input, Output] (0 => SD_DATA2 1 => SPIHD 3 => HS1_DATA2 4 => U1RXD) (0 => SD_DATA2 1 => SPIHD 3 => HS1_DATA2))
        (10, [Input, Output] ( 0 => SD_DATA3 1 => SPIWP 3 => HS1_DATA3) (0 => SD_DATA3 1 => SPIWP 3 => HS1_DATA3 4 => U1TXD))
        (11, [Input, Output] ( 1 => SPICS0) (0 => SD_CMD 1 => SPICS0 3 => HS1_CMD 4 => U1RTS))
        (12, [Input, Output, Analog, RtcIo, Touch] (0 => MTDI 1 => HSPIQ 3 => HS2_DATA2 4 => SD_DATA2) (1 => HSPIQ 3 => HS2_DATA2 4 => SD_DATA2 5 => EMAC_TXD3))
        (13, [Input, Output, Analog, RtcIo, Touch] (0 => MTCK 1 => HSPID 3 => HS2_DATA3 4 => SD_DATA3) (1 => HSPID 3 => HS2_DATA3 4 => SD_DATA3 5 => EMAC_RX_ER))
        (14, [Input, Output, Analog, RtcIo, Touch] (0 => MTMS 1 => HSPICLK) (1 => HSPICLK 3 => HS2_CLK 4 => SD_CLK 5 => EMAC_TXD2))
        (15, [Input, Output, Analog, RtcIo, Touch] (1 => HSPICS0 5 => EMAC_RXD3) (0 => MTDO 1 => HSPICS0 3 => HS2_CMD 4 => SD_CMD))
        (16, [Input, Output] (3 => HS1_DATA4 4 => U2RXD) (3 => HS1_DATA4 5 => EMAC_CLK_OUT))
        (17, [Input, Output] (3 => HS1_DATA5) (3 => HS1_DATA5 4 => U2TXD 5 => EMAC_CLK_180))
        (18, [Input, Output] (1 => VSPICLK 3 => HS1_DATA7) (1 => VSPICLK 3 => HS1_DATA7))
        (19, [Input, Output] (1 => VSPIQ 3 => U0CTS) (1 => VSPIQ 5 => EMAC_TXD0))
        (20, [Input, Output])
        (21, [Input, Output] (1 => VSPIHD) (1 => VSPIHD 5 => EMAC_TX_EN))
        (22, [Input, Output] (1 => VSPIWP) (1 => VSPIWP 3 => U0RTS 5 => EMAC_TXD1))
        (23, [Input, Output] (1 => VSPID) (1 => VSPID 3 => HS1_STROBE))
        (24, [Input, Output])
        (25, [Input, Output, Analog, RtcIo] (5 => EMAC_RXD0) ())
        (26, [Input, Output, Analog, RtcIo] (5 => EMAC_RXD1) ())
        (27, [Input, Output, Analog, RtcIo, Touch] (5 => EMAC_RX_DV) ())
        (32, [Input, Output, Analog, RtcIo, Touch])
        (33, [Input, Output, Analog, RtcIo, Touch])
        (34, [Input, Analog, RtcIoInput])
        (35, [Input, Analog, RtcIoInput])
        (36, [Input, Analog, RtcIoInput])
        (37, [Input, Analog, RtcIoInput])
        (38, [Input, Analog, RtcIoInput])
        (39, [Input, Analog, RtcIoInput])
    ]
}
