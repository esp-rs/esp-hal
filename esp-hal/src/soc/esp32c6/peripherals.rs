//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

pub(crate) use esp32c6 as pac;
// We need to export this for users to use
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
        AES <= AES,
        APB_SARADC <= APB_SARADC,
        ASSIST_DEBUG <= ASSIST_DEBUG,
        ATOMIC <= ATOMIC,
        BT <= virtual,
        DMA <= DMA,
        DS <= DS,
        ECC <= ECC,
        EFUSE <= EFUSE,
        EXTMEM <= EXTMEM,
        GPIO <= GPIO,
        GPIO_SD <= GPIO_SD,
        HINF <= HINF,
        HMAC <= HMAC,
        HP_APM <= HP_APM,
        HP_SYS <= HP_SYS,
        I2S0 <= I2S0 (I2S0),
        IEEE802154 <= IEEE802154,
        INTERRUPT_CORE0 <= INTERRUPT_CORE0,
        INTPRI <= INTPRI,
        IO_MUX <= IO_MUX,
        LEDC <= LEDC,
        LPWR <= LP_CLKRST,
        LP_CORE <= virtual,
        LP_PERI <= LP_PERI,
        LP_ANA <= LP_ANA,
        LP_AON <= LP_AON,
        LP_APM <= LP_APM,
        LP_APM0 <= LP_APM0,
        LP_I2C0 <= LP_I2C0,
        LP_I2C_ANA_MST <= LP_I2C_ANA_MST,
        LP_IO <= LP_IO,
        LP_TEE <= LP_TEE,
        LP_TIMER <= LP_TIMER,
        LP_UART <= LP_UART,
        LP_WDT <= LP_WDT,
        MCPWM0 <= MCPWM0,
        MEM_MONITOR <= MEM_MONITOR,
        MODEM_LPCON <= MODEM_LPCON,
        MODEM_SYSCON <= MODEM_SYSCON,
        OTP_DEBUG <= OTP_DEBUG,
        PARL_IO <= PARL_IO (PARL_IO),
        PAU <= PAU,
        PCR <= PCR,
        PCNT <= PCNT,
        PLIC_MX <= PLIC_MX,
        PMU <= PMU,
        RADIO_CLK <= virtual,
        RMT <= RMT,
        RNG <= RNG,
        RSA <= RSA,
        SHA <= SHA,
        SLCHOST <= SLCHOST,
        SOC_ETM <= SOC_ETM,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SYSTEM <= PCR,
        SYSTIMER <= SYSTIMER,
        SW_INTERRUPT <= virtual,
        TEE <= TEE,
        TIMG0 <= TIMG0,
        TIMG1 <= TIMG1,
        TRACE0 <= TRACE,
        TSENS <= virtual,
        TWAI0 <= TWAI0,
        TWAI1 <= TWAI1,
        UHCI0 <= UHCI0,
        USB_DEVICE <= USB_DEVICE,
        WIFI <= virtual,
        MEM2MEM1 <= virtual,
        MEM2MEM4 <= virtual,
        MEM2MEM5 <= virtual,
        MEM2MEM10 <= virtual,
        MEM2MEM11 <= virtual,
        MEM2MEM12 <= virtual,
        MEM2MEM13 <= virtual,
        MEM2MEM14 <= virtual,
        MEM2MEM15 <= virtual,
    ],
    pins: [
        (0, [Input, Output, Analog, RtcIo])
        (1, [Input, Output, Analog, RtcIo])
        (2, [Input, Output, Analog, RtcIo] (2 => FSPIQ) (2 => FSPIQ))
        (3, [Input, Output, Analog, RtcIo])
        (4, [Input, Output, Analog, RtcIo] (2 => FSPIHD) (0 => USB_JTAG_TMS 2 => FSPIHD))
        (5, [Input, Output, Analog, RtcIo] (2 => FSPIWP) (0 => USB_JTAG_TDI 2 => FSPIWP))
        (6, [Input, Output, Analog, RtcIo] (2 => FSPICLK) (0 => USB_JTAG_TCK 2 => FSPICLK_MUX))
        (7, [Input, Output, Analog, RtcIo] (2 => FSPID) (0 => USB_JTAG_TDO 2 => FSPID))
        (8, [Input, Output])
        (9, [Input, Output])
        (10, [Input, Output])
        (11, [Input, Output])
        (12, [Input, Output])
        (13, [Input, Output])
        (14, [Input, Output])
        (15, [Input, Output])
        (16, [Input, Output] (0 => U0RXD) (2 => FSPICS0))
        (17, [Input, Output] () (0 => U0TXD 2 => FSPICS1))
        (18, [Input, Output] () (2 => FSPICS2)) //  0 => SDIO_CMD but there are no signals since it's a fixed pin
        (19, [Input, Output] () (2 => FSPICS3)) //  0 => SDIO_CLK but there are no signals since it's a fixed pin
        (20, [Input, Output] () (2 => FSPICS4)) // 0 => SDIO_DATA0 but there are no signals since it's a fixed pin
        (21, [Input, Output] () (2 => FSPICS5)) // 0 => SDIO_DATA1 but there are no signals since it's a fixed pin
        (22, [Input, Output] () ()) // 0 => SDIO_DATA2 but there are no signals since it's a fixed pin
        (23, [Input, Output] () ()) // 0 => SDIO_DATA3 but there are no signals since it's a fixed pin
        (24, [Input, Output] () (0 => SPICS0))
        (25, [Input, Output] (0 => SPIQ) (0 => SPIQ))
        (26, [Input, Output] (0 => SPIWP) (0 => SPIWP))
        (27, [Input, Output])
        (28, [Input, Output] (0 => SPIHD) (0 => SPIHD))
        (29, [Input, Output] () (0 => SPICLK_MUX))
        (30, [Input, Output] (0 => SPID) (0 => SPID))
    ],
    dma_channels: [
        DMA_CH0: DmaChannel0,
        DMA_CH1: DmaChannel1,
        DMA_CH2: DmaChannel2,
    ]
}
