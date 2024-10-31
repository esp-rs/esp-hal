//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

use esp32h2 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

// Note that certain are marked with `virtual` in the invocation of the
// `peripherals!` macro below. Basically, this indicates there's no physical
// peripheral (no `PSRAM`, `RADIO`, etc. peripheral in the PACs), so we're
// creating "virtual peripherals" for them.
crate::peripherals! {
    ADC1 <= virtual,
    AES <= AES,
    ASSIST_DEBUG <= ASSIST_DEBUG,
    BT <= virtual,
    DMA <= DMA (DMA_IN_CH0,DMA_IN_CH1,DMA_IN_CH2,DMA_OUT_CH0,DMA_OUT_CH1,DMA_OUT_CH2),
    DS <= DS,
    ECC <= ECC,
    EFUSE <= EFUSE,
    GPIO <= GPIO (GPIO,GPIO_NMI),
    GPIO_SD <= GPIO_SD,
    HMAC <= HMAC,
    HP_APM <= HP_APM,
    HP_SYS <= HP_SYS,
    [I2cExt0] I2C0 <= I2C0,
    [I2cExt1] I2C1 <= I2C1,
    [I2s0] I2S0 <= I2S0 (I2S0),
    IEEE802154 <= IEEE802154,
    INTERRUPT_CORE0 <= INTERRUPT_CORE0,
    INTPRI <= INTPRI,
    IO_MUX <= IO_MUX,
    LEDC <= LEDC,
    LPWR <= LP_CLKRST,
    LP_ANA <= LP_ANA,
    LP_AON <= LP_AON,
    LP_APM <= LP_APM,
    LP_PERI <= LP_PERI,
    LP_TIMER <= LP_TIMER,
    LP_WDT <= LP_WDT,
    MCPWM0 <= MCPWM0,
    MEM_MONITOR <= MEM_MONITOR,
    MODEM_LPCON <= MODEM_LPCON,
    MODEM_SYSCON <= MODEM_SYSCON,
    OTP_DEBUG <= OTP_DEBUG,
    PARL_IO <= PARL_IO (PARL_IO_TX, PARL_IO_RX),
    PAU <= PAU,
    PCNT <= PCNT,
    PMU <= PMU,
    RADIO_CLK <= virtual,
    RMT <= RMT,
    RNG <= RNG,
    RSA <= RSA,
    SHA <= SHA,
    SOC_ETM <= SOC_ETM,
    SPI0 <= SPI0,
    SPI1 <= SPI1,
    [Spi2] SPI2 <= SPI2 (SPI2),
    SYSTEM <= PCR,
    SYSTIMER <= SYSTIMER,
    SW_INTERRUPT <= virtual,
    TEE <= TEE,
    TIMG0 <= TIMG0,
    TIMG1 <= TIMG1,
    TRACE0 <= TRACE,
    [Twai0] TWAI0 <= TWAI0,
    [Uart0] UART0 <= UART0,
    [Uart1] UART1 <= UART1,
    UHCI0 <= UHCI0,
    USB_DEVICE <= USB_DEVICE,
    MEM2MEM1 <= virtual,
    MEM2MEM4 <= virtual,
    MEM2MEM5 <= virtual,
    MEM2MEM10 <= virtual,
    MEM2MEM11 <= virtual,
    MEM2MEM12 <= virtual,
    MEM2MEM13 <= virtual,
    MEM2MEM14 <= virtual,
    MEM2MEM15 <= virtual,
}
