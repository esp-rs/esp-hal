//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

use esp32s2 as pac;
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
    ADC2 <= virtual,
    AES <= AES,
    DAC1 <= virtual,
    DAC2 <= virtual,
    DMA <= virtual,
    DEDICATED_GPIO <= DEDICATED_GPIO,
    DS <= DS,
    EFUSE <= EFUSE,
    EXTMEM <= EXTMEM,
    GPIO <= GPIO (GPIO,GPIO_NMI),
    GPIO_SD <= GPIO_SD,
    HMAC <= HMAC,
    I2C0 <= I2C0,
    I2C1 <= I2C1,
    [I2s0] I2S0 <= I2S0 (I2S0),
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
    SHA <= SHA,
    SPI0 <= SPI0,
    SPI1 <= SPI1,
    [Spi2] SPI2 <= SPI2 (SPI2_DMA, SPI2),
    [Spi3] SPI3 <= SPI3 (SPI3_DMA, SPI3),
    SYSCON <= SYSCON,
    SYSTEM <= SYSTEM,
    SYSTIMER <= SYSTIMER,
    SW_INTERRUPT <= virtual,
    TIMG0 <= TIMG0,
    TIMG1 <= TIMG1,
    [Twai0] TWAI0 <= TWAI0,
    [Uart0] UART0 <= UART0,
    [Uart1] UART1 <= UART1,
    UHCI0 <= UHCI0,
    ULP_RISCV_CORE <= virtual,
    USB0 <= USB0,
    USB_WRAP <= USB_WRAP,
    WIFI <= virtual,
    XTS_AES <= XTS_AES,
}
