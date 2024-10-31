//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

use esp32c2 as pac;
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
    APB_CTRL <= APB_CTRL,
    ASSIST_DEBUG <= ASSIST_DEBUG,
    BT <= virtual,
    DMA <= DMA (DMA_CH0),
    ECC <= ECC,
    EFUSE <= EFUSE,
    EXTMEM <= EXTMEM,
    GPIO <= GPIO (GPIO,GPIO_NMI),
    [I2cExt0] I2C0 <= I2C0,
    INTERRUPT_CORE0 <= INTERRUPT_CORE0,
    IO_MUX <= IO_MUX,
    LEDC <= LEDC,
    LPWR <= RTC_CNTL,
    RADIO_CLK <= virtual,
    RNG <= RNG,
    SENSITIVE <= SENSITIVE,
    SHA <= SHA,
    SPI0 <= SPI0,
    SPI1 <= SPI1,
    [Spi2] SPI2 <= SPI2 (SPI2),
    SYSTEM <= SYSTEM,
    SYSTIMER <= SYSTIMER,
    SW_INTERRUPT <= virtual,
    TIMG0 <= TIMG0,
    [Uart0] UART0 <= UART0,
    [Uart1] UART1 <= UART1,
    WIFI <= virtual,
    XTS_AES <= XTS_AES,
    MEM2MEM1 <= virtual,
    MEM2MEM2 <= virtual,
    MEM2MEM3 <= virtual,
    MEM2MEM4 <= virtual,
    MEM2MEM5 <= virtual,
    MEM2MEM6 <= virtual,
    MEM2MEM8 <= virtual,
}
