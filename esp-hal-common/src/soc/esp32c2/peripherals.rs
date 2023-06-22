//! Peripheral instance singletons

use esp32c2 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    APB_CTRL => true,
    APB_SARADC => true,
    ASSIST_DEBUG => true,
    DMA => true,
    ECC => true,
    EFUSE => true,
    EXTMEM => true,
    GPIO => true,
    I2C0 => true,
    INTERRUPT_CORE0 => true,
    IO_MUX => true,
    LEDC => true,
    RNG => true,
    RTC_CNTL => true,
    SENSITIVE => true,
    SHA => true,
    SPI0 => true,
    SPI1 => true,
    SPI2 => true,
    SYSTEM => true,
    SYSTIMER => true,
    TIMG0 => true,
    UART0 => true,
    UART1 => true,
    XTS_AES => true,
    RADIO => false
}
