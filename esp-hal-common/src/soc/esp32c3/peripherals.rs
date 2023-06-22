//! Peripheral instance singletons

use esp32c3 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES => true,
    APB_CTRL => true,
    APB_SARADC => true,
    ASSIST_DEBUG => true,
    DMA => true,
    DS => true,
    EFUSE => true,
    EXTMEM => true,
    GPIO => true,
    GPIO_SD => true,
    HMAC => true,
    I2C0 => true,
    I2S0 => true,
    INTERRUPT_CORE0 => true,
    IO_MUX => true,
    LEDC => true,
    RMT => true,
    RNG => true,
    RSA => true,
    RTC_CNTL => true,
    SENSITIVE => true,
    SHA => true,
    SPI0 => true,
    SPI1 => true,
    SPI2 => true,
    SYSTEM => true,
    SYSTIMER => true,
    TIMG0 => true,
    TIMG1 => true,
    TWAI0 => true,
    UART0 => true,
    UART1 => true,
    UHCI0 => true,
    UHCI1 => true,
    USB_DEVICE => true,
    XTS_AES => true,
    RADIO => false
}
