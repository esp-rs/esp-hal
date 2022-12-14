use esp32c3 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES,
    APB_CTRL,
    APB_SARADC,
    ASSIST_DEBUG,
    DMA,
    DS,
    EFUSE,
    EXTMEM,
    GPIO,
    GPIOSD,
    HMAC,
    I2C0,
    I2S,
    INTERRUPT_CORE0,
    IO_MUX,
    LEDC,
    RMT,
    RNG,
    RSA,
    RTC_CNTL,
    SENSITIVE,
    SHA,
    SPI0,
    SPI1,
    SPI2,
    SYSTEM,
    SYSTIMER,
    TIMG0,
    TIMG1,
    TWAI,
    UART0,
    UART1,
    UHCI0,
    UHCI1,
    USB_DEVICE,
    XTS_AES,
}

mod peripherals {
    pub use super::pac::*;

    crate::create_peripherals! {
        I2C0,
        RNG,
        SHA,
        SPI0,
        SPI1,
        SPI2,
        SYSTIMER,
        UART0,
        UART1,
        USB_DEVICE,
        SYSTEM,
        LEDC,
        RMT,
    }
}
