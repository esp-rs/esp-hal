pub use pac::Interrupt;

use crate::pac; // We need to export this for users to use

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
        UART0,
        UART1,
        SPI0,
        SPI1,
        SPI2,
    }
}
