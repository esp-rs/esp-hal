pub use pac::Interrupt;

use crate::pac; // We need to export this for users to use

crate::peripherals! {
    AES,
    APB_SARADC,
    DEDICATED_GPIO,
    DS,
    EFUSE,
    EXTMEM,
    GPIO,
    GPIO_SD,
    HMAC,
    I2C0,
    I2C1,
    I2S,
    INTERRUPT,
    IO_MUX,
    LEDC,
    PCNT,
    PMS,
    RMT,
    RNG,
    RSA,
    RTCIO,
    RTC_CNTL,
    RTC_I2C,
    SENS,
    SHA,
    SPI0,
    SPI1,
    SPI2,
    SPI3,
    SPI4,
    SYSTEM,
    SYSTIMER,
    TIMG0,
    TIMG1,
    TWAI,
    UART0,
    UART1,
    UHCI0,
    USB0,
    USB_WRAP,
    XTS_AES,
}

mod peripherals {
    pub use super::pac::*;

    crate::create_peripherals! {
        I2C0,
        I2C1,
        RNG,
        SPI0,
        SPI1,
        SPI2,
        SPI3,
        SPI4,
        SYSTIMER,
        UART0,
        UART1,
        SYSTEM,
        LEDC,
        RMT,
    }
}
