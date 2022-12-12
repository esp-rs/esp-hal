pub use pac::Interrupt;

use crate::pac; // We need to export this for users to use

crate::peripherals! {
    APB_CTRL,
    APB_SARADC,
    ASSIST_DEBUG,
    DMA,
    ECC,
    EFUSE,
    EXTMEM,
    GPIO,
    I2C0,
    INTERRUPT_CORE0,
    IO_MUX,
    LEDC,
    RNG,
    RTC_CNTL,
    SENSITIVE,
    SHA,
    SPI0,
    SPI1,
    SPI2,
    SYSTEM,
    SYSTIMER,
    TIMG0,
    UART0,
    UART1,
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
