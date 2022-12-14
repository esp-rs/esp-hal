use esp32 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

crate::peripherals! {
    AES,
    APB_CTRL,
    BB,
    DPORT,
    EFUSE,
    FLASH_ENCRYPTION,
    FRC_TIMER,
    GPIO,
    GPIO_SD,
    HINF,
    I2C0,
    I2C1,
    I2S0,
    I2S1,
    IO_MUX,
    LEDC,
    PWM0,
    PWM1,
    NRX,
    PCNT,
    RMT,
    RNG,
    RSA,
    RTC_CNTL,
    RTCIO,
    RTC_I2C,
    SDMMC,
    SENS,
    SHA,
    SLC,
    SLCHOST,
    SPI0,
    SPI1,
    SPI2,
    SPI3,
    TIMG0,
    TIMG1,
    TWAI,
    UART0,
    UART1,
    UART2,
    UHCI0,
    UHCI1,
}

mod peripherals {
    pub use super::pac::*;

    crate::create_peripherals! {
        I2C0,
        I2C1,
        RNG,
        SHA,
        SPI0,
        SPI1,
        SPI2,
        SPI3,
        UART0,
        UART1,
        UART2,
        DPORT,
        LEDC,
        RMT,
    }
}
