pub use pac::Interrupt;

use crate::pac; // We need to export this for users to use

crate::peripherals! {
    AES,
    APB_CTRL,
    APB_SARADC,
    DEBUG_ASSIST,
    DMA,
    DS,
    EFUSE,
    EXTMEM,
    GPIO,
    GPIOSD,
    HMAC,
    I2C0,
    I2C1,
    I2S0,
    I2S1,
    INTERRUPT_CORE0,
    INTERRUPT_CORE1,
    IO_MUX,
    LCD_CAM,
    LEDC,
    PCNT,
    PERI_BACKUP,
    PWM0,
    PWM1,
    RMT,
    RNG,
    RSA,
    RTC_CNTL,
    RTC_I2C,
    RTCIO,
    SENS,
    SENSITIVE,
    SHA,
    SPI0,
    SPI1,
    SPI2,
    SPI3,
    SYSTEM,
    SYSTIMER,
    TIMG0,
    TIMG1,
    TWAI,
    UART0,
    UART1,
    UART2,
    UHCI0,
    UHCI1,
    USB0,
    USB_DEVICE,
    USB_WRAP,
    WCL,
    XTS_AES,
}

mod peripherals {
    pub use super::pac::*;

    crate::create_peripherals! {
        UART0,
        UART1,
    }
}
