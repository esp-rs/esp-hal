//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

pub(crate) use esp32s3 as pac;
// We need to export this for users to use
#[doc(hidden)]
pub use pac::Interrupt;

// Note that certain are marked with `virtual` in the invocation of the
// `peripherals!` macro below. Basically, this indicates there's no physical
// peripheral (no `PSRAM`, `RADIO`, etc. peripheral in the PACs), so we're
// creating "virtual peripherals" for them.
crate::peripherals! {
    peripherals: [
        I2C0 <= I2C0,
        I2C1 <= I2C1,
        SPI2 <= SPI2 (SPI2),
        SPI3 <= SPI3 (SPI3),
        UART0 <= UART0,
        UART1 <= UART1,
        UART2 <= UART2,
    ],
    unstable_peripherals: [
        ADC1 <= virtual,
        ADC2 <= virtual,
        AES <= AES,
        APB_SARADC <= APB_SARADC,
        APB_CTRL <= APB_CTRL,
        ASSIST_DEBUG <= ASSIST_DEBUG,
        BT <= virtual,
        CPU_CTRL <= virtual,
        DMA <= DMA,
        DS <= DS,
        EFUSE <= EFUSE,
        EXTMEM <= EXTMEM,
        GPIO <= GPIO,
        GPIO_SD <= GPIO_SD,
        HMAC <= HMAC,
        I2S0 <= I2S0 (I2S0),
        I2S1 <= I2S1 (I2S1),
        INTERRUPT_CORE0 <= INTERRUPT_CORE0,
        INTERRUPT_CORE1 <= INTERRUPT_CORE1,
        IO_MUX <= IO_MUX,
        LCD_CAM <= LCD_CAM,
        LEDC <= LEDC,
        LPWR <= RTC_CNTL,
        PCNT <= PCNT,
        PERI_BACKUP <= PERI_BACKUP,
        PSRAM <= virtual,
        MCPWM0 <= MCPWM0,
        MCPWM1 <= MCPWM1,
        RMT <= RMT,
        RNG <= RNG,
        RSA <= RSA,
        RTC_I2C <= RTC_I2C,
        RTC_IO <= RTC_IO,
        SENS <= SENS,
        SENSITIVE <= SENSITIVE,
        SHA <= SHA,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SYSTEM <= SYSTEM,
        SYSTIMER <= SYSTIMER,
        SW_INTERRUPT <= virtual,
        TIMG0 <= TIMG0,
        TIMG1 <= TIMG1,
        TWAI0 <= TWAI0,
        UHCI0 <= UHCI0,
        UHCI1 <= UHCI1,
        ULP_RISCV_CORE <= virtual,
        USB0 <= USB0,
        USB_DEVICE <= USB_DEVICE,
        USB_WRAP <= USB_WRAP,
        WCL <= WCL,
        WIFI <= virtual,
        XTS_AES <= XTS_AES,

        DMA_CH0 <= virtual,
        DMA_CH1 <= virtual,
        DMA_CH2 <= virtual,
        DMA_CH3 <= virtual,
        DMA_CH4 <= virtual,
    ],
    pins: [
        0,
        1,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        16,
        17,
        18,
        19,
        20,
        21,
        26,
        27,
        28,
        29,
        30,
        31,
        32,
        33,
        34,
        35,
        36,
        37,
        38,
        39,
        40,
        41,
        42,
        43,
        44,
        45,
        46,
        47,
        48,
    ]
}

include!(concat!(env!("OUT_DIR"), "/_generated_peris.rs"));
include!(concat!(env!("OUT_DIR"), "/_generated_gpio.rs"));
