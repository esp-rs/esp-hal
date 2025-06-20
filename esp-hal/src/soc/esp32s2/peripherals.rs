//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

pub(crate) use esp32s2 as pac;
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
        SPI2 <= SPI2 (SPI2_DMA, SPI2),
        SPI3 <= SPI3 (SPI3_DMA, SPI3),
        UART0 <= UART0,
        UART1 <= UART1,
    ],
    unstable_peripherals: [
        ADC1 <= virtual,
        ADC2 <= virtual,
        AES <= AES,
        APB_SARADC <= APB_SARADC,
        DAC1 <= virtual,
        DAC2 <= virtual,
        DEDICATED_GPIO <= DEDICATED_GPIO,
        DS <= DS,
        EFUSE <= EFUSE,
        EXTMEM <= EXTMEM,
        GPIO <= GPIO,
        GPIO_SD <= GPIO_SD,
        HMAC <= HMAC,
        I2C_ANA_MST <= I2C_ANA_MST,
        I2S0 <= I2S0 (I2S0),
        INTERRUPT_CORE0 <= INTERRUPT_CORE0,
        IO_MUX <= IO_MUX,
        LEDC <= LEDC,
        LPWR <= RTC_CNTL,
        PCNT <= PCNT,
        PMS <= PMS,
        PSRAM <= virtual,
        RADIO_CLK <= virtual,
        RMT <= RMT,
        RNG <= RNG,
        RSA <= RSA,
        RTC_IO <= RTC_IO,
        RTC_I2C <= RTC_I2C,
        SENS <= SENS,
        SHA <= SHA,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SYSCON <= SYSCON,
        SYSTEM <= SYSTEM,
        SYSTIMER <= SYSTIMER,
        SW_INTERRUPT <= virtual,
        TIMG0 <= TIMG0,
        TIMG1 <= TIMG1,
        TWAI0 <= TWAI0,
        UHCI0 <= UHCI0,
        ULP_RISCV_CORE <= virtual,
        USB0 <= USB0,
        USB_WRAP <= USB_WRAP,
        WIFI <= WIFI,
        XTS_AES <= XTS_AES,
        NRX <= NRX,
        FE <= FE,
        FE2 <= FE2,

        DMA_SPI2 <= SPI2,
        DMA_SPI3 <= SPI3,
        DMA_I2S0 <= I2S0,
        DMA_CRYPTO <= CRYPTO_DMA,
        DMA_COPY <= COPY_DMA,
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
    ]
}

include!(concat!(env!("OUT_DIR"), "/_generated_gpio.rs"));
