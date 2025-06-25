//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

pub(crate) use esp32 as pac;
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
        UART2 <= UART2,
    ],
    unstable_peripherals: [
        ADC1 <= virtual,
        ADC2 <= virtual,
        AES <= AES,
        APB_CTRL <= APB_CTRL,
        BB <= BB,
        BT <= virtual,
        CPU_CTRL <= virtual,
        DAC1 <= virtual,
        DAC2 <= virtual,
        DPORT <= DPORT,
        EFUSE <= EFUSE,
        FLASH_ENCRYPTION <= FLASH_ENCRYPTION,
        FRC_TIMER <= FRC_TIMER,
        GPIO <= GPIO,
        GPIO_SD <= GPIO_SD,
        HINF <= HINF,
        I2S0 <= I2S0 (I2S0),
        I2S1 <= I2S1 (I2S1),
        IO_MUX <= IO_MUX,
        LEDC <= LEDC,
        LPWR <= RTC_CNTL,
        MCPWM0 <= MCPWM0,
        MCPWM1 <= MCPWM1,
        NRX <= NRX,
        PCNT <= PCNT,
        PSRAM <= virtual,
        RADIO_CLK <= virtual,
        RMT <= RMT,
        RNG <= RNG,
        RSA <= RSA,
        RTC_I2C <= RTC_I2C,
        RTC_IO <= RTC_IO,
        SDHOST <= SDHOST,
        SENS <= SENS,
        SHA <= SHA,
        SLC <= SLC,
        SLCHOST <= SLCHOST,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SYSTEM <= DPORT,
        SW_INTERRUPT <= virtual,
        TIMG0 <= TIMG0,
        TIMG1 <= TIMG1,
        TOUCH <= virtual,
        TWAI0 <= TWAI0,
        UHCI0 <= UHCI0,
        UHCI1 <= UHCI1,
        WIFI <= WIFI,

        DMA_SPI2 <= SPI2,
        DMA_SPI3 <= SPI3,
        DMA_I2S0 <= I2S0,
        DMA_I2S1 <= I2S1,
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
        22,
        23,
        24,
        25,
        26,
        27,
        32,
        33,
        34,
        35,
        36,
        37,
        38,
        39,
    ]
}

include!(concat!(env!("OUT_DIR"), "/_generated_gpio.rs"));
