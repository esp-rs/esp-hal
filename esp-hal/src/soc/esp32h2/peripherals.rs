//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

pub(crate) use esp32h2 as pac;
// We need to export this for users to use
#[doc(hidden)]
pub use pac::Interrupt;

// Note that certain are marked with `virtual` in the invocation of the
// `peripherals!` macro below. Basically, this indicates there's no physical
// peripheral (no `PSRAM`, `RADIO`, etc. peripheral in the PACs), so we're
// creating "virtual peripherals" for them.
crate::peripherals! {
    peripherals: [
        I2C0 <= I2C0 (peri => I2C_EXT0),
        I2C1 <= I2C1 (peri => I2C_EXT1),
        SPI2 <= SPI2 (peri => SPI2),
        UART0 <= UART0,
        UART1 <= UART1,
    ],
    unstable_peripherals: [
        ADC1 <= virtual,
        AES <= AES,
        APB_SARADC <= APB_SARADC,
        ASSIST_DEBUG <= ASSIST_DEBUG,
        BT <= virtual,
        DMA <= DMA,
        DS <= DS,
        ECC <= ECC,
        EFUSE <= EFUSE,
        GPIO <= GPIO,
        GPIO_SD <= GPIO_SD,
        HMAC <= HMAC,
        HP_APM <= HP_APM,
        HP_SYS <= HP_SYS,
        I2C_ANA_MST <= I2C_ANA_MST,
        I2S0 <= I2S0 (peri => I2S0),
        IEEE802154 <= IEEE802154,
        INTERRUPT_CORE0 <= INTERRUPT_CORE0,
        INTPRI <= INTPRI,
        IO_MUX <= IO_MUX,
        LEDC <= LEDC,
        LPWR <= LP_CLKRST,
        LP_ANA <= LP_ANA,
        LP_AON <= LP_AON,
        LP_APM <= LP_APM,
        LP_APM0 <= LP_APM0,
        LP_PERI <= LP_PERI,
        LP_TIMER <= LP_TIMER,
        LP_WDT <= LP_WDT,
        MCPWM0 <= MCPWM0,
        MEM_MONITOR <= MEM_MONITOR,
        MODEM_LPCON <= MODEM_LPCON,
        MODEM_SYSCON <= MODEM_SYSCON,
        OTP_DEBUG <= OTP_DEBUG,
        PARL_IO <= PARL_IO (tx => PARL_IO_TX, rx => PARL_IO_RX),
        PAU <= PAU,
        PCNT <= PCNT,
        PCR <= PCR,
        PLIC_MX <= PLIC_MX,
        PMU <= PMU,
        RADIO_CLK <= virtual,
        RMT <= RMT,
        RNG <= RNG,
        RSA <= RSA,
        SHA <= SHA,
        SOC_ETM <= SOC_ETM,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SYSTEM <= PCR,
        SYSTIMER <= SYSTIMER,
        SW_INTERRUPT <= virtual,
        TEE <= TEE,
        TIMG0 <= TIMG0,
        TIMG1 <= TIMG1,
        TRACE0 <= TRACE,
        TWAI0 <= TWAI0,
        UHCI0 <= UHCI0,
        USB_DEVICE <= USB_DEVICE,
        MEM2MEM1 <= virtual,
        MEM2MEM4 <= virtual,
        MEM2MEM5 <= virtual,
        MEM2MEM10 <= virtual,
        MEM2MEM11 <= virtual,
        MEM2MEM12 <= virtual,
        MEM2MEM13 <= virtual,
        MEM2MEM14 <= virtual,
        MEM2MEM15 <= virtual,

        DMA_CH0 <= virtual,
        DMA_CH1 <= virtual,
        DMA_CH2 <= virtual,
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
    ]
}

include!(concat!(env!("OUT_DIR"), "/_generated_peris.rs"));
include!(concat!(env!("OUT_DIR"), "/_generated_gpio.rs"));
