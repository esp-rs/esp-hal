//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

use esp32p4 as pac;
// We need to export this for users to use
pub use pac::Interrupt;

// We need to export this in the hal for the drivers to use
pub(crate) use self::peripherals::*;

// Note that certain are marked with `virtual` in the invocation of the
// `peripherals!` macro below. Basically, this indicates there's no physical
// peripheral (no `PSRAM`, `RADIO`, etc. peripheral in the PACs), so we're
// creating "virtual peripherals" for them.
crate::peripherals! {
    peripherals: [
        ADC <= ADC,
        AES <= AES,
        AHB_DMA <= AHB_DMA,
        ASSIST_DEBUG <= ASSIST_DEBUG,
        AXI_DMA <= AXI_DMA,
        AXI_ICM <= AXI_ICM,
        BITSCRAMBLER <= BITSCRAMBLER,
        CACHE <= CACHE,
        DMA <= DMA (AHB_PDMA_IN_CH0, AHB_PDMA_IN_CH1, AHB_PDMA_IN_CH2, AHB_PDMA_OUT_CH0, AHB_PDMA_OUT_CH1, AHB_PDMA_OUT_CH2, AXI_PDMA_IN_CH0, AXI_PDMA_IN_CH1, AXI_PDMA_IN_CH2, AXI_PDMA_OUT_CH0, AXI_PDMA_OUT_CH1, AXI_PDMA_OUT_CH2),
        DS <= DS,
        ECC <= ECC,
        ECDSA <= ECDSA,
        EFUSE <= EFUSE,
        GPIO <= GPIO,
        GPIO_SD <= GPIO_SD,
        H264 <= H264,
        H264_DMA <= H264_DMA,
        HMAC <= HMAC,
        HP_SYS_CLKRST <= HP_SYS_CLKRST,
        I2C0 <= I2C0,
        I2C1 <= I2C1,
        I2S0 <= I2S0,
        I2S1 <= I2S1,
        I2S2 <= I2S2,
        I3C_MST <= I3C_MST,
        I3C_MST_MEM <= I3C_MST_MEM,
        I3C_SLV <= I3C_SLV,
        INTERRUPT_CORE0 <= INTERRUPT_CORE0,
        INTERRUPT_CORE1 <= INTERRUPT_CORE1,
        IO_MUX <= IO_MUX,
        ISP <= ISP,
        JPEG <= JPEG,
        LCD_CAM <= LCD_CAM,
        LEDC <= LEDC,
        LP_ADC <= LP_ADC,
        LP_ANA <= LP_ANA,
        LP_AON_CLKRST <= LP_AON_CLKRST,
        LP_GPIO <= LP_GPIO,
        LP_HUK <= LP_HUK,
        LP_I2C0 <= LP_I2C0,
        LP_I2C_ANA_MST <= LP_I2C_ANA_MST,
        LP_I2S0 <= LP_I2S0,
        LP_INTR <= LP_INTR,
        LP_IO_MUX <= LP_IO_MUX,
        LP_PERI <= LP_PERI,
        LP_SYS <= LP_SYS,
        LP_TIMER <= LP_TIMER,
        LP_TOUCH <= LP_TOUCH,
        LP_TSENS <= LP_TSENS,
        LP_UART <= LP_UART,
        LP_WDT <= LP_WDT,
        MCPWM0 <= MCPWM0,
        MCPWM1 <= MCPWM1,
        MIPI_CSI_BRIDGE <= MIPI_CSI_BRIDGE,
        MIPI_CSI_HOST <= MIPI_CSI_HOST,
        MIPI_DSI_BRIDGE <= MIPI_DSI_BRIDGE,
        MIPI_DSI_HOST <= MIPI_DSI_HOST,
        PARL_IO <= PARL_IO,
        PAU <= PAU,
        PCNT <= PCNT,
        PMU <= PMU,
        PPA <= PPA,
        PVT <= PVT,
        RMT <= RMT,
        RSA <= RSA,
        SDHOST <= SDHOST,
        SHA <= SHA,
        SOC_ETM <= SOC_ETM,
        SPI0 <= SPI0,
        SPI1 <= SPI1,
        SPI2 <= SPI2,
        SPI3 <= SPI3,
        SW_INTERRUPT <= virtual,
        SYSTEM <= HP_SYS,
        SYSTIMER <= SYSTIMER,
        TIMG0 <= TIMG0,
        TIMG1 <= TIMG1,
        TRACE0 <= TRACE0,
        TRACE1 <= TRACE1,
        TWAI0 <= TWAI0,
        TWAI1 <= TWAI1,
        TWAI2 <= TWAI2,
        UART0 <= UART0,
        UART1 <= UART1,
        UART2 <= UART2,
        UART3 <= UART3,
        UART4 <= UART4,
        UHCI0 <= UHCI0,
        USB_DEVICE <= USB_DEVICE,
        USB_WRAP <= USB_WRAP,
    ],
    pins: [
        (0, [Input, Output, RtcIo])
        (1, [Input, Output, RtcIo])
        (2, [Input, Output, RtcIo])
        (3, [Input, Output, RtcIo])
        (4, [Input, Output, RtcIo])
        (5, [Input, Output, RtcIo])
        (6, [Input, Output, RtcIo] (3 => SPI2_HOLD_PAD) (3 => SPI2_HOLD_PAD))
        (7, [Input, Output, RtcIo] (3 => SPI2_CS_PAD) (3 => SPI2_CS_PAD))
        (8, [Input, Output, RtcIo] (3 => SPI2_D_PAD) (2 => U0RTS 3 => SPI2_D_PAD))
        (9, [Input, Output, RtcIo] (2 => U0CTS) (3 => SPI2_CK_PAD))
        (10, [Input, Output, RtcIo] (3 => SPI2_Q_PAD) (2 => U1TXD))
        (11, [Input, Output, RtcIo] (2 => U1RXD 3 => SPI2_WP_PAD) (3 => SPI2_WP_PAD))
        (12, [Input, Output, RtcIo] () (2 => U1RTS))
        (13, [Input, Output, RtcIo] (2 => U1CTS) ())
        (14, [Input, Output, RtcIo])
        (15, [Input, Output, RtcIo])
        (16, [Input, Output, RtcIo])
        (17, [Input, Output, RtcIo])
        (18, [Input, Output, RtcIo])
        (19, [Input, Output, RtcIo])
        (20, [Input, Output, RtcIo])
        (21, [Input, Output, RtcIo])
        (22, [Input, Output, RtcIo])
        (23, [Input, Output, RtcIo])
        (24, [Input, Output, RtcIo])
        (25, [Input, Output, RtcIo])
        (26, [Input, Output, RtcIo])
        (27, [Input, Output, RtcIo])
        (28, [Input, Output, RtcIo] (2 => SPI2_CS_PAD 3 => GMAC_PHY_RXDV_PAD) ())
        (29, [Input, Output, RtcIo] (2 => SPI2_D_PAD 3 => GMAC_PHY_RXD0_PAD) ())
        (30, [Input, Output, RtcIo] (3 => GMAC_PHY_RXD1_PAD) (2 => SPI2_CK_PAD))
        (31, [Input, Output, RtcIo] (2 => SPI2_Q_PAD 3 => GMAC_PHY_RXER_PAD) ())
        (32, [Input, Output] (2 => SPI2_HOLD_PAD 3 => GMAC_RX_CLK_PAD) (2 => SPI2_HOLD_PAD))
        (33, [Input, Output] (2 => SPI2_WP_PAD) (2 => SPI2_WP_PAD 3 => GMAC_PHY_TXEN_PAD))
        (34, [Input, Output] (2 => SPI2_IO4_PAD) (2 => SPI2_IO4_PAD 3 => GMAC_PHY_TXD0_PAD))
        (35, [Input, Output] (2 => SPI2_IO5_PAD) (2 => SPI2_IO5_PAD 3 => GMAC_PHY_TXD1_PAD))
        (36, [Input, Output] (2 => SPI2_IO6_PAD) (2 => SPI2_IO6_PAD 3 => GMAC_PHY_TXER_PAD))
        (37, [Input, Output] (2 => SPI2_IO7_PAD) (2 => SPI2_IO7_PAD))
        (38, [Input, Output] (2 => SPI2_Q_PAD) (2 => SPI2_Q_PAD))
        (39, [Input, Output])
        (40, [Input, Output] () (3 => GMAC_PHY_TXEN_PAD))
        (41, [Input, Output] () (3 => GMAC_PHY_TXD0_PAD))
        (42, [Input, Output] () (3 => GMAC_PHY_TXD1_PAD))
        (43, [Input, Output] () (3 => GMAC_PHY_TXER_PAD))
        (44, [Input, Output] (3 => GMAC_RX_CLK_PAD) ())
        (45, [Input, Output] (3 => GMAC_PHY_RXDV_PAD) ())
        (46, [Input, Output] (3 => GMAC_PHY_RXD0_PAD) ())
        (47, [Input, Output] (3 => GMAC_PHY_RXD1_PAD) ())
        (48, [Input, Output] (3 => GMAC_PHY_RXER_PAD) ())
        (49, [Input, Output] () (3 => GMAC_PHY_TXEN_PAD))
        (50, [Input, Output] (3 => GMAC_RX_CLK_PAD) ())
        (51, [Input, Output] (3 => GMAC_PHY_RXDV_PAD) ())
        (52, [Input, Output] (3 => GMAC_PHY_RXD0_PAD) ())
        (53, [Input, Output] (3 => GMAC_PHY_RXD1_PAD) ())
        (54, [Input, Output] (3 => GMAC_PHY_RXER_PAD) ())
    ]
}
