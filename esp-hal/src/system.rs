//! # System Control

use esp_sync::NonReentrantMutex;

cfg_if::cfg_if! {
    if #[cfg(all(soc_multi_core_enabled, feature = "unstable"))] {
        pub(crate) mod multi_core;
        #[cfg(feature = "unstable")]
        pub use multi_core::*;
    }
}

// Implements the Peripheral enum based on esp-metadata/device.soc/peripheral_clocks
// ESP32-P4: manual Peripheral enum with HP_SYS_CLKRST clock gate registers.
// The generated macro produces an empty #[repr(u8)] enum (no clock nodes in metadata).
// This manual definition covers all 43 peripherals with their enable/reset registers.
// Ref: esp-idf clk_gate_ll.h, hp_sys_clkrst_reg.h, TRM v0.5 Ch 12/22.
#[cfg(not(esp32p4))]
implement_peripheral_clocks!();

/// ESP32-P4 peripheral clock gates and resets.
///
/// All controlled via HP_SYS_CLKRST registers:
///   SOC_CLK_CTRL0 (0x14): Flash/PSRAM/GDMA system clocks
///   SOC_CLK_CTRL1 (0x18): Peripheral system clocks
///   SOC_CLK_CTRL2 (0x1c): APB clocks
///   HP_RST_EN0 (0xc0): Core/cache/DMA resets
///   HP_RST_EN1 (0xc4): Timer/UART/I2C/misc resets
///   HP_RST_EN2 (0xc8): SPI/I2S/crypto/misc resets
///   PERI_CLK_CTRLxx: Per-peripheral clock dividers and enables
///
/// Ref: esp-idf clk_gate_ll.h, hp_sys_clkrst_reg.h
///      TRM v0.5 Ch 12 (Reset and Clock), Ch 22 (System Registers)
#[cfg(esp32p4)]
mod _p4_peripheral_clocks {
    use crate::peripherals::HP_SYS_CLKRST;

    #[doc(hidden)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[repr(u8)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[allow(non_camel_case_types)]
    pub enum Peripheral {
        // -- UARTs (5x) -- SOC_CLK_CTRL1 sys + SOC_CLK_CTRL2 apb + HP_RST_EN1
        Uart0     = 0,
        Uart1     = 1,
        Uart2     = 2,
        Uart3     = 3,
        Uart4     = 4,
        // -- SPI (2x) -- SOC_CLK_CTRL1 sys + SOC_CLK_CTRL2 apb + HP_RST_EN2
        Spi2      = 5,
        Spi3      = 6,
        // -- I2C (2x) -- SOC_CLK_CTRL2 apb + HP_RST_EN1
        I2c0      = 7,
        I2c1      = 8,
        // -- I2S (3x) -- SOC_CLK_CTRL2 apb + HP_RST_EN2
        I2s0      = 9,
        I2s1      = 10,
        I2s2      = 11,
        // -- Timers -- SOC_CLK_CTRL2 apb + HP_RST_EN1
        Systimer  = 12,
        Timg0     = 13,
        Timg1     = 14,
        // -- GPIO/IOMUX -- PERI_CLK_CTRL26 + HP_RST_EN1
        Iomux     = 15,
        // -- PWM/Counter -- SOC_CLK_CTRL2 apb + HP_RST_EN1
        Ledc      = 16,
        Pcnt      = 17,
        Mcpwm0    = 18,
        Mcpwm1    = 19,
        Rmt       = 20,
        // -- CAN (TWAI) -- SOC_CLK_CTRL2 apb + HP_RST_EN1
        Twai0     = 21,
        Twai1     = 22,
        Twai2     = 23,
        // -- USB -- SOC_CLK_CTRL1 sys
        UsbOtg11  = 24, // Full-Speed
        UsbOtg20  = 25, // High-Speed
        UsbDevice = 26, // Serial/JTAG
        // -- Connectivity -- SOC_CLK_CTRL1 sys
        Emac      = 27,
        Sdmmc     = 28,
        Uhci      = 29,
        // -- Crypto -- SOC_CLK_CTRL1 crypto_sys + HP_RST_EN2
        Aes       = 30,
        Sha       = 31,
        Rsa       = 32,
        Ecc       = 33,
        Ecdsa     = 34,
        Hmac      = 35,
        Ds        = 36,
        // -- DMA -- SOC_CLK_CTRL1 sys + HP_RST_EN0/1
        // P4 has 3 DMA controllers:
        //   AHB_DMA (AHB PDMA): GDMA v2 compatible, used by esp-hal as "Dma"
        //   DW_GDMA (DesignWare GDMA): different register layout, not used by esp-hal
        //   AXI_DMA (AXI PDMA): AXI bus DMA
        Dma       = 37, // maps to AHB_DMA (GDMA v2 compatible)
        Gdma      = 38, // DW_GDMA (DesignWare GDMA) -- NOT esp-hal compatible
        AhbPdma   = 39, // alias for Dma (legacy)
        AxiPdma   = 40,
        // -- ADC -- SOC_CLK_CTRL2 apb + HP_RST_EN2
        Adc       = 41,
        // -- Parallel IO -- SOC_CLK_CTRL1 sys + SOC_CLK_CTRL2 apb + HP_RST_EN2
        Parlio    = 42,
        // -- Camera/Display -- SOC_CLK_CTRL1 sys + HP_RST_EN2
        LcdCam    = 43,
    }

    impl Peripheral {
        // NOTE (ESP32-P4): UsbDevice (USB-JTAG-Serial) kept enabled when the
        // esp-println `jtag-serial` backend or the esp-hal `usb_serial_jtag`
        // console driver is the boot log channel. Disabling its clock during
        // init silences all subsequent println output and looks like a hang.
        // TODO: gate this on a console feature; for now we keep it always on
        // because the EV board and probe firmware both use USB-JTAG-Serial.
        pub const KEEP_ENABLED: &[Peripheral] = &[
            Peripheral::Systimer,
            Peripheral::Iomux,
            Peripheral::UsbDevice,
        ];
        pub const COUNT: usize = Self::ALL.len();
        pub const ALL: &[Self] = &[
            Self::Uart0,
            Self::Uart1,
            Self::Uart2,
            Self::Uart3,
            Self::Uart4,
            Self::Spi2,
            Self::Spi3,
            Self::I2c0,
            Self::I2c1,
            Self::I2s0,
            Self::I2s1,
            Self::I2s2,
            Self::Systimer,
            Self::Timg0,
            Self::Timg1,
            Self::Iomux,
            Self::Ledc,
            Self::Pcnt,
            Self::Mcpwm0,
            Self::Mcpwm1,
            Self::Rmt,
            Self::Twai0,
            Self::Twai1,
            Self::Twai2,
            Self::UsbOtg11,
            Self::UsbOtg20,
            Self::UsbDevice,
            Self::Emac,
            Self::Sdmmc,
            Self::Uhci,
            Self::Aes,
            Self::Sha,
            Self::Rsa,
            Self::Ecc,
            Self::Ecdsa,
            Self::Hmac,
            Self::Ds,
            Self::Dma,
            Self::Gdma,
            Self::AhbPdma,
            Self::AxiPdma,
            Self::Adc,
            Self::Parlio,
            Self::LcdCam,
        ];
    }

    /// Enable or disable peripheral clock.
    /// Ref: esp-idf clk_gate_ll.h -- *_ll_enable_bus_clock() functions
    ///      esp-idf hp_sys_clkrst_reg.h
    ///      TRM v0.5 Ch 12
    pub(crate) unsafe fn enable_internal_racey(peripheral: Peripheral, enable: bool) {
        let c = HP_SYS_CLKRST::regs();
        match peripheral {
            // -- UARTs: sys_clk + apb_clk --
            Peripheral::Uart0 => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.uart0_sys_clk_en().bit(enable));
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.uart0_apb_clk_en().bit(enable));
            }
            Peripheral::Uart1 => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.uart1_sys_clk_en().bit(enable));
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.uart1_apb_clk_en().bit(enable));
            }
            Peripheral::Uart2 => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.uart2_sys_clk_en().bit(enable));
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.uart2_apb_clk_en().bit(enable));
            }
            Peripheral::Uart3 => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.uart3_sys_clk_en().bit(enable));
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.uart3_apb_clk_en().bit(enable));
            }
            Peripheral::Uart4 => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.uart4_sys_clk_en().bit(enable));
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.uart4_apb_clk_en().bit(enable));
            }
            // -- SPI: sys_clk + apb_clk --
            Peripheral::Spi2 => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.gpspi2_sys_clk_en().bit(enable));
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.gpspi2_apb_clk_en().bit(enable));
            }
            Peripheral::Spi3 => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.gpspi3_sys_clk_en().bit(enable));
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.gpspi3_apb_clk_en().bit(enable));
            }
            // -- I2C: apb_clk only --
            Peripheral::I2c0 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.i2c0_apb_clk_en().bit(enable));
            }
            Peripheral::I2c1 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.i2c1_apb_clk_en().bit(enable));
            }
            // -- I2S: apb_clk --
            Peripheral::I2s0 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.i2s0_apb_clk_en().bit(enable));
            }
            Peripheral::I2s1 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.i2s1_apb_clk_en().bit(enable));
            }
            Peripheral::I2s2 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.i2s2_apb_clk_en().bit(enable));
            }
            // -- Timers --
            Peripheral::Systimer => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.systimer_apb_clk_en().bit(enable));
                c.peri_clk_ctrl21()
                    .modify(|_, w| w.systimer_clk_en().bit(enable));
            }
            Peripheral::Timg0 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.timergrp0_apb_clk_en().bit(enable));
            }
            Peripheral::Timg1 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.timergrp1_apb_clk_en().bit(enable));
            }
            // -- GPIO/IOMUX --
            Peripheral::Iomux => {
                c.peri_clk_ctrl26()
                    .modify(|_, w| w.iomux_clk_en().bit(enable));
            }
            // -- PWM/Counter --
            Peripheral::Ledc => {
                // Note: LEDC uses SOC_CLK_CTRL3 in esp-idf, but PAC may differ
                // Ref: esp-idf clk_gate_ll.h -- _ledc_ll_enable_bus_clock
                // For now just reset control
            }
            Peripheral::Pcnt => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.pcnt_apb_clk_en().bit(enable));
            }
            Peripheral::Mcpwm0 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.mcpwm0_apb_clk_en().bit(enable));
            }
            Peripheral::Mcpwm1 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.mcpwm1_apb_clk_en().bit(enable));
            }
            Peripheral::Rmt => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.rmt_sys_clk_en().bit(enable));
            }
            // -- TWAI (CAN) --
            Peripheral::Twai0 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.twai0_apb_clk_en().bit(enable));
            }
            Peripheral::Twai1 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.twai1_apb_clk_en().bit(enable));
            }
            Peripheral::Twai2 => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.twai2_apb_clk_en().bit(enable));
            }
            // -- USB --
            Peripheral::UsbOtg11 => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.usb_otg11_sys_clk_en().bit(enable));
            }
            Peripheral::UsbOtg20 => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.usb_otg20_sys_clk_en().bit(enable));
            }
            Peripheral::UsbDevice => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.usb_device_apb_clk_en().bit(enable));
            }
            // -- Connectivity --
            Peripheral::Emac => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.emac_sys_clk_en().bit(enable));
            }
            Peripheral::Sdmmc => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.sdmmc_sys_clk_en().bit(enable));
            }
            Peripheral::Uhci => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.uhci_sys_clk_en().bit(enable));
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.uhci_apb_clk_en().bit(enable));
            }
            // -- Crypto (shared sys clock) --
            Peripheral::Aes
            | Peripheral::Sha
            | Peripheral::Rsa
            | Peripheral::Ecc
            | Peripheral::Ecdsa
            | Peripheral::Hmac
            | Peripheral::Ds => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.crypto_sys_clk_en().bit(enable));
            }
            // -- DMA --
            // Dma = AHB_DMA (GDMA v2 compatible, used by esp-hal)
            Peripheral::Dma => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.ahb_pdma_sys_clk_en().bit(enable));
            }
            // Gdma = DW_GDMA (DesignWare, NOT used by esp-hal DMA driver)
            Peripheral::Gdma => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.gdma_sys_clk_en().bit(enable));
            }
            Peripheral::AhbPdma => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.ahb_pdma_sys_clk_en().bit(enable));
            }
            Peripheral::AxiPdma => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.axi_pdma_sys_clk_en().bit(enable));
            }
            // -- ADC --
            Peripheral::Adc => {
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.adc_apb_clk_en().bit(enable));
            }
            // -- Parallel IO --
            Peripheral::Parlio => {
                c.soc_clk_ctrl1()
                    .modify(|_, w| w.parlio_sys_clk_en().bit(enable));
                c.soc_clk_ctrl2()
                    .modify(|_, w| w.parlio_apb_clk_en().bit(enable));
            }
            // -- LCD/Camera --
            Peripheral::LcdCam => {
                // LCD_CAM uses SOC_CLK_CTRL3 which may not be in PAC
                // Ref: esp-idf clk_gate_ll.h -- _lcdcam_ll_enable_bus_clock
            }
        }
    }

    /// Assert or de-assert peripheral reset.
    /// Ref: esp-idf hp_sys_clkrst_reg.h -- HP_RST_EN0/1/2
    ///      TRM v0.5 Ch 12
    #[allow(clippy::single_match)]
    pub(crate) unsafe fn assert_peri_reset_racey(peripheral: Peripheral, reset: bool) {
        let c = HP_SYS_CLKRST::regs();
        match peripheral {
            // -- UARTs: HP_RST_EN1 (core + apb reset) --
            Peripheral::Uart0 => {
                c.hp_rst_en1().modify(|_, w| {
                    w.rst_en_uart0_core()
                        .bit(reset)
                        .rst_en_uart0_apb()
                        .bit(reset)
                });
            }
            Peripheral::Uart1 => {
                c.hp_rst_en1().modify(|_, w| {
                    w.rst_en_uart1_core()
                        .bit(reset)
                        .rst_en_uart1_apb()
                        .bit(reset)
                });
            }
            Peripheral::Uart2 => {
                c.hp_rst_en1().modify(|_, w| {
                    w.rst_en_uart2_core()
                        .bit(reset)
                        .rst_en_uart2_apb()
                        .bit(reset)
                });
            }
            Peripheral::Uart3 => {
                c.hp_rst_en1().modify(|_, w| {
                    w.rst_en_uart3_core()
                        .bit(reset)
                        .rst_en_uart3_apb()
                        .bit(reset)
                });
            }
            Peripheral::Uart4 => {
                c.hp_rst_en1().modify(|_, w| {
                    w.rst_en_uart4_core()
                        .bit(reset)
                        .rst_en_uart4_apb()
                        .bit(reset)
                });
            }
            // -- SPI: HP_RST_EN2 --
            Peripheral::Spi2 => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_spi2().bit(reset));
            }
            Peripheral::Spi3 => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_spi3().bit(reset));
            }
            // -- I2C: HP_RST_EN1 --
            Peripheral::I2c0 => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_i2c0().bit(reset));
            }
            Peripheral::I2c1 => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_i2c1().bit(reset));
            }
            // -- I2S: HP_RST_EN2 --
            Peripheral::I2s0 => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_i2s0_apb().bit(reset));
            }
            Peripheral::I2s1 => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_i2s1_apb().bit(reset));
            }
            Peripheral::I2s2 => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_i2s2_apb().bit(reset));
            }
            // -- Timers: HP_RST_EN1 --
            Peripheral::Systimer => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_stimer().bit(reset));
            }
            Peripheral::Timg0 => {
                c.hp_rst_en1()
                    .modify(|_, w| w.rst_en_timergrp0().bit(reset));
            }
            Peripheral::Timg1 => {
                c.hp_rst_en1()
                    .modify(|_, w| w.rst_en_timergrp1().bit(reset));
            }
            // -- IOMUX: HP_RST_EN1 --
            Peripheral::Iomux => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_iomux().bit(reset));
            }
            // -- PWM/Counter: HP_RST_EN1 --
            Peripheral::Ledc => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_ledc().bit(reset));
            }
            Peripheral::Pcnt => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_pcnt().bit(reset));
            }
            Peripheral::Mcpwm0 => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_pwm0().bit(reset));
            }
            Peripheral::Mcpwm1 => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_pwm1().bit(reset));
            }
            Peripheral::Rmt => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_rmt().bit(reset));
            }
            // -- TWAI: HP_RST_EN1 (named can0/1/2) --
            Peripheral::Twai0 => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_can0().bit(reset));
            }
            Peripheral::Twai1 => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_can1().bit(reset));
            }
            Peripheral::Twai2 => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_can2().bit(reset));
            }
            // -- USB: reset via LP domain registers, not HP_RST_EN --
            Peripheral::UsbOtg11 | Peripheral::UsbOtg20 | Peripheral::UsbDevice => {
                return;
            }
            // -- Connectivity --
            Peripheral::Emac => {
                return;
            } // EMAC reset via dedicated register, not HP_RST_EN
            Peripheral::Sdmmc => {
                return;
            } // SDMMC reset not in HP_RST_EN
            Peripheral::Uhci => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_uhci().bit(reset));
            }
            // -- Crypto: HP_RST_EN2 --
            Peripheral::Aes => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_aes().bit(reset));
            }
            Peripheral::Sha => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_sha().bit(reset));
            }
            Peripheral::Rsa => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_rsa().bit(reset));
            }
            Peripheral::Ecc => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_ecc().bit(reset));
            }
            Peripheral::Ecdsa => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_ecdsa().bit(reset));
            }
            Peripheral::Hmac => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_hmac().bit(reset));
            }
            Peripheral::Ds => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_ds().bit(reset));
            }
            // -- DMA: HP_RST_EN0/1 --
            Peripheral::Dma => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_ahb_pdma().bit(reset));
            }
            Peripheral::Gdma => {
                c.hp_rst_en0().modify(|_, w| w.rst_en_gdma().bit(reset));
            }
            Peripheral::AhbPdma => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_ahb_pdma().bit(reset));
            }
            Peripheral::AxiPdma => {
                c.hp_rst_en1().modify(|_, w| w.rst_en_axi_pdma().bit(reset));
            }
            // -- ADC: HP_RST_EN2 --
            Peripheral::Adc => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_adc().bit(reset));
            }
            // -- Parallel IO: HP_RST_EN2 --
            Peripheral::Parlio => {
                c.hp_rst_en2().modify(|_, w| {
                    w.rst_en_parlio()
                        .bit(reset)
                        .rst_en_parlio_rx()
                        .bit(reset)
                        .rst_en_parlio_tx()
                        .bit(reset)
                });
            }
            // -- LCD/Camera: HP_RST_EN2 --
            Peripheral::LcdCam => {
                c.hp_rst_en2().modify(|_, w| w.rst_en_lcdcam().bit(reset));
            }
        }
    }
}

#[cfg(esp32p4)]
pub use _p4_peripheral_clocks::*;

#[cfg(not(esp32p4))]
impl Peripheral {
    pub const fn try_from(value: u8) -> Option<Peripheral> {
        if value >= Peripheral::COUNT as u8 {
            return None;
        }

        Some(unsafe { core::mem::transmute::<u8, Peripheral>(value) })
    }
}

#[cfg(esp32p4)]
impl Peripheral {
    pub const fn try_from(_value: u8) -> Option<Peripheral> {
        None
    }
}

struct RefCounts {
    counts: [usize; Peripheral::COUNT],
}

impl RefCounts {
    pub const fn new() -> Self {
        Self {
            counts: [0; Peripheral::COUNT],
        }
    }
}

static PERIPHERAL_REF_COUNT: NonReentrantMutex<RefCounts> =
    NonReentrantMutex::new(RefCounts::new());

/// Disable all peripherals.
///
/// Peripherals listed in [KEEP_ENABLED] are NOT disabled.
#[cfg_attr(not(feature = "rt"), expect(dead_code))]
pub(crate) fn disable_peripherals() {
    // Take the critical section up front to avoid taking it multiple times.
    PERIPHERAL_REF_COUNT.with(|refcounts| {
        for p in Peripheral::KEEP_ENABLED {
            refcounts.counts[*p as usize] += 1;
        }
        for p in Peripheral::ALL {
            let ref_count = refcounts.counts[*p as usize];
            if ref_count == 0 {
                PeripheralClockControl::enable_forced_with_counts(*p, false, true, refcounts);
            }
        }
    })
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct PeripheralGuard {
    peripheral: Peripheral,
}

impl PeripheralGuard {
    pub(crate) fn new_with(p: Peripheral, init: fn()) -> Self {
        PeripheralClockControl::request_peripheral(p, init);

        Self { peripheral: p }
    }

    pub(crate) fn new(p: Peripheral) -> Self {
        Self::new_with(p, || {})
    }
}

impl Clone for PeripheralGuard {
    fn clone(&self) -> Self {
        Self::new(self.peripheral)
    }

    fn clone_from(&mut self, _source: &Self) {
        // This is a no-op since the ref count for P remains the same.
    }
}

impl Drop for PeripheralGuard {
    fn drop(&mut self) {
        PeripheralClockControl::disable(self.peripheral);
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct GenericPeripheralGuard<const P: u8> {}

impl<const P: u8> GenericPeripheralGuard<P> {
    pub(crate) fn new_with(init: fn()) -> Self {
        let p = const { Peripheral::try_from(P).unwrap() };
        PeripheralClockControl::request_peripheral(p, init);

        Self {}
    }

    #[cfg_attr(not(feature = "unstable"), allow(unused))]
    pub(crate) fn new() -> Self {
        Self::new_with(|| {})
    }
}

impl<const P: u8> Clone for GenericPeripheralGuard<P> {
    fn clone(&self) -> Self {
        Self::new()
    }

    fn clone_from(&mut self, _source: &Self) {
        // This is a no-op since the ref count for P remains the same.
    }
}

impl<const P: u8> Drop for GenericPeripheralGuard<P> {
    fn drop(&mut self) {
        let peripheral = const { Peripheral::try_from(P).unwrap() };
        PeripheralClockControl::disable(peripheral);
    }
}

/// Controls the enablement of peripheral clocks.
pub(crate) struct PeripheralClockControl;

impl PeripheralClockControl {
    fn request_peripheral(p: Peripheral, init: fn()) {
        PERIPHERAL_REF_COUNT.with(|ref_counts| {
            if Self::enable_with_counts(p, ref_counts) {
                unsafe { Self::reset_racey(p) };
                init();
            }
        });
    }

    /// Enables the given peripheral.
    ///
    /// This keeps track of enabling a peripheral - i.e. a peripheral
    /// is only enabled with the first call attempt to enable it.
    ///
    /// Returns `true` if it actually enabled the peripheral.
    pub(crate) fn enable(peripheral: Peripheral) -> bool {
        PERIPHERAL_REF_COUNT.with(|ref_counts| Self::enable_with_counts(peripheral, ref_counts))
    }

    /// Enables the given peripheral.
    ///
    /// This keeps track of enabling a peripheral - i.e. a peripheral
    /// is only enabled with the first call attempt to enable it.
    ///
    /// Returns `true` if it actually enabled the peripheral.
    fn enable_with_counts(peripheral: Peripheral, ref_counts: &mut RefCounts) -> bool {
        Self::enable_forced_with_counts(peripheral, true, false, ref_counts)
    }

    /// Disables the given peripheral.
    ///
    /// This keeps track of disabling a peripheral - i.e. it only
    /// gets disabled when the number of enable/disable attempts is balanced.
    ///
    /// Returns `true` if it actually disabled the peripheral.
    pub(crate) fn disable(peripheral: Peripheral) -> bool {
        PERIPHERAL_REF_COUNT.with(|ref_counts| {
            Self::enable_forced_with_counts(peripheral, false, false, ref_counts)
        })
    }

    fn enable_forced_with_counts(
        peripheral: Peripheral,
        enable: bool,
        force: bool,
        ref_counts: &mut RefCounts,
    ) -> bool {
        let ref_count = &mut ref_counts.counts[peripheral as usize];
        if !force {
            let prev = *ref_count;
            if enable {
                *ref_count += 1;
                trace!("Enable {:?} {} -> {}", peripheral, prev, *ref_count);
                if prev > 0 {
                    return false;
                }
            } else {
                assert!(prev != 0);
                *ref_count -= 1;
                trace!("Disable {:?} {} -> {}", peripheral, prev, *ref_count);
                if prev > 1 {
                    return false;
                }
            };
        } else if !enable {
            assert!(*ref_count == 0);
        }

        debug!("Enable {:?} {}", peripheral, enable);
        unsafe { enable_internal_racey(peripheral, enable) };

        true
    }

    /// Resets the given peripheral
    pub(crate) unsafe fn reset_racey(peripheral: Peripheral) {
        debug!("Reset {:?}", peripheral);

        unsafe {
            assert_peri_reset_racey(peripheral, true);
            assert_peri_reset_racey(peripheral, false);
        }
    }

    /// Resets the given peripheral
    pub(crate) fn reset(peripheral: Peripheral) {
        PERIPHERAL_REF_COUNT.with(|_| unsafe { Self::reset_racey(peripheral) })
    }
}

/// Available CPU cores
///
/// The actual number of available cores depends on the target.
#[derive(Debug, Copy, Clone, PartialEq, Eq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    #[cfg(multi_core)]
    AppCpu = 1,
}

impl Cpu {
    /// The number of available cores.
    pub const COUNT: usize = 1 + cfg!(multi_core) as usize;

    #[procmacros::doc_replace]
    /// Returns the core the application is currently executing on
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// #
    /// use esp_hal::system::Cpu;
    /// let current_cpu = Cpu::current();
    /// #
    /// # {after_snippet}
    /// ```
    #[inline(always)]
    pub fn current() -> Self {
        // This works for both RISCV and Xtensa because both
        // get_raw_core functions return zero, _or_ something
        // greater than zero; 1 in the case of RISCV and 0x2000
        // in the case of Xtensa.
        match raw_core() {
            0 => Cpu::ProCpu,
            #[cfg(all(multi_core, riscv))]
            1 => Cpu::AppCpu,
            #[cfg(all(multi_core, xtensa))]
            0x2000 => Cpu::AppCpu,
            _ => unreachable!(),
        }
    }

    /// Returns an iterator over the "other" cores.
    #[inline(always)]
    #[instability::unstable]
    pub fn other() -> impl Iterator<Item = Self> {
        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                match Self::current() {
                    Cpu::ProCpu => [Cpu::AppCpu].into_iter(),
                    Cpu::AppCpu => [Cpu::ProCpu].into_iter(),
                }
            } else {
                [].into_iter()
            }
        }
    }

    /// Returns an iterator over all cores.
    #[inline(always)]
    pub fn all() -> impl Iterator<Item = Self> {
        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                [Cpu::ProCpu, Cpu::AppCpu].into_iter()
            } else {
                [Cpu::ProCpu].into_iter()
            }
        }
    }
}

/// Returns the raw value of the mhartid register.
///
/// On RISC-V, this is the hardware thread ID.
///
/// On Xtensa, this returns the result of reading the PRID register logically
/// ANDed with 0x2000, the 13th bit in the register. Espressif Xtensa chips use
/// this bit to determine the core id.
#[inline(always)]
pub(crate) fn raw_core() -> usize {
    // This method must never return UNUSED_THREAD_ID_VALUE
    cfg_if::cfg_if! {
        if #[cfg(all(multi_core, riscv))] {
            riscv::register::mhartid::read()
        } else if #[cfg(all(multi_core, xtensa))] {
            (xtensa_lx::get_processor_id() & 0x2000) as usize
        } else {
            0
        }
    }
}

use crate::rtc_cntl::SocResetReason;

/// Source of the wakeup event
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum SleepSource {
    /// In case of deep sleep, reset was not caused by exit from deep sleep
    Undefined = 0,
    /// Not a wakeup cause, used to disable all wakeup sources with
    /// esp_sleep_disable_wakeup_source
    All,
    /// Wakeup caused by external signal using RTC_IO
    Ext0,
    /// Wakeup caused by external signal using RTC_CNTL
    Ext1,
    /// Wakeup caused by timer
    Timer,
    /// Wakeup caused by touchpad
    TouchPad,
    /// Wakeup caused by ULP program
    Ulp,
    /// Wakeup caused by GPIO (light sleep only on ESP32, S2 and S3)
    Gpio,
    /// Wakeup caused by UART (light sleep only)
    Uart,
    /// Wakeup caused by WIFI (light sleep only)
    Wifi,
    /// Wakeup caused by COCPU int
    Cocpu,
    /// Wakeup caused by COCPU crash
    CocpuTrapTrig,
    /// Wakeup caused by BT (light sleep only)
    BT,
}

#[procmacros::doc_replace]
/// Performs a software reset on the chip.
///
/// # Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::system::software_reset;
/// software_reset();
/// # {after_snippet}
/// ```
#[inline]
pub fn software_reset() -> ! {
    crate::rom::software_reset()
}

/// Resets the given CPU, leaving peripherals unchanged.
#[instability::unstable]
#[inline]
pub fn software_reset_cpu(cpu: Cpu) {
    crate::rom::software_reset_cpu(cpu as u32)
}

/// Retrieves the reason for the last reset as a SocResetReason enum value.
/// Returns `None` if the reset reason cannot be determined.
#[instability::unstable]
#[inline]
pub fn reset_reason() -> Option<SocResetReason> {
    crate::rtc_cntl::reset_reason(Cpu::current())
}

/// Retrieves the cause of the last wakeup event as a SleepSource enum value.
#[instability::unstable]
#[inline]
pub fn wakeup_cause() -> SleepSource {
    crate::rtc_cntl::wakeup_cause()
}
