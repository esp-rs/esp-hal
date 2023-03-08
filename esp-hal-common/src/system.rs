//! System
//!
//! The SYSTEM/DPORT peripheral needs to be split into several logical parts.
//!
//! Example
//! ```no_run
//! let peripherals = Peripherals::take();
//! let system = peripherals.SYSTEM.split();
//! let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
//! ```

use crate::peripheral::PeripheralRef;
#[cfg(esp32)]
type SystemPeripheral = crate::peripherals::DPORT;
#[cfg(esp32c6)]
type SystemPeripheral = crate::peripherals::PCR;
#[cfg(not(any(esp32, esp32c6)))]
type SystemPeripheral = crate::peripherals::SYSTEM;
#[cfg(not(any(esp32, esp32c6)))]
pub enum SoftwareInterrupt {
    SoftwareInterrupt0,
    SoftwareInterrupt1,
    SoftwareInterrupt2,
    SoftwareInterrupt3,
}

/// Peripherals which can be enabled via [PeripheralClockControl]
pub enum Peripheral {
    Spi2,
    #[cfg(spi3)]
    Spi3,
    I2cExt0,
    #[cfg(i2c1)]
    I2cExt1,
    #[cfg(rmt)]
    Rmt,
    Ledc,
    #[cfg(mcpwm)]
    Mcpwm0,
    #[cfg(mcpwm)]
    Mcpwm1,
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c6))]
    Pcnt,
    #[cfg(any(esp32c2, esp32c3, esp32c6))]
    ApbSarAdc,
    #[cfg(gdma)]
    Gdma,
    #[cfg(pdma)]
    Dma,
    #[cfg(not(esp32c2))]
    I2s0,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32s2)))]
    I2s1,
    #[cfg(usb_otg)]
    Usb,
    #[cfg(any(esp32s3, esp32c3))]
    Twai,
    #[cfg(aes)]
    Aes,
    #[cfg(esp32c6)]
    Twai0,
    #[cfg(esp32c6)]
    Twai1,
}
#[cfg(not(any(esp32, esp32c6)))]
pub struct SoftwareInterruptControl {
    _private: (),
}
#[cfg(not(any(esp32, esp32c6)))]
impl SoftwareInterruptControl {
    pub fn raise(&mut self, interrupt: SoftwareInterrupt) {
        let system = unsafe { &*SystemPeripheral::PTR };
        match interrupt {
            SoftwareInterrupt::SoftwareInterrupt0 => {
                system
                    .cpu_intr_from_cpu_0
                    .write(|w| w.cpu_intr_from_cpu_0().bit(true));
            }
            SoftwareInterrupt::SoftwareInterrupt1 => {
                system
                    .cpu_intr_from_cpu_1
                    .write(|w| w.cpu_intr_from_cpu_1().bit(true));
            }
            SoftwareInterrupt::SoftwareInterrupt2 => {
                system
                    .cpu_intr_from_cpu_2
                    .write(|w| w.cpu_intr_from_cpu_2().bit(true));
            }
            SoftwareInterrupt::SoftwareInterrupt3 => {
                system
                    .cpu_intr_from_cpu_3
                    .write(|w| w.cpu_intr_from_cpu_3().bit(true));
            }
        }
    }
    pub fn reset(&mut self, interrupt: SoftwareInterrupt) {
        let system = unsafe { &*SystemPeripheral::PTR };
        match interrupt {
            SoftwareInterrupt::SoftwareInterrupt0 => {
                system
                    .cpu_intr_from_cpu_0
                    .write(|w| w.cpu_intr_from_cpu_0().bit(false));
            }
            SoftwareInterrupt::SoftwareInterrupt1 => {
                system
                    .cpu_intr_from_cpu_1
                    .write(|w| w.cpu_intr_from_cpu_1().bit(false));
            }
            SoftwareInterrupt::SoftwareInterrupt2 => {
                system
                    .cpu_intr_from_cpu_2
                    .write(|w| w.cpu_intr_from_cpu_2().bit(false));
            }
            SoftwareInterrupt::SoftwareInterrupt3 => {
                system
                    .cpu_intr_from_cpu_3
                    .write(|w| w.cpu_intr_from_cpu_3().bit(false));
            }
        }
    }
}
/// Controls the enablement of peripheral clocks.
pub struct PeripheralClockControl {
    _private: (),
}

#[cfg(not(esp32c6))]
impl PeripheralClockControl {
    /// Enables and resets the given peripheral
    pub fn enable(&mut self, peripheral: Peripheral) {
        let system = unsafe { &*SystemPeripheral::PTR };

        #[cfg(not(esp32))]
        let (perip_clk_en0, perip_rst_en0) = { (&system.perip_clk_en0, &system.perip_rst_en0) };
        #[cfg(esp32)]
        let (perip_clk_en0, perip_rst_en0, peri_clk_en, peri_rst_en) = {
            (
                &system.perip_clk_en,
                &system.perip_rst_en,
                &system.peri_clk_en,
                &system.peri_rst_en,
            )
        };

        #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
        let (perip_clk_en1, perip_rst_en1) = { (&system.perip_clk_en1, &system.perip_rst_en1) };

        match peripheral {
            Peripheral::Spi2 => {
                perip_clk_en0.modify(|_, w| w.spi2_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.spi2_rst().clear_bit());
            }
            #[cfg(spi3)]
            Peripheral::Spi3 => {
                perip_clk_en0.modify(|_, w| w.spi3_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.spi3_rst().clear_bit());
            }
            #[cfg(esp32)]
            Peripheral::I2cExt0 => {
                perip_clk_en0.modify(|_, w| w.i2c0_ext0_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.i2c0_ext0_rst().clear_bit());
            }
            #[cfg(not(esp32))]
            Peripheral::I2cExt0 => {
                perip_clk_en0.modify(|_, w| w.i2c_ext0_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.i2c_ext0_rst().clear_bit());
            }
            #[cfg(i2c1)]
            Peripheral::I2cExt1 => {
                perip_clk_en0.modify(|_, w| w.i2c_ext1_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.i2c_ext1_rst().clear_bit());
            }
            #[cfg(rmt)]
            Peripheral::Rmt => {
                perip_clk_en0.modify(|_, w| w.rmt_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.rmt_rst().clear_bit());
            }
            Peripheral::Ledc => {
                perip_clk_en0.modify(|_, w| w.ledc_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.ledc_rst().clear_bit());
            }
            #[cfg(mcpwm)]
            Peripheral::Mcpwm0 => {
                perip_clk_en0.modify(|_, w| w.pwm0_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.pwm0_rst().clear_bit());
            }
            #[cfg(mcpwm)]
            Peripheral::Mcpwm1 => {
                perip_clk_en0.modify(|_, w| w.pwm1_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.pwm1_rst().clear_bit());
            }
            #[cfg(any(esp32, esp32s2, esp32s3))]
            Peripheral::Pcnt => {
                perip_clk_en0.modify(|_, w| w.pcnt_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.pcnt_rst().clear_bit());
            }
            #[cfg(any(esp32c2, esp32c3))]
            Peripheral::ApbSarAdc => {
                perip_clk_en0.modify(|_, w| w.apb_saradc_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.apb_saradc_rst().clear_bit());
            }
            #[cfg(gdma)]
            Peripheral::Gdma => {
                perip_clk_en1.modify(|_, w| w.dma_clk_en().set_bit());
                perip_rst_en1.modify(|_, w| w.dma_rst().clear_bit());
            }
            #[cfg(esp32)]
            Peripheral::Dma => {
                perip_clk_en0.modify(|_, w| w.spi_dma_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.spi_dma_rst().clear_bit());
            }
            #[cfg(esp32s2)]
            Peripheral::Dma => {
                perip_clk_en0.modify(|_, w| w.spi2_dma_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.spi2_dma_rst().clear_bit());
                perip_clk_en0.modify(|_, w| w.spi3_dma_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.spi3_dma_rst().clear_bit());
            }
            #[cfg(esp32c3)]
            Peripheral::I2s0 => {
                // on ESP32-C3 note that i2s1_clk_en / rst is really I2s0
                perip_clk_en0.modify(|_, w| w.i2s1_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.i2s1_rst().clear_bit());
            }
            #[cfg(any(esp32s3, esp32, esp32s2))]
            Peripheral::I2s0 => {
                perip_clk_en0.modify(|_, w| w.i2s0_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.i2s0_rst().clear_bit());
            }
            #[cfg(any(esp32s3, esp32))]
            Peripheral::I2s1 => {
                perip_clk_en0.modify(|_, w| w.i2s1_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.i2s1_rst().clear_bit());
            }
            #[cfg(usb_otg)]
            Peripheral::Usb => {
                perip_clk_en0.modify(|_, w| w.usb_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.usb_rst().clear_bit());
            }
            #[cfg(any(esp32s3, esp32c3))]
            Peripheral::Twai => {
                perip_clk_en0.modify(|_, w| w.twai_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.twai_rst().clear_bit());
            }
            #[cfg(esp32)]
            Peripheral::Aes => {
                peri_clk_en.modify(|r, w| unsafe { w.bits(r.bits() | 1) });
                peri_rst_en.modify(|r, w| unsafe { w.bits(r.bits() & (!1)) });
            }
            #[cfg(any(esp32c3, esp32s2, esp32s3))]
            Peripheral::Aes => {
                perip_clk_en1.modify(|_, w| w.crypto_aes_clk_en().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_aes_rst().clear_bit());
            }
        }
    }
}

#[cfg(esp32c6)]
impl PeripheralClockControl {
    /// Enables and resets the given peripheral
    pub fn enable(&mut self, peripheral: Peripheral) {
        let system = unsafe { &*SystemPeripheral::PTR };

        match peripheral {
            Peripheral::Spi2 => {
                system.spi2_conf.modify(|_, w| w.spi2_clk_en().set_bit());
                system.spi2_conf.modify(|_, w| w.spi2_rst_en().clear_bit());
            }
            Peripheral::I2cExt0 => {
                system.i2c_conf.modify(|_, w| w.i2c_clk_en().set_bit());
                system.i2c_conf.modify(|_, w| w.i2c_rst_en().clear_bit());
            }
            Peripheral::Rmt => {
                system.rmt_conf.modify(|_, w| w.rmt_clk_en().set_bit());
                system.rmt_conf.modify(|_, w| w.rmt_rst_en().clear_bit());
            }
            Peripheral::Ledc => {
                system.ledc_conf.modify(|_, w| w.ledc_clk_en().set_bit());
                system.ledc_conf.modify(|_, w| w.ledc_rst_en().clear_bit());
            }
            Peripheral::Mcpwm0 | Peripheral::Mcpwm1 => {
                system.pwm_conf.modify(|_, w| w.pwm_clk_en().set_bit());
                system.pwm_conf.modify(|_, w| w.pwm_rst_en().clear_bit());
            }
            Peripheral::ApbSarAdc => {
                system
                    .saradc_conf
                    .modify(|_, w| w.saradc_reg_clk_en().set_bit());
                system
                    .saradc_conf
                    .modify(|_, w| w.saradc_reg_rst_en().clear_bit());
            }
            Peripheral::Gdma => {
                system.gdma_conf.modify(|_, w| w.gdma_clk_en().set_bit());
                system.gdma_conf.modify(|_, w| w.gdma_rst_en().clear_bit());
            }
            Peripheral::I2s0 => {
                system.i2s_conf.modify(|_, w| w.i2s_clk_en().set_bit());
                system.i2s_conf.modify(|_, w| w.i2s_rst_en().clear_bit());
            }
            Peripheral::Twai0 => {
                system.twai0_conf.modify(|_, w| w.twai0_clk_en().set_bit());
                system
                    .twai0_conf
                    .modify(|_, w| w.twai0_rst_en().clear_bit());
            }
            Peripheral::Twai1 => {
                system.twai1_conf.modify(|_, w| w.twai1_clk_en().set_bit());
                system
                    .twai1_conf
                    .modify(|_, w| w.twai1_rst_en().clear_bit());
            }
            Peripheral::Aes => {
                system.aes_conf.modify(|_, w| w.aes_clk_en().set_bit());
                system.aes_conf.modify(|_, w| w.aes_rst_en().clear_bit());
            }
            Peripheral::Pcnt => {
                system.pcnt_conf.modify(|_, w| w.pcnt_clk_en().set_bit());
                system.pcnt_conf.modify(|_, w| w.pcnt_rst_en().clear_bit());
            }
        }
    }
}

/// Controls the configuration of the chip's clocks.
pub struct SystemClockControl {
    _private: (),
}

/// Controls the configuration of the chip's clocks.
pub struct CpuControl {
    _private: (),
}

/// Dummy DMA peripheral.
#[cfg(pdma)]
pub struct Dma {
    _private: (),
}

/// The SYSTEM/DPORT splitted into it's different logical parts.
pub struct SystemParts<'d> {
    _private: PeripheralRef<'d, SystemPeripheral>,
    #[cfg(not(any(esp32c6, esp32)))]
    pub software_interrupt_control: SoftwareInterruptControl,
    pub peripheral_clock_control: PeripheralClockControl,
    pub clock_control: SystemClockControl,
    pub cpu_control: CpuControl,
    #[cfg(pdma)]
    pub dma: Dma,
}

/// Extension trait to split a SYSTEM/DPORT peripheral in independent logical
/// parts
pub trait SystemExt<'d> {
    type Parts;

    /// Splits the SYSTEM/DPORT peripheral into it's parts.
    fn split(self) -> Self::Parts;
}
#[cfg(any(esp32c6, esp32))]
impl<'d, T: crate::peripheral::Peripheral<P = SystemPeripheral> + 'd> SystemExt<'d> for T {
    type Parts = SystemParts<'d>;

    fn split(self) -> Self::Parts {
        Self::Parts {
            _private: self.into_ref(),
            peripheral_clock_control: PeripheralClockControl { _private: () },
            clock_control: SystemClockControl { _private: () },
            cpu_control: CpuControl { _private: () },
            #[cfg(pdma)]
            dma: Dma { _private: () },
        }
    }
}
#[cfg(not(any(esp32c6, esp32)))]
impl<'d, T: crate::peripheral::Peripheral<P = SystemPeripheral> + 'd> SystemExt<'d> for T {
    type Parts = SystemParts<'d>;

    fn split(self) -> Self::Parts {
        Self::Parts {
            _private: self.into_ref(),
            software_interrupt_control: SoftwareInterruptControl { _private: () },
            peripheral_clock_control: PeripheralClockControl { _private: () },
            clock_control: SystemClockControl { _private: () },
            cpu_control: CpuControl { _private: () },
            #[cfg(pdma)]
            dma: Dma { _private: () },
        }
    }
}

impl crate::peripheral::Peripheral for SystemClockControl {
    type P = SystemClockControl;

    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        SystemClockControl { _private: () }
    }
}

impl crate::peripheral::sealed::Sealed for SystemClockControl {}

#[cfg(pdma)]
mod dma_peripheral {
    use super::Dma;

    impl crate::peripheral::Peripheral for Dma {
        type P = Dma;
        #[inline]
        unsafe fn clone_unchecked(&mut self) -> Self::P {
            Dma { _private: () }
        }
    }

    impl crate::peripheral::sealed::Sealed for Dma {}
}
