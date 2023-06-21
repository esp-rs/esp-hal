//! System Control
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
#[cfg(any(esp32c6, esp32h2))]
type SystemPeripheral = crate::peripherals::PCR;
#[cfg(any(esp32c6, esp32h2))]
type IntPri = crate::peripherals::INTPRI;
#[cfg(not(any(esp32, esp32c6, esp32h2)))]
type SystemPeripheral = crate::peripherals::SYSTEM;

pub enum SoftwareInterrupt {
    SoftwareInterrupt0,
    SoftwareInterrupt1,
    SoftwareInterrupt2,
    SoftwareInterrupt3,
}

/// Peripherals which can be enabled via [PeripheralClockControl]
pub enum Peripheral {
    #[cfg(spi2)]
    Spi2,
    #[cfg(spi3)]
    Spi3,
    #[cfg(i2c0)]
    I2cExt0,
    #[cfg(i2c1)]
    I2cExt1,
    #[cfg(rmt)]
    Rmt,
    #[cfg(ledc)]
    Ledc,
    #[cfg(mcpwm0)]
    Mcpwm0,
    #[cfg(mcpwm1)]
    Mcpwm1,
    #[cfg(pcnt)]
    Pcnt,
    #[cfg(apb_saradc)]
    ApbSarAdc,
    #[cfg(gdma)]
    Gdma,
    #[cfg(pdma)]
    Dma,
    #[cfg(i2s0)]
    I2s0,
    #[cfg(i2s1)]
    I2s1,
    #[cfg(usb0)]
    Usb,
    #[cfg(aes)]
    Aes,
    #[cfg(twai0)]
    Twai0,
    #[cfg(twai1)]
    Twai1,
    #[cfg(timg0)]
    Timg0,
    #[cfg(timg1)]
    Timg1,
    #[cfg(lp_wdt)]
    Wdt,
    #[cfg(sha)]
    Sha,
    #[cfg(usb_device)]
    UsbDevice,
    #[cfg(uart0)]
    Uart0,
    #[cfg(uart1)]
    Uart1,
    #[cfg(uart2)]
    Uart2,
    #[cfg(rsa)]
    Rsa,
}

pub struct SoftwareInterruptControl {
    _private: (),
}

impl SoftwareInterruptControl {
    pub fn raise(&mut self, interrupt: SoftwareInterrupt) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let system = unsafe { &*SystemPeripheral::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let system = unsafe { &*IntPri::PTR };

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
        #[cfg(not(any(esp32c6, esp32h2)))]
        let system = unsafe { &*SystemPeripheral::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let system = unsafe { &*IntPri::PTR };

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

#[cfg(not(any(esp32c6, esp32h2)))]
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
            #[cfg(spi2)]
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
            #[cfg(mcpwm0)]
            Peripheral::Mcpwm0 => {
                perip_clk_en0.modify(|_, w| w.pwm0_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.pwm0_rst().clear_bit());
            }
            #[cfg(mcpwm1)]
            Peripheral::Mcpwm1 => {
                perip_clk_en0.modify(|_, w| w.pwm1_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.pwm1_rst().clear_bit());
            }
            #[cfg(pcnt)]
            Peripheral::Pcnt => {
                perip_clk_en0.modify(|_, w| w.pcnt_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.pcnt_rst().clear_bit());
            }
            #[cfg(apb_saradc)]
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
            #[cfg(usb0)]
            Peripheral::Usb => {
                perip_clk_en0.modify(|_, w| w.usb_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.usb_rst().clear_bit());
            }
            #[cfg(twai0)]
            Peripheral::Twai0 => {
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
            #[cfg(timg0)]
            Peripheral::Timg0 => {
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_clk_en0.modify(|_, w| w.timers_clk_en().set_bit());
                perip_clk_en0.modify(|_, w| w.timergroup_clk_en().set_bit());

                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_rst_en0.modify(|_, w| w.timers_rst().clear_bit());
                perip_rst_en0.modify(|_, w| w.timergroup_rst().clear_bit());
            }
            #[cfg(timg1)]
            Peripheral::Timg1 => {
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_clk_en0.modify(|_, w| w.timers_clk_en().set_bit());
                perip_clk_en0.modify(|_, w| w.timergroup1_clk_en().set_bit());

                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_rst_en0.modify(|_, w| w.timers_rst().clear_bit());
                perip_rst_en0.modify(|_, w| w.timergroup1_rst().clear_bit());
            }
            Peripheral::Sha => {
                #[cfg(not(esp32))]
                perip_clk_en1.modify(|_, w| w.crypto_sha_clk_en().set_bit());
                #[cfg(not(esp32))]
                perip_rst_en1.modify(|_, w| w.crypto_sha_rst().clear_bit());
            }
            #[cfg(esp32c3)]
            Peripheral::UsbDevice => {
                perip_clk_en0.modify(|_, w| w.usb_device_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.usb_device_rst().clear_bit());
            }
            #[cfg(esp32s3)]
            Peripheral::UsbDevice => {
                perip_clk_en1.modify(|_, w| w.usb_device_clk_en().set_bit());
                perip_rst_en1.modify(|_, w| w.usb_device_rst().clear_bit());
            }
            Peripheral::Uart0 => {
                perip_clk_en0.modify(|_, w| w.uart_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.uart_rst().clear_bit());
            }
            Peripheral::Uart1 => {
                perip_clk_en0.modify(|_, w| w.uart1_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.uart1_rst().clear_bit());
            }
            #[cfg(all(uart2, esp32s3))]
            Peripheral::Uart2 => {
                perip_clk_en1.modify(|_, w| w.uart2_clk_en().set_bit());
                perip_rst_en1.modify(|_, w| w.uart2_rst().clear_bit());
            }
            #[cfg(all(uart2, esp32))]
            Peripheral::Uart2 => {
                perip_clk_en0.modify(|_, w| w.uart2_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.uart2_rst().clear_bit());
            }
            #[cfg(esp32)]
            Peripheral::Rsa => {
                peri_clk_en.modify(|r, w| unsafe { w.bits(r.bits() | 1 << 2) });
                peri_rst_en.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 2)) });
            }
            #[cfg(any(esp32c3, esp32s2, esp32s3))]
            Peripheral::Rsa => {
                perip_clk_en1.modify(|_, w| w.crypto_rsa_clk_en().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_rsa_rst().clear_bit());
                system.rsa_pd_ctrl.modify(|_, w| w.rsa_mem_pd().clear_bit());
            }
        }
    }
}

#[cfg(any(esp32c6, esp32h2))]
impl PeripheralClockControl {
    /// Enables and resets the given peripheral
    pub fn enable(&mut self, peripheral: Peripheral) {
        let system = unsafe { &*SystemPeripheral::PTR };

        match peripheral {
            #[cfg(spi2)]
            Peripheral::Spi2 => {
                system.spi2_conf.modify(|_, w| w.spi2_clk_en().set_bit());
                system.spi2_conf.modify(|_, w| w.spi2_rst_en().clear_bit());
            }
            #[cfg(i2c0)]
            Peripheral::I2cExt0 => {
                #[cfg(any(esp32c6, esp32h2))]
                {
                    system.i2c0_conf.modify(|_, w| w.i2c0_clk_en().set_bit());
                    system.i2c0_conf.modify(|_, w| w.i2c0_rst_en().clear_bit());
                }
            }
            #[cfg(i2c1)]
            Peripheral::I2cExt1 => {
                #[cfg(esp32h2)]
                {
                    system.i2c1_conf.modify(|_, w| w.i2c1_clk_en().set_bit());
                    system.i2c1_conf.modify(|_, w| w.i2c1_rst_en().clear_bit());
                }
            }
            #[cfg(rmt)]
            Peripheral::Rmt => {
                system.rmt_conf.modify(|_, w| w.rmt_clk_en().set_bit());
                system.rmt_conf.modify(|_, w| w.rmt_rst_en().clear_bit());
            }
            #[cfg(ledc)]
            Peripheral::Ledc => {
                system.ledc_conf.modify(|_, w| w.ledc_clk_en().set_bit());
                system.ledc_conf.modify(|_, w| w.ledc_rst_en().clear_bit());
            }
            #[cfg(mcpwm0)]
            Peripheral::Mcpwm0 => {
                system.pwm_conf.modify(|_, w| w.pwm_clk_en().set_bit());
                system.pwm_conf.modify(|_, w| w.pwm_rst_en().clear_bit());
            }
            #[cfg(mcpwm1)]
            Peripheral::Mcpwm1 => {
                system.pwm_conf.modify(|_, w| w.pwm_clk_en().set_bit());
                system.pwm_conf.modify(|_, w| w.pwm_rst_en().clear_bit());
            }
            #[cfg(apb_saradc)]
            Peripheral::ApbSarAdc => {
                system
                    .saradc_conf
                    .modify(|_, w| w.saradc_reg_clk_en().set_bit());
                system
                    .saradc_conf
                    .modify(|_, w| w.saradc_reg_rst_en().clear_bit());
            }
            #[cfg(gdma)]
            Peripheral::Gdma => {
                system.gdma_conf.modify(|_, w| w.gdma_clk_en().set_bit());
                system.gdma_conf.modify(|_, w| w.gdma_rst_en().clear_bit());
            }
            #[cfg(i2s0)]
            Peripheral::I2s0 => {
                system.i2s_conf.modify(|_, w| w.i2s_clk_en().set_bit());
                system.i2s_conf.modify(|_, w| w.i2s_rst_en().clear_bit());
            }
            #[cfg(twai0)]
            Peripheral::Twai0 => {
                system.twai0_conf.modify(|_, w| w.twai0_clk_en().set_bit());
                system
                    .twai0_conf
                    .modify(|_, w| w.twai0_rst_en().clear_bit());
            }
            #[cfg(twai1)]
            Peripheral::Twai1 => {
                system.twai1_conf.modify(|_, w| w.twai1_clk_en().set_bit());
                system
                    .twai1_conf
                    .modify(|_, w| w.twai1_rst_en().clear_bit());
            }
            #[cfg(aes)]
            Peripheral::Aes => {
                system.aes_conf.modify(|_, w| w.aes_clk_en().set_bit());
                system.aes_conf.modify(|_, w| w.aes_rst_en().clear_bit());
            }
            #[cfg(pcnt)]
            Peripheral::Pcnt => {
                system.pcnt_conf.modify(|_, w| w.pcnt_clk_en().set_bit());
                system.pcnt_conf.modify(|_, w| w.pcnt_rst_en().clear_bit());
            }
            #[cfg(timg0)]
            Peripheral::Timg0 => {
                system
                    .timergroup0_timer_clk_conf
                    .modify(|_, w| w.tg0_timer_clk_en().set_bit());
            }
            #[cfg(timg1)]
            Peripheral::Timg1 => {
                system
                    .timergroup1_timer_clk_conf
                    .modify(|_, w| w.tg1_timer_clk_en().set_bit());
            }
            #[cfg(lp_wdt)]
            Peripheral::Wdt => {
                system
                    .timergroup0_wdt_clk_conf
                    .modify(|_, w| w.tg0_wdt_clk_en().set_bit());
                system
                    .timergroup1_timer_clk_conf
                    .modify(|_, w| w.tg1_timer_clk_en().set_bit());
            }
            #[cfg(sha)]
            Peripheral::Sha => {
                system.sha_conf.modify(|_, w| w.sha_clk_en().set_bit());
                system.sha_conf.modify(|_, w| w.sha_rst_en().clear_bit());
            }
            #[cfg(usb_device)]
            Peripheral::UsbDevice => {
                system
                    .usb_device_conf
                    .modify(|_, w| w.usb_device_clk_en().set_bit());
                system
                    .usb_device_conf
                    .modify(|_, w| w.usb_device_rst_en().clear_bit());
            }
            #[cfg(uart0)]
            Peripheral::Uart0 => {
                system.uart0_conf.modify(|_, w| w.uart0_clk_en().set_bit());
                system
                    .uart0_conf
                    .modify(|_, w| w.uart0_rst_en().clear_bit());
            }
            #[cfg(uart1)]
            Peripheral::Uart1 => {
                system.uart1_conf.modify(|_, w| w.uart1_clk_en().set_bit());
                system
                    .uart1_conf
                    .modify(|_, w| w.uart1_rst_en().clear_bit());
            }
            #[cfg(rsa)]
            Peripheral::Rsa => {
                system.rsa_conf.modify(|_, w| w.rsa_clk_en().set_bit());
                system.rsa_conf.modify(|_, w| w.rsa_rst_en().clear_bit());
                system.rsa_pd_ctrl.modify(|_, w| w.rsa_mem_pd().clear_bit());
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

pub enum RadioPeripherals {
    #[cfg(phy)]
    Phy,
    #[cfg(bt)]
    Bt,
    #[cfg(wifi)]
    Wifi,
    #[cfg(ieee802154)]
    Ieee802154,
}

pub struct RadioClockControl {
    _private: (),
}

/// Control the radio peripheral clocks
pub trait RadioClockController {
    /// Enable the peripheral
    fn enable(&mut self, peripheral: RadioPeripherals);

    /// Disable the peripheral
    fn disable(&mut self, peripheral: RadioPeripherals);

    /// Reset the MAC
    fn reset_mac(&mut self);

    /// Do any common initial initialization needed
    fn init_clocks(&mut self);

    /// Initialize BLE RTC clocks
    fn ble_rtc_clk_init(&mut self);

    fn reset_rpa(&mut self);
}

/// The SYSTEM/DPORT splitted into it's different logical parts.
pub struct SystemParts<'d> {
    _private: PeripheralRef<'d, SystemPeripheral>,
    pub peripheral_clock_control: PeripheralClockControl,
    pub clock_control: SystemClockControl,
    pub cpu_control: CpuControl,
    #[cfg(pdma)]
    pub dma: Dma,
    pub radio_clock_control: RadioClockControl,
    pub software_interrupt_control: SoftwareInterruptControl,
}

/// Extension trait to split a SYSTEM/DPORT peripheral in independent logical
/// parts
pub trait SystemExt<'d> {
    type Parts;

    /// Splits the SYSTEM/DPORT peripheral into it's parts.
    fn split(self) -> Self::Parts;
}

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
            radio_clock_control: RadioClockControl { _private: () },
            software_interrupt_control: SoftwareInterruptControl { _private: () },
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
