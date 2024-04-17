//! # System Control
//!
//! ## Overview
//! This `system` driver provides an interface to control and configure various
//! system-related features and peripherals on ESP chips. It includes
//! functionality to control peripheral clocks, manage software interrupts,
//! configure chip clocks, and control radio peripherals.
//!
//! ### Software Interrupts
//! The `SoftwareInterruptControl` struct gives access to the available software
//! interrupts.
//!
//! The `SoftwareInterrupt` struct allows raising or resetting software
//! interrupts using the `raise()` and `reset()` methods.
//!
//! ### Peripheral Clock Control
//! The `PeripheralClockControl` struct controls the enablement of peripheral
//! clocks.
//!
//! It provides an `enable()` method to enable and reset specific peripherals.
//! The available peripherals are represented by the `Peripheral` enum
//!
//! ## Example
//! ```no_run
//! let peripherals = Peripherals::take();
//! let system = peripherals.SYSTEM.split();
//! let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
//! ```

use crate::{interrupt::InterruptHandler, peripheral::PeripheralRef, peripherals::SYSTEM};

/// Peripherals which can be enabled via `PeripheralClockControl`
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
    #[cfg(parl_io)]
    ParlIo,
    #[cfg(hmac)]
    Hmac,
    #[cfg(ecc)]
    Ecc,
    #[cfg(soc_etm)]
    Etm,
    #[cfg(trace0)]
    Trace0,
    #[cfg(lcd_cam)]
    LcdCam,
}

/// A software interrupt can be triggered by software.
#[non_exhaustive]
pub struct SoftwareInterrupt<const NUM: u8> {}

impl<const NUM: u8> SoftwareInterrupt<NUM> {
    /// Sets the interrupt handler for this software-interrupt
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        let interrupt = match NUM {
            0 => crate::peripherals::Interrupt::FROM_CPU_INTR0,
            1 => crate::peripherals::Interrupt::FROM_CPU_INTR1,
            2 => crate::peripherals::Interrupt::FROM_CPU_INTR2,
            3 => crate::peripherals::Interrupt::FROM_CPU_INTR3,
            _ => unreachable!(),
        };

        unsafe {
            crate::interrupt::bind_interrupt(interrupt, handler.handler());
            crate::interrupt::enable(interrupt, handler.priority()).unwrap();
        }
    }

    /// Trigger this software-interrupt
    pub fn raise(&mut self) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let system = unsafe { &*SYSTEM::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let system = unsafe { &*crate::peripherals::INTPRI::PTR };

        match NUM {
            0 => {
                system
                    .cpu_intr_from_cpu_0()
                    .write(|w| w.cpu_intr_from_cpu_0().set_bit());
            }
            1 => {
                system
                    .cpu_intr_from_cpu_1()
                    .write(|w| w.cpu_intr_from_cpu_1().set_bit());
            }
            2 => {
                system
                    .cpu_intr_from_cpu_2()
                    .write(|w| w.cpu_intr_from_cpu_2().set_bit());
            }
            3 => {
                system
                    .cpu_intr_from_cpu_3()
                    .write(|w| w.cpu_intr_from_cpu_3().set_bit());
            }
            _ => unreachable!(),
        }
    }

    /// Resets this software-interrupt
    pub fn reset(&mut self) {
        #[cfg(not(any(esp32c6, esp32h2)))]
        let system = unsafe { &*SYSTEM::PTR };
        #[cfg(any(esp32c6, esp32h2))]
        let system = unsafe { &*crate::peripherals::INTPRI::PTR };

        match NUM {
            0 => {
                system
                    .cpu_intr_from_cpu_0()
                    .write(|w| w.cpu_intr_from_cpu_0().clear_bit());
            }
            1 => {
                system
                    .cpu_intr_from_cpu_1()
                    .write(|w| w.cpu_intr_from_cpu_1().clear_bit());
            }
            2 => {
                system
                    .cpu_intr_from_cpu_2()
                    .write(|w| w.cpu_intr_from_cpu_2().clear_bit());
            }
            3 => {
                system
                    .cpu_intr_from_cpu_3()
                    .write(|w| w.cpu_intr_from_cpu_3().clear_bit());
            }
            _ => unreachable!(),
        }
    }

    /// Unsafely create an instance of this peripheral out of thin air.
    ///
    /// # Safety
    ///
    /// You must ensure that you're only using one instance of this type at a
    /// time.
    #[inline]
    pub unsafe fn steal() -> Self {
        Self {}
    }
}

impl<const NUM: u8> crate::peripheral::Peripheral for SoftwareInterrupt<NUM> {
    type P = SoftwareInterrupt<NUM>;

    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self::steal()
    }
}

impl<const NUM: u8> crate::private::Sealed for SoftwareInterrupt<NUM> {}

/// This gives access to the available software interrupts.
///
/// Please note: Software interrupt 0 is not available when using the
/// `embassy-executor-thread` feature
#[non_exhaustive]
pub struct SoftwareInterruptControl {
    #[cfg(not(all(feature = "embassy-executor-thread", multi_core)))]
    pub software_interrupt0: SoftwareInterrupt<0>,
    pub software_interrupt1: SoftwareInterrupt<1>,
    pub software_interrupt2: SoftwareInterrupt<2>,
    pub software_interrupt3: SoftwareInterrupt<3>,
}

impl SoftwareInterruptControl {
    fn new_internal() -> Self {
        // the thread-executor uses SW-INT0 when used on a multi-core system
        // we cannot easily require `software_interrupt0` there since it's created
        // before `main` via proc-macro

        SoftwareInterruptControl {
            #[cfg(not(all(feature = "embassy-executor-thread", multi_core)))]
            software_interrupt0: SoftwareInterrupt {},
            software_interrupt1: SoftwareInterrupt {},
            software_interrupt2: SoftwareInterrupt {},
            software_interrupt3: SoftwareInterrupt {},
        }
    }
}

/// Controls the enablement of peripheral clocks.
pub(crate) struct PeripheralClockControl;

#[cfg(not(any(esp32c6, esp32h2)))]
impl PeripheralClockControl {
    /// Enables and resets the given peripheral
    pub(crate) fn enable(peripheral: Peripheral) {
        let system = unsafe { &*SYSTEM::PTR };

        #[cfg(esp32)]
        let (perip_clk_en0, perip_rst_en0, peri_clk_en, peri_rst_en) = {
            (
                &system.perip_clk_en(),
                &system.perip_rst_en(),
                &system.peri_clk_en(),
                &system.peri_rst_en(),
            )
        };
        #[cfg(not(esp32))]
        let (perip_clk_en0, perip_rst_en0) = { (&system.perip_clk_en0(), &system.perip_rst_en0()) };

        #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
        let (perip_clk_en1, perip_rst_en1) = { (&system.perip_clk_en1(), &system.perip_rst_en1()) };

        critical_section::with(|_cs| match peripheral {
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
            #[cfg(all(i2c0, esp32))]
            Peripheral::I2cExt0 => {
                perip_clk_en0.modify(|_, w| w.i2c0_ext0_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.i2c0_ext0_rst().clear_bit());
            }
            #[cfg(all(i2c0, not(esp32)))]
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
            #[cfg(ledc)]
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
            #[cfg(sha)]
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
            #[cfg(uart0)]
            Peripheral::Uart0 => {
                perip_clk_en0.modify(|_, w| w.uart_clk_en().set_bit());
                perip_rst_en0.modify(|_, w| w.uart_rst().clear_bit());
            }
            #[cfg(uart1)]
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
            #[cfg(all(rsa, esp32))]
            Peripheral::Rsa => {
                peri_clk_en.modify(|r, w| unsafe { w.bits(r.bits() | 1 << 2) });
                peri_rst_en.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 2)) });
            }
            #[cfg(all(rsa, any(esp32c3, esp32s2, esp32s3)))]
            Peripheral::Rsa => {
                perip_clk_en1.modify(|_, w| w.crypto_rsa_clk_en().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_rsa_rst().clear_bit());
                system
                    .rsa_pd_ctrl()
                    .modify(|_, w| w.rsa_mem_pd().clear_bit());
            }
            #[cfg(hmac)]
            Peripheral::Hmac => {
                perip_clk_en1.modify(|_, w| w.crypto_hmac_clk_en().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_hmac_rst().clear_bit());
            }
            #[cfg(ecc)]
            Peripheral::Ecc => {
                perip_clk_en1.modify(|_, w| w.crypto_ecc_clk_en().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_ecc_rst().clear_bit());
            }
            #[cfg(lcd_cam)]
            Peripheral::LcdCam => {
                perip_clk_en1.modify(|_, w| w.lcd_cam_clk_en().set_bit());
                perip_rst_en1.modify(|_, w| w.lcd_cam_rst().clear_bit());
            }
        });
    }

    /// Resets the given peripheral
    pub(crate) fn reset(peripheral: Peripheral) {
        let system = unsafe { &*SYSTEM::PTR };

        #[cfg(esp32)]
        let (perip_rst_en0, peri_rst_en) = { (&system.perip_rst_en(), &system.peri_rst_en()) };
        #[cfg(not(esp32))]
        let perip_rst_en0 = { &system.perip_rst_en0() };

        #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
        let perip_rst_en1 = { &system.perip_rst_en1() };

        critical_section::with(|_cs| match peripheral {
            #[cfg(spi2)]
            Peripheral::Spi2 => {
                perip_rst_en0.modify(|_, w| w.spi2_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.spi2_rst().clear_bit());
            }
            #[cfg(spi3)]
            Peripheral::Spi3 => {
                perip_rst_en0.modify(|_, w| w.spi3_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.spi3_rst().clear_bit());
            }
            #[cfg(all(i2c0, esp32))]
            Peripheral::I2cExt0 => {
                perip_rst_en0.modify(|_, w| w.i2c0_ext0_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.i2c0_ext0_rst().clear_bit());
            }
            #[cfg(all(i2c0, not(esp32)))]
            Peripheral::I2cExt0 => {
                perip_rst_en0.modify(|_, w| w.i2c_ext0_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.i2c_ext0_rst().clear_bit());
            }
            #[cfg(i2c1)]
            Peripheral::I2cExt1 => {
                perip_rst_en0.modify(|_, w| w.i2c_ext1_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.i2c_ext1_rst().clear_bit());
            }
            #[cfg(rmt)]
            Peripheral::Rmt => {
                perip_rst_en0.modify(|_, w| w.rmt_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.rmt_rst().clear_bit());
            }
            #[cfg(ledc)]
            Peripheral::Ledc => {
                perip_rst_en0.modify(|_, w| w.ledc_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.ledc_rst().clear_bit());
            }
            #[cfg(mcpwm0)]
            Peripheral::Mcpwm0 => {
                perip_rst_en0.modify(|_, w| w.pwm0_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.pwm0_rst().clear_bit());
            }
            #[cfg(mcpwm1)]
            Peripheral::Mcpwm1 => {
                perip_rst_en0.modify(|_, w| w.pwm1_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.pwm1_rst().clear_bit());
            }
            #[cfg(pcnt)]
            Peripheral::Pcnt => {
                perip_rst_en0.modify(|_, w| w.pcnt_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.pcnt_rst().clear_bit());
            }
            #[cfg(apb_saradc)]
            Peripheral::ApbSarAdc => {
                perip_rst_en0.modify(|_, w| w.apb_saradc_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.apb_saradc_rst().clear_bit());
            }
            #[cfg(gdma)]
            Peripheral::Gdma => {
                perip_rst_en1.modify(|_, w| w.dma_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.dma_rst().clear_bit());
            }
            #[cfg(esp32)]
            Peripheral::Dma => {
                perip_rst_en0.modify(|_, w| w.spi_dma_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.spi_dma_rst().clear_bit());
            }
            #[cfg(esp32s2)]
            Peripheral::Dma => {
                perip_rst_en0.modify(|_, w| w.spi2_dma_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.spi2_dma_rst().clear_bit());
                perip_rst_en0.modify(|_, w| w.spi3_dma_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.spi3_dma_rst().clear_bit());
            }
            #[cfg(esp32c3)]
            Peripheral::I2s0 => {
                // on ESP32-C3 note that i2s1_clk_en / rst is really I2s0
                perip_rst_en0.modify(|_, w| w.i2s1_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.i2s1_rst().clear_bit());
            }
            #[cfg(any(esp32s3, esp32, esp32s2))]
            Peripheral::I2s0 => {
                perip_rst_en0.modify(|_, w| w.i2s0_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.i2s0_rst().clear_bit());
            }
            #[cfg(any(esp32s3, esp32))]
            Peripheral::I2s1 => {
                perip_rst_en0.modify(|_, w| w.i2s1_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.i2s1_rst().clear_bit());
            }
            #[cfg(usb0)]
            Peripheral::Usb => {
                perip_rst_en0.modify(|_, w| w.usb_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.usb_rst().clear_bit());
            }
            #[cfg(twai0)]
            Peripheral::Twai0 => {
                perip_rst_en0.modify(|_, w| w.twai_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.twai_rst().clear_bit());
            }
            #[cfg(esp32)]
            Peripheral::Aes => {
                peri_rst_en.modify(|r, w| unsafe { w.bits(r.bits() | 1) });
                peri_rst_en.modify(|r, w| unsafe { w.bits(r.bits() & (!1)) });
            }
            #[cfg(any(esp32c3, esp32s2, esp32s3))]
            Peripheral::Aes => {
                perip_rst_en1.modify(|_, w| w.crypto_aes_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_aes_rst().clear_bit());
            }
            #[cfg(timg0)]
            Peripheral::Timg0 => {
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_rst_en0.modify(|_, w| w.timers_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.timergroup_rst().set_bit());
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_rst_en0.modify(|_, w| w.timers_rst().clear_bit());
                perip_rst_en0.modify(|_, w| w.timergroup_rst().clear_bit());
            }
            #[cfg(timg1)]
            Peripheral::Timg1 => {
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_rst_en0.modify(|_, w| w.timers_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.timergroup1_rst().set_bit());
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_rst_en0.modify(|_, w| w.timers_rst().clear_bit());
                perip_rst_en0.modify(|_, w| w.timergroup1_rst().clear_bit());
            }
            #[cfg(sha)]
            Peripheral::Sha => {
                #[cfg(not(esp32))]
                perip_rst_en1.modify(|_, w| w.crypto_sha_rst().set_bit());
                #[cfg(not(esp32))]
                perip_rst_en1.modify(|_, w| w.crypto_sha_rst().clear_bit());
            }
            #[cfg(esp32c3)]
            Peripheral::UsbDevice => {
                perip_rst_en0.modify(|_, w| w.usb_device_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.usb_device_rst().clear_bit());
            }
            #[cfg(esp32s3)]
            Peripheral::UsbDevice => {
                perip_rst_en1.modify(|_, w| w.usb_device_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.usb_device_rst().clear_bit());
            }
            #[cfg(uart0)]
            Peripheral::Uart0 => {
                perip_rst_en0.modify(|_, w| w.uart_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.uart_rst().clear_bit());
            }
            #[cfg(uart1)]
            Peripheral::Uart1 => {
                perip_rst_en0.modify(|_, w| w.uart1_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.uart1_rst().clear_bit());
            }
            #[cfg(all(uart2, esp32s3))]
            Peripheral::Uart2 => {
                perip_rst_en1.modify(|_, w| w.uart2_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.uart2_rst().clear_bit());
            }
            #[cfg(all(uart2, esp32))]
            Peripheral::Uart2 => {
                perip_rst_en0.modify(|_, w| w.uart2_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.uart2_rst().clear_bit());
            }
            #[cfg(all(rsa, esp32))]
            Peripheral::Rsa => {
                peri_rst_en.modify(|r, w| unsafe { w.bits(r.bits() | 1 << 2) });
                peri_rst_en.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 2)) });
            }
            #[cfg(all(rsa, any(esp32c3, esp32s2, esp32s3)))]
            Peripheral::Rsa => {
                perip_rst_en1.modify(|_, w| w.crypto_rsa_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_rsa_rst().clear_bit());
            }
            #[cfg(hmac)]
            Peripheral::Hmac => {
                perip_rst_en1.modify(|_, w| w.crypto_hmac_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_hmac_rst().clear_bit());
            }
            #[cfg(ecc)]
            Peripheral::Ecc => {
                perip_rst_en1.modify(|_, w| w.crypto_ecc_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_ecc_rst().clear_bit());
            }
            #[cfg(lcd_cam)]
            Peripheral::LcdCam => {
                perip_rst_en1.modify(|_, w| w.lcd_cam_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.lcd_cam_rst().clear_bit());
            }
        });
    }
}

#[cfg(any(esp32c6, esp32h2))]
impl PeripheralClockControl {
    /// Enables and resets the given peripheral
    pub(crate) fn enable(peripheral: Peripheral) {
        let system = unsafe { &*SYSTEM::PTR };

        match peripheral {
            #[cfg(spi2)]
            Peripheral::Spi2 => {
                system.spi2_conf().modify(|_, w| w.spi2_clk_en().set_bit());
                system
                    .spi2_conf()
                    .modify(|_, w| w.spi2_rst_en().clear_bit());
            }
            #[cfg(i2c0)]
            Peripheral::I2cExt0 => {
                #[cfg(any(esp32c6, esp32h2))]
                {
                    system.i2c0_conf().modify(|_, w| w.i2c0_clk_en().set_bit());
                    system
                        .i2c0_conf()
                        .modify(|_, w| w.i2c0_rst_en().clear_bit());
                }
            }
            #[cfg(i2c1)]
            Peripheral::I2cExt1 => {
                #[cfg(esp32h2)]
                {
                    system.i2c1_conf().modify(|_, w| w.i2c1_clk_en().set_bit());
                    system
                        .i2c1_conf()
                        .modify(|_, w| w.i2c1_rst_en().clear_bit());
                }
            }
            #[cfg(rmt)]
            Peripheral::Rmt => {
                system.rmt_conf().modify(|_, w| w.rmt_clk_en().set_bit());
                system.rmt_conf().modify(|_, w| w.rmt_rst_en().clear_bit());
            }
            #[cfg(ledc)]
            Peripheral::Ledc => {
                system.ledc_conf().modify(|_, w| w.ledc_clk_en().set_bit());
                system
                    .ledc_conf()
                    .modify(|_, w| w.ledc_rst_en().clear_bit());
            }
            #[cfg(mcpwm0)]
            Peripheral::Mcpwm0 => {
                system.pwm_conf().modify(|_, w| w.pwm_clk_en().set_bit());
                system.pwm_conf().modify(|_, w| w.pwm_rst_en().clear_bit());
            }
            #[cfg(mcpwm1)]
            Peripheral::Mcpwm1 => {
                system.pwm_conf.modify(|_, w| w.pwm_clk_en().set_bit());
                system.pwm_conf.modify(|_, w| w.pwm_rst_en().clear_bit());
            }
            #[cfg(apb_saradc)]
            Peripheral::ApbSarAdc => {
                system
                    .saradc_conf()
                    .modify(|_, w| w.saradc_reg_clk_en().set_bit());
                system
                    .saradc_conf()
                    .modify(|_, w| w.saradc_reg_rst_en().clear_bit());
            }
            #[cfg(gdma)]
            Peripheral::Gdma => {
                system.gdma_conf().modify(|_, w| w.gdma_clk_en().set_bit());
                system
                    .gdma_conf()
                    .modify(|_, w| w.gdma_rst_en().clear_bit());
            }
            #[cfg(i2s0)]
            Peripheral::I2s0 => {
                system.i2s_conf().modify(|_, w| w.i2s_clk_en().set_bit());
                system.i2s_conf().modify(|_, w| w.i2s_rst_en().clear_bit());
            }
            #[cfg(twai0)]
            Peripheral::Twai0 => {
                system
                    .twai0_conf()
                    .modify(|_, w| w.twai0_clk_en().set_bit());
                system
                    .twai0_conf()
                    .modify(|_, w| w.twai0_rst_en().clear_bit());

                // use Xtal clk-src
                system.twai0_func_clk_conf().modify(|_, w| {
                    w.twai0_func_clk_en()
                        .set_bit()
                        .twai0_func_clk_sel()
                        .variant(false)
                });
            }
            #[cfg(twai1)]
            Peripheral::Twai1 => {
                system
                    .twai1_conf()
                    .modify(|_, w| w.twai1_clk_en().set_bit());
                system
                    .twai1_conf()
                    .modify(|_, w| w.twai1_rst_en().clear_bit());
            }
            #[cfg(aes)]
            Peripheral::Aes => {
                system.aes_conf().modify(|_, w| w.aes_clk_en().set_bit());
                system.aes_conf().modify(|_, w| w.aes_rst_en().clear_bit());
            }
            #[cfg(pcnt)]
            Peripheral::Pcnt => {
                system.pcnt_conf().modify(|_, w| w.pcnt_clk_en().set_bit());
                system
                    .pcnt_conf()
                    .modify(|_, w| w.pcnt_rst_en().clear_bit());
            }
            #[cfg(timg0)]
            Peripheral::Timg0 => {
                system
                    .timergroup0_timer_clk_conf()
                    .modify(|_, w| w.tg0_timer_clk_en().set_bit());
            }
            #[cfg(timg1)]
            Peripheral::Timg1 => {
                system
                    .timergroup1_timer_clk_conf()
                    .modify(|_, w| w.tg1_timer_clk_en().set_bit());
            }
            #[cfg(lp_wdt)]
            Peripheral::Wdt => {
                system
                    .timergroup0_wdt_clk_conf()
                    .modify(|_, w| w.tg0_wdt_clk_en().set_bit());
                system
                    .timergroup1_timer_clk_conf()
                    .modify(|_, w| w.tg1_timer_clk_en().set_bit());
            }
            #[cfg(sha)]
            Peripheral::Sha => {
                system.sha_conf().modify(|_, w| w.sha_clk_en().set_bit());
                system.sha_conf().modify(|_, w| w.sha_rst_en().clear_bit());
            }
            #[cfg(usb_device)]
            Peripheral::UsbDevice => {
                system
                    .usb_device_conf()
                    .modify(|_, w| w.usb_device_clk_en().set_bit());
                system
                    .usb_device_conf()
                    .modify(|_, w| w.usb_device_rst_en().clear_bit());
            }
            #[cfg(uart0)]
            Peripheral::Uart0 => {
                system
                    .uart0_conf()
                    .modify(|_, w| w.uart0_clk_en().set_bit());
                system
                    .uart0_conf()
                    .modify(|_, w| w.uart0_rst_en().clear_bit());
            }
            #[cfg(uart1)]
            Peripheral::Uart1 => {
                system
                    .uart1_conf()
                    .modify(|_, w| w.uart1_clk_en().set_bit());
                system
                    .uart1_conf()
                    .modify(|_, w| w.uart1_rst_en().clear_bit());
            }
            #[cfg(rsa)]
            Peripheral::Rsa => {
                system.rsa_conf().modify(|_, w| w.rsa_clk_en().set_bit());
                system.rsa_conf().modify(|_, w| w.rsa_rst_en().clear_bit());
                system
                    .rsa_pd_ctrl()
                    .modify(|_, w| w.rsa_mem_pd().clear_bit());
            }
            #[cfg(parl_io)]
            Peripheral::ParlIo => {
                system
                    .parl_io_conf()
                    .modify(|_, w| w.parl_clk_en().set_bit());
                system
                    .parl_io_conf()
                    .modify(|_, w| w.parl_rst_en().set_bit());
                system
                    .parl_io_conf()
                    .modify(|_, w| w.parl_rst_en().clear_bit());
            }
            #[cfg(hmac)]
            Peripheral::Hmac => {
                system.hmac_conf().modify(|_, w| w.hmac_clk_en().set_bit());
                system
                    .hmac_conf()
                    .modify(|_, w| w.hmac_rst_en().clear_bit());
            }
            #[cfg(ecc)]
            Peripheral::Ecc => {
                system.ecc_conf().modify(|_, w| w.ecc_clk_en().set_bit());
                system.ecc_conf().modify(|_, w| w.ecc_rst_en().clear_bit());
            }
            #[cfg(soc_etm)]
            Peripheral::Etm => {
                system.etm_conf().modify(|_, w| w.etm_clk_en().set_bit());
                system.etm_conf().modify(|_, w| w.etm_rst_en().clear_bit());
            }
            #[cfg(trace0)]
            Peripheral::Trace0 => {
                system
                    .trace_conf()
                    .modify(|_, w| w.trace_clk_en().set_bit());
                system
                    .trace_conf()
                    .modify(|_, w| w.trace_rst_en().clear_bit());
            }
        }
    }

    /// Resets the given peripheral
    pub(crate) fn reset(peripheral: Peripheral) {
        let system = unsafe { &*SYSTEM::PTR };

        match peripheral {
            #[cfg(spi2)]
            Peripheral::Spi2 => {
                system.spi2_conf().modify(|_, w| w.spi2_rst_en().set_bit());
                system
                    .spi2_conf()
                    .modify(|_, w| w.spi2_rst_en().clear_bit());
            }
            #[cfg(i2c0)]
            Peripheral::I2cExt0 => {
                #[cfg(any(esp32c6, esp32h2))]
                {
                    system.i2c0_conf().modify(|_, w| w.i2c0_rst_en().set_bit());
                    system
                        .i2c0_conf()
                        .modify(|_, w| w.i2c0_rst_en().clear_bit());
                }
            }
            #[cfg(i2c1)]
            Peripheral::I2cExt1 => {
                #[cfg(esp32h2)]
                {
                    system.i2c1_conf().modify(|_, w| w.i2c1_rst_en().set_bit());
                    system
                        .i2c1_conf()
                        .modify(|_, w| w.i2c1_rst_en().clear_bit());
                }
            }
            #[cfg(rmt)]
            Peripheral::Rmt => {
                system.rmt_conf().modify(|_, w| w.rmt_rst_en().set_bit());
                system.rmt_conf().modify(|_, w| w.rmt_rst_en().clear_bit());
            }
            #[cfg(ledc)]
            Peripheral::Ledc => {
                system.ledc_conf().modify(|_, w| w.ledc_rst_en().set_bit());
                system
                    .ledc_conf()
                    .modify(|_, w| w.ledc_rst_en().clear_bit());
            }
            #[cfg(mcpwm0)]
            Peripheral::Mcpwm0 => {
                system.pwm_conf().modify(|_, w| w.pwm_rst_en().set_bit());
                system.pwm_conf().modify(|_, w| w.pwm_rst_en().clear_bit());
            }
            #[cfg(mcpwm1)]
            Peripheral::Mcpwm1 => {
                system.pwm_conf.modify(|_, w| w.pwm_rst_en().set_bit());
                system.pwm_conf.modify(|_, w| w.pwm_rst_en().clear_bit());
            }
            #[cfg(apb_saradc)]
            Peripheral::ApbSarAdc => {
                system
                    .saradc_conf()
                    .modify(|_, w| w.saradc_reg_rst_en().set_bit());
                system
                    .saradc_conf()
                    .modify(|_, w| w.saradc_reg_rst_en().clear_bit());
            }
            #[cfg(gdma)]
            Peripheral::Gdma => {
                system.gdma_conf().modify(|_, w| w.gdma_rst_en().set_bit());
                system
                    .gdma_conf()
                    .modify(|_, w| w.gdma_rst_en().clear_bit());
            }
            #[cfg(i2s0)]
            Peripheral::I2s0 => {
                system.i2s_conf().modify(|_, w| w.i2s_rst_en().set_bit());
                system.i2s_conf().modify(|_, w| w.i2s_rst_en().clear_bit());
            }
            #[cfg(twai0)]
            Peripheral::Twai0 => {
                system
                    .twai0_conf()
                    .modify(|_, w| w.twai0_rst_en().set_bit());
                system
                    .twai0_conf()
                    .modify(|_, w| w.twai0_rst_en().clear_bit());
            }
            #[cfg(twai1)]
            Peripheral::Twai1 => {
                system
                    .twai1_conf()
                    .modify(|_, w| w.twai1_rst_en().set_bit());
                system
                    .twai1_conf()
                    .modify(|_, w| w.twai1_rst_en().clear_bit());
            }
            #[cfg(aes)]
            Peripheral::Aes => {
                system.aes_conf().modify(|_, w| w.aes_rst_en().set_bit());
                system.aes_conf().modify(|_, w| w.aes_rst_en().clear_bit());
            }
            #[cfg(pcnt)]
            Peripheral::Pcnt => {
                system.pcnt_conf().modify(|_, w| w.pcnt_rst_en().set_bit());
                system
                    .pcnt_conf()
                    .modify(|_, w| w.pcnt_rst_en().clear_bit());
            }
            #[cfg(timg0)]
            Peripheral::Timg0 => {
                // no reset?
            }
            #[cfg(timg1)]
            Peripheral::Timg1 => {
                // no reset?
            }
            #[cfg(lp_wdt)]
            Peripheral::Wdt => {
                // no reset?
            }
            #[cfg(sha)]
            Peripheral::Sha => {
                system.sha_conf().modify(|_, w| w.sha_rst_en().set_bit());
                system.sha_conf().modify(|_, w| w.sha_rst_en().clear_bit());
            }
            #[cfg(usb_device)]
            Peripheral::UsbDevice => {
                system
                    .usb_device_conf()
                    .modify(|_, w| w.usb_device_rst_en().set_bit());
                system
                    .usb_device_conf()
                    .modify(|_, w| w.usb_device_rst_en().clear_bit());
            }
            #[cfg(uart0)]
            Peripheral::Uart0 => {
                system
                    .uart0_conf()
                    .modify(|_, w| w.uart0_rst_en().set_bit());
                system
                    .uart0_conf()
                    .modify(|_, w| w.uart0_rst_en().clear_bit());
            }
            #[cfg(uart1)]
            Peripheral::Uart1 => {
                system
                    .uart1_conf()
                    .modify(|_, w| w.uart1_rst_en().set_bit());
                system
                    .uart1_conf()
                    .modify(|_, w| w.uart1_rst_en().clear_bit());
            }
            #[cfg(rsa)]
            Peripheral::Rsa => {
                system.rsa_conf().modify(|_, w| w.rsa_rst_en().set_bit());
                system.rsa_conf().modify(|_, w| w.rsa_rst_en().clear_bit());
            }
            #[cfg(parl_io)]
            Peripheral::ParlIo => {
                system
                    .parl_io_conf()
                    .modify(|_, w| w.parl_rst_en().set_bit());
                system
                    .parl_io_conf()
                    .modify(|_, w| w.parl_rst_en().clear_bit());
            }
            #[cfg(hmac)]
            Peripheral::Hmac => {
                system.hmac_conf().modify(|_, w| w.hmac_rst_en().set_bit());
                system
                    .hmac_conf()
                    .modify(|_, w| w.hmac_rst_en().clear_bit());
            }
            #[cfg(ecc)]
            Peripheral::Ecc => {
                system.ecc_conf().modify(|_, w| w.ecc_rst_en().set_bit());
                system.ecc_conf().modify(|_, w| w.ecc_rst_en().clear_bit());
            }
            #[cfg(soc_etm)]
            Peripheral::Etm => {
                system.etm_conf().modify(|_, w| w.etm_rst_en().set_bit());
                system.etm_conf().modify(|_, w| w.etm_rst_en().clear_bit());
            }
            #[cfg(trace0)]
            Peripheral::Trace0 => {
                system
                    .trace_conf()
                    .modify(|_, w| w.trace_rst_en().set_bit());
                system
                    .trace_conf()
                    .modify(|_, w| w.trace_rst_en().clear_bit());
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

/// Enumeration of the available radio peripherals for this chip.
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

/// Functionality of clocks controlling the radio peripherals.
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
    _private: PeripheralRef<'d, SYSTEM>,
    pub clock_control: SystemClockControl,
    pub cpu_control: CpuControl,
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

impl<'d, T: crate::peripheral::Peripheral<P = SYSTEM> + 'd> SystemExt<'d> for T {
    type Parts = SystemParts<'d>;

    fn split(self) -> Self::Parts {
        Self::Parts {
            _private: self.into_ref(),
            clock_control: SystemClockControl { _private: () },
            cpu_control: CpuControl { _private: () },
            radio_clock_control: RadioClockControl { _private: () },
            software_interrupt_control: SoftwareInterruptControl::new_internal(),
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

impl crate::private::Sealed for SystemClockControl {}
