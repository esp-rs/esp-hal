//! # System Control
//!
//! ## Overview
//!
//! This `system` module defines the available radio peripherals and provides an
//! interface to control and configure radio clocks.

use core::sync::atomic::Ordering;

use critical_section::CriticalSection;
use portable_atomic::AtomicUsize;
use strum::{EnumCount, EnumIter, IntoEnumIterator};

use crate::peripherals::SYSTEM;

pub(crate) const KEEP_ENABLED: &[Peripheral] = &[
    Peripheral::Uart0,
    #[cfg(usb_device)]
    Peripheral::UsbDevice,
    #[cfg(systimer)]
    Peripheral::Systimer,
    Peripheral::Timg0,
];

/// Peripherals which can be enabled via `PeripheralClockControl`.
///
/// This enum represents various hardware peripherals that can be enabled
/// by the system's clock control. Depending on the target device, different
/// peripherals will be available for enabling.
// FIXME: This enum needs to be public because it's exposed via a bunch of traits, but it's not
// useful to users.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, EnumCount, EnumIter)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Peripheral {
    /// SPI2 peripheral.
    #[cfg(spi2)]
    Spi2,
    /// SPI3 peripheral.
    #[cfg(spi3)]
    Spi3,
    /// External I2C0 peripheral.
    #[cfg(i2c0)]
    I2cExt0,
    /// External I2C1 peripheral.
    #[cfg(i2c1)]
    I2cExt1,
    /// RMT peripheral (Remote Control).
    #[cfg(rmt)]
    Rmt,
    /// LEDC peripheral (LED PWM Controller).
    #[cfg(ledc)]
    Ledc,
    /// MCPWM0 peripheral (Motor Control PWM 0).
    #[cfg(mcpwm0)]
    Mcpwm0,
    /// MCPWM1 peripheral (Motor Control PWM 1).
    #[cfg(mcpwm1)]
    Mcpwm1,
    /// PCNT peripheral (Pulse Counter).
    #[cfg(pcnt)]
    Pcnt,
    /// APB SAR ADC peripheral.
    #[cfg(apb_saradc)]
    ApbSarAdc,
    /// General DMA (GDMA) peripheral.
    #[cfg(gdma)]
    Gdma,
    /// Peripheral DMA (PDMA) peripheral.
    #[cfg(pdma)]
    Dma,
    /// I2S0 peripheral (Inter-IC Sound).
    #[cfg(i2s0)]
    I2s0,
    /// I2S1 peripheral (Inter-IC Sound).
    #[cfg(i2s1)]
    I2s1,
    /// USB0 peripheral.
    #[cfg(usb0)]
    Usb,
    /// AES peripheral (Advanced Encryption Standard).
    #[cfg(aes)]
    Aes,
    /// TWAI0 peripheral.
    #[cfg(twai0)]
    Twai0,
    /// TWAI1 peripheral.
    #[cfg(twai1)]
    Twai1,
    /// Timer Group 0 peripheral.
    #[cfg(timg0)]
    Timg0,
    /// Timer Group 1 peripheral.
    #[cfg(timg1)]
    Timg1,
    /// SHA peripheral (Secure Hash Algorithm).
    #[cfg(sha)]
    Sha,
    /// USB Device peripheral.
    #[cfg(usb_device)]
    UsbDevice,
    /// UART0 peripheral.
    #[cfg(uart0)]
    Uart0,
    /// UART1 peripheral.
    #[cfg(uart1)]
    Uart1,
    /// UART2 peripheral.
    #[cfg(uart2)]
    Uart2,
    /// RSA peripheral (Rivest-Shamir-Adleman encryption).
    #[cfg(rsa)]
    Rsa,
    /// Parallel IO peripheral.
    #[cfg(parl_io)]
    ParlIo,
    /// HMAC peripheral (Hash-based Message Authentication Code).
    #[cfg(hmac)]
    Hmac,
    /// ECC peripheral (Elliptic Curve Cryptography).
    #[cfg(ecc)]
    Ecc,
    /// SOC ETM peripheral (Event Task Manager).
    #[cfg(soc_etm)]
    Etm,
    /// TRACE0 peripheral (Debug trace).
    #[cfg(trace0)]
    Trace0,
    /// LCD Camera peripheral.
    #[cfg(lcd_cam)]
    LcdCam,
    /// Systimer peripheral.
    #[cfg(systimer)]
    Systimer,
}

impl Peripheral {
    pub fn try_from(value: u8) -> Option<Peripheral> {
        if value >= Peripheral::COUNT as u8 {
            return None;
        }

        Some(unsafe { core::mem::transmute::<u8, Peripheral>(value) })
    }
}

static PERIPHERAL_REF_COUNT: [AtomicUsize; Peripheral::COUNT] =
    [const { AtomicUsize::new(0) }; Peripheral::COUNT];

/// Disable all peripherals.
///
/// Peripherals listed in [KEEP_ENABLED] are NOT disabled.
pub(crate) fn disable_peripherals() {
    for p in Peripheral::iter() {
        if KEEP_ENABLED.contains(&p) {
            continue;
        }
        PeripheralClockControl::enable_forced(p, false, true);
    }
}

#[derive(Debug)]
pub(crate) struct PeripheralGuard {
    peripheral: Peripheral,
}

impl PeripheralGuard {
    pub(crate) fn new(p: Peripheral) -> Self {
        if !KEEP_ENABLED.contains(&p) && PeripheralClockControl::enable(p) {
            PeripheralClockControl::reset(p);
        }

        Self { peripheral: p }
    }
}

impl Drop for PeripheralGuard {
    fn drop(&mut self) {
        if !KEEP_ENABLED.contains(&self.peripheral) {
            PeripheralClockControl::disable(self.peripheral);
        }
    }
}

#[derive(Debug)]
pub(crate) struct GenericPeripheralGuard<const P: u8> {}

impl<const P: u8> GenericPeripheralGuard<P> {
    pub(crate) fn new() -> Self {
        let peripheral = unwrap!(Peripheral::try_from(P));
        if !KEEP_ENABLED.contains(&peripheral) && PeripheralClockControl::enable(peripheral) {
            PeripheralClockControl::reset(peripheral);
        }

        Self {}
    }
}

impl<const P: u8> Drop for GenericPeripheralGuard<P> {
    fn drop(&mut self) {
        let peripheral = unwrap!(Peripheral::try_from(P));
        if !KEEP_ENABLED.contains(&peripheral) {
            PeripheralClockControl::disable(peripheral);
        }
    }
}

/// Controls the enablement of peripheral clocks.
pub(crate) struct PeripheralClockControl;

#[cfg(not(any(esp32c6, esp32h2)))]
impl PeripheralClockControl {
    fn enable_internal(peripheral: Peripheral, enable: bool, _cs: &CriticalSection<'_>) {
        debug!("Enable {:?} {}", peripheral, enable);

        let system = unsafe { &*SYSTEM::PTR };

        #[cfg(esp32)]
        let (perip_clk_en0, peri_clk_en) = { (&system.perip_clk_en(), &system.peri_clk_en()) };
        #[cfg(not(esp32))]
        let perip_clk_en0 = &system.perip_clk_en0();

        #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
        let perip_clk_en1 = &system.perip_clk_en1();

        match peripheral {
            #[cfg(spi2)]
            Peripheral::Spi2 => {
                perip_clk_en0.modify(|_, w| w.spi2_clk_en().bit(enable));
            }
            #[cfg(spi3)]
            Peripheral::Spi3 => {
                perip_clk_en0.modify(|_, w| w.spi3_clk_en().bit(enable));
            }
            #[cfg(all(i2c0, esp32))]
            Peripheral::I2cExt0 => {
                perip_clk_en0.modify(|_, w| w.i2c0_ext0_clk_en().bit(enable));
            }
            #[cfg(all(i2c0, not(esp32)))]
            Peripheral::I2cExt0 => {
                perip_clk_en0.modify(|_, w| w.i2c_ext0_clk_en().bit(enable));
            }
            #[cfg(i2c1)]
            Peripheral::I2cExt1 => {
                perip_clk_en0.modify(|_, w| w.i2c_ext1_clk_en().bit(enable));
            }
            #[cfg(rmt)]
            Peripheral::Rmt => {
                perip_clk_en0.modify(|_, w| w.rmt_clk_en().bit(enable));
            }
            #[cfg(ledc)]
            Peripheral::Ledc => {
                perip_clk_en0.modify(|_, w| w.ledc_clk_en().bit(enable));
            }
            #[cfg(mcpwm0)]
            Peripheral::Mcpwm0 => {
                perip_clk_en0.modify(|_, w| w.pwm0_clk_en().bit(enable));
            }
            #[cfg(mcpwm1)]
            Peripheral::Mcpwm1 => {
                perip_clk_en0.modify(|_, w| w.pwm1_clk_en().bit(enable));
            }
            #[cfg(pcnt)]
            Peripheral::Pcnt => {
                perip_clk_en0.modify(|_, w| w.pcnt_clk_en().bit(enable));
            }
            #[cfg(apb_saradc)]
            Peripheral::ApbSarAdc => {
                perip_clk_en0.modify(|_, w| w.apb_saradc_clk_en().bit(enable));
            }
            #[cfg(gdma)]
            Peripheral::Gdma => {
                perip_clk_en1.modify(|_, w| w.dma_clk_en().bit(enable));
            }
            #[cfg(esp32)]
            Peripheral::Dma => {
                perip_clk_en0.modify(|_, w| w.spi_dma_clk_en().bit(enable));
            }
            #[cfg(esp32s2)]
            Peripheral::Dma => {
                perip_clk_en0.modify(|_, w| w.spi2_dma_clk_en().bit(enable));
                perip_clk_en0.modify(|_, w| w.spi3_dma_clk_en().bit(enable));
            }
            #[cfg(esp32c3)]
            Peripheral::I2s0 => {
                // on ESP32-C3 note that i2s1_clk_en / rst is really I2s0
                perip_clk_en0.modify(|_, w| w.i2s1_clk_en().bit(enable));
            }
            #[cfg(any(esp32s3, esp32, esp32s2))]
            Peripheral::I2s0 => {
                perip_clk_en0.modify(|_, w| w.i2s0_clk_en().bit(enable));
            }
            #[cfg(any(esp32s3, esp32))]
            Peripheral::I2s1 => {
                perip_clk_en0.modify(|_, w| w.i2s1_clk_en().bit(enable));
            }
            #[cfg(usb0)]
            Peripheral::Usb => {
                perip_clk_en0.modify(|_, w| w.usb_clk_en().bit(enable));
            }
            #[cfg(twai0)]
            Peripheral::Twai0 => {
                perip_clk_en0.modify(|_, w| w.twai_clk_en().bit(enable));
            }
            #[cfg(esp32)]
            Peripheral::Aes => {
                peri_clk_en.modify(|r, w| unsafe { w.bits(r.bits() | enable as u32) });
            }
            #[cfg(any(esp32c3, esp32s2, esp32s3))]
            Peripheral::Aes => {
                perip_clk_en1.modify(|_, w| w.crypto_aes_clk_en().bit(enable));
            }
            #[cfg(timg0)]
            Peripheral::Timg0 => {
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_clk_en0.modify(|_, w| w.timers_clk_en().bit(enable));
                perip_clk_en0.modify(|_, w| w.timergroup_clk_en().bit(enable));
            }
            #[cfg(timg1)]
            Peripheral::Timg1 => {
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_clk_en0.modify(|_, w| w.timers_clk_en().bit(enable));
                perip_clk_en0.modify(|_, w| w.timergroup1_clk_en().bit(enable));
            }
            #[cfg(sha)]
            Peripheral::Sha => {
                #[cfg(not(esp32))]
                perip_clk_en1.modify(|_, w| w.crypto_sha_clk_en().bit(enable));
            }
            #[cfg(esp32c3)]
            Peripheral::UsbDevice => {
                perip_clk_en0.modify(|_, w| w.usb_device_clk_en().bit(enable));
            }
            #[cfg(esp32s3)]
            Peripheral::UsbDevice => {
                perip_clk_en1.modify(|_, w| w.usb_device_clk_en().bit(enable));
            }
            #[cfg(uart0)]
            Peripheral::Uart0 => {
                perip_clk_en0.modify(|_, w| w.uart_clk_en().bit(enable));
            }
            #[cfg(uart1)]
            Peripheral::Uart1 => {
                perip_clk_en0.modify(|_, w| w.uart1_clk_en().bit(enable));
            }
            #[cfg(all(uart2, esp32s3))]
            Peripheral::Uart2 => {
                perip_clk_en1.modify(|_, w| w.uart2_clk_en().set_bit());
            }
            #[cfg(all(uart2, esp32))]
            Peripheral::Uart2 => {
                perip_clk_en0.modify(|_, w| w.uart2_clk_en().bit(enable));
            }
            #[cfg(all(rsa, esp32))]
            Peripheral::Rsa => {
                peri_clk_en.modify(|r, w| unsafe { w.bits(r.bits() | (enable as u32) << 2) });
            }
            #[cfg(all(rsa, any(esp32c3, esp32s2, esp32s3)))]
            Peripheral::Rsa => {
                perip_clk_en1.modify(|_, w| w.crypto_rsa_clk_en().bit(enable));
                system
                    .rsa_pd_ctrl()
                    .modify(|_, w| w.rsa_mem_pd().bit(!enable));
            }
            #[cfg(hmac)]
            Peripheral::Hmac => {
                perip_clk_en1.modify(|_, w| w.crypto_hmac_clk_en().bit(enable));
            }
            #[cfg(ecc)]
            Peripheral::Ecc => {
                perip_clk_en1.modify(|_, w| w.crypto_ecc_clk_en().bit(enable));
            }
            #[cfg(lcd_cam)]
            Peripheral::LcdCam => {
                perip_clk_en1.modify(|_, w| w.lcd_cam_clk_en().bit(enable));
            }
            #[cfg(systimer)]
            Peripheral::Systimer => {
                perip_clk_en0.modify(|_, w| w.systimer_clk_en().bit(enable));
            }
        }
    }

    /// Resets the given peripheral
    pub(crate) fn reset(peripheral: Peripheral) {
        debug!("Reset {:?}", peripheral);
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
            #[cfg(systimer)]
            Peripheral::Systimer => {
                perip_rst_en0.modify(|_, w| w.systimer_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.systimer_rst().clear_bit());
            }
        });
    }
}

#[cfg(any(esp32c6, esp32h2))]
impl PeripheralClockControl {
    fn enable_internal(peripheral: Peripheral, enable: bool, _cs: &CriticalSection<'_>) {
        debug!("Enable {:?} {}", peripheral, enable);
        let system = unsafe { &*SYSTEM::PTR };

        match peripheral {
            #[cfg(spi2)]
            Peripheral::Spi2 => {
                system
                    .spi2_conf()
                    .modify(|_, w| w.spi2_clk_en().bit(enable));
            }
            #[cfg(i2c0)]
            Peripheral::I2cExt0 => {
                system
                    .i2c0_conf()
                    .modify(|_, w| w.i2c0_clk_en().bit(enable));
            }
            #[cfg(i2c1)]
            Peripheral::I2cExt1 => {
                system
                    .i2c1_conf()
                    .modify(|_, w| w.i2c1_clk_en().bit(enable));
            }
            #[cfg(rmt)]
            Peripheral::Rmt => {
                system.rmt_conf().modify(|_, w| w.rmt_clk_en().bit(enable));
            }
            #[cfg(ledc)]
            Peripheral::Ledc => {
                system
                    .ledc_conf()
                    .modify(|_, w| w.ledc_clk_en().bit(enable));
            }
            #[cfg(mcpwm0)]
            Peripheral::Mcpwm0 => {
                system.pwm_conf().modify(|_, w| w.pwm_clk_en().bit(enable));
            }
            #[cfg(mcpwm1)]
            Peripheral::Mcpwm1 => {
                system.pwm_conf.modify(|_, w| w.pwm_clk_en().bit(enable));
            }
            #[cfg(apb_saradc)]
            Peripheral::ApbSarAdc => {
                system
                    .saradc_conf()
                    .modify(|_, w| w.saradc_reg_clk_en().bit(enable));
            }
            #[cfg(gdma)]
            Peripheral::Gdma => {
                system
                    .gdma_conf()
                    .modify(|_, w| w.gdma_clk_en().bit(enable));
            }
            #[cfg(i2s0)]
            Peripheral::I2s0 => {
                system.i2s_conf().modify(|_, w| w.i2s_clk_en().bit(enable));
            }
            #[cfg(twai0)]
            Peripheral::Twai0 => {
                system
                    .twai0_conf()
                    .modify(|_, w| w.twai0_clk_en().bit(enable));

                if enable {
                    // use Xtal clk-src
                    system.twai0_func_clk_conf().modify(|_, w| {
                        w.twai0_func_clk_en()
                            .set_bit()
                            .twai0_func_clk_sel()
                            .variant(false)
                    });
                }
            }
            #[cfg(twai1)]
            Peripheral::Twai1 => {
                system
                    .twai1_conf()
                    .modify(|_, w| w.twai1_clk_en().bit(enable));
            }
            #[cfg(aes)]
            Peripheral::Aes => {
                system.aes_conf().modify(|_, w| w.aes_clk_en().bit(enable));
            }
            #[cfg(pcnt)]
            Peripheral::Pcnt => {
                system
                    .pcnt_conf()
                    .modify(|_, w| w.pcnt_clk_en().bit(enable));
            }
            #[cfg(timg0)]
            Peripheral::Timg0 => {
                system
                    .timergroup0_timer_clk_conf()
                    .modify(|_, w| w.tg0_timer_clk_en().bit(enable));
            }
            #[cfg(timg1)]
            Peripheral::Timg1 => {
                system
                    .timergroup1_timer_clk_conf()
                    .modify(|_, w| w.tg1_timer_clk_en().bit(enable));
            }
            #[cfg(sha)]
            Peripheral::Sha => {
                system.sha_conf().modify(|_, w| w.sha_clk_en().bit(enable));
            }
            #[cfg(usb_device)]
            Peripheral::UsbDevice => {
                system
                    .usb_device_conf()
                    .modify(|_, w| w.usb_device_clk_en().bit(enable));
            }
            #[cfg(uart0)]
            Peripheral::Uart0 => {
                system
                    .uart0_conf()
                    .modify(|_, w| w.uart0_clk_en().bit(enable));
            }
            #[cfg(uart1)]
            Peripheral::Uart1 => {
                system
                    .uart1_conf()
                    .modify(|_, w| w.uart1_clk_en().bit(enable));
            }
            #[cfg(rsa)]
            Peripheral::Rsa => {
                system.rsa_conf().modify(|_, w| w.rsa_clk_en().bit(enable));
                system
                    .rsa_pd_ctrl()
                    .modify(|_, w| w.rsa_mem_pd().clear_bit());
            }
            #[cfg(parl_io)]
            Peripheral::ParlIo => {
                system
                    .parl_io_conf()
                    .modify(|_, w| w.parl_clk_en().bit(enable));
            }
            #[cfg(hmac)]
            Peripheral::Hmac => {
                system
                    .hmac_conf()
                    .modify(|_, w| w.hmac_clk_en().bit(enable));
            }
            #[cfg(ecc)]
            Peripheral::Ecc => {
                system.ecc_conf().modify(|_, w| w.ecc_clk_en().bit(enable));
            }
            #[cfg(soc_etm)]
            Peripheral::Etm => {
                system.etm_conf().modify(|_, w| w.etm_clk_en().bit(enable));
            }
            #[cfg(trace0)]
            Peripheral::Trace0 => {
                system
                    .trace_conf()
                    .modify(|_, w| w.trace_clk_en().bit(enable));
            }
            #[cfg(systimer)]
            Peripheral::Systimer => {
                system
                    .systimer_conf()
                    .modify(|_, w| w.systimer_clk_en().bit(enable));
            }
        }
    }

    /// Resets the given peripheral
    pub(crate) fn reset(peripheral: Peripheral) {
        debug!("Reset {:?}", peripheral);

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
            #[cfg(systimer)]
            Peripheral::Systimer => {
                system
                    .systimer_conf()
                    .modify(|_, w| w.systimer_rst_en().set_bit());
                system
                    .systimer_conf()
                    .modify(|_, w| w.systimer_rst_en().clear_bit());
            }
        }
    }
}

impl PeripheralClockControl {
    /// Enables the given peripheral.
    ///
    /// This keeps track of enabling a peripheral - i.e. a peripheral
    /// is only enabled with the first call attempt to enable it.
    ///
    /// Returns `true` if it actually enabled the peripheral.
    pub(crate) fn enable(peripheral: Peripheral) -> bool {
        Self::enable_forced(peripheral, true, false)
    }

    /// Disables the given peripheral.
    ///
    /// This keeps track of disabling a peripheral - i.e. it only
    /// gets disabled when the number of enable/disable attempts is balanced.
    ///
    /// Returns `true` if it actually disabled the peripheral.
    ///
    /// Before disabling a peripheral it will also get reset
    pub(crate) fn disable(peripheral: Peripheral) -> bool {
        Self::enable_forced(peripheral, false, false)
    }

    pub(crate) fn enable_forced(peripheral: Peripheral, enable: bool, force: bool) -> bool {
        critical_section::with(|cs| {
            if !force {
                if enable {
                    let prev =
                        PERIPHERAL_REF_COUNT[peripheral as usize].fetch_add(1, Ordering::Relaxed);
                    if prev > 0 {
                        return false;
                    }
                } else {
                    let prev =
                        PERIPHERAL_REF_COUNT[peripheral as usize].fetch_sub(1, Ordering::Relaxed);
                    assert!(prev != 0);
                    if prev > 1 {
                        return false;
                    }
                };
            } else if !enable {
                assert!(PERIPHERAL_REF_COUNT[peripheral as usize].swap(0, Ordering::Relaxed) == 0);
            }

            if !enable {
                Self::reset(peripheral);
            }

            Self::enable_internal(peripheral, enable, &cs);

            true
        })
    }
}

/// Enumeration of the available radio peripherals for this chip.
#[cfg(any(bt, ieee802154, wifi))]
pub enum RadioPeripherals {
    /// Represents the PHY (Physical Layer) peripheral.
    #[cfg(phy)]
    Phy,
    /// Represents the Bluetooth peripheral.
    #[cfg(bt)]
    Bt,
    /// Represents the WiFi peripheral.
    #[cfg(wifi)]
    Wifi,
    /// Represents the IEEE 802.15.4 peripheral.
    #[cfg(ieee802154)]
    Ieee802154,
}

/// Control the radio peripheral clocks
#[cfg(any(bt, ieee802154, wifi))]
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

    /// Reset the Resolvable Private Address (RPA).
    fn reset_rpa(&mut self);
}
