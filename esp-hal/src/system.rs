//! # System Control

use core::cell::RefCell;

use critical_section::{CriticalSection, Mutex};

use crate::peripherals::SYSTEM;

/// Peripherals which can be enabled via `PeripheralClockControl`.
///
/// This enum represents various hardware peripherals that can be enabled
/// by the system's clock control. Depending on the target device, different
/// peripherals will be available for enabling.
// FIXME: This enum needs to be public because it's exposed via a bunch of traits, but it's not
// useful to users.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    /// Temperature sensor peripheral.
    #[cfg(tsens)]
    Tsens,
}

impl Peripheral {
    const KEEP_ENABLED: &[Peripheral] = &[
        Peripheral::Uart0,
        #[cfg(usb_device)]
        Peripheral::UsbDevice,
        #[cfg(systimer)]
        Peripheral::Systimer,
        Peripheral::Timg0,
        #[cfg(esp32c6)] // used by some wifi calibration steps.
        // TODO: We should probably automatically enable this when needed.
        Peripheral::ApbSarAdc,
    ];

    const COUNT: usize = Self::ALL.len();

    const ALL: &[Self] = &[
        #[cfg(spi2)]
        Self::Spi2,
        #[cfg(spi3)]
        Self::Spi3,
        #[cfg(i2c0)]
        Self::I2cExt0,
        #[cfg(i2c1)]
        Self::I2cExt1,
        #[cfg(rmt)]
        Self::Rmt,
        #[cfg(ledc)]
        Self::Ledc,
        #[cfg(mcpwm0)]
        Self::Mcpwm0,
        #[cfg(mcpwm1)]
        Self::Mcpwm1,
        #[cfg(pcnt)]
        Self::Pcnt,
        #[cfg(apb_saradc)]
        Self::ApbSarAdc,
        #[cfg(gdma)]
        Self::Gdma,
        #[cfg(pdma)]
        Self::Dma,
        #[cfg(i2s0)]
        Self::I2s0,
        #[cfg(i2s1)]
        Self::I2s1,
        #[cfg(usb0)]
        Self::Usb,
        #[cfg(aes)]
        Self::Aes,
        #[cfg(twai0)]
        Self::Twai0,
        #[cfg(twai1)]
        Self::Twai1,
        #[cfg(timg0)]
        Self::Timg0,
        #[cfg(timg1)]
        Self::Timg1,
        #[cfg(sha)]
        Self::Sha,
        #[cfg(usb_device)]
        Self::UsbDevice,
        #[cfg(uart0)]
        Self::Uart0,
        #[cfg(uart1)]
        Self::Uart1,
        #[cfg(uart2)]
        Self::Uart2,
        #[cfg(rsa)]
        Self::Rsa,
        #[cfg(parl_io)]
        Self::ParlIo,
        #[cfg(hmac)]
        Self::Hmac,
        #[cfg(ecc)]
        Self::Ecc,
        #[cfg(soc_etm)]
        Self::Etm,
        #[cfg(trace0)]
        Self::Trace0,
        #[cfg(lcd_cam)]
        Self::LcdCam,
        #[cfg(systimer)]
        Self::Systimer,
        #[cfg(tsens)]
        Self::Tsens,
    ];
}

impl Peripheral {
    pub fn try_from(value: u8) -> Option<Peripheral> {
        if value >= Peripheral::COUNT as u8 {
            return None;
        }

        Some(unsafe { core::mem::transmute::<u8, Peripheral>(value) })
    }
}

static PERIPHERAL_REF_COUNT: Mutex<RefCell<[usize; Peripheral::COUNT]>> =
    Mutex::new(RefCell::new([0; Peripheral::COUNT]));

/// Disable all peripherals.
///
/// Peripherals listed in [KEEP_ENABLED] are NOT disabled.
pub(crate) fn disable_peripherals() {
    // Take the critical section up front to avoid taking it multiple times.
    critical_section::with(|cs| {
        for p in Peripheral::ALL {
            if Peripheral::KEEP_ENABLED.contains(p) {
                continue;
            }
            PeripheralClockControl::enable_forced_with_cs(*p, false, true, cs);
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
        if !Peripheral::KEEP_ENABLED.contains(&p) && PeripheralClockControl::enable(p) {
            PeripheralClockControl::reset(p);
            init();
        }

        Self { peripheral: p }
    }

    pub(crate) fn new(p: Peripheral) -> Self {
        Self::new_with(p, || {})
    }
}

impl Drop for PeripheralGuard {
    fn drop(&mut self) {
        if !Peripheral::KEEP_ENABLED.contains(&self.peripheral) {
            PeripheralClockControl::disable(self.peripheral);
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct GenericPeripheralGuard<const P: u8> {}

impl<const P: u8> GenericPeripheralGuard<P> {
    pub(crate) fn new_with(init: fn(CriticalSection<'_>)) -> Self {
        let peripheral = unwrap!(Peripheral::try_from(P));
        critical_section::with(|cs| {
            if !Peripheral::KEEP_ENABLED.contains(&peripheral)
                && PeripheralClockControl::enable_with_cs(peripheral, cs)
            {
                PeripheralClockControl::reset(peripheral);
                init(cs);
            }
        });

        Self {}
    }

    pub(crate) fn new() -> Self {
        Self::new_with(|_| {})
    }
}

impl<const P: u8> Drop for GenericPeripheralGuard<P> {
    fn drop(&mut self) {
        let peripheral = unwrap!(Peripheral::try_from(P));
        if !Peripheral::KEEP_ENABLED.contains(&peripheral) {
            PeripheralClockControl::disable(peripheral);
        }
    }
}

/// Controls the enablement of peripheral clocks.
pub(crate) struct PeripheralClockControl;

#[cfg(not(any(esp32c6, esp32h2)))]
impl PeripheralClockControl {
    fn enable_internal(peripheral: Peripheral, enable: bool, _cs: CriticalSection<'_>) {
        debug!("Enable {:?} {}", peripheral, enable);

        let system = SYSTEM::regs();

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
                perip_clk_en1.modify(|_, w| w.crypto_dma_clk_en().bit(enable));
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
                peri_clk_en.modify(|r, w| unsafe { w.bits(r.bits() | ((enable as u32) << 2)) });
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
            #[cfg(tsens)]
            Peripheral::Tsens => {
                perip_clk_en1.modify(|_, w| w.tsens_clk_en().bit(enable));
            }
        }
    }

    /// Resets the given peripheral
    pub(crate) fn reset(peripheral: Peripheral) {
        debug!("Reset {:?}", peripheral);
        let system = SYSTEM::regs();

        #[cfg(esp32)]
        let (perip_rst_en0, peri_rst_en) = (system.perip_rst_en(), system.peri_rst_en());
        #[cfg(not(esp32))]
        let perip_rst_en0 = system.perip_rst_en0();

        #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
        let perip_rst_en1 = system.perip_rst_en1();

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
                perip_rst_en1.modify(|_, w| w.crypto_dma_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.crypto_dma_rst().clear_bit());
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
                peri_rst_en.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 2)) });
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
            #[cfg(all(tsens, esp32c6))]
            Peripheral::Tsens => {
                perip_rst_en0.modify(|_, w| w.tsens_rst().set_bit());
                perip_rst_en0.modify(|_, w| w.tsens_rst().clear_bit());
            }
            #[cfg(all(tsens, esp32c3))]
            Peripheral::Tsens => {
                perip_rst_en1.modify(|_, w| w.tsens_rst().set_bit());
                perip_rst_en1.modify(|_, w| w.tsens_rst().clear_bit());
            }
        });
    }
}

#[cfg(any(esp32c6, esp32h2))]
impl PeripheralClockControl {
    fn enable_internal(peripheral: Peripheral, enable: bool, _cs: CriticalSection<'_>) {
        debug!("Enable {:?} {}", peripheral, enable);
        let system = SYSTEM::regs();

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
                system.uart(0).conf().modify(|_, w| w.clk_en().bit(enable));
            }
            #[cfg(uart1)]
            Peripheral::Uart1 => {
                system.uart(1).conf().modify(|_, w| w.clk_en().bit(enable));
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
            #[cfg(tsens)]
            Peripheral::Tsens => {
                system
                    .tsens_clk_conf()
                    .modify(|_, w| w.tsens_clk_en().bit(enable));

                system
                    .tsens_clk_conf()
                    .modify(|_, w| w.tsens_clk_sel().bit(enable));
            }
        }
    }

    /// Resets the given peripheral
    pub(crate) fn reset(peripheral: Peripheral) {
        debug!("Reset {:?}", peripheral);

        let system = SYSTEM::regs();

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
                system.uart(0).conf().modify(|_, w| w.rst_en().set_bit());
                system.uart(0).conf().modify(|_, w| w.rst_en().clear_bit());
            }
            #[cfg(uart1)]
            Peripheral::Uart1 => {
                system.uart(1).conf().modify(|_, w| w.rst_en().set_bit());
                system.uart(1).conf().modify(|_, w| w.rst_en().clear_bit());
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
            #[cfg(tsens)]
            Peripheral::Tsens => {
                system
                    .tsens_clk_conf()
                    .modify(|_, w| w.tsens_rst_en().set_bit());
                system
                    .tsens_clk_conf()
                    .modify(|_, w| w.tsens_rst_en().clear_bit());
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

    /// Enables the given peripheral.
    ///
    /// This keeps track of enabling a peripheral - i.e. a peripheral
    /// is only enabled with the first call attempt to enable it.
    ///
    /// Returns `true` if it actually enabled the peripheral.
    pub(crate) fn enable_with_cs(peripheral: Peripheral, cs: CriticalSection<'_>) -> bool {
        Self::enable_forced_with_cs(peripheral, true, false, cs)
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
        critical_section::with(|cs| Self::enable_forced_with_cs(peripheral, enable, force, cs))
    }

    pub(crate) fn enable_forced_with_cs(
        peripheral: Peripheral,
        enable: bool,
        force: bool,
        cs: CriticalSection<'_>,
    ) -> bool {
        let mut ref_counts = PERIPHERAL_REF_COUNT.borrow_ref_mut(cs);
        let ref_count = &mut ref_counts[peripheral as usize];
        if !force {
            if enable {
                let prev = *ref_count;
                *ref_count += 1;
                trace!("Enable {:?} {} -> {}", peripheral, prev, *ref_count);
                if prev > 0 {
                    return false;
                }
            } else {
                let prev = *ref_count;
                *ref_count -= 1;
                trace!("Disable {:?} {} -> {}", peripheral, prev, *ref_count);
                if prev > 1 {
                    return false;
                }
                assert!(prev != 0);
            };
        } else if !enable {
            assert!(*ref_count == 0);
        }

        if !enable {
            Self::reset(peripheral);
        }

        Self::enable_internal(peripheral, enable, cs);

        true
    }
}

#[cfg(any(esp32, esp32s3))]
#[allow(unused_imports)]
pub use crate::soc::cpu_control::*;

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

    /// Returns the core the application is currently executing on
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
    pub(crate) fn other() -> impl Iterator<Item = Self> {
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
    pub(crate) fn all() -> impl Iterator<Item = Self> {
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

/// Performs a software reset on the chip.
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
