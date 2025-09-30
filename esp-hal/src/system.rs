//! # System Control

use esp_sync::NonReentrantMutex;

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
    #[cfg(soc_has_spi2)]
    Spi2,
    /// SPI3 peripheral.
    #[cfg(soc_has_spi3)]
    Spi3,
    /// External I2C0 peripheral.
    #[cfg(soc_has_i2c0)]
    I2cExt0,
    /// External I2C1 peripheral.
    #[cfg(soc_has_i2c1)]
    I2cExt1,
    /// RMT peripheral (Remote Control).
    #[cfg(soc_has_rmt)]
    Rmt,
    /// LEDC peripheral (LED PWM Controller).
    #[cfg(soc_has_ledc)]
    Ledc,
    /// MCPWM0 peripheral (Motor Control PWM 0).
    #[cfg(soc_has_mcpwm0)]
    Mcpwm0,
    /// MCPWM1 peripheral (Motor Control PWM 1).
    #[cfg(soc_has_mcpwm1)]
    Mcpwm1,
    /// PCNT peripheral (Pulse Counter).
    #[cfg(soc_has_pcnt)]
    Pcnt,
    /// APB SAR ADC peripheral.
    #[cfg(soc_has_apb_saradc)]
    ApbSarAdc,
    /// General DMA (GDMA) peripheral.
    #[cfg(gdma)]
    Gdma,
    /// Peripheral DMA (PDMA) peripheral.
    #[cfg(pdma)]
    Dma,
    /// I2S0 peripheral (Inter-IC Sound).
    #[cfg(soc_has_i2s0)]
    I2s0,
    /// I2S1 peripheral (Inter-IC Sound).
    #[cfg(soc_has_i2s1)]
    I2s1,
    /// USB0 peripheral.
    #[cfg(soc_has_usb0)]
    Usb,
    /// AES peripheral (Advanced Encryption Standard).
    #[cfg(soc_has_aes)]
    Aes,
    /// TWAI0 peripheral.
    #[cfg(soc_has_twai0)]
    Twai0,
    /// TWAI1 peripheral.
    #[cfg(soc_has_twai1)]
    Twai1,
    /// Timer Group 0 peripheral.
    #[cfg(soc_has_timg0)]
    Timg0,
    /// Timer Group 1 peripheral.
    #[cfg(soc_has_timg1)]
    Timg1,
    /// SHA peripheral (Secure Hash Algorithm).
    #[cfg(soc_has_sha)]
    Sha,
    /// USB Device peripheral.
    #[cfg(soc_has_usb_device)]
    UsbDevice,
    /// UART0 peripheral.
    #[cfg(soc_has_uart0)]
    Uart0,
    /// UART1 peripheral.
    #[cfg(soc_has_uart1)]
    Uart1,
    /// UART2 peripheral.
    #[cfg(soc_has_uart2)]
    Uart2,
    /// RSA peripheral (Rivest-Shamir-Adleman encryption).
    #[cfg(soc_has_rsa)]
    Rsa,
    /// Parallel IO peripheral.
    #[cfg(soc_has_parl_io)]
    ParlIo,
    /// HMAC peripheral (Hash-based Message Authentication Code).
    #[cfg(soc_has_hmac)]
    Hmac,
    /// ECC peripheral (Elliptic Curve Cryptography).
    #[cfg(soc_has_ecc)]
    Ecc,
    /// SOC ETM peripheral (Event Task Manager).
    #[cfg(soc_has_etm)]
    Etm,
    /// TRACE0 peripheral (Debug trace).
    #[cfg(soc_has_trace0)]
    Trace0,
    /// LCD Camera peripheral.
    #[cfg(soc_has_lcd_cam)]
    LcdCam,
    /// Systimer peripheral.
    #[cfg(soc_has_systimer)]
    Systimer,
    /// Temperature sensor peripheral.
    #[cfg(soc_has_tsens)]
    Tsens,
    /// UHCI0
    #[cfg(soc_has_uhci0)]
    Uhci0,
}

impl Peripheral {
    const KEEP_ENABLED: &[Peripheral] = &[
        Peripheral::Uart0,
        #[cfg(soc_has_usb_device)]
        Peripheral::UsbDevice,
        #[cfg(soc_has_systimer)]
        Peripheral::Systimer,
        #[cfg(soc_has_timg0)]
        Peripheral::Timg0,
        #[cfg(esp32c6)] // used by some wifi calibration steps.
        // TODO: We should probably automatically enable this when needed.
        Peripheral::ApbSarAdc,
    ];

    const COUNT: usize = Self::ALL.len();

    const ALL: &[Self] = &[
        #[cfg(soc_has_spi2)]
        Self::Spi2,
        #[cfg(soc_has_spi3)]
        Self::Spi3,
        #[cfg(soc_has_i2c0)]
        Self::I2cExt0,
        #[cfg(soc_has_i2c1)]
        Self::I2cExt1,
        #[cfg(soc_has_rmt)]
        Self::Rmt,
        #[cfg(soc_has_ledc)]
        Self::Ledc,
        #[cfg(soc_has_mcpwm0)]
        Self::Mcpwm0,
        #[cfg(soc_has_mcpwm1)]
        Self::Mcpwm1,
        #[cfg(soc_has_pcnt)]
        Self::Pcnt,
        #[cfg(soc_has_apb_saradc)]
        Self::ApbSarAdc,
        #[cfg(gdma)]
        Self::Gdma,
        #[cfg(pdma)]
        Self::Dma,
        #[cfg(soc_has_i2s0)]
        Self::I2s0,
        #[cfg(soc_has_i2s1)]
        Self::I2s1,
        #[cfg(soc_has_usb0)]
        Self::Usb,
        #[cfg(soc_has_aes)]
        Self::Aes,
        #[cfg(soc_has_twai0)]
        Self::Twai0,
        #[cfg(soc_has_twai1)]
        Self::Twai1,
        #[cfg(soc_has_timg0)]
        Self::Timg0,
        #[cfg(soc_has_timg1)]
        Self::Timg1,
        #[cfg(soc_has_sha)]
        Self::Sha,
        #[cfg(soc_has_usb_device)]
        Self::UsbDevice,
        #[cfg(soc_has_uart0)]
        Self::Uart0,
        #[cfg(soc_has_uart1)]
        Self::Uart1,
        #[cfg(soc_has_uart2)]
        Self::Uart2,
        #[cfg(soc_has_rsa)]
        Self::Rsa,
        #[cfg(soc_has_parl_io)]
        Self::ParlIo,
        #[cfg(soc_has_hmac)]
        Self::Hmac,
        #[cfg(soc_has_ecc)]
        Self::Ecc,
        #[cfg(soc_has_etm)]
        Self::Etm,
        #[cfg(soc_has_trace0)]
        Self::Trace0,
        #[cfg(soc_has_lcd_cam)]
        Self::LcdCam,
        #[cfg(soc_has_systimer)]
        Self::Systimer,
        #[cfg(soc_has_tsens)]
        Self::Tsens,
        #[cfg(soc_has_uhci0)]
        Self::Uhci0,
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
        for p in Peripheral::ALL {
            if Peripheral::KEEP_ENABLED.contains(p) {
                continue;
            }
            PeripheralClockControl::enable_forced_with_counts(*p, false, true, refcounts);
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
    pub(crate) fn new_with(init: fn()) -> Self {
        let peripheral = unwrap!(Peripheral::try_from(P));
        if !Peripheral::KEEP_ENABLED.contains(&peripheral) {
            PERIPHERAL_REF_COUNT.with(|ref_counts| {
                if PeripheralClockControl::enable_with_counts(peripheral, ref_counts) {
                    unsafe { PeripheralClockControl::reset_racey(peripheral) };
                    init();
                }
            });
        }

        Self {}
    }

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
    unsafe fn enable_internal_racey(peripheral: Peripheral, enable: bool) {
        debug!("Enable {:?} {}", peripheral, enable);

        let system = SYSTEM::regs();

        #[cfg(esp32)]
        let (perip_clk_en0, perip_clk_en1) = { (&system.perip_clk_en(), &system.peri_clk_en()) };
        #[cfg(not(esp32))]
        let perip_clk_en0 = &system.perip_clk_en0();

        #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
        let perip_clk_en1 = &system.perip_clk_en1();

        match peripheral {
            #[cfg(soc_has_spi2)]
            Peripheral::Spi2 => {
                perip_clk_en0.modify(|_, w| w.spi2_clk_en().bit(enable));
            }
            #[cfg(soc_has_spi3)]
            Peripheral::Spi3 => {
                perip_clk_en0.modify(|_, w| w.spi3_clk_en().bit(enable));
            }
            #[cfg(soc_has_i2c0)]
            Peripheral::I2cExt0 => {
                perip_clk_en0.modify(|_, w| w.i2c_ext0_clk_en().bit(enable));
            }
            #[cfg(soc_has_i2c1)]
            Peripheral::I2cExt1 => {
                perip_clk_en0.modify(|_, w| w.i2c_ext1_clk_en().bit(enable));
            }
            #[cfg(soc_has_rmt)]
            Peripheral::Rmt => {
                perip_clk_en0.modify(|_, w| w.rmt_clk_en().bit(enable));
            }
            #[cfg(soc_has_ledc)]
            Peripheral::Ledc => {
                perip_clk_en0.modify(|_, w| w.ledc_clk_en().bit(enable));
            }
            #[cfg(soc_has_mcpwm0)]
            Peripheral::Mcpwm0 => {
                perip_clk_en0.modify(|_, w| w.pwm0_clk_en().bit(enable));
            }
            #[cfg(soc_has_mcpwm1)]
            Peripheral::Mcpwm1 => {
                perip_clk_en0.modify(|_, w| w.pwm1_clk_en().bit(enable));
            }
            #[cfg(soc_has_pcnt)]
            Peripheral::Pcnt => {
                perip_clk_en0.modify(|_, w| w.pcnt_clk_en().bit(enable));
            }
            #[cfg(soc_has_apb_saradc)]
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
            #[cfg(soc_has_i2s0)]
            Peripheral::I2s0 => {
                perip_clk_en0.modify(|_, w| w.i2s0_clk_en().bit(enable));
            }
            #[cfg(soc_has_i2s1)]
            Peripheral::I2s1 => {
                perip_clk_en0.modify(|_, w| w.i2s1_clk_en().bit(enable));
            }
            #[cfg(soc_has_usb0)]
            Peripheral::Usb => {
                perip_clk_en0.modify(|_, w| w.usb_clk_en().bit(enable));
            }
            #[cfg(soc_has_twai0)]
            Peripheral::Twai0 => {
                perip_clk_en0.modify(|_, w| w.twai_clk_en().bit(enable));
            }
            #[cfg(soc_has_aes)]
            Peripheral::Aes => {
                perip_clk_en1.modify(|_, w| w.crypto_aes_clk_en().bit(enable));
            }
            #[cfg(soc_has_timg0)]
            Peripheral::Timg0 => {
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_clk_en0.modify(|_, w| w.timers_clk_en().bit(enable));
                perip_clk_en0.modify(|_, w| w.timergroup_clk_en().bit(enable));
            }
            #[cfg(soc_has_timg1)]
            Peripheral::Timg1 => {
                #[cfg(any(esp32c3, esp32s2, esp32s3))]
                perip_clk_en0.modify(|_, w| w.timers_clk_en().bit(enable));
                perip_clk_en0.modify(|_, w| w.timergroup1_clk_en().bit(enable));
            }
            #[cfg(soc_has_sha)]
            Peripheral::Sha => {
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
            #[cfg(soc_has_uart0)]
            Peripheral::Uart0 => {
                perip_clk_en0.modify(|_, w| w.uart_clk_en().bit(enable));
            }
            #[cfg(soc_has_uart1)]
            Peripheral::Uart1 => {
                perip_clk_en0.modify(|_, w| w.uart1_clk_en().bit(enable));
            }
            #[cfg(all(soc_has_uart2, esp32s3))]
            Peripheral::Uart2 => {
                perip_clk_en1.modify(|_, w| w.uart2_clk_en().set_bit());
            }
            #[cfg(all(soc_has_uart2, esp32))]
            Peripheral::Uart2 => {
                perip_clk_en0.modify(|_, w| w.uart2_clk_en().bit(enable));
            }
            #[cfg(all(rsa, esp32))]
            Peripheral::Rsa => {
                perip_clk_en1.modify(|_, w| w.crypto_rsa_clk_en().bit(enable));
            }
            #[cfg(all(rsa, any(esp32c3, esp32s2, esp32s3)))]
            Peripheral::Rsa => {
                perip_clk_en1.modify(|_, w| w.crypto_rsa_clk_en().bit(enable));
                system
                    .rsa_pd_ctrl()
                    .modify(|_, w| w.rsa_mem_pd().bit(!enable));
            }
            #[cfg(soc_has_hmac)]
            Peripheral::Hmac => {
                perip_clk_en1.modify(|_, w| w.crypto_hmac_clk_en().bit(enable));
            }
            #[cfg(soc_has_ecc)]
            Peripheral::Ecc => {
                perip_clk_en1.modify(|_, w| w.crypto_ecc_clk_en().bit(enable));
            }
            #[cfg(soc_has_lcd_cam)]
            Peripheral::LcdCam => {
                perip_clk_en1.modify(|_, w| w.lcd_cam_clk_en().bit(enable));
            }
            #[cfg(soc_has_systimer)]
            Peripheral::Systimer => {
                perip_clk_en0.modify(|_, w| w.systimer_clk_en().bit(enable));
            }
            #[cfg(soc_has_tsens)]
            Peripheral::Tsens => {
                perip_clk_en1.modify(|_, w| w.tsens_clk_en().bit(enable));
            }
            #[cfg(soc_has_uhci0)]
            Peripheral::Uhci0 => {
                perip_clk_en0.modify(|_, w| w.uhci0_clk_en().bit(enable));
            }
        }
    }
}

#[cfg(any(esp32c6, esp32h2))]
impl PeripheralClockControl {
    unsafe fn enable_internal_racey(peripheral: Peripheral, enable: bool) {
        debug!("Enable {:?} {}", peripheral, enable);
        let system = SYSTEM::regs();

        match peripheral {
            #[cfg(soc_has_spi2)]
            Peripheral::Spi2 => {
                system
                    .spi2_conf()
                    .modify(|_, w| w.spi2_clk_en().bit(enable));
            }
            #[cfg(soc_has_i2c0)]
            Peripheral::I2cExt0 => {
                system
                    .i2c0_conf()
                    .modify(|_, w| w.i2c0_clk_en().bit(enable));
            }
            #[cfg(soc_has_i2c1)]
            Peripheral::I2cExt1 => {
                system
                    .i2c1_conf()
                    .modify(|_, w| w.i2c1_clk_en().bit(enable));
            }
            #[cfg(soc_has_rmt)]
            Peripheral::Rmt => {
                system.rmt_conf().modify(|_, w| w.rmt_clk_en().bit(enable));
            }
            #[cfg(soc_has_ledc)]
            Peripheral::Ledc => {
                system
                    .ledc_conf()
                    .modify(|_, w| w.ledc_clk_en().bit(enable));
            }
            #[cfg(soc_has_mcpwm0)]
            Peripheral::Mcpwm0 => {
                system.pwm_conf().modify(|_, w| w.pwm_clk_en().bit(enable));
            }
            #[cfg(soc_has_mcpwm1)]
            Peripheral::Mcpwm1 => {
                system.pwm_conf.modify(|_, w| w.pwm_clk_en().bit(enable));
            }
            #[cfg(soc_has_apb_saradc)]
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
            #[cfg(soc_has_i2s0)]
            Peripheral::I2s0 => {
                system.i2s_conf().modify(|_, w| w.i2s_clk_en().bit(enable));
            }
            #[cfg(soc_has_twai0)]
            Peripheral::Twai0 => {
                system
                    .twai0_conf()
                    .modify(|_, w| w.twai0_clk_en().bit(enable));

                if enable {
                    // use Xtal clk-src
                    system.twai0_func_clk_conf().modify(|_, w| {
                        w.twai0_func_clk_en().set_bit();
                        w.twai0_func_clk_sel().variant(false)
                    });
                }
            }
            #[cfg(soc_has_twai1)]
            Peripheral::Twai1 => {
                system
                    .twai1_conf()
                    .modify(|_, w| w.twai1_clk_en().bit(enable));
            }
            #[cfg(soc_has_aes)]
            Peripheral::Aes => {
                system.aes_conf().modify(|_, w| w.aes_clk_en().bit(enable));
            }
            #[cfg(soc_has_pcnt)]
            Peripheral::Pcnt => {
                system
                    .pcnt_conf()
                    .modify(|_, w| w.pcnt_clk_en().bit(enable));
            }
            #[cfg(soc_has_timg0)]
            Peripheral::Timg0 => {
                system
                    .timergroup0_timer_clk_conf()
                    .modify(|_, w| w.tg0_timer_clk_en().bit(enable));
            }
            #[cfg(soc_has_timg1)]
            Peripheral::Timg1 => {
                system
                    .timergroup1_timer_clk_conf()
                    .modify(|_, w| w.tg1_timer_clk_en().bit(enable));
            }
            #[cfg(soc_has_sha)]
            Peripheral::Sha => {
                system.sha_conf().modify(|_, w| w.sha_clk_en().bit(enable));
            }
            #[cfg(soc_has_usb_device)]
            Peripheral::UsbDevice => {
                system
                    .usb_device_conf()
                    .modify(|_, w| w.usb_device_clk_en().bit(enable));
            }
            #[cfg(soc_has_uart0)]
            Peripheral::Uart0 => {
                system.uart(0).conf().modify(|_, w| w.clk_en().bit(enable));
            }
            #[cfg(soc_has_uart1)]
            Peripheral::Uart1 => {
                system.uart(1).conf().modify(|_, w| w.clk_en().bit(enable));
            }
            #[cfg(soc_has_rsa)]
            Peripheral::Rsa => {
                system.rsa_conf().modify(|_, w| w.rsa_clk_en().bit(enable));
                system
                    .rsa_pd_ctrl()
                    .modify(|_, w| w.rsa_mem_pd().clear_bit());
            }
            #[cfg(soc_has_parl_io)]
            Peripheral::ParlIo => {
                system
                    .parl_io_conf()
                    .modify(|_, w| w.parl_clk_en().bit(enable));
            }
            #[cfg(soc_has_hmac)]
            Peripheral::Hmac => {
                system
                    .hmac_conf()
                    .modify(|_, w| w.hmac_clk_en().bit(enable));
            }
            #[cfg(soc_has_ecc)]
            Peripheral::Ecc => {
                system.ecc_conf().modify(|_, w| w.ecc_clk_en().bit(enable));
            }
            #[cfg(soc_has_etm)]
            Peripheral::Etm => {
                system.etm_conf().modify(|_, w| w.etm_clk_en().bit(enable));
            }
            #[cfg(soc_has_trace0)]
            Peripheral::Trace0 => {
                system
                    .trace_conf()
                    .modify(|_, w| w.trace_clk_en().bit(enable));
            }
            #[cfg(soc_has_systimer)]
            Peripheral::Systimer => {
                system
                    .systimer_conf()
                    .modify(|_, w| w.systimer_clk_en().bit(enable));
            }
            #[cfg(soc_has_tsens)]
            Peripheral::Tsens => {
                system.tsens_clk_conf().modify(|_, w| {
                    w.tsens_clk_en().bit(enable);
                    w.tsens_clk_sel().bit(enable)
                });
            }
            #[cfg(soc_has_uhci0)]
            Peripheral::Uhci0 => {
                system
                    .uhci_conf()
                    .modify(|_, w| w.uhci_clk_en().bit(enable));
            }
        }
    }
}

#[cfg(not(any(esp32c6, esp32h2)))]
/// Resets the given peripheral
unsafe fn assert_peri_reset_racey(peripheral: Peripheral, reset: bool) {
    let system = SYSTEM::regs();

    #[cfg(esp32)]
    let (perip_rst_en0, perip_rst_en1) = (system.perip_rst_en(), system.peri_rst_en());
    #[cfg(not(esp32))]
    let perip_rst_en0 = system.perip_rst_en0();

    #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
    let perip_rst_en1 = system.perip_rst_en1();

    match peripheral {
        #[cfg(soc_has_spi2)]
        Peripheral::Spi2 => {
            perip_rst_en0.modify(|_, w| w.spi2_rst().bit(reset));
        }
        #[cfg(soc_has_spi3)]
        Peripheral::Spi3 => {
            perip_rst_en0.modify(|_, w| w.spi3_rst().bit(reset));
        }
        #[cfg(soc_has_i2c0)]
        Peripheral::I2cExt0 => {
            perip_rst_en0.modify(|_, w| w.i2c_ext0_rst().bit(reset));
        }
        #[cfg(soc_has_i2c1)]
        Peripheral::I2cExt1 => {
            perip_rst_en0.modify(|_, w| w.i2c_ext1_rst().bit(reset));
        }
        #[cfg(soc_has_rmt)]
        Peripheral::Rmt => {
            perip_rst_en0.modify(|_, w| w.rmt_rst().bit(reset));
        }
        #[cfg(soc_has_ledc)]
        Peripheral::Ledc => {
            perip_rst_en0.modify(|_, w| w.ledc_rst().bit(reset));
        }
        #[cfg(soc_has_mcpwm0)]
        Peripheral::Mcpwm0 => {
            perip_rst_en0.modify(|_, w| w.pwm0_rst().bit(reset));
        }
        #[cfg(soc_has_mcpwm1)]
        Peripheral::Mcpwm1 => {
            perip_rst_en0.modify(|_, w| w.pwm1_rst().bit(reset));
        }
        #[cfg(soc_has_pcnt)]
        Peripheral::Pcnt => {
            perip_rst_en0.modify(|_, w| w.pcnt_rst().bit(reset));
        }
        #[cfg(soc_has_apb_saradc)]
        Peripheral::ApbSarAdc => {
            perip_rst_en0.modify(|_, w| w.apb_saradc_rst().bit(reset));
        }
        #[cfg(gdma)]
        Peripheral::Gdma => {
            perip_rst_en1.modify(|_, w| w.dma_rst().bit(reset));
        }
        #[cfg(esp32)]
        Peripheral::Dma => {
            perip_rst_en0.modify(|_, w| w.spi_dma_rst().bit(reset));
        }
        #[cfg(esp32s2)]
        Peripheral::Dma => {
            perip_rst_en0.modify(|_, w| w.spi2_dma_rst().bit(reset));
            perip_rst_en0.modify(|_, w| w.spi3_dma_rst().bit(reset));
            perip_rst_en1.modify(|_, w| w.crypto_dma_rst().bit(reset));
        }
        #[cfg(soc_has_i2s0)]
        Peripheral::I2s0 => {
            perip_rst_en0.modify(|_, w| w.i2s0_rst().bit(reset));
        }
        #[cfg(soc_has_i2s1)]
        Peripheral::I2s1 => {
            perip_rst_en0.modify(|_, w| w.i2s1_rst().bit(reset));
        }
        #[cfg(soc_has_usb0)]
        Peripheral::Usb => {
            perip_rst_en0.modify(|_, w| w.usb_rst().bit(reset));
        }
        #[cfg(soc_has_twai0)]
        Peripheral::Twai0 => {
            perip_rst_en0.modify(|_, w| w.twai_rst().bit(reset));
        }
        #[cfg(soc_has_aes)]
        Peripheral::Aes => {
            perip_rst_en1.modify(|_, w| w.crypto_aes_rst().bit(reset));
        }
        #[cfg(soc_has_timg0)]
        Peripheral::Timg0 => {
            #[cfg(any(esp32c3, esp32s2, esp32s3))]
            perip_rst_en0.modify(|_, w| w.timers_rst().bit(reset));
            perip_rst_en0.modify(|_, w| w.timergroup_rst().bit(reset));
        }
        #[cfg(soc_has_timg1)]
        Peripheral::Timg1 => {
            #[cfg(any(esp32c3, esp32s2, esp32s3))]
            perip_rst_en0.modify(|_, w| w.timers_rst().bit(reset));
            perip_rst_en0.modify(|_, w| w.timergroup1_rst().bit(reset));
        }
        #[cfg(soc_has_sha)]
        Peripheral::Sha => {
            perip_rst_en1.modify(|_, w| w.crypto_sha_rst().bit(reset));
        }
        #[cfg(soc_has_usb_device)]
        Peripheral::UsbDevice => {
            cfg_if::cfg_if! {
                if #[cfg(esp32c3)] {
                    perip_rst_en0.modify(|_, w| w.usb_device_rst().bit(reset));
                } else {
                    perip_rst_en1.modify(|_, w| w.usb_device_rst().bit(reset));
                }
            }
        }
        #[cfg(soc_has_uart0)]
        Peripheral::Uart0 => {
            perip_rst_en0.modify(|_, w| w.uart_rst().bit(reset));
        }
        #[cfg(soc_has_uart1)]
        Peripheral::Uart1 => {
            perip_rst_en0.modify(|_, w| w.uart1_rst().bit(reset));
        }
        #[cfg(soc_has_uart2)]
        Peripheral::Uart2 => {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    perip_rst_en0.modify(|_, w| w.uart2_rst().bit(reset));
                } else {
                    perip_rst_en1.modify(|_, w| w.uart2_rst().bit(reset));
                }
            }
        }
        #[cfg(soc_has_rsa)]
        Peripheral::Rsa => {
            perip_rst_en1.modify(|_, w| w.crypto_rsa_rst().bit(reset));
        }
        #[cfg(soc_has_hmac)]
        Peripheral::Hmac => {
            perip_rst_en1.modify(|_, w| w.crypto_hmac_rst().bit(reset));
        }
        #[cfg(soc_has_ecc)]
        Peripheral::Ecc => {
            perip_rst_en1.modify(|_, w| w.crypto_ecc_rst().bit(reset));
        }
        #[cfg(soc_has_lcd_cam)]
        Peripheral::LcdCam => {
            perip_rst_en1.modify(|_, w| w.lcd_cam_rst().bit(reset));
        }
        #[cfg(soc_has_systimer)]
        Peripheral::Systimer => {
            perip_rst_en0.modify(|_, w| w.systimer_rst().bit(reset));
        }
        #[cfg(soc_has_tsens)]
        Peripheral::Tsens => {
            cfg_if::cfg_if! {
                if #[cfg(esp32c3)] {
                    perip_rst_en1.modify(|_, w| w.tsens_rst().bit(reset));
                } else {
                    perip_rst_en0.modify(|_, w| w.tsens_rst().bit(reset));
                }
            }
        }
        #[cfg(soc_has_uhci0)]
        Peripheral::Uhci0 => {
            perip_rst_en0.modify(|_, w| w.uhci0_rst().bit(reset));
        }
    }
}

#[cfg(any(esp32c6, esp32h2))]
unsafe fn assert_peri_reset_racey(peripheral: Peripheral, reset: bool) {
    let system = SYSTEM::regs();

    // No need to lock, different peripherals' bits are in separate registers. In theory this may
    // race with accessing the clk_enable bits, but the peripheral singleton pattern, as well as the
    // general usage patterns of this code should prevent that.

    match peripheral {
        #[cfg(soc_has_spi2)]
        Peripheral::Spi2 => {
            system.spi2_conf().modify(|_, w| w.spi2_rst_en().bit(reset));
        }
        #[cfg(soc_has_i2c0)]
        Peripheral::I2cExt0 => {
            system.i2c0_conf().modify(|_, w| w.i2c0_rst_en().bit(reset));
        }
        #[cfg(soc_has_i2c1)]
        Peripheral::I2cExt1 => {
            system.i2c1_conf().modify(|_, w| w.i2c1_rst_en().bit(reset));
        }
        #[cfg(soc_has_rmt)]
        Peripheral::Rmt => {
            system.rmt_conf().modify(|_, w| w.rmt_rst_en().bit(reset));
        }
        #[cfg(soc_has_ledc)]
        Peripheral::Ledc => {
            system.ledc_conf().modify(|_, w| w.ledc_rst_en().bit(reset));
        }
        #[cfg(soc_has_mcpwm0)]
        Peripheral::Mcpwm0 => {
            system.pwm_conf().modify(|_, w| w.pwm_rst_en().bit(reset));
        }
        #[cfg(soc_has_apb_saradc)]
        Peripheral::ApbSarAdc => {
            system
                .saradc_conf()
                .modify(|_, w| w.saradc_reg_rst_en().bit(reset));
        }
        #[cfg(gdma)]
        Peripheral::Gdma => {
            system.gdma_conf().modify(|_, w| w.gdma_rst_en().bit(reset));
        }
        #[cfg(soc_has_i2s0)]
        Peripheral::I2s0 => {
            system.i2s_conf().modify(|_, w| w.i2s_rst_en().bit(reset));
        }
        #[cfg(soc_has_twai0)]
        Peripheral::Twai0 => {
            system
                .twai0_conf()
                .modify(|_, w| w.twai0_rst_en().bit(reset));
        }
        #[cfg(soc_has_twai1)]
        Peripheral::Twai1 => {
            system
                .twai1_conf()
                .modify(|_, w| w.twai1_rst_en().bit(reset));
        }
        #[cfg(soc_has_aes)]
        Peripheral::Aes => {
            system.aes_conf().modify(|_, w| w.aes_rst_en().bit(reset));
        }
        #[cfg(soc_has_pcnt)]
        Peripheral::Pcnt => {
            system.pcnt_conf().modify(|_, w| w.pcnt_rst_en().bit(reset));
        }
        #[cfg(soc_has_timg0)]
        Peripheral::Timg0 => {
            // no reset?
        }
        #[cfg(soc_has_timg1)]
        Peripheral::Timg1 => {
            // no reset?
        }
        #[cfg(soc_has_sha)]
        Peripheral::Sha => {
            system.sha_conf().modify(|_, w| w.sha_rst_en().bit(reset));
        }
        #[cfg(soc_has_usb_device)]
        Peripheral::UsbDevice => {
            system
                .usb_device_conf()
                .modify(|_, w| w.usb_device_rst_en().bit(reset));
        }
        #[cfg(soc_has_uart0)]
        Peripheral::Uart0 => {
            system.uart(0).conf().modify(|_, w| w.rst_en().bit(reset));
        }
        #[cfg(soc_has_uart1)]
        Peripheral::Uart1 => {
            system.uart(1).conf().modify(|_, w| w.rst_en().bit(reset));
        }
        #[cfg(soc_has_rsa)]
        Peripheral::Rsa => {
            system.rsa_conf().modify(|_, w| w.rsa_rst_en().bit(reset));
        }
        #[cfg(soc_has_parl_io)]
        Peripheral::ParlIo => {
            system
                .parl_io_conf()
                .modify(|_, w| w.parl_rst_en().bit(reset));
        }
        #[cfg(soc_has_hmac)]
        Peripheral::Hmac => {
            system.hmac_conf().modify(|_, w| w.hmac_rst_en().bit(reset));
        }
        #[cfg(soc_has_ecc)]
        Peripheral::Ecc => {
            system.ecc_conf().modify(|_, w| w.ecc_rst_en().bit(reset));
        }
        #[cfg(soc_has_etm)]
        Peripheral::Etm => {
            system.etm_conf().modify(|_, w| w.etm_rst_en().bit(reset));
        }
        #[cfg(soc_has_trace0)]
        Peripheral::Trace0 => {
            system
                .trace_conf()
                .modify(|_, w| w.trace_rst_en().bit(reset));
        }
        #[cfg(soc_has_systimer)]
        Peripheral::Systimer => {
            system
                .systimer_conf()
                .modify(|_, w| w.systimer_rst_en().bit(reset));
        }
        #[cfg(soc_has_tsens)]
        Peripheral::Tsens => {
            system
                .tsens_clk_conf()
                .modify(|_, w| w.tsens_rst_en().bit(reset));
        }
        #[cfg(soc_has_uhci0)]
        Peripheral::Uhci0 => {
            system.uhci_conf().modify(|_, w| w.uhci_rst_en().bit(reset));
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
    ///
    /// Before disabling a peripheral it will also get reset
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

        if !enable {
            unsafe { Self::reset_racey(peripheral) };
        }

        unsafe { Self::enable_internal_racey(peripheral, enable) };

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
