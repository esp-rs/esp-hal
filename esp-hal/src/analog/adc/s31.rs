use core::{
    marker::PhantomData,
    pin::Pin,
    task::{Context, Poll},
};

// Both SAR units report completion through the single LP_ADC interrupt line.
use Interrupt::LP_ADC as InterruptSource;
// Both units share one interrupt, so the enabled instances have to be counted.
use portable_atomic::{AtomicU32, Ordering};
use procmacros::handler;

use super::{AdcCalScheme, AdcChannel, AdcConfig, AdcPin};
use crate::{
    Async,
    Blocking,
    asynch::AtomicWaker,
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripherals::{APB_SARADC, Interrupt, LP_PERI},
    rtc_cntl::WakeLock,
    system::{GenericPeripheralGuard, Peripheral},
};

/// Both units have 8 channels, and the attenuation table is indexed by channel.
pub(super) const NUM_ATTENS: usize = 8;

/// Mask of the conversion data in `sarN_data_status`. The rest of the register
/// holds the channel and the unit that produced the sample.
const ADC_DATA_MASK: u32 = 0x1_ffff;

/// The largest value a conversion can return.
///
/// The SAR has 17 redundant comparator bits with non-uniform weights, and the
/// hardware reports their weighted sum. The result is therefore not a 17-bit
/// number: it spans a little more than 12 bits.
///
/// See `adc_digi_output_data_t` for the ESP32-S31 in
/// `components/esp_hal_ana_conv/include/hal/adc_types.h`.
#[instability::unstable]
pub const FULL_SCALE: u16 = 4393;

/// The value a conversion returns for a zero input difference.
///
/// The SAR is differential, so an input tied to ground reads about this value
/// instead of zero, and only the codes above it are available to a single-ended
/// measurement.
///
/// See `ADC_LL_ZERO_DIFF_CODE` in
/// `components/esp_hal_ana_conv/esp32s31/include/hal/adc_ll.h`.
#[instability::unstable]
pub const ZERO_DIFF_CODE: u16 = 2198;

/// Software trigger mode, used for one-shot conversions.
const TRIGGER_MODE_SW: u8 = 2;
/// Trigger disabled.
const TRIGGER_MODE_OFF: u8 = 0;

/// Power the SAR up by software, instead of leaving it to the FSM.
const FORCE_XPD_SAR_PU: u8 = 3;

/// Clock divider for the digital controller. The divider is `CLK_DIV_NUM + 1`.
///
/// See `ADC_LL_CLKM_DIV_NUM_DEFAULT` in
/// `components/esp_hal_ana_conv/esp32s31/include/hal/adc_ll.h`.
const CLK_DIV_NUM: u8 = 4;

/// Digital controller clock source select value for XTAL.
const CLK_SRC_XTAL: u8 = 1;

/// Enable the reference generator shared by both units.
fn enable_refgen() {
    APB_SARADC::regs().ref_control().modify(|_, w| {
        w.rtc_xpd_refgen().set_bit();
        w.rtc_pre_charge().set_bit();
        w.rtc_ref_delay().set_bit()
    });
}

#[doc(hidden)]
pub trait RegisterAccess {
    /// Power the SAR up and put the unit into single-conversion mode.
    fn enable();

    /// Program the single-entry pattern table with the given channel.
    fn program_pattern(channel: u8);

    /// Trigger one conversion.
    fn start_sample();

    /// Check if sampling is done
    fn is_done() -> bool;

    /// Read sample data
    fn read_data() -> u16;

    /// Clear the done flag and stop triggering.
    fn reset();
}

#[cfg(adc_adc1)]
impl RegisterAccess for crate::peripherals::ADC1<'_> {
    fn enable() {
        APB_SARADC::regs()
            .ctrl2()
            .modify(|_, w| w.timer_en().clear_bit());

        enable_refgen();

        APB_SARADC::regs().ctrl0().modify(|_, w| unsafe {
            w.xpd_sar1_force().bits(FORCE_XPD_SAR_PU);
            w.sar1_continue_mode_en().clear_bit();
            w.sar1_trigger_stop().set_bit()
        });
    }

    fn program_pattern(channel: u8) {
        let regs = APB_SARADC::regs();

        // Each pattern entry is 6 bits wide and the first entry occupies the
        // most significant bits of the 24-bit table.
        let entry = (((channel & 0xf) as u32) << 2) << 18;

        regs.ctrl0().modify(|_, w| unsafe {
            w.sar1_patt_type().set_bit();
            w.sar1_patt_len().bits(0)
        });
        regs.sar1_patt_tab1()
            .write(|w| unsafe { w.sar1_patt_tab1().bits(entry) });

        regs.ctrl0().modify(|_, w| w.sar1_patt_p_clear().set_bit());
        regs.ctrl0()
            .modify(|_, w| w.sar1_patt_p_clear().clear_bit());

        regs.ctrl0()
            .modify(|_, w| unsafe { w.sar1_trigger_mode().bits(TRIGGER_MODE_SW) });
    }

    fn start_sample() {
        APB_SARADC::regs()
            .ctrl0()
            .modify(|_, w| w.sar1_trigger_start().set_bit());
    }

    fn is_done() -> bool {
        APB_SARADC::regs().int_raw().read().sar1_done().bit_is_set()
    }

    fn read_data() -> u16 {
        (APB_SARADC::regs()
            .sar1_data_status()
            .read()
            .apb_saradc1_data()
            .bits()
            & ADC_DATA_MASK) as u16
    }

    fn reset() {
        APB_SARADC::regs()
            .int_clr()
            .write(|w| w.sar1_done().clear_bit_by_one());

        APB_SARADC::regs()
            .ctrl0()
            .modify(|_, w| unsafe { w.sar1_trigger_mode().bits(TRIGGER_MODE_OFF) });
    }
}

#[cfg(adc_adc2)]
impl RegisterAccess for crate::peripherals::ADC2<'_> {
    fn enable() {
        APB_SARADC::regs()
            .ctrl2()
            .modify(|_, w| w.timer_en().clear_bit());

        enable_refgen();

        APB_SARADC::regs().ctrl1().modify(|_, w| unsafe {
            w.xpd_sar2_force().bits(FORCE_XPD_SAR_PU);
            w.sar2_continue_mode_en().clear_bit();
            w.sar2_trigger_stop().set_bit()
        });
    }

    fn program_pattern(channel: u8) {
        let regs = APB_SARADC::regs();

        let entry = (((channel & 0xf) as u32) << 2) << 18;

        regs.ctrl1().modify(|_, w| unsafe {
            w.sar2_patt_type().set_bit();
            w.sar2_patt_len().bits(0)
        });
        regs.sar2_patt_tab1()
            .write(|w| unsafe { w.sar2_patt_tab1().bits(entry) });

        regs.ctrl1().modify(|_, w| w.sar2_patt_p_clear().set_bit());
        regs.ctrl1()
            .modify(|_, w| w.sar2_patt_p_clear().clear_bit());

        regs.ctrl1()
            .modify(|_, w| unsafe { w.sar2_trigger_mode().bits(TRIGGER_MODE_SW) });
    }

    fn start_sample() {
        APB_SARADC::regs()
            .ctrl1()
            .modify(|_, w| w.sar2_trigger_start().set_bit());
    }

    fn is_done() -> bool {
        APB_SARADC::regs().int_raw().read().sar2_done().bit_is_set()
    }

    fn read_data() -> u16 {
        (APB_SARADC::regs()
            .sar2_data_status()
            .read()
            .apb_saradc2_data()
            .bits()
            & ADC_DATA_MASK) as u16
    }

    fn reset() {
        APB_SARADC::regs()
            .int_clr()
            .write(|w| w.sar2_done().clear_bit_by_one());

        APB_SARADC::regs()
            .ctrl1()
            .modify(|_, w| unsafe { w.sar2_trigger_mode().bits(TRIGGER_MODE_OFF) });
    }
}

/// Analog-to-Digital Converter peripheral driver.
pub struct Adc<'d, ADC, Dm: crate::DriverMode> {
    _adc: ADC,
    active_channel: Option<u8>,
    _guard: GenericPeripheralGuard<{ Peripheral::ApbSarAdc as u8 }>,
    _phantom: PhantomData<(Dm, &'d mut ())>,
}

impl<'d, ADCX> Adc<'d, ADCX, Blocking>
where
    ADCX: RegisterAccess + 'd,
{
    /// Configure a given ADC instance using the provided configuration, and
    /// initialize the ADC for use
    ///
    /// The ESP32-S31 SAR ADC has a single attenuation setting, so the
    /// attenuation given per pin has no effect.
    pub fn new(adc_instance: ADCX, _config: AdcConfig<ADCX>) -> Self {
        let guard = GenericPeripheralGuard::new();

        // Select XTAL as the digital controller clock and divide it down.
        LP_PERI::regs().adc_ctrl().modify(|_, w| unsafe {
            w.lp_adc_clk_sel().bits(CLK_SRC_XTAL);
            w.lp_adc_div_num().bits(CLK_DIV_NUM)
        });

        // Function clock of the digital controller.
        APB_SARADC::regs()
            .ctrl_date()
            .modify(|_, w| w.clk_en().set_bit());

        ADCX::enable();

        Adc {
            _adc: adc_instance,
            active_channel: None,
            _guard: guard,
            _phantom: PhantomData,
        }
    }

    /// Reconfigures the ADC driver to operate in asynchronous mode.
    pub fn into_async(mut self) -> Adc<'d, ADCX, Async> {
        acquire_async_adc();
        self.set_interrupt_handler(adc_interrupt_handler);

        // Clear a stale done flag so that the first future does not complete early.
        ADCX::reset();

        Adc {
            _adc: self._adc,
            active_channel: self.active_channel,
            _guard: self._guard,
            _phantom: PhantomData,
        }
    }

    /// Start and wait for a conversion on the specified pin and return the
    /// result
    ///
    /// The result is in the range 0..=[`FULL_SCALE`], and an input tied to
    /// ground reads about [`ZERO_DIFF_CODE`].
    pub fn read_blocking<PIN, CS>(&mut self, pin: &mut AdcPin<PIN, ADCX, CS>) -> u16
    where
        PIN: AdcChannel,
        CS: AdcCalScheme<ADCX>,
    {
        ADCX::program_pattern(pin.pin.adc_channel());
        ADCX::start_sample();

        while !ADCX::is_done() {}

        let converted_value = ADCX::read_data();
        ADCX::reset();

        converted_value
    }

    /// Request that the ADC begin a conversion on the specified pin
    ///
    /// This method takes an [AdcPin](super::AdcPin) reference, as it is
    /// expected that the ADC will be able to sample whatever channel
    /// underlies the pin.
    ///
    /// The result is in the range 0..=[`FULL_SCALE`], and an input tied to
    /// ground reads about [`ZERO_DIFF_CODE`].
    pub fn read_oneshot<PIN, CS>(
        &mut self,
        pin: &mut super::AdcPin<PIN, ADCX, CS>,
    ) -> nb::Result<u16, ()>
    where
        PIN: super::AdcChannel,
        CS: super::AdcCalScheme<ADCX>,
    {
        if let Some(active_channel) = self.active_channel {
            // There is conversion in progress:
            // - if it's for a different channel try again later
            // - if it's for the given channel, go ahead and check progress
            if active_channel != pin.pin.adc_channel() {
                return Err(nb::Error::WouldBlock);
            }
        } else {
            // If no conversions are in progress, start a new one for given channel
            self.active_channel = Some(pin.pin.adc_channel());

            ADCX::program_pattern(pin.pin.adc_channel());
            ADCX::start_sample();
        }

        if !ADCX::is_done() {
            return Err(nb::Error::WouldBlock);
        }

        let converted_value = ADCX::read_data();
        ADCX::reset();

        // Mark that no conversions are currently in progress
        self.active_channel = None;

        Ok(converted_value)
    }
}

impl<ADCX> crate::private::Sealed for Adc<'_, ADCX, Blocking> {}

impl<ADCX> InterruptConfigurable for Adc<'_, ADCX, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, InterruptSource);
        }
        crate::interrupt::bind_handler(InterruptSource, handler);
    }
}

impl<'d, ADCX> Adc<'d, ADCX, Async>
where
    ADCX: RegisterAccess + 'd,
{
    /// Create a new instance in [crate::Blocking] mode.
    pub fn into_blocking(self) -> Adc<'d, ADCX, Blocking> {
        if release_async_adc() {
            // Disable the ADC interrupt on all cores once the last async instance goes away.
            for cpu in crate::system::Cpu::all() {
                crate::interrupt::disable(cpu, InterruptSource);
            }
        }
        Adc {
            _adc: self._adc,
            active_channel: self.active_channel,
            _guard: self._guard,
            _phantom: PhantomData,
        }
    }

    /// Start a conversion on the specified pin and wait for the result.
    ///
    /// This method takes an [AdcPin](super::AdcPin) reference, as it is
    /// expected that the ADC will be able to sample whatever channel
    /// underlies the pin.
    ///
    /// The result is in the range 0..=[`FULL_SCALE`], and an input tied to
    /// ground reads about [`ZERO_DIFF_CODE`].
    pub async fn read_oneshot<PIN, CS>(&mut self, pin: &mut super::AdcPin<PIN, ADCX, CS>) -> u16
    where
        ADCX: Instance,
        PIN: super::AdcChannel,
        CS: super::AdcCalScheme<ADCX>,
    {
        ADCX::program_pattern(pin.pin.adc_channel());
        ADCX::start_sample();

        AdcFuture::<ADCX>::new(self).await;

        let converted_value = ADCX::read_data();
        ADCX::reset();

        converted_value
    }
}

static ASYNC_ADC_COUNT: AtomicU32 = AtomicU32::new(0);

fn acquire_async_adc() {
    ASYNC_ADC_COUNT.fetch_add(1, Ordering::Relaxed);
}

fn release_async_adc() -> bool {
    ASYNC_ADC_COUNT.fetch_sub(1, Ordering::Relaxed) == 1
}

#[handler]
pub(crate) fn adc_interrupt_handler() {
    let interrupt_status = APB_SARADC::regs().int_st().read();

    #[cfg(adc_adc1)]
    if interrupt_status.sar1_done().bit_is_set() {
        unsafe { handle_async(crate::peripherals::ADC1::steal()) }
    }

    #[cfg(adc_adc2)]
    if interrupt_status.sar2_done().bit_is_set() {
        unsafe { handle_async(crate::peripherals::ADC2::steal()) }
    }
}

fn handle_async<ADCX: Instance>(_instance: ADCX) {
    ADCX::waker().wake();
    ADCX::unlisten();
}

/// Enable asynchronous access.
pub trait Instance: crate::private::Sealed {
    /// Enable the ADC interrupt
    fn listen();

    /// Disable the ADC interrupt
    fn unlisten();

    /// Clear the ADC interrupt
    fn clear_interrupt();

    /// Obtain the waker for the ADC interrupt
    fn waker() -> &'static AtomicWaker;
}

#[cfg(adc_adc1)]
impl Instance for crate::peripherals::ADC1<'_> {
    fn listen() {
        APB_SARADC::regs()
            .int_ena()
            .modify(|_, w| w.sar1_done().set_bit());
    }

    fn unlisten() {
        APB_SARADC::regs()
            .int_ena()
            .modify(|_, w| w.sar1_done().clear_bit());
    }

    fn clear_interrupt() {
        APB_SARADC::regs()
            .int_clr()
            .write(|w| w.sar1_done().clear_bit_by_one());
    }

    fn waker() -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();

        &WAKER
    }
}

#[cfg(adc_adc2)]
impl Instance for crate::peripherals::ADC2<'_> {
    fn listen() {
        APB_SARADC::regs()
            .int_ena()
            .modify(|_, w| w.sar2_done().set_bit());
    }

    fn unlisten() {
        APB_SARADC::regs()
            .int_ena()
            .modify(|_, w| w.sar2_done().clear_bit());
    }

    fn clear_interrupt() {
        APB_SARADC::regs()
            .int_clr()
            .write(|w| w.sar2_done().clear_bit_by_one());
    }

    fn waker() -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();

        &WAKER
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct AdcFuture<ADCX: Instance> {
    phantom: PhantomData<ADCX>,
    _wake_lock: WakeLock,
}

impl<ADCX: Instance> AdcFuture<ADCX> {
    pub fn new(_self: &super::Adc<'_, ADCX, Async>) -> Self {
        Self {
            phantom: PhantomData,
            _wake_lock: WakeLock::new(),
        }
    }
}

impl<ADCX: Instance + super::RegisterAccess> core::future::Future for AdcFuture<ADCX> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if ADCX::is_done() {
            ADCX::clear_interrupt();
            Poll::Ready(())
        } else {
            ADCX::waker().register(cx.waker());
            ADCX::listen();
            Poll::Pending
        }
    }
}

impl<ADCX: Instance> Drop for AdcFuture<ADCX> {
    fn drop(&mut self) {
        ADCX::unlisten();
    }
}
