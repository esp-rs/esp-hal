use core::{
    marker::PhantomData,
    pin::Pin,
    task::{Context, Poll},
};

// One-shot ADC is handled by the LP_ADC peripheral.
use Interrupt::LP_ADC as InterruptSource;
// Both units share one interrupt, so the enabled instances have to be counted.
use portable_atomic::{AtomicU32, Ordering};
use procmacros::handler;

pub use self::calibration::*;
use super::{AdcCalScheme, AdcCalSource, AdcChannel, AdcConfig, AdcPin, Attenuation};
use crate::{
    Async,
    Blocking,
    asynch::AtomicWaker,
    efuse::AdcCalibUnit,
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripherals::{Interrupt, LP_ADC, LP_PERI},
    rtc_cntl::WakeLock,
    soc::regi2c,
    system::{GenericPeripheralGuard, Peripheral},
};

mod calibration;

/// ADC1 covers channels 0..=7, ADC2 covers channels 0..=5. The attenuation
/// table is indexed by channel, so it has to hold the larger of the two.
pub(super) const NUM_ATTENS: usize = 8;

const ADC_VAL_MASK: u16 = 0xfff;
const ADC_CAL_CNT_MAX: u16 = 32;
const ADC_CAL_CHANNEL: u16 = 15;

/// Powers the SAR up by software, instead of leaving it to the FSM.
const FORCE_XPD_SAR_PU: u8 = 3;

impl<ADCX> AdcConfig<ADCX>
where
    ADCX: RegisterAccess,
{
    /// Calibrates ADC with specified attenuation and voltage source.
    pub fn adc_calibrate(atten: Attenuation, source: AdcCalSource) -> u16
    where
        ADCX: super::CalibrationAccess,
    {
        let mut adc_max: u16 = 0;
        let mut adc_min: u16 = u16::MAX;
        let mut adc_sum: u32 = 0;

        ADCX::enable_vdef(true);

        // Start sampling
        ADCX::set_en_pad(ADCX::ADC_CAL_CHANNEL as u8);
        ADCX::set_attenuation(ADCX::ADC_CAL_CHANNEL as usize, atten as u8);

        // Connect calibration source
        ADCX::connect_cal(source, true);

        ADCX::calibration_init();
        ADCX::set_init_code(0);

        for _ in 0..ADCX::ADC_CAL_CNT_MAX {
            // Trigger ADC sampling
            ADCX::start_sample();

            // Wait until ADC sampling is done
            while !ADCX::is_done() {}

            let adc = ADCX::read_data() & ADCX::ADC_VAL_MASK;

            ADCX::reset();

            adc_sum += adc as u32;
            adc_max = adc.max(adc_max);
            adc_min = adc.min(adc_min);
        }

        let cal_val =
            (adc_sum - adc_max as u32 - adc_min as u32) as u16 / (ADCX::ADC_CAL_CNT_MAX - 2);

        // Disconnect calibration source
        ADCX::connect_cal(source, false);

        cal_val
    }
}

#[doc(hidden)]
pub trait RegisterAccess {
    fn set_attenuation(channel: usize, attenuation: u8);

    /// Routes the unit to the RTC controller, driven by software.
    fn set_rtc_controller();

    fn set_en_pad(channel: u8);

    fn clear_start_sample();

    fn start_sample();

    /// Returns whether sampling is done.
    fn is_done() -> bool;

    /// Reads sample data.
    fn read_data() -> u16;

    /// Powers the SAR up.
    fn power_up();

    /// Sets up ADC hardware for calibration.
    fn calibration_init();

    /// Sets calibration parameter to ADC hardware.
    fn set_init_code(data: u16);

    /// Resets flags.
    fn reset();
}

#[cfg(adc_adc1)]
impl RegisterAccess for crate::peripherals::ADC1<'_> {
    fn set_attenuation(channel: usize, attenuation: u8) {
        LP_ADC::regs().atten1().modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation & 0b11) as u32) << (channel * 2));

            unsafe { w.sar1_atten().bits(new_value) }
        });
    }

    fn set_rtc_controller() {
        LP_ADC::regs()
            .meas1_mux()
            .modify(|_, w| w.sar1_dig_force().clear_bit());
        LP_ADC::regs().meas1_ctrl2().modify(|_, w| {
            w.meas1_start_force().set_bit();
            w.sar1_en_pad_force().set_bit()
        });
    }

    fn set_en_pad(channel: u8) {
        LP_ADC::regs()
            .meas1_ctrl2()
            .modify(|_, w| unsafe { w.sar1_en_pad().bits(1 << channel) });
    }

    fn clear_start_sample() {
        LP_ADC::regs()
            .meas1_ctrl2()
            .modify(|_, w| w.meas1_start_sar().clear_bit());
    }

    fn start_sample() {
        LP_ADC::regs()
            .meas1_ctrl2()
            .modify(|_, w| w.meas1_start_sar().set_bit());
    }

    fn is_done() -> bool {
        LP_ADC::regs()
            .meas1_ctrl2()
            .read()
            .meas1_done_sar()
            .bit_is_set()
    }

    fn read_data() -> u16 {
        LP_ADC::regs().meas1_ctrl2().read().meas1_data_sar().bits()
    }

    fn power_up() {
        LP_ADC::regs()
            .force_wpd_sar()
            .modify(|_, w| unsafe { w.force_xpd_sar1().bits(FORCE_XPD_SAR_PU) });
    }

    fn calibration_init() {
        // https://github.com/espressif/esp-idf/blob/08e0d30a74a/components/esp_hal_ana_conv/esp32p4/include/hal/adc_ll.h#L727
        regi2c::ADC_SAR1_DREF.write_field(4);
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(msb);
        regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(lsb);
    }

    fn reset() {
        // The conversion-done interrupt latches even when it is masked, so clear it
        // here to keep a later `into_async` from seeing a stale completion.
        LP_ADC::regs()
            .int_clr()
            .write(|w| w.cocpu_saradc1_int_clr().set_bit());

        LP_ADC::regs()
            .meas1_ctrl2()
            .modify(|_, w| w.meas1_start_sar().clear_bit());
    }
}

#[cfg(adc_adc1)]
impl super::CalibrationAccess for crate::peripherals::ADC1<'_> {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        regi2c::ADC_SAR1_DREF.write_field(enable as u8);
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        match source {
            AdcCalSource::Gnd => regi2c::ADC_SAR1_ENCAL_GND.write_field(enable as u8),
            AdcCalSource::Ref => regi2c::ADC_SAR1_ENCAL_REF.write_field(enable as u8),
        }
    }
}

/// ADC2 channels are wired to SAR pads 2..=7, so the pad index is the channel
/// number plus two.
///
/// See `ADC_LL_UNIT2_CHANNEL_SUBSTRATION` in
/// `components/esp_hal_ana_conv/esp32p4/include/hal/adc_ll.h`
#[cfg(adc_adc2)]
const ADC2_PAD_OFFSET: u8 = 2;

#[cfg(adc_adc2)]
impl RegisterAccess for crate::peripherals::ADC2<'_> {
    fn set_attenuation(channel: usize, attenuation: u8) {
        let pad = channel + ADC2_PAD_OFFSET as usize;
        LP_ADC::regs().atten2().modify(|r, w| {
            let new_value =
                (r.bits() & !(0b11 << (pad * 2))) | (((attenuation & 0b11) as u32) << (pad * 2));

            unsafe { w.sar2_atten().bits(new_value) }
        });
    }

    fn set_rtc_controller() {
        LP_ADC::regs()
            .meas2_mux()
            .modify(|_, w| w.sar2_rtc_force().set_bit());
        LP_ADC::regs().meas2_ctrl2().modify(|_, w| {
            w.meas2_start_force().set_bit();
            w.sar2_en_pad_force().set_bit()
        });
    }

    fn set_en_pad(channel: u8) {
        LP_ADC::regs()
            .meas2_ctrl2()
            .modify(|_, w| unsafe { w.sar2_en_pad().bits(1 << (channel + ADC2_PAD_OFFSET)) });
    }

    fn clear_start_sample() {
        LP_ADC::regs()
            .meas2_ctrl2()
            .modify(|_, w| w.meas2_start_sar().clear_bit());
    }

    fn start_sample() {
        LP_ADC::regs()
            .meas2_ctrl2()
            .modify(|_, w| w.meas2_start_sar().set_bit());
    }

    fn is_done() -> bool {
        LP_ADC::regs()
            .meas2_ctrl2()
            .read()
            .meas2_done_sar()
            .bit_is_set()
    }

    fn read_data() -> u16 {
        LP_ADC::regs().meas2_ctrl2().read().meas2_data_sar().bits()
    }

    fn power_up() {
        LP_ADC::regs()
            .force_wpd_sar()
            .modify(|_, w| unsafe { w.force_xpd_sar2().bits(FORCE_XPD_SAR_PU) });
    }

    fn calibration_init() {
        regi2c::ADC_SAR2_DREF.write_field(4);
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(msb);
        regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(lsb);
    }

    fn reset() {
        // The conversion-done interrupt latches even when it is masked, so clear it
        // here to keep a later `into_async` from seeing a stale completion.
        LP_ADC::regs()
            .int_clr()
            .write(|w| w.cocpu_saradc2_int_clr().set_bit());

        LP_ADC::regs()
            .meas2_ctrl2()
            .modify(|_, w| w.meas2_start_sar().clear_bit());
    }
}

#[cfg(adc_adc2)]
impl super::CalibrationAccess for crate::peripherals::ADC2<'_> {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        regi2c::ADC_SAR2_DREF.write_field(enable as u8);
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        match source {
            AdcCalSource::Gnd => regi2c::ADC_SAR2_ENCAL_GND.write_field(enable as u8),
            AdcCalSource::Ref => regi2c::ADC_SAR2_ENCAL_REF.write_field(enable as u8),
        }
    }
}

#[cfg(adc_adc1)]
impl super::AdcCalEfuse for crate::peripherals::ADC1<'_> {
    fn init_code(atten: Attenuation) -> Option<u16> {
        crate::efuse::rtc_calib_init_code(AdcCalibUnit::ADC1, atten)
    }

    fn cal_mv(atten: Attenuation) -> u16 {
        crate::efuse::rtc_calib_cal_mv(AdcCalibUnit::ADC1, atten)
    }

    fn cal_code(atten: Attenuation) -> Option<u16> {
        crate::efuse::rtc_calib_cal_code(AdcCalibUnit::ADC1, atten)
    }
}

#[cfg(adc_adc2)]
impl super::AdcCalEfuse for crate::peripherals::ADC2<'_> {
    fn init_code(atten: Attenuation) -> Option<u16> {
        crate::efuse::rtc_calib_init_code(AdcCalibUnit::ADC2, atten)
    }

    fn cal_mv(atten: Attenuation) -> u16 {
        crate::efuse::rtc_calib_cal_mv(AdcCalibUnit::ADC2, atten)
    }

    fn cal_code(atten: Attenuation) -> Option<u16> {
        crate::efuse::rtc_calib_cal_code(AdcCalibUnit::ADC2, atten)
    }
}

/// Analog-to-Digital Converter peripheral driver.
pub struct Adc<'d, ADC, Dm: crate::DriverMode> {
    _adc: ADC,
    active_channel: Option<u8>,
    last_init_code: u16,
    _guard: GenericPeripheralGuard<{ Peripheral::ApbSarAdc as u8 }>,
    _phantom: PhantomData<(Dm, &'d mut ())>,
}

impl<ADCX, Dm> Adc<'_, ADCX, Dm>
where
    ADCX: RegisterAccess,
    Dm: crate::DriverMode,
{
    fn start_sample<PIN, CS>(&mut self, pin: &mut AdcPin<PIN, ADCX, CS>)
    where
        PIN: AdcChannel,
        CS: AdcCalScheme<ADCX>,
    {
        // Set ADC unit calibration according used scheme for pin
        let init_code = pin.cal_scheme.adc_cal();
        if self.last_init_code != init_code {
            ADCX::calibration_init();
            ADCX::set_init_code(init_code);
            self.last_init_code = init_code;
        }

        ADCX::set_en_pad(pin.pin.adc_channel());

        ADCX::clear_start_sample();
        ADCX::start_sample();
    }
}

impl<'d, ADCX> Adc<'d, ADCX, Blocking>
where
    ADCX: RegisterAccess + 'd,
{
    /// Configures a given ADC instance using the provided configuration, and
    /// initializes the ADC for use.
    pub fn new(adc_instance: ADCX, config: AdcConfig<ADCX>) -> Self {
        let guard = GenericPeripheralGuard::new();

        // The RTC controller lives in the LP domain and has its own clock gate.
        LP_PERI::regs()
            .clk_en()
            .modify(|_, w| w.ck_en_lp_adc().set_bit());

        for (channel, attenuation) in config.attenuations.iter().enumerate() {
            if let Some(attenuation) = attenuation {
                ADCX::set_attenuation(channel, *attenuation as u8);
            }
        }

        ADCX::set_rtc_controller();
        ADCX::power_up();

        Adc {
            _adc: adc_instance,
            active_channel: None,
            last_init_code: 0,
            _guard: guard,
            _phantom: PhantomData,
        }
    }

    /// Starts and waits for a conversion on the specified pin and returns the
    /// result.
    pub fn read_blocking<PIN, CS>(&mut self, pin: &mut AdcPin<PIN, ADCX, CS>) -> u16
    where
        PIN: AdcChannel,
        CS: AdcCalScheme<ADCX>,
    {
        self.start_sample(pin);

        // Wait for ADC to finish conversion
        while !ADCX::is_done() {}

        // Get converted value
        let converted_value = ADCX::read_data();
        ADCX::reset();

        // Postprocess converted value according to calibration scheme used for pin
        pin.cal_scheme.adc_val(converted_value)
    }

    /// Requests that the ADC begin a conversion on the specified pin.
    ///
    /// Takes an [`AdcPin`](super::AdcPin) reference, as it is
    /// expected that the ADC will be able to sample whatever channel
    /// underlies the pin.
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

            self.start_sample(pin);
        }

        // Wait for ADC to finish conversion
        let conversion_finished = ADCX::is_done();
        if !conversion_finished {
            return Err(nb::Error::WouldBlock);
        }

        // Get converted value
        let converted_value = ADCX::read_data();
        ADCX::reset();

        // Postprocess converted value according to calibration scheme used for pin
        let converted_value = pin.cal_scheme.adc_val(converted_value);

        // Mark that no conversions are currently in progress
        self.active_channel = None;

        Ok(converted_value)
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
            last_init_code: self.last_init_code,
            _guard: self._guard,
            _phantom: PhantomData,
        }
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
    /// Reconfigures the ADC driver to operate in [`Blocking`] mode.
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
            last_init_code: self.last_init_code,
            _guard: self._guard,
            _phantom: PhantomData,
        }
    }

    /// Starts a conversion on the specified pin and waits for the result.
    ///
    /// Takes an [`AdcPin`](super::AdcPin) reference, as it is
    /// expected that the ADC will be able to sample whatever channel
    /// underlies the pin.
    pub async fn read_oneshot<PIN, CS>(&mut self, pin: &mut super::AdcPin<PIN, ADCX, CS>) -> u16
    where
        ADCX: Instance,
        PIN: super::AdcChannel,
        CS: super::AdcCalScheme<ADCX>,
    {
        self.start_sample(pin);

        AdcFuture::<ADCX>::new(self).await;

        let converted_value = ADCX::read_data();
        ADCX::reset();

        // Postprocess converted value according to calibration scheme used for pin
        pin.cal_scheme.adc_val(converted_value)
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
    let interrupt_status = LP_ADC::regs().int_st().read();

    #[cfg(adc_adc1)]
    if interrupt_status.cocpu_saradc1_int_st().bit_is_set() {
        unsafe { handle_async(crate::peripherals::ADC1::steal()) }
    }

    #[cfg(adc_adc2)]
    if interrupt_status.cocpu_saradc2_int_st().bit_is_set() {
        unsafe { handle_async(crate::peripherals::ADC2::steal()) }
    }
}

fn handle_async<ADCX: Instance>(_instance: ADCX) {
    ADCX::waker().wake();
    ADCX::unlisten();
}

/// Enables asynchronous access.
pub trait Instance: crate::private::Sealed {
    /// Enables the ADC interrupt.
    fn listen();

    /// Disables the ADC interrupt.
    fn unlisten();

    /// Clears the ADC interrupt.
    fn clear_interrupt();

    /// Obtains the waker for the ADC interrupt.
    fn waker() -> &'static AtomicWaker;
}

#[cfg(adc_adc1)]
impl Instance for crate::peripherals::ADC1<'_> {
    fn listen() {
        LP_ADC::regs()
            .int_ena_w1ts()
            .write(|w| w.cocpu_saradc1_int_ena_w1ts().set_bit());
    }

    fn unlisten() {
        LP_ADC::regs()
            .int_ena_w1tc()
            .write(|w| w.cocpu_saradc1_int_ena_w1tc().set_bit());
    }

    fn clear_interrupt() {
        LP_ADC::regs()
            .int_clr()
            .write(|w| w.cocpu_saradc1_int_clr().set_bit());
    }

    fn waker() -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();

        &WAKER
    }
}

#[cfg(adc_adc2)]
impl Instance for crate::peripherals::ADC2<'_> {
    fn listen() {
        LP_ADC::regs()
            .int_ena_w1ts()
            .write(|w| w.cocpu_saradc2_int_ena_w1ts().set_bit());
    }

    fn unlisten() {
        LP_ADC::regs()
            .int_ena_w1tc()
            .write(|w| w.cocpu_saradc2_int_ena_w1tc().set_bit());
    }

    fn clear_interrupt() {
        LP_ADC::regs()
            .int_clr()
            .write(|w| w.cocpu_saradc2_int_clr().set_bit());
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
        ADCX::listen();
        Self {
            phantom: PhantomData,
            _wake_lock: WakeLock::new(),
        }
    }
}

impl<ADCX: Instance + super::RegisterAccess> core::future::Future for AdcFuture<ADCX> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        ADCX::waker().register(cx.waker());
        if ADCX::is_done() {
            ADCX::clear_interrupt();
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl<ADCX: Instance> Drop for AdcFuture<ADCX> {
    fn drop(&mut self) {
        ADCX::unlisten();
    }
}
