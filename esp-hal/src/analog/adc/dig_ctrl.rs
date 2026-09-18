use core::marker::PhantomData;

cfg_select! {
    any(esp32c6, esp32c61) => {
        use Interrupt::APB_SARADC as InterruptSource;
    }
    _ => {
        use Interrupt::APB_ADC as InterruptSource;
    }
}

use core::{
    pin::Pin,
    task::{Context, Poll},
};

use procmacros::handler;

pub use self::calibration::*;
use super::{AdcConfig, Attenuation};
use crate::{
    Async,
    Blocking,
    asynch::AtomicWaker,
    efuse::AdcCalibUnit,
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripherals::{APB_SARADC, Interrupt},
    rtc_cntl::WakeLock,
    soc::regi2c,
    system::{GenericPeripheralGuard, Peripheral},
};

mod calibration;

// Constants taken from:
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32c2/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32c3/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32c6/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32h2/include/soc/regi2c_saradc.h
const ADC_VAL_MASK: u16 = 0xfff;

// The number of analog IO pins, and in turn the number of attentuations,
// depends on which chip is being used
cfg_select! {
    esp32c6 => {
        pub(super) const NUM_ATTENS: usize = 7;
    }
    esp32c5 => {
        pub(super) const NUM_ATTENS: usize = 6;
    }
    esp32c61 => {
        pub(super) const NUM_ATTENS: usize = 4;
    }
    _ => {
        pub(super) const NUM_ATTENS: usize = 5;
    }
}

impl<ADCX> AdcConfig<ADCX>
where
    ADCX: RegisterAccess,
{
    /// Measures the initial code that cancels the unit's zero-voltage offset at `atten`.
    pub fn adc_calibrate(atten: Attenuation) -> u16
    where
        ADCX: super::CalibrationAccess,
    {
        // A calibration scheme measures the ADC while building itself, which happens before
        // `Adc::new` runs, so the unit cannot be assumed to be clocked and powered yet.
        init_hardware();

        ADCX::enable_vdef(true);

        // Start sampling
        ADCX::setup_calibration(atten);

        // Connect calibration source
        ADCX::connect_gnd(true);

        ADCX::calibration_init();
        let cal_val = super::search_init_code::<ADCX>(read_cal_channel::<ADCX>);
        ADCX::reset();

        // Disconnect calibration source
        ADCX::connect_gnd(false);

        cal_val
    }
}

/// Converts the connected calibration source once.
///
/// Clearing the done flag before the trigger keeps it from coming back up from a conversion that
/// was still in flight; a stale result would send the search off in the wrong direction.
///
/// The trigger goes the other way around than it does for a pad conversion: ESP-IDF puts the low
/// period up front here, holds it for a fixed five microseconds rather than the delay it computes
/// there, and leaves the level asserted. It also reads the result without waiting out
/// [`settle_after_done`] - which would make the ESP32-H2 read zero, so we wait.
///
/// See `read_cal_channel` in
/// <https://github.com/espressif/esp-idf/blob/v6.1/components/esp_hal_ana_conv/adc_hal_common.c>
fn read_cal_channel<ADCX: RegisterAccess>() -> u16 {
    ADCX::reset();

    ADCX::set_onetime_start(false);
    crate::rom::ets_delay_us(5);
    ADCX::set_onetime_start(true);

    while !ADCX::is_done() {}
    settle_after_done();

    ADCX::read_data() & ADC_VAL_MASK
}

/// Clocks and powers the SAR ADC up, the way ESP-IDF's oneshot driver does.
///
/// The SAR clock is divided down from the digital controller clock, and the calibration data in
/// eFuse was characterized at the frequency ESP-IDF picks. Programming the dividers explicitly
/// also keeps the results independent of what the bootloader left behind - it clocks the ADC
/// too, to seed the RNG.
///
/// Writes nothing that depends on previous state, so it is safe to run more than once.
///
/// See `adc_oneshot_hal_setup` and `ADC_LL_CLKM_DIV_*_DEFAULT` in
/// <https://github.com/espressif/esp-idf/blob/v6.1/components/esp_hal_ana_conv/adc_oneshot_hal.c>
fn init_hardware() {
    // controller_clk = source / (DIV_NUM + DIV_A / DIV_B + 1)
    // The ESP32-H2 divides its faster source further, landing at the same 5 MHz as the other chips.
    #[cfg(esp32h2)]
    const DIV: (u8, u8, u8) = (18, 5, 1);
    #[cfg(not(esp32h2))]
    const DIV: (u8, u8, u8) = (15, 1, 0);

    let (div_num, div_b, div_a) = DIV;

    // The ESP32-C2 and ESP32-C3 keep the divider in the ADC itself, and only support APB as the
    // source. The other chips take the PLL output via the PCR - 80 MHz, or 96 MHz on the
    // ESP32-H2.
    #[cfg(any(esp32c2, esp32c3))]
    APB_SARADC::regs().clkm_conf().modify(|_, w| unsafe {
        w.clk_sel().bits(2);
        w.clkm_div_num().bits(div_num);
        w.clkm_div_b().bits(div_b);
        w.clkm_div_a().bits(div_a)
    });

    #[cfg(not(any(esp32c2, esp32c3)))]
    {
        // The PLL is source 1 on the ESP32-C6 and ESP32-H2, but source 2 on the ESP32-C5 and
        // ESP32-C61 - there, 1 selects the imprecise RC_FAST oscillator instead.
        #[cfg(any(esp32c5, esp32c61))]
        const PLL: u8 = 2;
        #[cfg(not(any(esp32c5, esp32c61)))]
        const PLL: u8 = 1;

        // The peripheral guard only enables the bus clock (`saradc_reg_clk_en`); the
        // controller runs off a separate function clock that has to be enabled here.
        // See `adc_ll_enable_func_clock`.
        crate::peripherals::PCR::regs()
            .saradc_clkm_conf()
            .modify(|_, w| unsafe {
                w.saradc_clkm_en().set_bit();
                w.saradc_clkm_sel().bits(PLL);
                w.saradc_clkm_div_num().bits(div_num);
                w.saradc_clkm_div_b().bits(div_b);
                w.saradc_clkm_div_a().bits(div_a)
            });
    }

    APB_SARADC::regs().ctrl().modify(|_, w| unsafe {
        // Conversions are triggered through `onetime_start`, so the software start of the
        // digital controller has to stay off - a concurrently running controller disturbs the
        // sampling and biases the results low.
        w.start_force().clear_bit();
        w.start().clear_bit();
        w.sar_clk_gated().set_bit();
        // Run the SAR at the digital controller clock, like ESP-IDF does.
        #[cfg(not(any(esp32c5, esp32c61, esp32h2)))]
        w.sar_clk_div().bits(1);
        w.xpd_sar_force().bits(0b11)
    });

    // The ESP32-C5, ESP32-C61 and ESP32-H2 moved that divider into the PCR; `APB_SARADC_CTRL` keeps
    // a field of the same name that the hardware no longer uses. Writing only that one leaves the
    // SAR at the divider of 15 the bootloader's RNG seeding leaves behind - its teardown restores
    // the controller divider but not this one.
    //
    // See `adc_ll_digi_set_clk_div`.
    #[cfg(any(esp32c5, esp32c61, esp32h2))]
    crate::peripherals::PCR::regs()
        .sar_clk_div()
        .modify(|_, w| unsafe { w.sar1_clk_div_num().bits(1) });
}

/// Triggers a single conversion.
///
/// The controller converts on the step of `onetime_start`, so one rising edge per conversion is
/// all it needs. The level stays asserted until `reset` clears it after the read - the way
/// ESP-IDF's calibration read does it. Pulsing the bit like ESP-IDF's pad-conversion path does
/// risks the digital controller missing the step: it needs three of its own clock cycles to
/// capture the level, and at the 5 MHz `init_hardware` configures, a pulse only two register
/// writes wide is never seen on the ESP32-H2.
///
/// Matching ESP-IDF's pulse exactly - assert, wait 3 us, de-assert, wait 3 us - was measured on
/// the ESP32-C3 to roughly double the gain error against ESP-IDF rather than remove it, so the
/// remaining difference is not the trigger shape.
///
/// See `adc_hal_onetime_start` and `read_cal_channel` in
/// <https://github.com/espressif/esp-idf/blob/v6.1/components/esp_hal_ana_conv/adc_oneshot_hal.c>
fn start_onetime_sample<ADCX: RegisterAccess>() {
    ADCX::set_onetime_start(true);
}

/// Waits for the conversion result to become readable after the done flag has come up.
///
/// The ESP32-H2 needs five SAR clock cycles here or it reports zero. `init_hardware` leaves the SAR
/// at the 5 MHz digital controller clock, which makes that one microsecond per cycle.
///
/// See `ADC_LL_DELAY_CYCLE_AFTER_DONE_SIGNAL`, which is zero on every other chip here.
#[inline]
fn settle_after_done() {
    #[cfg(esp32h2)]
    crate::rom::ets_delay_us(5);
}

#[doc(hidden)]
pub trait RegisterAccess {
    /// Configures one-time sampling parameters.
    fn config_onetime_sample(channel: u8, attenuation: u8);

    /// Drives the `onetime_start` trigger level.
    ///
    /// Callers go through [`start_onetime_sample`], which raises it once per conversion; `reset`
    /// lowers it again after the read.
    fn set_onetime_start(enable: bool);

    /// Returns whether sampling is done.
    fn is_done() -> bool;

    /// Reads sample data.
    fn read_data() -> u16;

    /// Resets flags.
    fn reset();

    /// Sets up ADC hardware for calibration.
    fn calibration_init();

    /// Sets calibration parameter to ADC hardware.
    fn set_init_code(data: u16);
}

impl RegisterAccess for crate::peripherals::ADC1<'_> {
    fn config_onetime_sample(channel: u8, attenuation: u8) {
        APB_SARADC::regs().onetime_sample().modify(|_, w| unsafe {
            // The channel, the attenuation and the trigger are one set of fields that the arm
            // bits steer at one SAR or the other, so an ADC2 left armed by the bootloader - it
            // samples the ADC to seed the RNG - would convert alongside this one and load the
            // bias both SARs share. That only applies to the ESP32-C3, the one chip here whose
            // ADC2 exists in silicon, and no driver arms it because it is not exposed - but
            // disarm it anyway, the way ESP-IDF does before every conversion.
            //
            // See `adc_oneshot_ll_disable_all_unit`.
            w.saradc2_onetime_sample().clear_bit();
            w.saradc1_onetime_sample().set_bit();
            w.onetime_channel().bits(channel);
            w.onetime_atten().bits(attenuation)
        });
    }

    fn set_onetime_start(enable: bool) {
        APB_SARADC::regs()
            .onetime_sample()
            .modify(|_, w| w.onetime_start().bit(enable));
    }

    fn is_done() -> bool {
        APB_SARADC::regs().int_raw().read().adc1_done().bit()
    }

    fn read_data() -> u16 {
        APB_SARADC::regs()
            .sar1data_status()
            .read()
            .saradc1_data()
            .bits() as u16
            & 0xfff
    }

    fn reset() {
        // Clear ADC1 sampling done interrupt bit
        APB_SARADC::regs()
            .int_clr()
            .write(|w| w.adc1_done().clear_bit_by_one());

        // Disarm both units along with the trigger. Leaving a unit armed keeps its SAR selected
        // for one-time sampling between conversions; ESP-IDF disarms after every result it reads.
        //
        // See `adc_oneshot_ll_disable_all_unit`.
        APB_SARADC::regs().onetime_sample().modify(|_, w| {
            w.onetime_start().clear_bit();
            w.saradc2_onetime_sample().clear_bit();
            w.saradc1_onetime_sample().clear_bit()
        });
    }

    fn calibration_init() {
        // e.g.
        // https://github.com/espressif/esp-idf/blob/800f141f94c0f880c162de476512e183df671307/components/hal/esp32c3/include/hal/adc_ll.h#L702
        regi2c::ADC_SAR1_DREF.write_field(1);
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(msb);
        regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(lsb);
    }
}

impl super::CalibrationAccess for crate::peripherals::ADC1<'_> {
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;
    const SUPPORTS_SELF_CALIBRATION: bool = !cfg!(esp32c5);

    fn enable_vdef(enable: bool) {
        regi2c::ADC_SAR1_DREF.write_field(enable as _);
    }

    fn connect_gnd(enable: bool) {
        regi2c::ADC_SAR1_ENCAL_GND.write_field(enable as _);
    }

    fn setup_calibration(atten: Attenuation) {
        // `adc_oneshot_ll_disable_channel` deselects every pad by writing
        // `(unit << 3) | 0xf` to the channel field. The attenuation is shared by all
        // channels on these chips, so programming it is enough.
        Self::config_onetime_sample(0xf, atten as u8);
    }
}

/// Analog-to-Digital Converter peripheral driver.
pub struct Adc<'d, ADCX, Dm: crate::DriverMode> {
    _adc: ADCX,
    attenuations: [Option<Attenuation>; NUM_ATTENS],
    /// Hardware calibration code per attenuation, indexed by [`Attenuation`].
    init_codes: [u16; super::ATTENUATION_COUNT],
    active_channel: Option<u8>,
    _guard: GenericPeripheralGuard<{ Peripheral::ApbSarAdc as u8 }>,
    _phantom: PhantomData<(Dm, &'d mut ())>,
}

impl<'d, ADCX> Adc<'d, ADCX, Blocking>
where
    ADCX: RegisterAccess + 'd,
{
    /// Configures a given ADC instance using the provided configuration, and
    /// initializes the ADC for use.
    pub fn new(adc_instance: ADCX, config: AdcConfig<ADCX>) -> Self
    where
        ADCX: super::AdcCalEfuse + super::CalibrationAccess,
    {
        let guard = GenericPeripheralGuard::new();

        init_hardware();

        let init_codes = super::hw_init_codes::<ADCX>(&config.attenuations);

        // The bootloader runs conversions of its own to seed the RNG, so the done flag can
        // already be set here and would make the first read return a stale result.
        ADCX::reset();

        Adc {
            _adc: adc_instance,
            attenuations: config.attenuations,
            init_codes,
            active_channel: None,
            _guard: guard,
            _phantom: PhantomData,
        }
    }

    /// Reconfigures the ADC driver to operate in asynchronous mode.
    pub fn into_async(mut self) -> Adc<'d, ADCX, Async> {
        self.set_interrupt_handler(adc_interrupt_handler);

        // Reset interrupt flags and disable oneshot reading to normalize state before
        // entering async mode, otherwise there can be '0' readings, happening initially
        // using ADC2
        ADCX::reset();

        Adc {
            _adc: self._adc,
            attenuations: self.attenuations,
            init_codes: self.init_codes,
            active_channel: self.active_channel,
            _guard: self._guard,
            _phantom: PhantomData,
        }
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
        let channel = pin.pin.adc_channel();
        let attenuation = super::channel_attenuation(&self.attenuations, channel);

        if let Some(active_channel) = self.active_channel {
            // There is conversion in progress:
            // - if it's for a different channel try again later
            // - if it's for the given channel, go ahead and check progress
            if active_channel != channel {
                return Err(nb::Error::WouldBlock);
            }
        } else {
            // If no conversions are in progress, start a new one for given channel
            self.active_channel = Some(channel);

            ADCX::calibration_init();
            ADCX::set_init_code(self.init_codes[attenuation as usize]);

            ADCX::config_onetime_sample(channel, attenuation as u8);
            start_onetime_sample::<ADCX>();
        }

        // Wait for ADC to finish conversion
        let conversion_finished = ADCX::is_done();
        if !conversion_finished {
            return Err(nb::Error::WouldBlock);
        }
        settle_after_done();

        // Get converted value
        let converted_value = ADCX::read_data();
        ADCX::reset();

        // Postprocess converted value according to calibration scheme used for pin
        let converted_value = pin.cal_scheme.adc_val(converted_value);

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

    #[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2))]
    fn cal_chan_compens(atten: Attenuation, channel: u8) -> Option<i32> {
        crate::efuse::rtc_calib_get_chan_compens(AdcCalibUnit::ADC1, channel, atten)
    }
}

impl<'d, ADCX> Adc<'d, ADCX, Async>
where
    ADCX: RegisterAccess + 'd,
{
    /// Reconfigures the ADC driver to operate in [`Blocking`] mode.
    pub fn into_blocking(self) -> Adc<'d, ADCX, Blocking> {
        // Every chip handled by this driver has a single ADC unit, so no other driver can be
        // sharing the interrupt and it is always safe to disable it here.
        for cpu in crate::system::Cpu::all() {
            crate::interrupt::disable(cpu, InterruptSource);
        }
        Adc {
            _adc: self._adc,
            attenuations: self.attenuations,
            init_codes: self.init_codes,
            active_channel: self.active_channel,
            _guard: self._guard,
            _phantom: PhantomData,
        }
    }

    /// Requests that the ADC begin a conversion on the specified pin.
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
        let channel = pin.pin.adc_channel();
        let attenuation = super::channel_attenuation(&self.attenuations, channel);

        ADCX::calibration_init();
        ADCX::set_init_code(self.init_codes[attenuation as usize]);

        ADCX::config_onetime_sample(channel, attenuation as u8);
        start_onetime_sample::<ADCX>();

        // Wait for ADC to finish conversion and get value
        let adc_ready_future = AdcFuture::new(self);
        adc_ready_future.await;
        settle_after_done();

        let converted_value = ADCX::read_data();

        ADCX::reset();

        // Postprocess converted value according to calibration scheme used for pin
        pin.cal_scheme.adc_val(converted_value)
    }
}

#[handler]
pub(crate) fn adc_interrupt_handler() {
    let saradc = APB_SARADC::regs();
    let interrupt_status = saradc.int_st().read();

    if interrupt_status.adc1_done().bit_is_set() {
        unsafe { handle_async(crate::peripherals::ADC1::steal()) }
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

impl Instance for crate::peripherals::ADC1<'_> {
    fn listen() {
        APB_SARADC::regs()
            .int_ena()
            .modify(|_, w| w.adc1_done().set_bit());
    }

    fn unlisten() {
        APB_SARADC::regs()
            .int_ena()
            .modify(|_, w| w.adc1_done().clear_bit());
    }

    fn clear_interrupt() {
        APB_SARADC::regs()
            .int_clr()
            .write(|w| w.adc1_done().clear_bit_by_one());
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
