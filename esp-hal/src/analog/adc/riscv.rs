use core::marker::PhantomData;

cfg_if::cfg_if! {
    if #[cfg(esp32c6)] {
        use Interrupt::APB_SARADC as InterruptSource;
    } else {
        use Interrupt::APB_ADC as InterruptSource;
    }
}

#[cfg(not(esp32h2))]
pub use self::calibration::*;
use super::{AdcCalSource, AdcConfig, Attenuation};
#[cfg(any(esp32c2, esp32c3, esp32c6))]
use crate::efuse::Efuse;
use crate::{
    analog::adc::asynch::AdcFuture,
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripheral::PeripheralRef,
    peripherals::{Interrupt, APB_SARADC},
    soc::regi2c,
    system::{GenericPeripheralGuard, Peripheral},
    Async,
    Blocking,
};

mod calibration;

// Constants taken from:
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32c2/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32c3/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32c6/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32h2/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32h4/include/soc/regi2c_saradc.h
cfg_if::cfg_if! {
    if #[cfg(adc1)] {
        const ADC_VAL_MASK: u16 = 0xfff;
        const ADC_CAL_CNT_MAX: u16 = 32;
        const ADC_CAL_CHANNEL: u16 = 15;
    }
}

// The number of analog IO pins, and in turn the number of attentuations,
// depends on which chip is being used
cfg_if::cfg_if! {
    if #[cfg(esp32c6)] {
        pub(super) const NUM_ATTENS: usize = 7;
    } else {
        pub(super) const NUM_ATTENS: usize = 5;
    }
}

/// The sampling/readout resolution of the ADC.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names, reason = "peripheral is unstable")]
pub enum Resolution {
    /// 12-bit resolution
    #[default]
    Resolution12Bit,
}

impl<ADCI> AdcConfig<ADCI>
where
    ADCI: RegisterAccess,
{
    /// Calibrate ADC with specified attenuation and voltage source
    pub fn adc_calibrate(atten: Attenuation, source: AdcCalSource) -> u16
    where
        ADCI: super::CalibrationAccess,
    {
        let mut adc_max: u16 = 0;
        let mut adc_min: u16 = u16::MAX;
        let mut adc_sum: u32 = 0;

        ADCI::enable_vdef(true);

        // Start sampling
        ADCI::config_onetime_sample(ADC_CAL_CHANNEL as u8, atten as u8);

        // Connect calibration source
        ADCI::connect_cal(source, true);

        for _ in 0..ADC_CAL_CNT_MAX {
            ADCI::set_init_code(0);

            // Trigger ADC sampling
            ADCI::start_onetime_sample();

            // Wait until ADC1 sampling is done
            while !ADCI::is_done() {}

            let adc = ADCI::read_data() & ADC_VAL_MASK;

            ADCI::reset();

            adc_sum += adc as u32;
            adc_max = adc.max(adc_max);
            adc_min = adc.min(adc_min);
        }

        let cal_val = (adc_sum - adc_max as u32 - adc_min as u32) as u16 / (ADC_CAL_CNT_MAX - 2);

        // Disconnect calibration source
        ADCI::connect_cal(source, false);

        cal_val
    }
}

#[doc(hidden)]
pub trait RegisterAccess {
    /// Configure onetime sampling parameters
    fn config_onetime_sample(channel: u8, attenuation: u8);

    /// Start onetime sampling
    fn start_onetime_sample();

    /// Check if sampling is done
    fn is_done() -> bool;

    /// Read sample data
    fn read_data() -> u16;

    /// Reset flags
    fn reset();

    /// Set calibration parameter to ADC hardware
    fn set_init_code(data: u16);
}

#[cfg(adc1)]
impl RegisterAccess for crate::peripherals::ADC1 {
    fn config_onetime_sample(channel: u8, attenuation: u8) {
        APB_SARADC::regs().onetime_sample().modify(|_, w| unsafe {
            w.saradc1_onetime_sample().set_bit();
            w.onetime_channel().bits(channel);
            w.onetime_atten().bits(attenuation)
        });
    }

    fn start_onetime_sample() {
        APB_SARADC::regs()
            .onetime_sample()
            .modify(|_, w| w.onetime_start().set_bit());
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

        // Disable ADC sampling
        APB_SARADC::regs()
            .onetime_sample()
            .modify(|_, w| w.onetime_start().clear_bit());
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(msb);
        regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(lsb);
    }
}

#[cfg(adc1)]
impl super::CalibrationAccess for crate::peripherals::ADC1 {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        regi2c::ADC_SAR1_DREF.write_field(enable as _);
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        match source {
            AdcCalSource::Gnd => regi2c::ADC_SAR1_ENCAL_GND.write_field(enable as _),
            #[cfg(not(esp32h2))]
            AdcCalSource::Ref => regi2c::ADC_SAR1_ENCAL_REF.write_field(enable as _),
        }
    }
}

#[cfg(adc2)]
impl RegisterAccess for crate::peripherals::ADC2 {
    fn config_onetime_sample(channel: u8, attenuation: u8) {
        APB_SARADC::regs().onetime_sample().modify(|_, w| unsafe {
            w.saradc2_onetime_sample().set_bit();
            w.onetime_channel().bits(channel);
            w.onetime_atten().bits(attenuation)
        });
    }

    fn start_onetime_sample() {
        APB_SARADC::regs()
            .onetime_sample()
            .modify(|_, w| w.onetime_start().set_bit());
    }

    fn is_done() -> bool {
        APB_SARADC::regs().int_raw().read().adc2_done().bit()
    }

    fn read_data() -> u16 {
        APB_SARADC::regs()
            .sar2data_status()
            .read()
            .saradc2_data()
            .bits() as u16
            & 0xfff
    }

    fn reset() {
        APB_SARADC::regs()
            .int_clr()
            .write(|w| w.adc2_done().clear_bit_by_one());

        APB_SARADC::regs()
            .onetime_sample()
            .modify(|_, w| w.onetime_start().clear_bit());
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(msb as _);
        regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(lsb as _);
    }
}

#[cfg(adc2)]
impl super::CalibrationAccess for crate::peripherals::ADC2 {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        regi2c::ADC_SAR2_DREF.write_field(enable as _);
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        match source {
            AdcCalSource::Gnd => regi2c::ADC_SAR2_ENCAL_GND.write_field(enable as _),
            AdcCalSource::Ref => regi2c::ADC_SAR2_ENCAL_REF.write_field(enable as _),
        }
    }
}

/// Analog-to-Digital Converter peripheral driver.
pub struct Adc<'d, ADCI, Dm: crate::DriverMode> {
    _adc: PeripheralRef<'d, ADCI>,
    attenuations: [Option<Attenuation>; NUM_ATTENS],
    active_channel: Option<u8>,
    _guard: GenericPeripheralGuard<{ Peripheral::ApbSarAdc as u8 }>,
    _phantom: PhantomData<Dm>,
}

impl<'d, ADCI> Adc<'d, ADCI, Blocking>
where
    ADCI: RegisterAccess + 'd,
{
    /// Configure a given ADC instance using the provided configuration, and
    /// initialize the ADC for use
    pub fn new(
        adc_instance: impl crate::peripheral::Peripheral<P = ADCI> + 'd,
        config: AdcConfig<ADCI>,
    ) -> Self {
        let guard = GenericPeripheralGuard::new();

        APB_SARADC::regs().ctrl().modify(|_, w| unsafe {
            w.start_force().set_bit();
            w.start().set_bit();
            w.sar_clk_gated().set_bit();
            w.xpd_sar_force().bits(0b11)
        });

        Adc {
            _adc: adc_instance.into_ref(),
            attenuations: config.attenuations,
            active_channel: None,
            _guard: guard,
            _phantom: PhantomData,
        }
    }

    /// Reconfigures the ADC driver to operate in asynchronous mode.
    pub fn into_async(mut self) -> Adc<'d, ADCI, Async> {
        asynch::acquire_async_adc();
        self.set_interrupt_handler(asynch::adc_interrupt_handler);

        // Reset interrupt flags and disable oneshot reading to normalize state before
        // entering async mode, otherwise there can be '0' readings, happening initially
        // using ADC2
        ADCI::reset();

        Adc {
            _adc: self._adc,
            attenuations: self.attenuations,
            active_channel: self.active_channel,
            _guard: self._guard,
            _phantom: PhantomData,
        }
    }

    /// Request that the ADC begin a conversion on the specified pin
    ///
    /// This method takes an [AdcPin](super::AdcPin) reference, as it is
    /// expected that the ADC will be able to sample whatever channel
    /// underlies the pin.
    pub fn read_oneshot<PIN, CS>(
        &mut self,
        pin: &mut super::AdcPin<PIN, ADCI, CS>,
    ) -> nb::Result<u16, ()>
    where
        PIN: super::AdcChannel,
        CS: super::AdcCalScheme<ADCI>,
    {
        if self.attenuations[PIN::CHANNEL as usize].is_none() {
            panic!("Channel {} is not configured reading!", PIN::CHANNEL);
        }

        if let Some(active_channel) = self.active_channel {
            // There is conversion in progress:
            // - if it's for a different channel try again later
            // - if it's for the given channel, go ahead and check progress
            if active_channel != PIN::CHANNEL {
                return Err(nb::Error::WouldBlock);
            }
        } else {
            // If no conversions are in progress, start a new one for given channel
            self.active_channel = Some(PIN::CHANNEL);

            // Set ADC unit calibration according used scheme for pin
            ADCI::set_init_code(pin.cal_scheme.adc_cal());

            let channel = self.active_channel.unwrap();
            let attenuation = self.attenuations[channel as usize].unwrap() as u8;
            ADCI::config_onetime_sample(channel, attenuation);
            ADCI::start_onetime_sample();

            // see https://github.com/espressif/esp-idf/blob/b4268c874a4cf8fcf7c0c4153cffb76ad2ddda4e/components/hal/adc_oneshot_hal.c#L105-L107
            // the delay might be a bit generous but longer delay seem to not cause problems
            #[cfg(esp32c6)]
            {
                crate::rom::ets_delay_us(40);
                ADCI::start_onetime_sample();
            }
        }

        // Wait for ADC to finish conversion
        let conversion_finished = ADCI::is_done();
        if !conversion_finished {
            return Err(nb::Error::WouldBlock);
        }

        // Get converted value
        let converted_value = ADCI::read_data();
        ADCI::reset();

        // Postprocess converted value according to calibration scheme used for pin
        let converted_value = pin.cal_scheme.adc_val(converted_value);

        // There is a hardware limitation. If the APB clock frequency is high, the step
        // of this reg signal: ``onetime_start`` may not be captured by the
        // ADC digital controller (when its clock frequency is too slow). A rough
        // estimate for this step should be at least 3 ADC digital controller
        // clock cycle.
        //
        // This limitation will be removed in hardware future versions.
        // We reset ``onetime_start`` in `reset` and assume enough time has passed until
        // the next sample is requested.

        // Mark that no conversions are currently in progress
        self.active_channel = None;

        Ok(converted_value)
    }
}

impl<ADCI> crate::private::Sealed for Adc<'_, ADCI, Blocking> {}

impl<ADCI> InterruptConfigurable for Adc<'_, ADCI, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, InterruptSource);
        }
        unsafe { crate::interrupt::bind_interrupt(InterruptSource, handler.handler()) };
        unwrap!(crate::interrupt::enable(
            InterruptSource,
            handler.priority()
        ));
    }
}

#[cfg(all(adc1, not(esp32h2)))]
impl super::AdcCalEfuse for crate::peripherals::ADC1 {
    fn init_code(atten: Attenuation) -> Option<u16> {
        Efuse::rtc_calib_init_code(1, atten)
    }

    fn cal_mv(atten: Attenuation) -> u16 {
        Efuse::rtc_calib_cal_mv(1, atten)
    }

    fn cal_code(atten: Attenuation) -> Option<u16> {
        Efuse::rtc_calib_cal_code(1, atten)
    }
}

#[cfg(adc2)]
impl super::AdcCalEfuse for crate::peripherals::ADC2 {
    fn init_code(atten: Attenuation) -> Option<u16> {
        Efuse::rtc_calib_init_code(2, atten)
    }

    fn cal_mv(atten: Attenuation) -> u16 {
        Efuse::rtc_calib_cal_mv(2, atten)
    }

    fn cal_code(atten: Attenuation) -> Option<u16> {
        Efuse::rtc_calib_cal_code(2, atten)
    }
}

#[cfg(esp32c2)]
mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (GpioPin<0>, 0),
            (GpioPin<1>, 1),
            (GpioPin<2>, 2),
            (GpioPin<3>, 3),
            (GpioPin<4>, 4),
        ]
    }
}

#[cfg(esp32c3)]
mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (GpioPin<0>, 0),
            (GpioPin<1>, 1),
            (GpioPin<2>, 2),
            (GpioPin<3>, 3),
            (GpioPin<4>, 4),
        ]
    }

    crate::analog::adc::impl_adc_interface! {
        ADC2 [
            (GpioPin<5>, 0),
        ]
    }
}

#[cfg(esp32c6)]
mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (GpioPin<0>, 0),
            (GpioPin<1>, 1),
            (GpioPin<2>, 2),
            (GpioPin<3>, 3),
            (GpioPin<4>, 4),
            (GpioPin<5>, 5),
            (GpioPin<6>, 6),
        ]
    }
}

#[cfg(esp32h2)]
mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (GpioPin<1>, 0),
            (GpioPin<2>, 1),
            (GpioPin<3>, 2),
            (GpioPin<4>, 3),
            (GpioPin<5>, 4),
        ]
    }
}

impl<'d, ADCI> Adc<'d, ADCI, Async>
where
    ADCI: RegisterAccess + 'd,
{
    /// Create a new instance in [crate::Blocking] mode.
    pub fn into_blocking(self) -> Adc<'d, ADCI, Blocking> {
        if asynch::release_async_adc() {
            // Disable ADC interrupt on all cores if the last async ADC instance is disabled
            for cpu in crate::system::Cpu::all() {
                crate::interrupt::disable(cpu, InterruptSource);
            }
        }
        Adc {
            _adc: self._adc,
            attenuations: self.attenuations,
            active_channel: self.active_channel,
            _guard: self._guard,
            _phantom: PhantomData,
        }
    }

    /// Request that the ADC begin a conversion on the specified pin
    ///
    /// This method takes an [AdcPin](super::AdcPin) reference, as it is
    /// expected that the ADC will be able to sample whatever channel
    /// underlies the pin.
    pub async fn read_oneshot<PIN, CS>(&mut self, pin: &mut super::AdcPin<PIN, ADCI, CS>) -> u16
    where
        ADCI: asynch::AsyncAccess,
        PIN: super::AdcChannel,
        CS: super::AdcCalScheme<ADCI>,
    {
        let channel = PIN::CHANNEL;
        if self.attenuations[channel as usize].is_none() {
            panic!("Channel {} is not configured reading!", channel);
        }

        // Set ADC unit calibration according used scheme for pin
        ADCI::set_init_code(pin.cal_scheme.adc_cal());

        let attenuation = self.attenuations[channel as usize].unwrap() as u8;
        ADCI::config_onetime_sample(channel, attenuation);
        ADCI::start_onetime_sample();

        // Wait for ADC to finish conversion and get value
        let adc_ready_future = AdcFuture::new(self);
        adc_ready_future.await;
        let converted_value = ADCI::read_data();

        // There is a hardware limitation. If the APB clock frequency is high, the step
        // of this reg signal: ``onetime_start`` may not be captured by the
        // ADC digital controller (when its clock frequency is too slow). A rough
        // estimate for this step should be at least 3 ADC digital controller
        // clock cycle.
        //
        // This limitation will be removed in hardware future versions.
        // We reset ``onetime_start`` in `reset` and assume enough time has passed until
        // the next sample is requested.

        ADCI::reset();

        // Postprocess converted value according to calibration scheme used for pin
        pin.cal_scheme.adc_val(converted_value)
    }
}

/// Async functionality
pub(crate) mod asynch {
    use core::{
        marker::PhantomData,
        pin::Pin,
        task::{Context, Poll},
    };

    // We only have to count on devices that have multiple ADCs sharing the same interrupt
    #[cfg(all(adc1, adc2))]
    use portable_atomic::{AtomicU32, Ordering};
    use procmacros::handler;

    use crate::{asynch::AtomicWaker, peripherals::APB_SARADC, Async};

    #[cfg(all(adc1, adc2))]
    static ASYNC_ADC_COUNT: AtomicU32 = AtomicU32::new(0);

    pub(super) fn acquire_async_adc() {
        #[cfg(all(adc1, adc2))]
        ASYNC_ADC_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    pub(super) fn release_async_adc() -> bool {
        cfg_if::cfg_if! {
            if #[cfg(all(adc1, adc2))] {
                ASYNC_ADC_COUNT.fetch_sub(1, Ordering::Relaxed) == 1
            } else {
                true
            }
        }
    }

    #[handler]
    pub(crate) fn adc_interrupt_handler() {
        let saradc = APB_SARADC::regs();
        let interrupt_status = saradc.int_st().read();

        #[cfg(adc1)]
        if interrupt_status.adc1_done().bit_is_set() {
            handle_async(crate::peripherals::ADC1)
        }

        #[cfg(adc2)]
        if interrupt_status.adc2_done().bit_is_set() {
            handle_async(crate::peripherals::ADC2)
        }
    }

    fn handle_async<ADCI: AsyncAccess>(_instance: ADCI) {
        ADCI::waker().wake();
        ADCI::disable_interrupt();
    }

    #[doc(hidden)]
    pub trait AsyncAccess {
        /// Enable the ADC interrupt
        fn enable_interrupt();

        /// Disable the ADC interrupt
        fn disable_interrupt();

        /// Clear the ADC interrupt
        fn clear_interrupt();

        /// Obtain the waker for the ADC interrupt
        fn waker() -> &'static AtomicWaker;
    }

    #[cfg(adc1)]
    impl AsyncAccess for crate::peripherals::ADC1 {
        fn enable_interrupt() {
            APB_SARADC::regs()
                .int_ena()
                .modify(|_, w| w.adc1_done().set_bit());
        }

        fn disable_interrupt() {
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

    #[cfg(adc2)]
    impl AsyncAccess for crate::peripherals::ADC2 {
        fn enable_interrupt() {
            APB_SARADC::regs()
                .int_ena()
                .modify(|_, w| w.adc2_done().set_bit());
        }

        fn disable_interrupt() {
            APB_SARADC::regs()
                .int_ena()
                .modify(|_, w| w.adc2_done().clear_bit());
        }

        fn clear_interrupt() {
            APB_SARADC::regs()
                .int_clr()
                .write(|w| w.adc2_done().clear_bit_by_one());
        }

        fn waker() -> &'static AtomicWaker {
            static WAKER: AtomicWaker = AtomicWaker::new();

            &WAKER
        }
    }

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub(crate) struct AdcFuture<ADCI: AsyncAccess> {
        phantom: PhantomData<ADCI>,
    }

    impl<ADCI: AsyncAccess> AdcFuture<ADCI> {
        pub fn new(_self: &super::Adc<'_, ADCI, Async>) -> Self {
            Self {
                phantom: PhantomData,
            }
        }
    }

    impl<ADCI: AsyncAccess + super::RegisterAccess> core::future::Future for AdcFuture<ADCI> {
        type Output = ();

        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            if ADCI::is_done() {
                ADCI::clear_interrupt();
                Poll::Ready(())
            } else {
                ADCI::waker().register(cx.waker());
                ADCI::enable_interrupt();
                Poll::Pending
            }
        }
    }

    impl<ADCI: AsyncAccess> Drop for AdcFuture<ADCI> {
        fn drop(&mut self) {
            ADCI::disable_interrupt();
        }
    }
}
