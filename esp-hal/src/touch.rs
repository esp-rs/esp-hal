//! # Capacitive Touch Sensor
//!
//! ## Overview
//!
//! The touch sensor peripheral allows for cheap and robust user interfaces by
//! e.g., dedicating a part of the pcb as touch button.
//!
//! ## Examples
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::touch::{Touch, TouchPad};
//! let touch_pin0 = peripherals.GPIO2;
//! let touch = Touch::continuous_mode(peripherals.TOUCH, None);
//! let mut touchpad = TouchPad::new(touch_pin0, &touch);
//! // ... give the peripheral some time for the measurement
//! let touch_val = touchpad.read();
//! # Ok(())
//! # }
//! ```
//! 
//! ## Implementation State:
//!
//! Mostly feature complete, missing:
//!
//! - Touch sensor slope control
//! - Deep Sleep support (wakeup from Deep Sleep)

use core::marker::PhantomData;

use crate::{
    gpio::TouchPin,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{LPWR, SENS, TOUCH},
    private::{Internal, Sealed},
    rtc_cntl::Rtc,
    Async,
    Blocking,
    DriverMode,
};

/// A marker trait describing the mode the touch pad is set to.
pub trait TouchMode: Sealed {}

/// Marker struct for the touch peripherals manual trigger mode. In the
/// technical reference manual, this is referred to as "start FSM via software".
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OneShot;

/// Marker struct for the touch peripherals continuous reading mode. In the
/// technical reference manual, this is referred to as "start FSM via timer".
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Continuous;

impl TouchMode for OneShot {}
impl TouchMode for Continuous {}
impl Sealed for OneShot {}
impl Sealed for Continuous {}

/// Touchpad threshold type.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ThresholdMode {
    /// Pad is considered touched if value is greater than threshold.
    GreaterThan,
    /// Pad is considered touched if value is less than threshold.
    LessThan,
}

/// Configurations for the touch pad driver
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchConfig {
    /// The [`ThresholdMode`] for the pads. Defaults to
    /// `ThresholdMode::LessThan`
    pub threshold_mode: Option<ThresholdMode>,
    /// Duration of a single measurement (in cycles of the 8 MHz touch clock).
    /// Defaults to `0x7fff`
    pub measurement_duration: Option<u16>,
    /// Sleep cycles for the touch timer in [`Continuous`]-mode. Defaults to
    /// `0x100`
    pub sleep_cycles: Option<u16>,
}

/// This struct marks a successfully initialized touch peripheral
pub struct Touch<'d, Tm: TouchMode, Dm: DriverMode> {
    _inner: PeripheralRef<'d, TOUCH>,
    _touch_mode: PhantomData<Tm>,
    _mode: PhantomData<Dm>,
}
impl<Tm: TouchMode, Dm: DriverMode> Touch<'_, Tm, Dm> {
    /// Common initialization of the touch peripheral.
    fn initialize_common(config: Option<TouchConfig>) {
        let rtccntl = LPWR::regs();
        let sens = SENS::regs();

        let mut threshold_mode = false;
        let mut meas_dur = 0x7fff;

        if let Some(config) = config {
            threshold_mode = match config.threshold_mode {
                Some(ThresholdMode::LessThan) => false,
                Some(ThresholdMode::GreaterThan) => true,
                None => false,
            };
            if let Some(dur) = config.measurement_duration {
                meas_dur = dur;
            }
        }

        // stop touch fsm
        rtccntl
            .state0()
            .write(|w| w.touch_slp_timer_en().clear_bit());
        // Disable touch interrupt
        rtccntl.int_ena().write(|w| w.touch().clear_bit());
        // Clear pending interrupts
        rtccntl.int_clr().write(|w| w.touch().bit(true));

        // Disable all interrupts and touch pads
        sens.sar_touch_enable().write(|w| unsafe {
            w.touch_pad_outen1()
                .bits(0b0)
                .touch_pad_outen2()
                .bits(0b0)
                .touch_pad_worken()
                .bits(0b0)
        });

        sens.sar_touch_ctrl1().write(|w| unsafe {
            w
                // Default to trigger when touch is below threshold
                .touch_out_sel()
                .bit(threshold_mode)
                // Interrupt only on set 1
                .touch_out_1en()
                .set_bit()
                .touch_meas_delay()
                .bits(meas_dur)
                // TODO Chip Specific
                .touch_xpd_wait()
                .bits(0xff)
        });
    }

    /// Common parts of the continuous mode initialization.
    fn initialize_common_continuous(config: Option<TouchConfig>) {
        let rtccntl = LPWR::regs();
        let sens = SENS::regs();

        // Default nr of sleep cycles from IDF
        let mut sleep_cyc = 0x1000;
        if let Some(config) = config {
            if let Some(slp) = config.sleep_cycles {
                sleep_cyc = slp;
            }
        }

        Self::initialize_common(config);

        sens.sar_touch_ctrl2().write(|w| unsafe {
            w
                // Reset existing touch measurements
                .touch_meas_en_clr()
                .set_bit()
                .touch_sleep_cycles()
                .bits(sleep_cyc)
                // Configure FSM for timer mode
                .touch_start_fsm_en()
                .set_bit()
                .touch_start_force()
                .clear_bit()
        });

        // start touch fsm
        rtccntl.state0().write(|w| w.touch_slp_timer_en().set_bit());
    }
}
// Async mode and OneShot does not seem to be a sensible combination....
impl<'d> Touch<'d, OneShot, Blocking> {
    /// Initializes the touch peripheral and returns this marker struct.
    /// Optionally accepts configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let touch_cfg = Some(TouchConfig {
    ///     measurement_duration: Some(0x2000),
    ///     ..Default::default()
    /// });
    /// let touch = Touch::one_shot_mode(peripherals.TOUCH, touch_cfg);
    /// # Ok(())
    /// # }
    /// ```
    pub fn one_shot_mode(
        touch_peripheral: impl Peripheral<P = TOUCH> + 'd,
        config: Option<TouchConfig>,
    ) -> Self {
        crate::into_ref!(touch_peripheral);
        let rtccntl = LPWR::regs();
        let sens = SENS::regs();

        // Default nr of sleep cycles from IDF
        let mut sleep_cyc = 0x1000;
        if let Some(config) = config {
            if let Some(slp) = config.sleep_cycles {
                sleep_cyc = slp;
            }
        }

        Self::initialize_common(config);

        sens.sar_touch_ctrl2().write(|w| unsafe {
            w
                // Reset existing touch measurements
                .touch_meas_en_clr()
                .set_bit()
                .touch_sleep_cycles()
                .bits(sleep_cyc)
                // Configure FSM for SW mode
                .touch_start_fsm_en()
                .set_bit()
                .touch_start_en()
                .clear_bit()
                .touch_start_force()
                .set_bit()
        });

        // start touch fsm
        rtccntl.state0().write(|w| w.touch_slp_timer_en().set_bit());

        Self {
            _inner: touch_peripheral,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }
}
impl<'d> Touch<'d, Continuous, Blocking> {
    /// Initializes the touch peripheral in continuous mode and returns this
    /// marker struct. Optionally accepts configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let touch_cfg = Some(TouchConfig {
    ///     measurement_duration: Some(0x3000),
    ///     ..Default::default()
    /// });
    /// let touch = Touch::continuous_mode(peripherals.TOUCH, touch_cfg);
    /// # Ok(())
    /// # }
    /// ```
    pub fn continuous_mode(
        touch_peripheral: impl Peripheral<P = TOUCH> + 'd,
        config: Option<TouchConfig>,
    ) -> Self {
        crate::into_ref!(touch_peripheral);

        Self::initialize_common_continuous(config);

        Self {
            _inner: touch_peripheral,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }
}
impl<'d> Touch<'d, Continuous, Async> {
    /// Initializes the touch peripheral in continuous async mode and returns
    /// this marker struct.
    ///
    /// ## Warning:
    ///
    /// This uses [`RTC_CORE`](crate::peripherals::Interrupt::RTC_CORE)
    /// interrupts under the hood. So the whole async part breaks if you install
    /// an interrupt handler with [`Rtc::set_interrupt_handler()`][1].
    ///
    /// [1]: ../rtc_cntl/struct.Rtc.html#method.set_interrupt_handler
    ///
    /// ## Parameters:
    ///
    /// - `rtc`: The RTC peripheral is needed to configure the required
    ///   interrupts.
    /// - `config`: Optional configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::rtc_cntl::Rtc;
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let mut rtc = Rtc::new(peripherals.LPWR);
    /// let touch = Touch::async_mode(peripherals.TOUCH, &mut rtc, None);
    /// # Ok(())
    /// # }
    /// ```
    pub fn async_mode(
        touch_peripheral: impl Peripheral<P = TOUCH> + 'd,
        rtc: &mut Rtc<'_>,
        config: Option<TouchConfig>,
    ) -> Self {
        crate::into_ref!(touch_peripheral);

        Self::initialize_common_continuous(config);

        rtc.set_interrupt_handler(asynch::handle_touch_interrupt);

        Self {
            _inner: touch_peripheral,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }
}

/// A pin that is configured as a TouchPad.
pub struct TouchPad<P: TouchPin, Tm: TouchMode, Dm: DriverMode> {
    pin: P,
    _touch_mode: PhantomData<Tm>,
    _mode: PhantomData<Dm>,
}
impl<P: TouchPin> TouchPad<P, OneShot, Blocking> {
    /// (Re-)Start a touch measurement on the pin. You can get the result by
    /// calling [`read`](Self::read) once it is finished.
    pub fn start_measurement(&mut self) {
        crate::peripherals::RTC_IO::regs()
            .touch_pad2()
            .write(|w| unsafe {
                w.start().set_bit();
                w.xpd().set_bit();
                // clear input_enable
                w.fun_ie().clear_bit();
                // Connect pin to analog / RTC module instead of standard GPIO
                w.mux_sel().set_bit();
                // Disable pull-up and pull-down resistors on the pin
                w.rue().clear_bit();
                w.rde().clear_bit();
                w.tie_opt().clear_bit();
                w.to_gpio().set_bit();
                // Select function "RTC function 1" (GPIO) for analog use
                w.fun_sel().bits(0b00)
            });

        crate::peripherals::SENS::regs()
            .sar_touch_ctrl2()
            .modify(|_, w| w.touch_start_en().clear_bit());
        crate::peripherals::SENS::regs()
            .sar_touch_ctrl2()
            .modify(|_, w| w.touch_start_en().set_bit());
    }
}
impl<P: TouchPin, Tm: TouchMode, Dm: DriverMode> TouchPad<P, Tm, Dm> {
    /// Construct a new instance of [`TouchPad`].
    ///
    /// ## Parameters:
    /// - `pin`: The pin that gets configured as touch pad
    /// - `touch`: The [`Touch`] struct indicating that touch is configured.
    pub fn new(pin: P, _touch: &Touch<'_, Tm, Dm>) -> Self {
        // TODO revert this on drop
        pin.set_touch(Internal);

        Self {
            pin,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }

    /// Read the current touch pad capacitance counter.
    ///
    /// Usually a lower value means higher capacitance, thus indicating touch
    /// event.
    ///
    /// Returns `None` if the value is not yet ready. (Note: Measurement must be
    /// started manually with [`start_measurement`](Self::start_measurement) if
    /// the touch peripheral is in [`OneShot`] mode).
    pub fn try_read(&mut self) -> Option<u16> {
        if unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_ctrl2()
            .read()
            .touch_meas_done()
            .bit_is_set()
        {
            Some(self.pin.touch_measurement(Internal))
        } else {
            None
        }
    }
}
impl<P: TouchPin, Tm: TouchMode> TouchPad<P, Tm, Blocking> {
    /// Blocking read of the current touch pad capacitance counter.
    ///
    /// Usually a lower value means higher capacitance, thus indicating touch
    /// event.
    ///
    /// ## Note for [`OneShot`] mode:
    ///
    /// This function might block forever, if
    /// [`start_measurement`](Self::start_measurement) was not called before. As
    /// measurements are not cleared, the touch values might also be
    /// outdated, if it has been some time since the last call to that
    /// function.
    pub fn read(&mut self) -> u16 {
        while unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_ctrl2()
            .read()
            .touch_meas_done()
            .bit_is_clear()
        {}
        self.pin.touch_measurement(Internal)
    }

    /// Enables the touch_pad interrupt.
    ///
    /// The raised interrupt is actually
    /// [`RTC_CORE`](crate::peripherals::Interrupt::RTC_CORE). A handler can
    /// be installed with [`Rtc::set_interrupt_handler()`][1].
    ///
    /// [1]: ../rtc_cntl/struct.Rtc.html#method.set_interrupt_handler
    ///
    /// ## Parameters:
    /// - `threshold`: The threshold above/below which the pin is considered
    ///   touched. Above/below depends on the configuration of `touch` in
    ///   [`new`](Self::new) (defaults to below).
    ///
    /// ## Example
    pub fn enable_interrupt(&mut self, threshold: u16) {
        self.pin.set_threshold(threshold, Internal);
        internal_enable_interrupt(self.pin.touch_nr(Internal))
    }

    /// Disables the touch pad's interrupt.
    ///
    /// If no other touch pad interrupts are active, the touch interrupt is
    /// disabled completely.
    pub fn disable_interrupt(&mut self) {
        internal_disable_interrupt(self.pin.touch_nr(Internal))
    }

    /// Clears a pending touch interrupt.
    ///
    /// ## Note on interrupt clearing behaviour:
    ///
    /// There is only a single interrupt for the touch pad.
    /// [`is_interrupt_set`](Self::is_interrupt_set) can be used to check
    /// which pins are touchted. However, this function clears the interrupt
    /// status for all pins. So only call it when all pins are handled.
    pub fn clear_interrupt(&mut self) {
        internal_clear_interrupt()
    }

    /// Checks if the pad is touched, based on the configured threshold value.
    pub fn is_interrupt_set(&mut self) -> bool {
        internal_is_interrupt_set(self.pin.touch_nr(Internal))
    }
}

fn internal_enable_interrupt(touch_nr: u8) {
    // enable touch interrupts
    LPWR::regs().int_ena().write(|w| w.touch().set_bit());

    SENS::regs().sar_touch_enable().modify(|r, w| unsafe {
        w.touch_pad_outen1()
            .bits(r.touch_pad_outen1().bits() | 1 << touch_nr)
    });
}

fn internal_disable_interrupt(touch_nr: u8) {
    SENS::regs().sar_touch_enable().modify(|r, w| unsafe {
        w.touch_pad_outen1()
            .bits(r.touch_pad_outen1().bits() & !(1 << touch_nr))
    });
    if SENS::regs()
        .sar_touch_enable()
        .read()
        .touch_pad_outen1()
        .bits()
        == 0
    {
        LPWR::regs().int_ena().write(|w| w.touch().clear_bit());
    }
}

fn internal_disable_interrupts() {
    SENS::regs()
        .sar_touch_enable()
        .write(|w| unsafe { w.touch_pad_outen1().bits(0) });
    if SENS::regs()
        .sar_touch_enable()
        .read()
        .touch_pad_outen1()
        .bits()
        == 0
    {
        LPWR::regs().int_ena().write(|w| w.touch().clear_bit());
    }
}

fn internal_clear_interrupt() {
    LPWR::regs()
        .int_clr()
        .write(|w| w.touch().clear_bit_by_one());
    SENS::regs()
        .sar_touch_ctrl2()
        .write(|w| w.touch_meas_en_clr().set_bit());
}

fn internal_pins_touched() -> u16 {
    // Only god knows, why the "interrupt flag" register is called "meas_en" on this
    // chip...
    SENS::regs().sar_touch_ctrl2().read().touch_meas_en().bits()
}

fn internal_is_interrupt_set(touch_nr: u8) -> bool {
    internal_pins_touched() & (1 << touch_nr) != 0
}

mod asynch {
    use core::{
        sync::atomic::{AtomicU16, Ordering},
        task::{Context, Poll},
    };

    use super::*;
    use crate::{asynch::AtomicWaker, handler, ram, Async};

    const NUM_TOUCH_PINS: usize = 10;

    static TOUCH_WAKERS: [AtomicWaker; NUM_TOUCH_PINS] =
        [const { AtomicWaker::new() }; NUM_TOUCH_PINS];

    // Helper variable to store which pins need handling.
    static TOUCHED_PINS: AtomicU16 = AtomicU16::new(0);

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub struct TouchFuture {
        touch_nr: u8,
    }

    impl TouchFuture {
        pub fn new(touch_nr: u8) -> Self {
            Self { touch_nr }
        }
    }

    impl core::future::Future for TouchFuture {
        type Output = ();

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            TOUCH_WAKERS[self.touch_nr as usize].register(cx.waker());

            let pins = TOUCHED_PINS.load(Ordering::Acquire);

            if pins & (1 << self.touch_nr) != 0 {
                // clear the pin to signal that this pin was handled.
                TOUCHED_PINS.fetch_and(!(1 << self.touch_nr), Ordering::Release);
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    #[handler]
    #[ram]
    pub(super) fn handle_touch_interrupt() {
        let touch_pads = internal_pins_touched();
        for (i, waker) in TOUCH_WAKERS.iter().enumerate() {
            if touch_pads & (1 << i) != 0 {
                waker.wake();
            }
        }
        TOUCHED_PINS.store(touch_pads, Ordering::Relaxed);
        internal_clear_interrupt();
        internal_disable_interrupts();
    }

    impl<P: TouchPin, Tm: TouchMode> TouchPad<P, Tm, Async> {
        /// Wait for the pad to be touched.
        pub async fn wait_for_touch(&mut self, threshold: u16) {
            self.pin.set_threshold(threshold, Internal);
            let touch_nr = self.pin.touch_nr(Internal);
            internal_enable_interrupt(touch_nr);
            TouchFuture::new(touch_nr).await;
        }
    }
}
