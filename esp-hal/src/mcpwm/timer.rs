#![cfg_attr(docsrs, procmacros::doc_replace)]

//! # MCPWM Timer Module
//!
//! ## Overview
//! The `timer` module provides an interface to configure and use timers for
//! generating `PWM` signals used in motor control and other applications.
//!
//! The `timer` module allows the configuration of setting the sync in source
//! for the timer, and the ability to get their sync out to given to other timers
//! as their sync in.
//!
//! ### Software Sync Events
//! The `timer` module supports the software triggering of syncs from Timers.
#![cfg_attr(
    soc_has_mcpwm_swsync_can_propagate,
    doc = "**Note:** Software triggered sync events from timers will propagate to their respective sync outputs on this chip."
)]
#![cfg_attr(
    not(soc_has_mcpwm_swsync_can_propagate),
    doc = "**Note:** Software triggered sync events do not propagate to other timers on this chip."
)]

use core::marker::PhantomData;

use enumset::{EnumSet, EnumSetType};

use super::PeripheralGuard;
use crate::{
    mcpwm::{
        Event,
        FrequencyError,
        Instance,
        PeripheralClockConfig,
        PwmClockGuard,
        sync::{SyncInSelect, SyncOut, SyncSource},
    },
    pac,
    time::Rate,
};

#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum TimerEvent {
    /// Event for when a timer stops
    TimerStop,
    /// Event for when a timer equals zero
    TimerEqualZero,
    /// Event for when a timer equals period
    TimerEqualPeriod,
}

impl Into<Event> for TimerEvent {
    fn into(self) -> Event {
        match self {
            TimerEvent::TimerEqualPeriod => Event::TimerEqualPeriod,
            TimerEvent::TimerEqualZero => Event::TimerEqualZero,
            TimerEvent::TimerStop => Event::TimerStop,
        }
    }
}

/// A MCPWM timer
///
/// Every timer of a particular [`MCPWM`](super::McPwm) peripheral can be used
/// as a timing reference for every
/// [`Operator`](super::operator::Operator) of that peripheral
pub struct Timer<'d, const TIM: u8, PWM: Instance> {
    /// Sync out for the timer that can be connected to other timers sync in or operators sync in
    pub sync_out: SyncOut<'d, TIM, PWM>,
    _phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
    config: TimerClockConfig,
}

impl<'d, const TIM: u8, PWM: Instance> Timer<'d, TIM, PWM> {
    pub(super) fn new(guard: PeripheralGuard, peripheral_clock: &PeripheralClockConfig) -> Self {
        // Default configuration for the timer
        let config = TimerClockConfig::with_prescaler(
            peripheral_clock,
            u16::MAX,
            PwmWorkingMode::Increase,
            0,
        );

        let mut timer = Timer {
            sync_out: SyncOut::new(),
            _phantom: PhantomData,
            _guard: guard,
            _pwm_clock_guard: PwmClockGuard::new::<PWM>(),
            config,
        };
        timer.configure();

        timer
    }

    /// Start the given timer with the provided configurationn
    pub fn start(&mut self) {
        // set timer to run with a stop condition
        let stop_condition = self.config.stop_condition as u8;
        let mode = self.config.mode as u8;
        self.cfg1().write(|w| unsafe {
            w.start().bits(stop_condition);
            w.mod_().bits(mode)
        });
    }

    /// Stop the timer in its current state
    pub fn stop(&mut self) {
        // freeze the timer
        self.cfg1().write(|w| unsafe { w.mod_().bits(0) });
    }

    /// ### Note:
    /// * The prescaler configuration will be applied immediately.
    /// * The period configuration will be applied based on the [`PeriodUpdatingMethod`] set in the
    ///   [`TimerClockConfig`].
    ///
    /// If the timer is already running you might want to call [`Timer::stop`] first.
    ///
    /// If your [`PeriodUpdatingMethod`] is set to [`PeriodUpdatingMethod::Immediately`]
    /// and if your new period is larger than the current counter value as this will cause weird
    /// behavior.
    pub fn set_config(&mut self, config: TimerClockConfig) {
        self.config = config;
        self.configure();
    }

    /// Set the timer counter to the provided value
    ///
    /// ## Overview
    /// Internally we set the timers phase and direction
    /// Then trigger a software sync event.
    #[cfg(soc_has_mcpwm_swsync_can_propagate)]
    /// **Note** This will cause the timer to fire a sync out event to other timers
    pub fn set_counter(&mut self, phase: u16, direction: CounterDirection) {
        self.set_sync_phase(phase);
        self.set_sync_counter_direction(direction);
        self.trigger_sync();
    }

    /// Trigger a software sync event
    #[cfg(soc_has_mcpwm_swsync_can_propagate)]
    /// **Note** This will cause the timer to fire a sync out event to other timers
    pub fn trigger_sync(&mut self) {
        // SAFETY: Only TIMERx_SYNC accessed; unique per TIM const
        let tmr = unsafe { Self::tmr() };
        tmr.sync().modify(|r, w| w.sw().bit(!r.sw().bit()));
    }

    /// Read the counter value and counter direction of the timer
    pub fn status(&self) -> (u16, CounterDirection) {
        // SAFETY: Only read TIMERx_STATUS; unique per TIM const
        let reg = unsafe { Self::tmr() }.status().read();
        (reg.value().bits(), reg.direction().bit_is_set().into())
    }

    /// Sets the capture timers sync source. Refer to how sync events are
    /// handled in the [`Timer`] documentation.
    pub fn set_sync_in(&mut self, sync_source: &impl SyncSource<PWM>) {
        let sync_in_sel = sync_source.get_kind().into();
        self.set_sync_in_select(sync_in_sel);
    }

    /// Clears the sync in for the timer and disables sync input
    pub fn clear_sync_in(&mut self) {
        self.set_sync_in_select(SyncInSelect::None);
    }

    #[instability::unstable]
    pub fn listen(&mut self, events: EnumSet<TimerEvent>) {
        self.enable_listen(events, true);
    }

    #[instability::unstable]
    pub fn unlisten(&mut self, events: EnumSet<TimerEvent>) {
        self.enable_listen(events, false);
    }

    #[instability::unstable]
    pub fn interrupts(&self) -> EnumSet<TimerEvent> {
        let mut res = EnumSet::new();
        let (info, _) = PWM::split();

        let ints = info.regs().int_st().read();
        if ints.timer_stop(TIM).bit() {
            res.insert(TimerEvent::TimerStop);
        }
        if ints.timer_tep(TIM).bit() {
            res.insert(TimerEvent::TimerEqualPeriod);
        }
        if ints.timer_tez(TIM).bit() {
            res.insert(TimerEvent::TimerEqualZero);
        }

        res
    }

    #[instability::unstable]
    pub fn clear_interrupts(&mut self, events: EnumSet<TimerEvent>) {
        let (info, _) = PWM::split();
        info.regs().int_clr().write(|w| {
            for event in events {
                match event {
                    TimerEvent::TimerStop => w.timer_stop(TIM).bit(true),
                    TimerEvent::TimerEqualPeriod => w.timer_tep(TIM).bit(true),
                    TimerEvent::TimerEqualZero => w.timer_tez(TIM).bit(true),
                };
            }
            w
        });
    }

    /// Update period of the timer on the fly. How the new period is applied depends on the
    /// [`PeriodUpdatingMethod`] set in the [`TimerClockConfig`].
    #[doc(hidden)]
    pub fn update_period(&mut self, new_period: u16) {
        self.cfg0()
            .modify(|_r, w| unsafe { w.period().bits(new_period) });
    }

    fn configure(&mut self) {
        // write prescaler and period
        let (prescaler, period, period_updating_method) = (
            self.config.prescaler,
            self.config.period,
            self.config.period_updating_method as u8,
        );
        self.cfg0().write(|w| unsafe {
            w.prescale().bits(prescaler);
            w.period().bits(period);
            w.period_upmethod().bits(period_updating_method as u8)
        });

        // write sync configure
        self.set_sync_counter_direction(self.config.sync_direction);
        self.set_sync_out_selection(self.config.sync_out);
        self.set_sync_phase(self.config.sync_phase);
    }

    fn set_sync_phase(&mut self, sync_phase: u16) {
        // SAFETY: Only TIMERx_SYNC accessed; unique per TIM const
        let tmr = unsafe { Self::tmr() };
        tmr.sync()
            .modify(|_r, w| unsafe { w.phase().bits(sync_phase) });
    }

    fn set_sync_counter_direction(&mut self, direction: CounterDirection) {
        // SAFETY: Only TIMERx_SYNC accessed; unique per TIM const
        let tmr = unsafe { Self::tmr() };
        tmr.sync()
            .modify(|_r, w| w.phase_direction().bit(direction as u8 != 0));
    }

    fn set_sync_out_selection(&mut self, sync_out: SyncOutSelect) {
        // SAFETY: Only TIMERx_SYNC accessed; unique per TIM const
        let timer = unsafe { Self::tmr() };
        timer
            .sync()
            .modify(|_r, w| unsafe { w.synco_sel().bits(sync_out as u8) });
    }

    fn set_sync_in_select(&mut self, sync_sel: SyncInSelect) {
        let (info, _) = PWM::split();

        // SAFETY: Only TIMER_SYNCI_CFG and TIMERx_SYNC accessed
        info.regs()
            .timer_synci_cfg()
            .modify(|_r, w| unsafe { w.timer_syncisel(TIM).bits(sync_sel as u8) });
        self.sync()
            .modify(|_r, w| w.synci_en().bit(sync_sel != SyncInSelect::None));
    }

    fn enable_listen(&mut self, events: EnumSet<TimerEvent>, value: bool) {
        let (info, state) = PWM::split();
        let mut int_events = EnumSet::new();
        for timer_event in events {
            int_events.insert(timer_event.into());
        }

        state.enable_listen::<TIM>(info, int_events, value);
    }

    fn cfg0(&mut self) -> &pac::mcpwm0::timer::CFG0 {
        // SAFETY: Unique register access per TIM
        unsafe { Self::tmr() }.cfg0()
    }

    fn cfg1(&mut self) -> &pac::mcpwm0::timer::CFG1 {
        // SAFETY: Unique register access per TIM
        unsafe { Self::tmr() }.cfg1()
    }

    fn sync(&mut self) -> &pac::mcpwm0::timer::SYNC {
        // SAFETY: Unique register access per TIM
        unsafe { Self::tmr() }.sync()
    }

    // Marked unsafe as the caller must ensure that only one timer
    // is accessing the registers for a given TIM const
    unsafe fn tmr() -> &'static pac::mcpwm0::TIMER {
        let (info, _) = PWM::split();
        info.regs().timer(TIM as usize)
    }
}

/// Sync out selection for the timer
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SyncOutSelect {
    /// Sync out is triggered when a timer receives a sync in
    SyncIn              = 0,
    /// Sync out is triggered when the timer equals zero
    SyncWhenEqualZero   = 1,
    /// Sync out is triggered when the timer equals the period
    SyncWhenEqualPeriod = 2,
}

/// Clock configuration of a MCPWM timer
///
/// Use [`PeripheralClockConfig::timer_clock_with_prescaler`](super::PeripheralClockConfig::timer_clock_with_prescaler) or
/// [`PeripheralClockConfig::timer_clock_with_frequency`](super::PeripheralClockConfig::timer_clock_with_frequency) to it.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimerClockConfig {
    frequency: Rate,
    period: u16,
    period_updating_method: PeriodUpdatingMethod,
    stop_condition: StopCondition,
    prescaler: u8,
    mode: PwmWorkingMode,
    sync_out: SyncOutSelect,
    sync_phase: u16,
    sync_direction: CounterDirection,
}

impl TimerClockConfig {
    pub(super) fn with_prescaler(
        clock: &PeripheralClockConfig,
        period: u16,
        mode: PwmWorkingMode,
        prescaler: u8,
    ) -> Self {
        let cycle_period = match mode {
            PwmWorkingMode::Increase | PwmWorkingMode::Decrease => period as u32 + 1,
            // The reference manual seems to provide an incorrect formula for UpDown
            PwmWorkingMode::UpDown => period as u32 * 2,
        };
        let frequency = clock.frequency / (prescaler as u32 + 1) / cycle_period;

        Self {
            frequency,
            prescaler,
            period,
            period_updating_method: PeriodUpdatingMethod::Immediately,
            stop_condition: StopCondition::RunContinuously,
            mode,
            sync_out: SyncOutSelect::SyncIn,
            sync_phase: 0,
            sync_direction: CounterDirection::Increasing,
        }
    }

    pub(super) fn with_frequency(
        clock: &PeripheralClockConfig,
        period: u16,
        mode: PwmWorkingMode,
        target_freq: Rate,
    ) -> Result<Self, FrequencyError> {
        let cycle_period = match mode {
            PwmWorkingMode::Increase | PwmWorkingMode::Decrease => period as u32 + 1,
            // The reference manual seems to provide an incorrect formula for UpDown
            PwmWorkingMode::UpDown => period as u32 * 2,
        };
        let target_timer_frequency = target_freq
            .as_hz()
            .checked_mul(cycle_period)
            .ok_or(FrequencyError)?;
        if target_timer_frequency == 0 || target_freq > clock.frequency {
            return Err(FrequencyError);
        }
        let prescaler = clock.frequency.as_hz() / target_timer_frequency - 1;
        if prescaler > u8::MAX as u32 {
            return Err(FrequencyError);
        }
        let frequency = clock.frequency / (prescaler + 1) / cycle_period;

        Ok(Self {
            frequency,
            prescaler: prescaler as u8,
            period,
            period_updating_method: PeriodUpdatingMethod::Immediately,
            stop_condition: StopCondition::RunContinuously,
            mode,
            sync_out: SyncOutSelect::SyncIn,
            sync_phase: 0,
            sync_direction: CounterDirection::Increasing,
        })
    }

    /// Set the method for updating the PWM period
    pub fn with_period_updating_method(self, method: PeriodUpdatingMethod) -> Self {
        Self {
            period_updating_method: method,
            ..self
        }
    }

    /// Sets the stop timer conditions
    pub fn with_stop_conditions(self, condition: StopCondition) -> Self {
        Self {
            stop_condition: condition,
            ..self
        }
    }

    /// Set sync out selection
    pub fn with_sync_out_selection(self, sync_out: SyncOutSelect) -> Self {
        Self { sync_out, ..self }
    }

    /// Sets the counter direction when the timer receives a sync event in up-down mode
    pub fn with_sync_direction(self, direction: CounterDirection) -> Self {
        Self {
            sync_direction: direction,
            ..self
        }
    }

    /// Sets the sync phase for the timer
    pub fn with_sync_phase(self, sync_phase: u16) -> Self {
        Self { sync_phase, ..self }
    }

    /// Get the timer clock frequency.
    ///
    /// ### Note:
    /// The actual value is rounded down to the nearest `u32` value
    pub fn frequency(&self) -> Rate {
        self.frequency
    }
}

/// Method for updating the PWM period
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PeriodUpdatingMethod {
    /// The period is updated immediately.
    Immediately           = 0,
    /// The period is updated when the timer equals zero.
    TimerEqualsZero       = 1,
    /// The period is updated on a synchronization event.
    Sync                  = 2,
    /// The period is updated either when the timer equals zero or on a
    /// synchronization event.
    TimerEqualsZeroOrSync = 3,
}

/// Method for stop conditions for timers
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum StopCondition {
    /// Defines to start the timer now and run till [`Timer::stop`] is called.
    RunContinuously = 2,
    /// Defines to start the timer now and run till the
    /// next time the timers counts equal zeros
    StopAtZero      = 3,
    /// Defines to start the timer now and run till the
    /// next time the timers counts equals period
    StopAtPeriod    = 4,
}

/// PWM working mode
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PwmWorkingMode {
    /// In this mode, the PWM timer increments from zero until reaching the
    /// value configured in the period field. Once done, the PWM timer
    /// returns to zero and starts increasing again. PWM period is equal to the
    /// value of the period field + 1.
    Increase = 1,
    /// The PWM timer decrements to zero, starting from the value configured in
    /// the period field. After reaching zero, it is set back to the period
    /// value. Then it starts to decrement again. In this case, the PWM period
    /// is also equal to the value of period field + 1.
    Decrease = 2,
    /// This is a combination of the two modes mentioned above. The PWM timer
    /// starts increasing from zero until the period value is reached. Then,
    /// the timer decreases back to zero. This pattern is then repeated. The
    /// PWM period is the result of the value of the period field × 2.
    UpDown   = 3,
}

/// The direction the timer counter is changing
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum CounterDirection {
    /// The timer counter is increasing
    Increasing = 0,
    /// The timer counter is decreasing
    Decreasing = 1,
}

impl From<bool> for CounterDirection {
    fn from(bit: bool) -> Self {
        match bit {
            false => CounterDirection::Increasing,
            true => CounterDirection::Decreasing,
        }
    }
}
