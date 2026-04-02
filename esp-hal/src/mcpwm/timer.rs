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
        sync::{SyncOut, SyncSelection, SyncSource},
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
    _phantom: PhantomData<&'d PWM>,
    _guard: PeripheralGuard,
    _pwm_clock_guard: PwmClockGuard,
}

impl<'d, const TIM: u8, PWM: Instance> Timer<'d, TIM, PWM> {
    pub(super) fn new(guard: PeripheralGuard) -> Self {
        Timer {
            phantom: PhantomData,
            _guard: guard,
            _pwm_clock_guard: PwmClockGuard::new::<PWM>(),
        }
    }

    /// Apply the given timer configuration.
    ///
    /// ### Note:
    /// The prescaler and period configuration will be applied immediately by
    /// default and before setting the [`PwmWorkingMode`].
    /// If the timer is already running you might want to call [`Timer::stop`]
    /// and/or [`Timer::set_counter`] first
    /// (if the new period is larger than the current counter value this will
    /// cause weird behavior).
    ///
    /// If configured via [`TimerClockConfig::with_period_updating_method`],
    /// another behavior can be applied. Currently, only
    /// [`PeriodUpdatingMethod::Immediately`]
    /// and [`PeriodUpdatingMethod::TimerEqualsZero`] are useful as the sync
    /// method is not yet implemented.
    ///
    /// The hardware supports writing these settings in sync with certain timer
    /// events but this HAL does not expose these for now.
    pub fn start(&mut self, timer_config: TimerClockConfig) {
        // write prescaler and period with immediate update method
        self.cfg0().write(|w| unsafe {
            w.prescale().bits(timer_config.prescaler);
            w.period().bits(timer_config.period);
            w.period_upmethod()
                .bits(timer_config.period_updating_method as u8)
        });

        // set timer to run with a stop condition
        self.cfg1().write(|w| unsafe {
            w.start().bits(timer_config.stop_condition as u8);
            w.mod_().bits(timer_config.mode as u8)
        });
    }

    /// Stop the timer in its current state
    pub fn stop(&mut self) {
        // freeze the timer
        self.cfg1().write(|w| unsafe { w.mod_().bits(0) });
    }

    /// Set the timer counter to the provided value
    ///
    /// ## Overview
    /// Internally we set the timers phase and direction
    /// Then trigger a software sync event.
    pub fn set_counter(&mut self, phase: u16, direction: CounterDirection) {
        self.set_sync_phase(phase);
        self.set_sync_counter_direction(direction);
        self.trigger_sync();
    }

    /// Set the timers sync phase
    pub fn set_sync_phase(&mut self, phase: u16) {
        // SAFETY: Only TIMERx_SYNC accessed; unique per TIM const
        let tmr = Self::tmr();
        tmr.sync().modify(|_r, w| unsafe { w.phase().bits(phase) });
    }

    /// Set the timers sync counter direction
    /// This specifies how the counter direction will be updated
    /// during a sync event.
    pub fn set_sync_counter_direction(&mut self, direction: CounterDirection) {
        // SAFETY: Only TIMERx_SYNC accessed; unique per TIM const
        let tmr = Self::tmr();
        tmr.sync()
            .modify(|_r, w| w.phase_direction().bit(direction as u8 != 0));
    }

    /// Trigger a software sync event
    pub fn trigger_sync(&mut self) {
        // SAFETY: Only TIMERx_SYNC accessed; unique per TIM const
        let tmr = Self::tmr();
        tmr.sync().modify(|r, w| w.sw().bit(!r.sw().bit()));
    }

    /// Read the counter value and counter direction of the timer
    pub fn status(&self) -> (u16, CounterDirection) {
        // SAFETY: Only read TIMERx_STATUS; unique per TIM const
        let reg = Self::tmr().status().read();
        (reg.value().bits(), reg.direction().bit_is_set().into())
    }

    /// Selects the how the timer fires sync out events
    pub fn set_sync_out_selection(&mut self, sync_out: SyncOutSelect) {
        // SAFETY: Only TIMERx_SYNC accessed; unique per TIM const
        let timer = Self::tmr();
        timer
            .sync()
            .modify(|_r, w| unsafe { w.synco_sel().bits(sync_out as u8) });
    }

    /// Returns the timers sync_out source
    pub fn get_sync_out(&self) -> SyncOut<'d, TIM, PWM> {
        SyncOut::new()
    }

    /// Sets the timer sync in source
    pub fn set_sync_in(&mut self, sync: &impl SyncSource<PWM>) {
        let sync_select: SyncSelection = sync.get_kind().into();
        self.enable_sync_in(sync_select, true);
    }

    /// Clears the sync in for the timer and disables sync input
    pub fn clear_sync_in(&mut self) {
        self.enable_sync_in(SyncSelection::None, false);
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

    fn enable_sync_in(&mut self, sync_sel: SyncSelection, enable: bool) {
        let (info, _) = PWM::split();
        let timer = Self::tmr();

        // SAFETY: Only TIMER_SYNCI_CFG and TIMERx_SYNC accessed
        info.regs()
            .timer_synci_cfg()
            .modify(|_r, w| unsafe { w.timer_syncisel(TIM).bits(sync_sel as u8) });
        timer.sync().modify(|_r, w| w.synci_en().bit(enable));
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
        Self::tmr().cfg0()
    }

    fn cfg1(&mut self) -> &pac::mcpwm0::timer::CFG1 {
        // SAFETY: Unique register access per TIM
        Self::tmr().cfg1()
    }

    fn tmr() -> &'static pac::mcpwm0::TIMER {
        let (info, _) = PWM::split();
        info.regs().timer(TIM as usize)
    }
}

/// Sync out selection for the timer
///
/// Note: For chips ESP32-S3, ESP32-C5, ESP32-C6, and ESP32-H2
///
/// During a software sync event triggered by [`Timer::trigger_sync`] or
/// [`Timer::set_counter`]. A software triggered sync event will
/// propagate to the sync out regardless if the timer was configured
/// with [`SyncOutSelection::SyncWhenEqualZero`] or [`SyncOutSelection::SyncWhenEqualPeriod`].
#[derive(Clone, Copy, Debug)]
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
#[derive(Copy, Clone)]
pub struct TimerClockConfig {
    frequency: Rate,
    period: u16,
    period_updating_method: PeriodUpdatingMethod,
    stop_condition: StopCondition,
    prescaler: u8,
    mode: PwmWorkingMode,
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

        TimerClockConfig {
            frequency,
            prescaler,
            period,
            period_updating_method: PeriodUpdatingMethod::Immediately,
            stop_condition: StopCondition::RunContinuously,
            mode,
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

        Ok(TimerClockConfig {
            frequency,
            prescaler: prescaler as u8,
            period,
            period_updating_method: PeriodUpdatingMethod::Immediately,
            stop_condition: StopCondition::RunContinuously,
            mode,
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

    /// Get the timer clock frequency.
    ///
    /// ### Note:
    /// The actual value is rounded down to the nearest `u32` value
    pub fn frequency(&self) -> Rate {
        self.frequency
    }
}

/// Method for updating the PWM period
#[derive(Clone, Copy)]
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
#[derive(Clone, Copy)]
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
#[derive(Copy, Clone)]
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
#[derive(Debug)]
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
