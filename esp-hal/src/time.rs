//! # Time
//!
//! The `time` module offers a way to get the system now.

use core::fmt::{Debug, Display, Formatter, Result as FmtResult};

#[cfg(esp32)]
use crate::peripherals::TIMG0;

type InnerRate = fugit::Rate<u32, 1, 1>;
type InnerInstant = fugit::Instant<u64, 1, 1_000_000>;
type InnerDuration = fugit::Duration<u64, 1, 1_000_000>;

/// Represents a rate or frequency of events.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Rate(InnerRate);

impl core::hash::Hash for Rate {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.as_hz().hash(state);
    }
}

impl Display for Rate {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "{} Hz", self.as_hz())
    }
}

impl Debug for Rate {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "Rate({} Hz)", self.as_hz())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Rate {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "{=u32} Hz", self.as_hz())
    }
}

impl Rate {
    /// Shorthand for creating a rate which represents hertz.
    #[inline]
    pub const fn from_hz(val: u32) -> Self {
        Self(InnerRate::Hz(val))
    }

    /// Shorthand for creating a rate which represents kilohertz.
    #[inline]
    pub const fn from_khz(val: u32) -> Self {
        Self(InnerRate::kHz(val))
    }

    /// Shorthand for creating a rate which represents megahertz.
    #[inline]
    pub const fn from_mhz(val: u32) -> Self {
        Self(InnerRate::MHz(val))
    }

    /// Convert the `Rate` to an interger number of Hz.
    #[inline]
    pub const fn as_hz(&self) -> u32 {
        self.0.to_Hz()
    }

    /// Convert the `Rate` to an interger number of kHz.
    #[inline]
    pub const fn as_khz(&self) -> u32 {
        self.0.to_kHz()
    }

    /// Convert the `Rate` to an interger number of MHz.
    #[inline]
    pub const fn as_mhz(&self) -> u32 {
        self.0.to_MHz()
    }

    /// Convert the `Rate` to a `Duration`.
    #[inline]
    pub const fn as_duration(&self) -> Duration {
        Duration::from_micros(1_000_000 / self.as_hz() as u64)
    }
}

impl core::ops::Div for Rate {
    type Output = u32;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

impl core::ops::Mul<u32> for Rate {
    type Output = Rate;

    fn mul(self, rhs: u32) -> Self::Output {
        Rate(self.0 * rhs)
    }
}

impl core::ops::Div<u32> for Rate {
    type Output = Rate;

    fn div(self, rhs: u32) -> Self::Output {
        Rate(self.0 / rhs)
    }
}

/// Represents an instant in time.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Instant(InnerInstant);

impl Debug for Instant {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(
            f,
            "Instant({} µs since epoch)",
            self.duration_since_epoch().as_micros()
        )
    }
}

impl Display for Instant {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(
            f,
            "{} µs since epoch",
            self.duration_since_epoch().as_micros()
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Instant {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "{=u64} µs since epoch",
            self.duration_since_epoch().as_micros()
        )
    }
}

impl Instant {
    /// Returns the current instant.
    pub fn now() -> Self {
        now()
    }

    pub(crate) fn from_ticks(ticks: u64) -> Self {
        Instant(InnerInstant::from_ticks(ticks))
    }

    /// Returns the elapsed time since boot.
    pub const fn duration_since_epoch(&self) -> Duration {
        Duration::from_micros(self.0.ticks())
    }

    /// Returns the elapsed `Duration` since this instant was created.
    pub fn elapsed(&self) -> Duration {
        Self::now() - *self
    }
}

impl core::ops::Add<Duration> for Instant {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self::Output {
        Instant(self.0 + rhs.0)
    }
}

impl core::ops::AddAssign<Duration> for Instant {
    fn add_assign(&mut self, rhs: Duration) {
        self.0 += rhs.0;
    }
}

impl core::ops::Sub for Instant {
    type Output = Duration;

    fn sub(self, rhs: Self) -> Self::Output {
        Duration(self.0 - rhs.0)
    }
}

impl core::ops::Sub<Duration> for Instant {
    type Output = Self;

    fn sub(self, rhs: Duration) -> Self::Output {
        Instant(self.0 - rhs.0)
    }
}

impl core::ops::SubAssign<Duration> for Instant {
    fn sub_assign(&mut self, rhs: Duration) {
        self.0 -= rhs.0;
    }
}

/// Represents a duration of time.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Duration(InnerDuration);

impl Debug for Duration {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "Duration({} µs)", self.as_micros())
    }
}

impl Display for Duration {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "{} µs", self.as_micros())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Duration {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "{=u64} µs", self.as_micros())
    }
}

impl Duration {
    /// A duration of zero time.
    pub const ZERO: Self = Self(InnerDuration::from_ticks(0));

    /// A duration representing the maximum possible time.
    pub const MAX: Self = Self(InnerDuration::from_ticks(u64::MAX));

    /// Creates a duration which represents microseconds.
    #[inline]
    pub const fn from_micros(val: u64) -> Self {
        Self(InnerDuration::micros(val))
    }

    /// Creates a duration which represents milliseconds.
    #[inline]
    pub const fn from_millis(val: u64) -> Self {
        Self(InnerDuration::millis(val))
    }

    /// Creates a duration which represents seconds.
    #[inline]
    pub const fn from_secs(val: u64) -> Self {
        Self(InnerDuration::secs(val))
    }

    /// Creates a duration which represents minutes.
    #[inline]
    pub const fn from_minutes(val: u64) -> Self {
        Self(InnerDuration::minutes(val))
    }

    /// Creates a duration which represents hours.
    #[inline]
    pub const fn from_hours(val: u64) -> Self {
        Self(InnerDuration::hours(val))
    }

    delegate::delegate! {
        #[inline]
        to self.0 {
            /// Convert the `Duration` to an interger number of microseconds.
            #[call(to_micros)]
            pub const fn as_micros(&self) -> u64;

            /// Convert the `Duration` to an interger number of milliseconds.
            #[call(to_millis)]
            pub const fn as_millis(&self) -> u64;

            /// Convert the `Duration` to an interger number of seconds.
            #[call(to_secs)]
            pub const fn as_secs(&self) -> u64;

            /// Convert the `Duration` to an interger number of minutes.
            #[call(to_minutes)]
            pub const fn as_minutes(&self) -> u64;

            /// Convert the `Duration` to an interger number of hours.
            #[call(to_hours)]
            pub const fn as_hours(&self) -> u64;
        }
    }

    /// Add two durations while checking for overflow.
    pub const fn checked_add(self, rhs: Self) -> Option<Self> {
        if let Some(val) = self.0.checked_add(rhs.0) {
            Some(Duration(val))
        } else {
            None
        }
    }

    /// Subtract two durations while checking for overflow.
    pub const fn checked_sub(self, rhs: Self) -> Option<Self> {
        if let Some(val) = self.0.checked_sub(rhs.0) {
            Some(Duration(val))
        } else {
            None
        }
    }

    /// Add two durations, returning the maximum value if overflow occurred.
    pub const fn saturating_add(self, rhs: Self) -> Self {
        if let Some(val) = self.checked_add(rhs) {
            val
        } else {
            Self::MAX
        }
    }

    /// Subtract two durations, returning the minimum value if the result would
    /// be negative.
    pub const fn saturating_sub(self, rhs: Self) -> Self {
        if let Some(val) = self.checked_sub(rhs) {
            val
        } else {
            Self::ZERO
        }
    }
}

impl core::ops::Add for Duration {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Duration(self.0 + rhs.0)
    }
}

impl core::ops::AddAssign for Duration {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl core::ops::Sub for Duration {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Duration(self.0 - rhs.0)
    }
}

impl core::ops::SubAssign for Duration {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl core::ops::Mul<u32> for Duration {
    type Output = Self;

    fn mul(self, rhs: u32) -> Self::Output {
        Duration(self.0 * rhs)
    }
}

impl core::ops::Div<u32> for Duration {
    type Output = Self;

    fn div(self, rhs: u32) -> Self::Output {
        Duration(self.0 / rhs)
    }
}

impl core::ops::Div<Duration> for Duration {
    type Output = u64;

    fn div(self, rhs: Duration) -> Self::Output {
        self.0 / rhs.0
    }
}

/// Provides time since system start in microseconds precision.
///
/// The counter won’t measure time in sleep-mode.
///
/// The timer will wrap after
#[cfg_attr(esp32, doc = "36_558 years")]
#[cfg_attr(esp32s2, doc = "7_311 years")]
#[cfg_attr(not(any(esp32, esp32s2)), doc = "more than 7 years")]
pub fn now() -> Instant {
    #[cfg(esp32)]
    let (ticks, div) = {
        // on ESP32 use LACT
        let tg0 = TIMG0::regs();
        tg0.lactupdate().write(|w| unsafe { w.update().bits(1) });

        // The peripheral doesn't have a bit to indicate that the update is done, so we
        // poll the lower 32 bit part of the counter until it changes, or a timeout
        // expires.
        let lo_initial = tg0.lactlo().read().bits();
        let mut div = tg0.lactconfig().read().divider().bits();
        let lo = loop {
            let lo = tg0.lactlo().read().bits();
            if lo != lo_initial || div == 0 {
                break lo;
            }
            div -= 1;
        };
        let hi = tg0.lacthi().read().bits();

        let ticks = (hi as u64) << 32u64 | lo as u64;
        (ticks, 16)
    };

    #[cfg(not(esp32))]
    let (ticks, div) = {
        use crate::timer::systimer::{SystemTimer, Unit};
        // otherwise use SYSTIMER
        let ticks = SystemTimer::unit_value(Unit::Unit0);
        (ticks, (SystemTimer::ticks_per_second() / 1_000_000))
    };

    Instant::from_ticks(ticks / div)
}

#[cfg(esp32)]
pub(crate) fn time_init() {
    let apb = crate::Clocks::get().apb_clock.as_hz();
    // we assume 80MHz APB clock source - there is no way to configure it in a
    // different way currently
    assert_eq!(apb, 80_000_000u32);

    let tg0 = TIMG0::regs();

    tg0.lactconfig().write(|w| unsafe { w.bits(0) });
    tg0.lactalarmhi().write(|w| unsafe { w.bits(u32::MAX) });
    tg0.lactalarmlo().write(|w| unsafe { w.bits(u32::MAX) });
    tg0.lactload().write(|w| unsafe { w.load().bits(1) });

    // 16 MHz counter
    tg0.lactconfig()
        .modify(|_, w| unsafe { w.divider().bits((apb / 16_000_000u32) as u16) });
    tg0.lactconfig().modify(|_, w| {
        w.increase().bit(true);
        w.autoreload().bit(true);
        w.en().bit(true)
    });
}
