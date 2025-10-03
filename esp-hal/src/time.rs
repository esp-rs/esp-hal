//! # Timekeeping
//!
//! This module provides types for representing frequency and duration, as well
//! as an instant in time. Time is measured since boot, and can be accessed
//! by the [`Instant::now`] function.

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
    #[inline]
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.as_hz().hash(state);
    }
}

impl Display for Rate {
    #[inline]
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "{} Hz", self.as_hz())
    }
}

impl Debug for Rate {
    #[inline]
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "Rate({} Hz)", self.as_hz())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Rate {
    #[inline]
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "{=u32} Hz", self.as_hz())
    }
}

impl Rate {
    #[procmacros::doc_replace]
    /// Shorthand for creating a rate which represents hertz.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Rate;
    /// let rate = Rate::from_hz(1000);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn from_hz(val: u32) -> Self {
        Self(InnerRate::Hz(val))
    }

    #[procmacros::doc_replace]
    /// Shorthand for creating a rate which represents kilohertz.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Rate;
    /// let rate = Rate::from_khz(1000);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn from_khz(val: u32) -> Self {
        Self(InnerRate::kHz(val))
    }

    #[procmacros::doc_replace]
    /// Shorthand for creating a rate which represents megahertz.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Rate;
    /// let rate = Rate::from_mhz(1000);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn from_mhz(val: u32) -> Self {
        Self(InnerRate::MHz(val))
    }

    #[procmacros::doc_replace]
    /// Convert the `Rate` to an interger number of Hz.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Rate;
    /// let rate = Rate::from_hz(1000);
    /// let hz = rate.as_hz();
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn as_hz(&self) -> u32 {
        self.0.to_Hz()
    }

    #[procmacros::doc_replace]
    /// Convert the `Rate` to an interger number of kHz.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Rate;
    /// let rate = Rate::from_khz(1000);
    /// let khz = rate.as_khz();
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn as_khz(&self) -> u32 {
        self.0.to_kHz()
    }

    #[procmacros::doc_replace]
    /// Convert the `Rate` to an interger number of MHz.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Rate;
    /// let rate = Rate::from_mhz(1000);
    /// let mhz = rate.as_mhz();
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn as_mhz(&self) -> u32 {
        self.0.to_MHz()
    }

    #[procmacros::doc_replace]
    /// Convert the `Rate` to a `Duration`.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Rate;
    /// let rate = Rate::from_hz(1000);
    /// let duration = rate.as_duration();
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn as_duration(&self) -> Duration {
        Duration::from_micros(1_000_000 / self.as_hz() as u64)
    }
}

impl core::ops::Div for Rate {
    type Output = u32;

    #[inline]
    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

impl core::ops::Mul<u32> for Rate {
    type Output = Rate;

    #[inline]
    fn mul(self, rhs: u32) -> Self::Output {
        Rate(self.0 * rhs)
    }
}

impl core::ops::Div<u32> for Rate {
    type Output = Rate;

    #[inline]
    fn div(self, rhs: u32) -> Self::Output {
        Rate(self.0 / rhs)
    }
}

/// Represents an instant in time.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Instant(InnerInstant);

impl Debug for Instant {
    #[inline]
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(
            f,
            "Instant({} µs since epoch)",
            self.duration_since_epoch().as_micros()
        )
    }
}

impl Display for Instant {
    #[inline]
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
    #[inline]
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "{=u64} µs since epoch",
            self.duration_since_epoch().as_micros()
        )
    }
}

impl core::hash::Hash for Instant {
    #[inline]
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.duration_since_epoch().hash(state);
    }
}

impl Instant {
    /// Represents the moment the system booted.
    pub const EPOCH: Instant = Instant(InnerInstant::from_ticks(0));

    #[procmacros::doc_replace(
        "wrap_after" => {
            cfg(esp32) => "36_558 years",
            cfg(esp32s2) => "7_311 years",
            _ => "more than 7 years"
        }
    )]
    /// Returns the current instant.
    ///
    /// The counter won’t measure time in sleep-mode.
    ///
    /// The timer has a 1 microsecond resolution and will wrap after
    /// # {wrap_after}
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Instant;
    /// let now = Instant::now();
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn now() -> Self {
        now()
    }

    #[inline]
    pub(crate) fn from_ticks(ticks: u64) -> Self {
        Instant(InnerInstant::from_ticks(ticks))
    }

    #[procmacros::doc_replace]
    /// Returns the elapsed `Duration` since boot.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Instant;
    /// let now = Instant::now();
    /// let duration = now.duration_since_epoch();
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn duration_since_epoch(&self) -> Duration {
        *self - Self::EPOCH
    }

    #[procmacros::doc_replace]
    /// Returns the elapsed `Duration` since this `Instant` was created.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Instant;
    /// let now = Instant::now();
    /// let duration = now.elapsed();
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn elapsed(&self) -> Duration {
        Self::now() - *self
    }
}

impl core::ops::Add<Duration> for Instant {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Duration) -> Self::Output {
        Instant(self.0 + rhs.0)
    }
}

impl core::ops::AddAssign<Duration> for Instant {
    #[inline]
    fn add_assign(&mut self, rhs: Duration) {
        self.0 += rhs.0;
    }
}

impl core::ops::Sub for Instant {
    type Output = Duration;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        // Avoid "Sub failed! Other > self" panics
        Duration::from_micros(self.0.ticks().wrapping_sub(rhs.0.ticks()))
    }
}

impl core::ops::Sub<Duration> for Instant {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Duration) -> Self::Output {
        Instant(self.0 - rhs.0)
    }
}

impl core::ops::SubAssign<Duration> for Instant {
    #[inline]
    fn sub_assign(&mut self, rhs: Duration) {
        self.0 -= rhs.0;
    }
}

/// Represents a duration of time.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Duration(InnerDuration);

impl Debug for Duration {
    #[inline]
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "Duration({} µs)", self.as_micros())
    }
}

impl Display for Duration {
    #[inline]
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "{} µs", self.as_micros())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Duration {
    #[inline]
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "{=u64} µs", self.as_micros())
    }
}

impl core::hash::Hash for Duration {
    #[inline]
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.as_micros().hash(state);
    }
}

impl Duration {
    /// A duration of zero time.
    pub const ZERO: Self = Self(InnerDuration::from_ticks(0));

    /// A duration representing the maximum possible time.
    pub const MAX: Self = Self(InnerDuration::from_ticks(u64::MAX));

    #[procmacros::doc_replace]
    /// Creates a duration which represents microseconds.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Duration;
    /// let duration = Duration::from_micros(1000);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn from_micros(val: u64) -> Self {
        Self(InnerDuration::micros(val))
    }

    #[procmacros::doc_replace]
    /// Creates a duration which represents milliseconds.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Duration;
    /// let duration = Duration::from_millis(100);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn from_millis(val: u64) -> Self {
        Self(InnerDuration::millis(val))
    }

    #[procmacros::doc_replace]
    /// Creates a duration which represents seconds.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Duration;
    /// let duration = Duration::from_secs(1);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn from_secs(val: u64) -> Self {
        Self(InnerDuration::secs(val))
    }

    #[procmacros::doc_replace]
    /// Creates a duration which represents minutes.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Duration;
    /// let duration = Duration::from_minutes(1);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn from_minutes(val: u64) -> Self {
        Self(InnerDuration::minutes(val))
    }

    #[procmacros::doc_replace]
    /// Creates a duration which represents hours.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Duration;
    /// let duration = Duration::from_hours(1);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn from_hours(val: u64) -> Self {
        Self(InnerDuration::hours(val))
    }

    delegate::delegate! {
        #[inline]
        to self.0 {
            #[procmacros::doc_replace]
            /// Convert the `Duration` to an interger number of microseconds.
            ///
            /// ## Example
            ///
            /// ```rust, no_run
            /// # {before_snippet}
            /// use esp_hal::time::Duration;
            /// let duration = Duration::from_micros(1000);
            /// let micros = duration.as_micros();
            /// # {after_snippet}
            /// ```
            #[call(to_micros)]
            pub const fn as_micros(&self) -> u64;

            #[procmacros::doc_replace]
            /// Convert the `Duration` to an interger number of milliseconds.
            ///
            /// ## Example
            ///
            /// ```rust, no_run
            /// # {before_snippet}
            /// use esp_hal::time::Duration;
            /// let duration = Duration::from_millis(100);
            /// let millis = duration.as_millis();
            /// # {after_snippet}
            /// ```
            #[call(to_millis)]
            pub const fn as_millis(&self) -> u64;

            #[procmacros::doc_replace]
            /// Convert the `Duration` to an interger number of seconds.
            ///
            /// ## Example
            ///
            /// ```rust, no_run
            /// # {before_snippet}
            /// use esp_hal::time::Duration;
            /// let duration = Duration::from_secs(1);
            /// let secs = duration.as_secs();
            /// # {after_snippet}
            /// ```
            #[call(to_secs)]
            pub const fn as_secs(&self) -> u64;

            #[procmacros::doc_replace]
            /// Convert the `Duration` to an interger number of minutes.
            ///
            /// ## Example
            ///
            /// ```rust, no_run
            /// # {before_snippet}
            /// use esp_hal::time::Duration;
            /// let duration = Duration::from_minutes(1);
            /// let minutes = duration.as_minutes();
            /// # {after_snippet}
            /// ```
            #[call(to_minutes)]
            pub const fn as_minutes(&self) -> u64;

            #[procmacros::doc_replace]
            /// Convert the `Duration` to an interger number of hours.
            ///
            /// ## Example
            ///
            /// ```rust, no_run
            /// # {before_snippet}
            /// use esp_hal::time::Duration;
            /// let duration = Duration::from_hours(1);
            /// let hours = duration.as_hours();
            /// # {after_snippet}
            /// ```
            #[call(to_hours)]
            pub const fn as_hours(&self) -> u64;
        }
    }

    #[procmacros::doc_replace]
    /// Add two durations while checking for overflow.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Duration;
    /// let duration = Duration::from_secs(1);
    /// let duration2 = Duration::from_secs(2);
    ///
    /// if let Some(sum) = duration.checked_add(duration2) {
    ///     println!("Sum: {}", sum);
    /// } else {
    ///     println!("Overflow occurred");
    /// }
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn checked_add(self, rhs: Self) -> Option<Self> {
        if let Some(val) = self.0.checked_add(rhs.0) {
            Some(Duration(val))
        } else {
            None
        }
    }

    #[procmacros::doc_replace]
    /// Subtract two durations while checking for overflow.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Duration;
    /// let duration = Duration::from_secs(3);
    /// let duration2 = Duration::from_secs(1);
    ///
    /// if let Some(diff) = duration.checked_sub(duration2) {
    ///     println!("Difference: {}", diff);
    /// } else {
    ///     println!("Underflow occurred");
    /// }
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn checked_sub(self, rhs: Self) -> Option<Self> {
        if let Some(val) = self.0.checked_sub(rhs.0) {
            Some(Duration(val))
        } else {
            None
        }
    }

    #[procmacros::doc_replace]
    /// Add two durations, returning the maximum value if overflow occurred.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Duration;
    /// let duration = Duration::from_secs(1);
    /// let duration2 = Duration::from_secs(2);
    ///
    /// let sum = duration.saturating_add(duration2);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub const fn saturating_add(self, rhs: Self) -> Self {
        if let Some(val) = self.checked_add(rhs) {
            val
        } else {
            Self::MAX
        }
    }

    #[procmacros::doc_replace]
    /// Subtract two durations, returning the minimum value if the result would
    /// be negative.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::time::Duration;
    /// let duration = Duration::from_secs(3);
    /// let duration2 = Duration::from_secs(1);
    ///
    /// let diff = duration.saturating_sub(duration2);
    /// # {after_snippet}
    /// ```
    #[inline]
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

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Duration(self.0 + rhs.0)
    }
}

impl core::ops::AddAssign for Duration {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl core::ops::Sub for Duration {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Duration(self.0 - rhs.0)
    }
}

impl core::ops::SubAssign for Duration {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl core::ops::Mul<u32> for Duration {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: u32) -> Self::Output {
        Duration(self.0 * rhs)
    }
}

impl core::ops::Div<u32> for Duration {
    type Output = Self;

    #[inline]
    fn div(self, rhs: u32) -> Self::Output {
        Duration(self.0 / rhs)
    }
}

impl core::ops::Div<Duration> for Duration {
    type Output = u64;

    #[inline]
    fn div(self, rhs: Duration) -> Self::Output {
        self.0 / rhs.0
    }
}

#[inline]
fn now() -> Instant {
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

        let ticks = ((hi as u64) << 32u64) | lo as u64;
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

#[cfg(all(esp32, feature = "rt"))]
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
