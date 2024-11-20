//! # Time
//!
//! The `time` module offers a way to get the system now.

/// Represents a duration of time.
///
/// The resolution is 1 microsecond, represented as a 64-bit unsigned integer.
pub type Duration = fugit::Duration<u64, 1, 1_000_000>;

/// Represents an instant in time.
///
/// The resolution is 1 microsecond, represented as a 64-bit unsigned integer.
pub type Instant = fugit::Instant<u64, 1, 1_000_000>;

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
        let tg0 = unsafe { crate::peripherals::TIMG0::steal() };
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
        // otherwise use SYSTIMER
        let ticks = crate::timer::systimer::SystemTimer::unit_count(Unit::Unit0);
        (
            ticks,
            (crate::timer::systimer::SystemTimer::ticks_per_second() / 1_000_000),
        )
    };

    Instant::from_ticks(ticks / div)
}

#[cfg(esp32)]
pub(crate) fn time_init() {
    let apb = crate::Clocks::get().apb_clock.to_Hz();
    // we assume 80MHz APB clock source - there is no way to configure it in a
    // different way currently
    assert_eq!(apb, 80_000_000u32);

    let tg0 = unsafe { crate::peripherals::TIMG0::steal() };

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
