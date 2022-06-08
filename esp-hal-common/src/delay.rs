//! Delay driver
//!
//! Implement the `DelayMs` and `DelayUs` traits from [embedded-hal].
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/

use embedded_hal::blocking::delay::{DelayMs, DelayUs};

pub use self::delay::Delay;

impl<T> DelayMs<T> for Delay
where
    T: Into<u32>,
{
    fn delay_ms(&mut self, ms: T) {
        for _ in 0..ms.into() {
            self.delay_us(1000u32);
        }
    }
}

impl<T> DelayUs<T> for Delay
where
    T: Into<u32>,
{
    fn delay_us(&mut self, us: T) {
        self.delay(us.into());
    }
}

#[cfg(feature = "esp32c3")]
mod delay {
    use fugit::HertzU64;

    use crate::{clock::Clocks, pac::SYSTIMER};

    /// Uses the `SYSTIMER` peripheral for counting clock cycles, as
    /// unfortunately the ESP32-C3 does NOT implement the `mcycle` CSR, which is
    /// how we would normally do this.
    pub struct Delay {
        systimer: SYSTIMER,
        freq: HertzU64,
    }

    impl Delay {
        /// Create a new Delay instance
        pub fn new(systimer: SYSTIMER, clocks: &Clocks) -> Self {
            // The counters and comparators are driven using `XTAL_CLK`. The average clock
            // frequency is fXTAL_CLK/2.5, which is 16 MHz. The timer counting is
            // incremented by 1/16 μs on each `CNT_CLK` cycle.

            Self {
                systimer,
                freq: HertzU64::MHz((clocks.xtal_clock.to_MHz() * 10 / 25) as u64),
            }
        }

        /// Return the raw interface to the underlying SYSTIMER instance
        pub fn free(self) -> SYSTIMER {
            self.systimer
        }

        /// Delay for the specified number of microseconds
        pub fn delay(&self, us: u32) {
            let t0 = self.unit0_value();
            let clocks = (us as u64 * self.freq.raw()) / HertzU64::MHz(1).raw();

            while self.unit0_value().wrapping_sub(t0) <= clocks {}
        }

        #[inline(always)]
        fn unit0_value(&self) -> u64 {
            self.systimer
                .unit0_op
                .write(|w| w.timer_unit0_update().set_bit());

            while !self
                .systimer
                .unit0_op
                .read()
                .timer_unit0_value_valid()
                .bit_is_set()
            {}

            let value_lo = self.systimer.unit0_value_lo.read().bits();
            let value_hi = self.systimer.unit0_value_hi.read().bits();

            ((value_hi as u64) << 32) | value_lo as u64
        }
    }
}

#[cfg(not(feature = "esp32c3"))]
mod delay {

    use fugit::HertzU64;

    use crate::clock::Clocks;

    /// Delay driver
    ///
    /// Uses the built-in Xtensa timer from the `xtensa_lx` crate.
    pub struct Delay {
        freq: HertzU64,
    }

    impl Delay {
        /// Instantiate the `Delay` driver
        pub fn new(clocks: &Clocks) -> Self {
            Self {
                freq: HertzU64::MHz(clocks.cpu_clock.to_MHz() as u64),
            }
        }

        /// Delay for the specified number of microseconds
        pub fn delay(&self, us: u32) {
            let clocks = (us as u64 * self.freq.raw()) / HertzU64::MHz(1).raw();
            xtensa_lx::timer::delay(clocks as u32);
        }
    }
}
