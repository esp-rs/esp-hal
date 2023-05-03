//! Delay driver
//!
//! Implement the `DelayMs` and `DelayUs` traits from [embedded-hal].
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/

pub use self::delay::Delay;

impl<T> embedded_hal::blocking::delay::DelayMs<T> for Delay
where
    T: Into<u32>,
{
    fn delay_ms(&mut self, ms: T) {
        for _ in 0..ms.into() {
            self.delay(1000u32);
        }
    }
}

impl<T> embedded_hal::blocking::delay::DelayUs<T> for Delay
where
    T: Into<u32>,
{
    fn delay_us(&mut self, us: T) {
        self.delay(us.into());
    }
}

#[cfg(feature = "eh1")]
impl embedded_hal_1::delay::DelayUs for Delay {
    fn delay_us(&mut self, us: u32) {
        self.delay(us);
    }
}

#[cfg(riscv)]
mod delay {
    use fugit::HertzU64;

    use crate::{clock::Clocks, systimer::SystemTimer};

    /// Uses the `SYSTIMER` peripheral for counting clock cycles, as
    /// unfortunately the ESP32-C3 does NOT implement the `mcycle` CSR, which is
    /// how we would normally do this.
    pub struct Delay {
        freq: HertzU64,
    }

    impl Delay {
        /// Create a new Delay instance
        pub fn new(clocks: &Clocks) -> Self {
            // The counters and comparators are driven using `XTAL_CLK`. The average clock
            // frequency is fXTAL_CLK/2.5, which is 16 MHz. The timer counting is
            // incremented by 1/16 μs on each `CNT_CLK` cycle.

            Self {
                freq: HertzU64::MHz((clocks.xtal_clock.to_MHz() * 10 / 25) as u64),
            }
        }

        /// Delay for the specified number of microseconds
        pub fn delay(&self, us: u32) {
            let t0 = SystemTimer::now();
            let clocks = (us as u64 * self.freq.raw()) / HertzU64::MHz(1).raw();

            while SystemTimer::now().wrapping_sub(t0) & SystemTimer::BIT_MASK <= clocks {}
        }
    }
}

#[cfg(xtensa)]
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
