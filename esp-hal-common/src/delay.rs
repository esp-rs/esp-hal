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

// The delay implementation for RISC-V devices (ESP32-C3) uses the `SYSTIMER`
// peripheral, as unfortunately this device does NOT implement the `mcycle` CSR.
#[cfg(feature = "esp32c3")]
mod delay {
    use crate::pac::SYSTIMER;

    // The counters and comparators are driven using `XTAL_CLK`. The average clock
    // frequency is fXTAL_CLK/2.5, which is 16 MHz. The timer counting is
    // incremented by 1/16 μs on each `CNT_CLK` cycle.
    const CLK_FREQ_HZ: u64 = 16_000_000;

    pub struct Delay {
        systimer: SYSTIMER,
    }

    impl Delay {
        pub fn new(systimer: SYSTIMER) -> Self {
            Self { systimer }
        }

        pub fn delay(&self, us: u32) {
            let t0 = self.unit0_value();
            let clocks = (us as u64 * CLK_FREQ_HZ) / 1_000_000;

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

// The delay implementation for Xtensa devices (ESP32, ESP32-S2, ESP32-S3) uses
// the built-in timer from the `xtensa_lx` crate.
#[cfg(not(feature = "esp32c3"))]
mod delay {
    // FIXME: The ESP32-S2 and ESP32-S3 have fixed crystal frequencies of 40MHz.
    //        This will not always be the case when using the ESP32.
    const CLK_FREQ_HZ: u64 = 40_000_000;

    #[derive(Default)]
    pub struct Delay;

    impl Delay {
        pub fn new() -> Self {
            Self
        }

        pub fn delay(&self, us: u32) {
            let clocks = (us as u64 * CLK_FREQ_HZ) / 1_000_000;
            xtensa_lx::timer::delay(clocks as u32);
        }
    }
}
