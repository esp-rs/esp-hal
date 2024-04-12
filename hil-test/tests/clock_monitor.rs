//! Clock Monitor Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, rtc_cntl::Rtc};

struct Context<'a> {
    rtc: Rtc<'a>,
}

impl Context<'_> {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = peripherals.SYSTEM.split();
        ClockControl::boot_defaults(system.clock_control).freeze();

        let rtc = Rtc::new(peripherals.LPWR, None);

        Context { rtc }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context<'static> {
        Context::init()
    }

    #[test]
    fn test_estimated_clock(mut ctx: Context<'static>) {
        // We call the function twice since sometimes the first call gives a
        // wrong result
        ctx.rtc.estimate_xtal_frequency();
        let estimated_xtal_freq = ctx.rtc.estimate_xtal_frequency();

        #[cfg(feature = "esp32c2")] // 26 MHz
        assert!(estimated_xtal_freq > 22 && estimated_xtal_freq < 30);
        #[cfg(feature = "esp32h2")] // 32 MHz
        assert!(estimated_xtal_freq > 26 && estimated_xtal_freq < 36);
        #[cfg(not(any(feature = "esp32h2", feature = "esp32c2")))] // 40 MHz
        assert!(estimated_xtal_freq > 34 && estimated_xtal_freq < 44);
    }
}
