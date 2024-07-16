//! Clock Monitor Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    rtc_cntl::Rtc,
    system::SystemControl,
};

struct Context<'a> {
    rtc: Rtc<'a>,
}

impl Context<'_> {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        ClockControl::boot_defaults(system.clock_control).freeze();

        let rtc = Rtc::new(peripherals.LPWR);

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
        #[cfg(feature = "esp32c2")] // 26 MHz
        defmt::assert!((23..=29).contains(&ctx.rtc.estimate_xtal_frequency()));
        #[cfg(feature = "esp32h2")] // 32 MHz
        defmt::assert!((29..=35).contains(&ctx.rtc.estimate_xtal_frequency()));
        #[cfg(not(any(feature = "esp32h2", feature = "esp32c2")))] // 40 MHz
        defmt::assert!((35..=45).contains(&ctx.rtc.estimate_xtal_frequency()));
    }
}
