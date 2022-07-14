//! Connect a potentiometer to PIN3 and see the read values change when
//! rotating the shaft. Alternatively you could also connect the PIN to GND or
//! 3V3 to see the maximum and minimum raw values read.

#![no_std]
#![no_main]

use esp32s2_hal::{
    adc::{AdcConfig, Attenuation, ADC, ADC1},
    clock::ClockControl,
    gpio::IO,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    RtcCntl,
};
use esp_println::println;
use panic_halt as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc_cntl.set_wdt_global_enable(false);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut pin3 = io.pins.gpio3.into_analog();

    // Create ADC instances
    let analog = peripherals.SENS.split();

    let mut adc1_config = AdcConfig::new();
    adc1_config.enable_pin(&pin3, Attenuation::Attenuation11dB);
    let mut adc1 = ADC::<ADC1>::adc(analog.adc1, adc1_config).unwrap();

    let mut delay = Delay::new(&clocks);

    loop {
        println!("About to read from ADC");
        let pin3_value: u16 = nb::block!(adc1.read(&mut pin3)).unwrap();
        println!("PIN3 ADC reading = {}", pin3_value);
        delay.delay_ms(1500u32);
    }
}
