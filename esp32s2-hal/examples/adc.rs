//! Connect a potentiometer to PIN3 and see the read values change when
//! rotating the shaft. If could also connect the PIN to GND or 3V3 to see the
//! maximum and minimum raw values read.
//! 
//! THIS CURRENTLY DOESN'T WORK IN DEBUG BUILDS! THIS NEEDS TO GET FIGURED OUT!

#![no_std]
#![no_main]

use esp32s2_hal::{
    clock::ClockControl,
    gpio::IO,
    pac::Peripherals,
    prelude::*,
    AdcConfig,
    Attenuation,
    Delay,
    RtcCntl,
    Timer,
    ADC,
    ADC1,
};
use esp_println::println;
use panic_halt as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut timer0 = Timer::new(peripherals.TIMG0, clocks.apb_clock);
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    timer0.disable();
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
