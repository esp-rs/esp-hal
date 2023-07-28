//! Connect a potentiometer to PIN2 and see the read values change when
//! rotating the shaft. Alternatively you could also connect the PIN to GND or
//! 3V3 to see the maximum and minimum raw values read.

#![no_std]
#![no_main]

use esp32c6_hal::{
    adc,
    adc::{AdcConfig, Attenuation, ADC, ADC1},
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create ADC instances
    let analog = peripherals.APB_SARADC.split();

    let mut adc1_config = AdcConfig::new();

    let atten = Attenuation::Attenuation11dB;

    // You can try any of the following calibration methods by uncommenting them
    // type AdcCal = ();
    // type AdcCal = adc::AdcCalBasic<ADC1>;
    // type AdcCal = adc::AdcCalLine<ADC1>;
    type AdcCal = adc::AdcCalCurve<ADC1>;

    let mut pin = adc1_config.enable_pin_with_cal::<_, AdcCal>(io.pins.gpio2.into_analog(), atten);

    let mut adc1 = ADC::<ADC1>::adc(
        &mut system.peripheral_clock_control,
        analog.adc1,
        adc1_config,
    )
    .unwrap();

    let mut delay = Delay::new(&clocks);

    loop {
        let pin_value: u16 = nb::block!(adc1.read(&mut pin)).unwrap();
        let pin_value_mv = pin_value as u32 * atten.ref_mv() as u32 / 4096;
        println!("PIN2 ADC reading = {pin_value} ({pin_value_mv} mV)");
        delay.delay_ms(1500u32);
    }
}
