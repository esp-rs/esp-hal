//! Connect a potentiometer to PIN2 and see the read values change when
//! rotating the shaft. Alternatively you could also connect the PIN to GND or
//! 3V3 to see the maximum and minimum raw values read.

#![no_std]
#![no_main]

use esp32c3_hal::{
    adc::{AdcConfig, Attenuation, ADC, ADC1},
    analog::SarAdcExt,
    clock::ClockControl,
    gpio::IO,
    pac::Peripherals,
    prelude::*,
    system::SystemExt,
    Delay,
    RtcCntl,
    Timer,
};
use esp_println::println;
use panic_halt as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0, clocks.apb_clock);
    let mut timer1 = Timer::new(peripherals.TIMG1, clocks.apb_clock);

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();
    timer1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut pin = io.pins.gpio2.into_analog();

    // Create ADC instances
    let analog = peripherals.APB_SARADC.split();

    let mut adc2_config = AdcConfig::new();
    adc2_config.enable_pin(&pin, Attenuation::Attenuation11dB);
    let mut adc2 = ADC::<ADC1>::adc(
        &mut system.peripheral_clock_control,
        analog.adc1,
        adc2_config,
    )
    .unwrap();

    let mut delay = Delay::new(&clocks);

    loop {
        let pin_value: u16 = nb::block!(adc2.read(&mut pin)).unwrap();
        println!("PIN ADC reading = {}", pin_value);
        delay.delay_ms(1500u32);
    }
}
