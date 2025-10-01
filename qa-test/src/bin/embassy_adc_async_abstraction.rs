//! This shows how to asynchronously read ADC data and offers an abstraction to
//! convert the raw value to a type-specific interpretation.
//!
//! PINS
//! GPIO4 for ADC1

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use core::marker::PhantomData;

use embassy_executor::Spawner;
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    Async,
    analog::adc::{
        Adc,
        AdcCalScheme,
        AdcChannel,
        AdcConfig,
        AdcPin,
        Attenuation,
        Instance,
        RegisterAccess,
    },
    delay::Delay,
    timer::timg::TimerGroup,
};
use esp_println::println;

trait Sensor {
    async fn measure(&mut self) -> u16;
}

trait Converter {
    /// Converts the raw ADC value to a valid metering.
    fn raw_to_metering(raw_value: u16) -> u16;
}

pub struct AdcSensor<'d, ADCI, PIN, CS, MC> {
    adc: Adc<'d, ADCI, Async>,
    pin: AdcPin<PIN, ADCI, CS>,
    _phantom: PhantomData<MC>,
}

impl<'d, ADCI, PIN, CS, MC> AdcSensor<'d, ADCI, PIN, CS, MC> {
    pub fn new(adc: Adc<'d, ADCI, Async>, pin: AdcPin<PIN, ADCI, CS>) -> Self {
        let _phantom = PhantomData::<MC> {};
        Self { adc, pin, _phantom }
    }
}

impl<'d, ADCI, PIN, CS, MC> Sensor for AdcSensor<'d, ADCI, PIN, CS, MC>
where
    ADCI: RegisterAccess + Instance + 'd,
    PIN: AdcChannel,
    CS: AdcCalScheme<ADCI>,
    MC: Converter,
{
    async fn measure(&mut self) -> u16 {
        let raw_value = self.adc.read_oneshot(&mut self.pin).await;

        MC::raw_to_metering(raw_value)
    }
}

/// Returns the provided raw value as is.
pub struct Identity;

impl Converter for Identity {
    fn raw_to_metering(raw_value: u16) -> u16 {
        raw_value
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    let mut adc1_config = AdcConfig::new();
    let analog_pin1 = peripherals.GPIO4;
    let pin1 = adc1_config.enable_pin(analog_pin1, Attenuation::_11dB);
    let adc1 = Adc::new(peripherals.ADC1, adc1_config).into_async();
    let mut id = AdcSensor::<_, _, _, Identity>::new(adc1, pin1);

    let delay = Delay::new();

    loop {
        let id_value = id.measure().await;
        println!("id value: {}", id_value);
        delay.delay_millis(1000);
    }
}
