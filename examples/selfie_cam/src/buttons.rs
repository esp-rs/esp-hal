//! The shutter: the SET key on the board's ADC key ladder, or the BOOT key.

use esp_hal::{
    Blocking,
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    gpio::{Input, InputConfig, Pull},
    peripherals::{ADC1, GPIO42},
};

use crate::ButtonResources;

/// The ladder readings while SET is down. The ADC reads 0 while no key is down.
const SET_LEVELS: core::ops::RangeInclusive<u16> = 71..=434;
/// Readings in a row that make a press. The ladder passes other levels on the way.
const STEADY_READINGS: u8 = 3;

pub struct Buttons {
    adc: Adc<'static, ADC1<'static>, Blocking>,
    ladder: AdcPin<GPIO42<'static>, ADC1<'static>>,
    boot: Input<'static>,
    /// Readings in a row with the shutter down, up to [`STEADY_READINGS`].
    down_readings: u8,
}

impl Buttons {
    pub fn new(r: ButtonResources<'static>) -> Self {
        let mut config = AdcConfig::new();
        let ladder = config.enable_pin(r.ladder, Attenuation::_11dB);
        Self {
            adc: Adc::new(r.adc, config).unwrap(),
            ladder,
            boot: Input::new(r.boot, InputConfig::default().with_pull(Pull::Up)),
            down_readings: 0,
        }
    }

    /// Whether the shutter went down since the last call.
    pub fn shutter_pressed(&mut self) -> bool {
        let level = self.adc.read_blocking(&mut self.ladder).unwrap();
        if !(self.boot.is_low() || SET_LEVELS.contains(&level)) {
            self.down_readings = 0;
            return false;
        }
        if self.down_readings == STEADY_READINGS {
            return false;
        }
        self.down_readings += 1;
        self.down_readings == STEADY_READINGS
    }
}
