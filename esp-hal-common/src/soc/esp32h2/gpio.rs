use paste::paste;

// https://github.com/espressif/esp-idf/blob/master/components/soc/esp32h2/gpio_periph.c#L42
pub const NUM_PINS: usize = 27;

pub type OutputSignalType = u8;
pub const OUTPUT_SIGNAL_MAX: u8 = 0; // FIXME
pub const INPUT_SIGNAL_MAX: u8 = 0; // FIXME

pub const ONE_INPUT: u8 = 0x1e;
pub const ZERO_INPUT: u8 = 0x1f;

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
pub enum InputSignal {}

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
pub enum OutputSignal {}

// crate::gpio::gpio! {}

// crate::gpio::analog! {}

// TODO USB pins
// implement marker traits on USB pins
// impl<T> crate::otg_fs::UsbSel for Gpio??<T> {}
// impl<T> crate::otg_fs::UsbDp for Gpio12<T> {}
// impl<T> crate::otg_fs::UsbDm for Gpio13<T> {}
