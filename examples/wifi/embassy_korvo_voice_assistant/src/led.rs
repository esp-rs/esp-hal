//! The WS2812 RGB LED of the Korvo-1 on GPIO37, driven by the RMT peripheral.

use esp_hal::{
    Async,
    gpio::Level,
    peripherals::{GPIO37, RMT},
    rmt::{Channel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator},
    time::Rate,
};

/// The RMT clock. One tick is 12.5 ns.
const RMT_CLOCK_MHZ: u32 = 80;
/// The pulses of a 0 and a 1 bit: 0.4 µs high and 0.85 µs low, or 0.8 µs high and 0.45 µs low.
const ZERO: PulseCode = PulseCode::new(Level::High, 32, Level::Low, 68);
const ONE: PulseCode = PulseCode::new(Level::High, 64, Level::Low, 36);

/// A color of the LED. Full brightness is glaring, so the colors use about a sixth of it.
#[derive(Clone, Copy)]
pub struct Color {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl Color {
    pub const OFF: Self = Self::new(0, 0, 0);
    pub const RED: Self = Self::new(40, 0, 0);
    pub const GREEN: Self = Self::new(0, 40, 0);
    pub const BLUE: Self = Self::new(0, 0, 40);
    pub const YELLOW: Self = Self::new(40, 30, 0);
    pub const PURPLE: Self = Self::new(30, 0, 40);
    pub const WHITE: Self = Self::new(40, 40, 40);

    const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self { red, green, blue }
    }
}

pub struct Led {
    channel: Channel<'static, Async, Tx>,
}

impl Led {
    pub fn new(rmt: RMT<'static>, pin: GPIO37<'static>) -> Self {
        let rmt = Rmt::new(rmt, Rate::from_mhz(RMT_CLOCK_MHZ))
            .unwrap()
            .into_async();
        let config = TxChannelConfig::default()
            .with_clk_divider(1)
            .with_idle_output_level(Level::Low)
            .with_idle_output(true);
        let channel = rmt.channel0.configure_tx(&config).unwrap().with_pin(pin);
        Self { channel }
    }

    /// Sends the color: 24 bits in green, red, blue order, most significant bit first.
    pub async fn set(&mut self, color: Color) {
        let bits = u32::from_be_bytes([0, color.green, color.red, color.blue]);
        let mut pulses = [PulseCode::end_marker(); 25];
        for (index, pulse) in pulses[..24].iter_mut().enumerate() {
            *pulse = if bits & (1 << (23 - index)) != 0 {
                ONE
            } else {
                ZERO
            };
        }
        if let Err(error) = self.channel.transmit(&pulses).await {
            log::error!("Failed to set the LED: {error:?}");
        }
    }
}
