//! lcd_cam i8080 tests

//% CHIPS: esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::dummy_pin::DummyPin,
    lcd_cam::{
        lcd::{
            i8080,
            i8080::{Command, TxEightBits, I8080},
        },
        LcdCam,
    },
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

const DATA_SIZE: usize = 1024 * 10;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn test_i8080_8bit() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let (tx_buffer, tx_descriptors, _, _) = dma_buffers!(DATA_SIZE, 0);

        let dma = Dma::new(peripherals.DMA);
        let channel = dma.channel0.configure(false, DmaPriority::Priority0);
        let pins = TxEightBits::new(
            DummyPin::new(),
            DummyPin::new(),
            DummyPin::new(),
            DummyPin::new(),
            DummyPin::new(),
            DummyPin::new(),
            DummyPin::new(),
            DummyPin::new(),
        );
        let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

        let mut i8080 = I8080::new(
            lcd_cam.lcd,
            channel.tx,
            tx_descriptors,
            pins,
            20.MHz(),
            i8080::Config::default(),
            &clocks,
        );

        let xfer = i8080.send_dma(Command::<u8>::None, 0, &tx_buffer).unwrap();
        xfer.wait().unwrap();
    }
}
