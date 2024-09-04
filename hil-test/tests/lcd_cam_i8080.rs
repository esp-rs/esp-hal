//! lcd_cam i8080 tests

//% CHIPS: esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Dma, DmaDescriptor, DmaPriority},
    dma_buffers,
    gpio::DummyPin,
    lcd_cam::{
        lcd::i8080::{Command, Config, TxEightBits, I8080},
        LcdCam,
    },
    prelude::*,
};
use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

struct Context<'d> {
    lcd_cam: LcdCam<'d, esp_hal::Blocking>,
    dma: Dma<'d>,
    tx_buffer: &'static [u8],
    tx_descriptors: &'static mut [DmaDescriptor],
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let dma = Dma::new(peripherals.DMA);
        let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
        let (tx_buffer, tx_descriptors, _, _) = dma_buffers!(DATA_SIZE, 0);

        Context {
            lcd_cam,
            dma,
            tx_buffer,
            tx_descriptors,
        }
    }

    #[test]
    fn test_i8080_8bit(ctx: Context<'static>) {
        let channel = ctx.dma.channel0.configure(false, DmaPriority::Priority0);

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

        let mut i8080 = I8080::new(
            ctx.lcd_cam.lcd,
            channel.tx,
            ctx.tx_descriptors,
            pins,
            20.MHz(),
            Config::default(),
        );

        let xfer = i8080
            .send_dma(Command::<u8>::None, 0, &ctx.tx_buffer)
            .unwrap();
        xfer.wait().unwrap();
    }

    #[test]
    fn test_i8080_8bit_async_channel(ctx: Context<'static>) {
        let channel = ctx
            .dma
            .channel0
            .configure_for_async(false, DmaPriority::Priority0);
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

        let mut i8080 = I8080::new(
            ctx.lcd_cam.lcd,
            channel.tx,
            ctx.tx_descriptors,
            pins,
            20.MHz(),
            Config::default(),
        );

        let xfer = i8080
            .send_dma(Command::<u8>::None, 0, &ctx.tx_buffer)
            .unwrap();
        xfer.wait().unwrap();
    }
}
