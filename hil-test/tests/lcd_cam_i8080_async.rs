//! lcd_cam i8080 tests

//% CHIPS: esp32s3
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use esp_hal::{
    dma::{DmaChannel0, DmaTxBuf},
    dma_buffers,
    gpio::NoPin,
    lcd_cam::{
        lcd::i8080::{Command, Config, TxEightBits, I8080},
        LcdCam,
    },
    prelude::*,
    Async,
};
use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

struct Context<'d> {
    lcd_cam: LcdCam<'d, Async>,
    dma: DmaChannel0,
    dma_buf: DmaTxBuf,
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    async fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let lcd_cam = LcdCam::new(peripherals.LCD_CAM).into_async();
        let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, DATA_SIZE);
        let dma_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        Context {
            lcd_cam,
            dma: peripherals.DMA_CH0,
            dma_buf,
        }
    }

    #[test]
    async fn test_i8080_8bit(ctx: Context<'static>) {
        let pins = TxEightBits::new(NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin);

        let i8080 = I8080::new(ctx.lcd_cam.lcd, ctx.dma, pins, {
            let mut config = Config::default();
            config.frequency = 20.MHz();
            config
        })
        .unwrap();

        let mut transfer = i8080.send(Command::<u8>::None, 0, ctx.dma_buf).unwrap();

        transfer.wait_for_done().await;

        // This should not block forever and should immediately return.
        transfer.wait_for_done().await;

        transfer.wait().0.unwrap();
    }
}
