//! lcd_cam i8080 tests

//% CHIPS: esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    Async,
    dma::DmaTxBuf,
    dma_buffers,
    gpio::NoPin,
    lcd_cam::{
        LcdCam,
        lcd::i8080::{Command, Config, I8080, TxEightBits},
    },
    peripherals::DMA_CH0,
    time::Rate,
};
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

const DATA_SIZE: usize = 1024 * 10;

struct Context<'d> {
    lcd_cam: LcdCam<'d, Async>,
    dma: DMA_CH0<'d>,
    dma_buf: DmaTxBuf,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
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

        let i8080 = I8080::new(
            ctx.lcd_cam.lcd,
            ctx.dma,
            pins,
            Config::default().with_frequency(Rate::from_mhz(20)),
        )
        .unwrap();

        // explicitly drop the camera half to see if it disables clocks (unexpectedly,
        // I8080 should keep it alive)
        core::mem::drop(ctx.lcd_cam.cam);

        let mut transfer = i8080.send(Command::<u8>::None, 0, ctx.dma_buf).unwrap();

        transfer.wait_for_done().await;

        // This should not block forever and should immediately return.
        transfer.wait_for_done().await;

        transfer.wait().0.unwrap();
    }
}
