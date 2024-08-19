//! lcd_cam i8080 tests

//% CHIPS: esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    clock::{ClockControl, Clocks},
    dma::{Dma, DmaDescriptor, DmaPriority},
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
use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

struct Context<'d> {
    lcd_cam: LcdCam<'d, esp_hal::Async>,
    clocks: Clocks<'d>,
    dma: Dma<'d>,
    tx_buffer: &'static [u8],
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> Context<'d> {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
        let dma = Dma::new(peripherals.DMA);
        let lcd_cam = LcdCam::new_async(peripherals.LCD_CAM);
        let (tx_buffer, tx_descriptors, _, _) = dma_buffers!(DATA_SIZE, 0);

        Self {
            lcd_cam,
            clocks,
            dma,
            tx_buffer,
            tx_descriptors,
        }
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    async fn init() -> Context<'static> {
        Context::init()
    }

    #[test]
    async fn test_i8080_8bit(ctx: Context<'static>) {
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
            i8080::Config::default(),
            &ctx.clocks,
        );

        i8080
            .send_dma_async(Command::<u8>::None, 0, &ctx.tx_buffer)
            .await
            .unwrap();
    }

    #[test]
    async fn test_i8080_8bit_async_channel(ctx: Context<'static>) {
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
            i8080::Config::default(),
            &ctx.clocks,
        );

        i8080
            .send_dma_async(Command::<u8>::None, 0, &ctx.tx_buffer)
            .await
            .unwrap();
    }
}
