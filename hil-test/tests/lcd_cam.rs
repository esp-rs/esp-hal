//! LCD_CAM Camera and DPI tests

//% CHIPS: esp32s3
//% FEATURES: defmt

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{
        interconnect::{InputSignal, OutputSignal},
        GpioPin,
        Io,
        Level,
        NoPin,
        OutputPin,
    },
    lcd_cam::{
        cam::{Camera, RxEightBits},
        lcd::{
            dpi,
            dpi::{Dpi, Format, FrameTiming},
            ClockMode,
            Phase,
            Polarity,
        },
        LcdCam,
    },
    Blocking,
};
use fugit::RateExtU32;
use hil_test as _;

struct Context {
    io: Io,
    dma: Dma<'static>,
    lcd_cam: LcdCam<'static, Blocking>,
    dma_tx_buf: DmaTxBuf,
    dma_rx_buf: DmaRxBuf,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let dma = Dma::new(peripherals.DMA);
        let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(50 * 50);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        Context {
            io,
            dma,
            lcd_cam,
            dma_tx_buf,
            dma_rx_buf,
        }
    }

    #[test]
    fn test_camera_can_receive_from_rgb(ctx: Context) {
        let channel = ctx.dma.channel2.configure(false, DmaPriority::Priority0);

        fn split_pin<const NUM: u8>(pin: GpioPin<NUM>) -> (InputSignal, OutputSignal)
        where
            GpioPin<NUM>: OutputPin,
        {
            let input = pin.peripheral_input();
            let output = pin.into_peripheral_output();
            (input, output)
        }

        let (vsync_in, vsync_out) = split_pin(ctx.io.pins.gpio6);
        let (hsync_in, hsync_out) = split_pin(ctx.io.pins.gpio7);
        let (de_in, de_out) = split_pin(ctx.io.pins.gpio14);
        let (pclk_in, pclk_out) = split_pin(ctx.io.pins.gpio13);
        let (d0_in, d0_out) = split_pin(ctx.io.pins.gpio11);
        let (d1_in, d1_out) = split_pin(ctx.io.pins.gpio9);
        let (d2_in, d2_out) = split_pin(ctx.io.pins.gpio8);
        let (d3_in, d3_out) = split_pin(ctx.io.pins.gpio47);
        let (d4_in, d4_out) = split_pin(ctx.io.pins.gpio12);
        let (d5_in, d5_out) = split_pin(ctx.io.pins.gpio18);
        let (d6_in, d6_out) = split_pin(ctx.io.pins.gpio17);
        let (d7_in, d7_out) = split_pin(ctx.io.pins.gpio16);

        let dpi = Dpi::new(
            ctx.lcd_cam.lcd,
            channel.tx,
            500u32.kHz(),
            dpi::Config {
                clock_mode: ClockMode {
                    polarity: Polarity::IdleHigh,
                    phase: Phase::ShiftLow,
                },
                format: Format {
                    enable_2byte_mode: false,
                    ..Default::default()
                },
                // Send a 50x50 video
                timing: FrameTiming {
                    horizontal_total_width: 65,
                    hsync_width: 5,
                    horizontal_blank_front_porch: 10,
                    horizontal_active_width: 50,

                    vertical_total_height: 65,
                    vsync_width: 5,
                    vertical_blank_front_porch: 10,
                    vertical_active_height: 50,

                    hsync_position: 0,
                },
                vsync_idle_level: Level::High,
                hsync_idle_level: Level::High,
                de_idle_level: Level::Low,
                disable_black_region: false,
                ..Default::default()
            },
        )
        .with_ctrl_pins(vsync_out, hsync_out, de_out, pclk_out)
        .with_data_pins(
            d0_out, d1_out, d2_out, d3_out, d4_out, d5_out, d6_out, d7_out, NoPin, NoPin, NoPin,
            NoPin, NoPin, NoPin, NoPin, NoPin,
        );

        let camera = Camera::new(
            ctx.lcd_cam.cam,
            channel.rx,
            RxEightBits::new(d0_in, d1_in, d2_in, d3_in, d4_in, d5_in, d6_in, d7_in),
            1u32.MHz(),
        )
        .with_ctrl_pins_and_de(vsync_in, hsync_in, de_in)
        .with_pixel_clock(pclk_in);

        let mut dma_tx_buf = ctx.dma_tx_buf;
        let mut dma_rx_buf = ctx.dma_rx_buf;

        for (i, b) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *b = ((i + 0) % 256) as u8;
        }

        let camera_transfer = camera.receive(dma_rx_buf).map_err(|e| e.0).unwrap();
        // Note: next_frame_en is not false because the RGB driver doesn't send a VSYNC
        // at the end of the frame, which means the DMA doesn't flush the last
        // few bytes it receives.
        let dpi_transfer = dpi.send(true, dma_tx_buf).map_err(|e| e.0).unwrap();

        (_, _, dma_rx_buf) = camera_transfer.wait();
        (_, dma_tx_buf) = dpi_transfer.stop();

        assert_eq!(dma_tx_buf.as_slice(), dma_rx_buf.as_slice());
    }
}
