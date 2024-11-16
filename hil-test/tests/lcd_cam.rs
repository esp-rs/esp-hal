//! LCD_CAM Camera and DPI tests

//% CHIPS: esp32s3
//% FEATURES: defmt

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::Level,
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
    peripherals::Peripherals,
};
use fugit::RateExtU32;
use hil_test as _;

struct Context {
    peripherals: Peripherals,
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

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(50 * 50);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        Context {
            peripherals,
            dma_tx_buf,
            dma_rx_buf,
        }
    }

    #[test]
    fn test_camera_can_receive_from_rgb(ctx: Context) {
        let peripherals = ctx.peripherals;

        let dma = Dma::new(peripherals.DMA);
        let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

        let channel = dma.channel2.configure(false, DmaPriority::Priority0);

        let (vsync_in, vsync_out) = peripherals.GPIO6.split();
        let (hsync_in, hsync_out) = peripherals.GPIO7.split();
        let (de_in, de_out) = peripherals.GPIO14.split();
        let (pclk_in, pclk_out) = peripherals.GPIO13.split();
        let (d0_in, d0_out) = peripherals.GPIO11.split();
        let (d1_in, d1_out) = peripherals.GPIO9.split();
        let (d2_in, d2_out) = peripherals.GPIO8.split();
        let (d3_in, d3_out) = peripherals.GPIO47.split();
        let (d4_in, d4_out) = peripherals.GPIO12.split();
        let (d5_in, d5_out) = peripherals.GPIO18.split();
        let (d6_in, d6_out) = peripherals.GPIO17.split();
        let (d7_in, d7_out) = peripherals.GPIO16.split();

        let config = dpi::Config {
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
        };
        let dpi = Dpi::new(lcd_cam.lcd, channel.tx, 500u32.kHz(), config)
            .with_vsync(vsync_out)
            .with_hsync(hsync_out)
            .with_de(de_out)
            .with_pclk(pclk_out)
            .with_data0(d0_out)
            .with_data1(d1_out)
            .with_data2(d2_out)
            .with_data3(d3_out)
            .with_data4(d4_out)
            .with_data5(d5_out)
            .with_data6(d6_out)
            .with_data7(d7_out);

        let camera = Camera::new(
            lcd_cam.cam,
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
