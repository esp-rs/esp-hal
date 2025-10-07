//! lcd_cam_i8080 and camera tests
//% CHIPS: esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    Async,
    Blocking,
    dma::{DmaChannel, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::Level,
    lcd_cam::{
        BitOrder,
        LcdCam,
        cam::{self, Camera, VhdeMode},
        lcd::{
            ClockMode,
            Phase,
            Polarity,
            dpi::{self, Dpi, Format, FrameTiming},
            i8080::{Command, Config, I8080},
        },
    },
    pcnt::{
        Pcnt,
        channel::{CtrlMode, EdgeMode},
    },
    peripherals::{DMA_CH0, Peripherals},
    time::Rate,
};
use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

#[allow(non_snake_case)]
struct Pins {
    pub GPIO8: esp_hal::peripherals::GPIO8<'static>,
    pub GPIO11: esp_hal::peripherals::GPIO11<'static>,
    pub GPIO12: esp_hal::peripherals::GPIO12<'static>,
    pub GPIO16: esp_hal::peripherals::GPIO16<'static>,
    pub GPIO17: esp_hal::peripherals::GPIO17<'static>,
}

struct AsyncContext<'d> {
    lcd_cam: LcdCam<'d, Async>,
    dma: DMA_CH0<'d>,
    dma_buf: DmaTxBuf,
}

struct BlockingContext<'d> {
    lcd_cam: LcdCam<'d, Blocking>,
    pcnt: Pcnt<'d>,
    pins: Pins,
    dma: DMA_CH0<'d>,
    dma_buf: DmaTxBuf,
}

struct CameraContext {
    peripherals: Peripherals,
    dma_tx_buf: DmaTxBuf,
    dma_rx_buf: DmaRxBuf,
}

// lcd_cam_i8080 tests
mod async_tests {
    use super::*;

    #[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
    mod tests {
        use super::*;

        #[init]
        async fn init() -> AsyncContext<'static> {
            let peripherals = esp_hal::init(esp_hal::Config::default());
            let lcd_cam = LcdCam::new(peripherals.LCD_CAM).into_async();
            let (_, _, tx_buffer, tx_desc) = dma_buffers!(0, DATA_SIZE);
            let dma_buf = DmaTxBuf::new(tx_desc, tx_buffer).unwrap();

            AsyncContext {
                lcd_cam,
                dma: peripherals.DMA_CH0,
                dma_buf,
            }
        }

        #[test]
        async fn test_i8080_8bit(ctx: AsyncContext<'static>) {
            let i8080 = I8080::new(
                ctx.lcd_cam.lcd,
                ctx.dma,
                Config::default().with_frequency(Rate::from_mhz(20)),
            )
            .unwrap();

            core::mem::drop(ctx.lcd_cam.cam);
            let mut transfer = i8080.send(Command::<u8>::None, 0, ctx.dma_buf).unwrap();

            transfer.wait_for_done().await;
            transfer.wait_for_done().await;
            transfer.wait().0.unwrap();
        }
    }
}

// lcd_cam_i8080 tests
mod blocking_tests {
    use super::*;

    #[embedded_test::tests(default_timeout = 3)]
    mod tests {
        use super::*;

        #[init]
        fn init() -> BlockingContext<'static> {
            let peripherals = esp_hal::init(esp_hal::Config::default());
            let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
            let pcnt = Pcnt::new(peripherals.PCNT);
            let (_, _, tx_buffer, tx_desc) = dma_buffers!(0, DATA_SIZE);
            let dma_buf = DmaTxBuf::new(tx_desc, tx_buffer).unwrap();

            BlockingContext {
                lcd_cam,
                dma: peripherals.DMA_CH0,
                pcnt,
                pins: Pins {
                    GPIO8: peripherals.GPIO8,
                    GPIO11: peripherals.GPIO11,
                    GPIO12: peripherals.GPIO12,
                    GPIO16: peripherals.GPIO16,
                    GPIO17: peripherals.GPIO17,
                },
                dma_buf,
            }
        }

        #[test]
        fn test_i8080_8bit(ctx: BlockingContext<'static>) {
            let i8080 = I8080::new(
                ctx.lcd_cam.lcd,
                ctx.dma,
                Config::default().with_frequency(Rate::from_mhz(20)),
            )
            .unwrap();

            let xfer = i8080.send(Command::<u8>::None, 0, ctx.dma_buf).unwrap();
            xfer.wait().0.unwrap();
        }

        #[test]
        fn test_i8080_8bit_is_seen_by_pcnt(ctx: BlockingContext<'static>) {
            let (unit_ctrl, cs_signal) = unsafe { ctx.pins.GPIO8.split() };
            let (unit0_input, unit0_signal) = unsafe { ctx.pins.GPIO11.split() };
            let (unit1_input, unit1_signal) = unsafe { ctx.pins.GPIO12.split() };
            let (unit2_input, unit2_signal) = unsafe { ctx.pins.GPIO16.split() };
            let (unit3_input, unit3_signal) = unsafe { ctx.pins.GPIO17.split() };

            let pcnt = ctx.pcnt;
            pcnt.unit0
                .channel0
                .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
            pcnt.unit1
                .channel0
                .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
            pcnt.unit2
                .channel0
                .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
            pcnt.unit3
                .channel0
                .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);

            pcnt.unit0
                .channel0
                .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
            pcnt.unit1
                .channel0
                .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
            pcnt.unit2
                .channel0
                .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
            pcnt.unit3
                .channel0
                .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

            let mut i8080 = I8080::new(
                ctx.lcd_cam.lcd,
                ctx.dma,
                Config::default().with_frequency(Rate::from_mhz(20)),
            )
            .unwrap()
            .with_cs(cs_signal)
            .with_data0(unit0_signal)
            .with_data1(unit1_signal)
            .with_data2(unit2_signal)
            .with_data3(unit3_signal);

            core::mem::drop(ctx.lcd_cam.cam);
            i8080.set_bit_order(BitOrder::Inverted);

            pcnt.unit0.channel0.set_edge_signal(unit0_input);
            pcnt.unit1.channel0.set_edge_signal(unit1_input);
            pcnt.unit2.channel0.set_edge_signal(unit2_input);
            pcnt.unit3.channel0.set_edge_signal(unit3_input);

            pcnt.unit0.channel0.set_ctrl_signal(unit_ctrl.clone());
            pcnt.unit1.channel0.set_ctrl_signal(unit_ctrl.clone());
            pcnt.unit2.channel0.set_ctrl_signal(unit_ctrl.clone());
            pcnt.unit3.channel0.set_ctrl_signal(unit_ctrl.clone());

            pcnt.unit0.resume();
            pcnt.unit1.resume();
            pcnt.unit2.resume();
            pcnt.unit3.resume();

            let data_to_send = [
                0b0000_0000,
                0b1010_0000,
                0b0110_0000,
                0b1110_0000,
                0b0000_0000,
                0b1000_0000,
                0b0100_0000,
                0b1010_0000,
                0b0101_0000,
                0b1000_0000,
            ];

            let mut dma_buf = ctx.dma_buf;
            dma_buf.as_mut_slice().fill(0);
            dma_buf.as_mut_slice()[..data_to_send.len()].copy_from_slice(&data_to_send);

            let xfer = i8080.send(Command::<u8>::None, 0, dma_buf).unwrap();
            xfer.wait().0.unwrap();

            let actual = [
                pcnt.unit0.value(),
                pcnt.unit1.value(),
                pcnt.unit2.value(),
                pcnt.unit3.value(),
            ];
            assert_eq!([5, 3, 2, 1], actual);
        }

        #[test]
        fn test_i8080_16bit_is_seen_by_pcnt(ctx: BlockingContext<'static>) {
            let (unit_ctrl, cs_signal) = unsafe { ctx.pins.GPIO8.split() };
            let (unit0_input, unit0_signal) = unsafe { ctx.pins.GPIO11.split() };
            let (unit1_input, unit1_signal) = unsafe { ctx.pins.GPIO12.split() };
            let (unit2_input, unit2_signal) = unsafe { ctx.pins.GPIO16.split() };
            let (unit3_input, unit3_signal) = unsafe { ctx.pins.GPIO17.split() };

            let pcnt = ctx.pcnt;
            pcnt.unit0
                .channel0
                .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
            pcnt.unit1
                .channel0
                .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
            pcnt.unit2
                .channel0
                .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
            pcnt.unit3
                .channel0
                .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);

            pcnt.unit0
                .channel0
                .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
            pcnt.unit1
                .channel0
                .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
            pcnt.unit2
                .channel0
                .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
            pcnt.unit3
                .channel0
                .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

            let mut i8080 = I8080::new(
                ctx.lcd_cam.lcd,
                ctx.dma,
                Config::default().with_frequency(Rate::from_mhz(20)),
            )
            .unwrap()
            .with_cs(cs_signal)
            .with_data3(unit0_signal)
            .with_data7(unit1_signal)
            .with_data11(unit2_signal)
            .with_data15(unit3_signal);

            i8080.set_bit_order(BitOrder::Inverted);

            pcnt.unit0.channel0.set_edge_signal(unit0_input);
            pcnt.unit1.channel0.set_edge_signal(unit1_input);
            pcnt.unit2.channel0.set_edge_signal(unit2_input);
            pcnt.unit3.channel0.set_edge_signal(unit3_input);

            pcnt.unit0.channel0.set_ctrl_signal(unit_ctrl.clone());
            pcnt.unit1.channel0.set_ctrl_signal(unit_ctrl.clone());
            pcnt.unit2.channel0.set_ctrl_signal(unit_ctrl.clone());
            pcnt.unit3.channel0.set_ctrl_signal(unit_ctrl.clone());

            pcnt.unit0.resume();
            pcnt.unit1.resume();
            pcnt.unit2.resume();
            pcnt.unit3.resume();

            let data_to_send = [
                0b0000_0000_0000_0000u16,
                0b0001_0000_0001_0000,
                0b0000_0001_0001_0000,
                0b0001_0001_0001_0000,
                0b0000_0000_0000_0000,
                0b0001_0000_0000_0000,
                0b0000_0001_0000_0000,
                0b0001_0000_0001_0000,
                0b0000_0001_0000_0001,
                0b0001_0000_0000_0000,
            ];

            let mut dma_buf = ctx.dma_buf;
            dma_buf
                .as_mut_slice()
                .iter_mut()
                .zip(data_to_send.iter().flat_map(|&d| d.to_ne_bytes()))
                .for_each(|(d, s)| *d = s);

            let xfer = i8080.send(Command::<u16>::None, 0, dma_buf).unwrap();
            xfer.wait().0.unwrap();

            let actual = [
                pcnt.unit0.value(),
                pcnt.unit1.value(),
                pcnt.unit2.value(),
                pcnt.unit3.value(),
            ];
            assert_eq!([5, 3, 2, 1], actual);
        }
    }
}

// LCD_CAM Camera and DPI tests
mod camera_tests {
    use super::*;

    #[embedded_test::tests]
    mod tests {
        use super::*;

        #[init]
        fn init() -> CameraContext {
            let peripherals = esp_hal::init(esp_hal::Config::default());
            let (rx_buf, rx_desc, tx_buf, tx_desc) = dma_buffers!(50 * 50);
            let dma_rx_buf = DmaRxBuf::new(rx_desc, rx_buf).unwrap();
            let dma_tx_buf = DmaTxBuf::new(tx_desc, tx_buf).unwrap();

            CameraContext {
                peripherals,
                dma_tx_buf,
                dma_rx_buf,
            }
        }

        #[test]
        fn test_camera_can_receive_from_rgb(ctx: CameraContext) {
            let peripherals = ctx.peripherals;
            let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
            let (rx_channel, tx_channel) = peripherals.DMA_CH2.split();

            let (
                (vsync_in, vsync_out),
                (hsync_in, hsync_out),
                (de_in, de_out),
                (pclk_in, pclk_out),
                (d0_in, d0_out),
                (d1_in, d1_out),
                (d2_in, d2_out),
                (d3_in, d3_out),
                (d4_in, d4_out),
                (d5_in, d5_out),
                (d6_in, d6_out),
                (d7_in, d7_out),
            ) = unsafe {
                (
                    peripherals.GPIO6.split(),
                    peripherals.GPIO7.split(),
                    peripherals.GPIO14.split(),
                    peripherals.GPIO13.split(),
                    peripherals.GPIO11.split(),
                    peripherals.GPIO9.split(),
                    peripherals.GPIO8.split(),
                    peripherals.GPIO47.split(),
                    peripherals.GPIO12.split(),
                    peripherals.GPIO18.split(),
                    peripherals.GPIO17.split(),
                    peripherals.GPIO16.split(),
                )
            };

            let config = dpi::Config::default()
                .with_clock_mode(ClockMode {
                    polarity: Polarity::IdleHigh,
                    phase: Phase::ShiftLow,
                })
                .with_frequency(Rate::from_khz(500))
                .with_format(Format {
                    enable_2byte_mode: false,
                    ..Default::default()
                })
                .with_timing(FrameTiming {
                    horizontal_total_width: 65,
                    hsync_width: 5,
                    horizontal_blank_front_porch: 10,
                    horizontal_active_width: 50,
                    vertical_total_height: 65,
                    vsync_width: 5,
                    vertical_blank_front_porch: 10,
                    vertical_active_height: 50,
                    hsync_position: 0,
                })
                .with_vsync_idle_level(Level::High)
                .with_hsync_idle_level(Level::High)
                .with_de_idle_level(Level::Low)
                .with_disable_black_region(false);

            let dpi = Dpi::new(lcd_cam.lcd, tx_channel, config)
                .unwrap()
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
                rx_channel,
                cam::Config::default()
                    .with_frequency(Rate::from_mhz(1))
                    .with_vh_de_mode(VhdeMode::VsyncHsync),
            )
            .unwrap()
            .with_vsync(vsync_in)
            .with_hsync(hsync_in)
            .with_h_enable(de_in)
            .with_pixel_clock(pclk_in)
            .with_data0(d0_in)
            .with_data1(d1_in)
            .with_data2(d2_in)
            .with_data3(d3_in)
            .with_data4(d4_in)
            .with_data5(d5_in)
            .with_data6(d6_in)
            .with_data7(d7_in);

            let mut dma_tx_buf = ctx.dma_tx_buf;
            let mut dma_rx_buf = ctx.dma_rx_buf;

            for (i, b) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
                *b = (i % 256) as u8;
            }

            let camera_transfer = camera.receive(dma_rx_buf).map_err(|e| e.0).unwrap();
            let dpi_transfer = dpi.send(true, dma_tx_buf).map_err(|e| e.0).unwrap();

            (_, _, dma_rx_buf) = camera_transfer.wait();
            (_, dma_tx_buf) = dpi_transfer.stop();

            assert_eq!(dma_tx_buf.as_slice(), dma_rx_buf.as_slice());
        }
    }
}
