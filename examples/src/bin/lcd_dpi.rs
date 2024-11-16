//! Drives the 16-bit parallel RGB display on ESP32-S3-LCD-EV-Board v1.5
//!
//! This example fills the screen with every color.
//!
//! The following wiring is assumed:
//! - LCD_VSYNC  => GPIO3
//! - LCD_HSYNC  => GPIO46
//! - LCD_DE     => GPIO17
//! - LCD_PCLK   => GPIO9
//! - LCD_DATA0  => GPIO10
//! - LCD_DATA1  => GPIO11
//! - LCD_DATA2  => GPIO12
//! - LCD_DATA3  => GPIO13
//! - LCD_DATA4  => GPIO14
//! - LCD_DATA5  => GPIO21
//! - LCD_DATA6  => GPIO8
//! - LCD_DATA7  => GPIO18
//! - LCD_DATA8  => GPIO45
//! - LCD_DATA9  => GPIO38
//! - LCD_DATA10 => GPIO39
//! - LCD_DATA11 => GPIO40
//! - LCD_DATA12 => GPIO41
//! - LCD_DATA13 => GPIO42
//! - LCD_DATA14 => GPIO2
//! - LCD_DATA15 => GPIO1

//% CHIPS: esp32s3

#![no_std]
#![no_main]

use core::iter::{empty, once};

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_loop_buffer,
    gpio::{Level, Output},
    i2c,
    i2c::master::I2c,
    lcd_cam::{
        lcd::{
            dpi::{Config, Dpi, Format, FrameTiming},
            ClockMode,
            Phase,
            Polarity,
        },
        LcdCam,
    },
    peripherals::Peripherals,
    prelude::*,
    Blocking,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals: Peripherals = esp_hal::init(esp_hal::Config::default());

    let i2c = I2c::new(
        peripherals.I2C0,
        i2c::master::Config {
            frequency: 400.kHz(),
            ..Default::default()
        },
    )
    .with_sda(peripherals.GPIO47)
    .with_scl(peripherals.GPIO48);

    let dma = Dma::new(peripherals.DMA);
    let channel = dma.channel2.configure(true, DmaPriority::Priority0);
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let mut expander = Tca9554::new(i2c);
    expander.write_output_reg(0b1111_0011).unwrap();
    expander.write_direction_reg(0b1111_0001).unwrap();

    let delay = Delay::new();

    println!("Initialising");

    let mut write_byte = |b: u8, is_cmd: bool| {
        const SCS_BIT: u8 = 0b0000_0010;
        const SCL_BIT: u8 = 0b0000_0100;
        const SDA_BIT: u8 = 0b0000_1000;

        let mut output = 0b1111_0001 & !SCS_BIT;
        expander.write_output_reg(output).unwrap();

        for bit in once(!is_cmd).chain((0..8).map(|i| (b >> i) & 0b1 != 0).rev()) {
            let prev = output;
            if bit {
                output |= SDA_BIT;
            } else {
                output &= !SDA_BIT;
            }
            if prev != output {
                expander.write_output_reg(output).unwrap();
            }

            output &= !SCL_BIT;
            expander.write_output_reg(output).unwrap();

            output |= SCL_BIT;
            expander.write_output_reg(output).unwrap();
        }

        output &= !SCL_BIT;
        expander.write_output_reg(output).unwrap();

        output &= !SDA_BIT;
        expander.write_output_reg(output).unwrap();

        output |= SCS_BIT;
        expander.write_output_reg(output).unwrap();
    };

    let mut vsync_pin = peripherals.GPIO3;

    let vsync_must_be_high_during_setup = Output::new(&mut vsync_pin, Level::High);
    for &init in INIT_CMDS.iter() {
        match init {
            InitCmd::Cmd(cmd, args) => {
                write_byte(cmd, true);
                for &arg in args {
                    write_byte(arg, false);
                }
            }
            InitCmd::Delay(ms) => {
                delay.delay_millis(ms as _);
            }
        }
    }
    drop(vsync_must_be_high_during_setup);

    let mut dma_buf = dma_loop_buffer!(2 * 16);

    let config = Config {
        clock_mode: ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftLow,
        },
        format: Format {
            enable_2byte_mode: true,
            ..Default::default()
        },
        timing: FrameTiming {
            horizontal_active_width: 480,
            horizontal_total_width: 520,
            horizontal_blank_front_porch: 10,

            vertical_active_height: 480,
            vertical_total_height: 510,
            vertical_blank_front_porch: 10,

            hsync_width: 10,
            vsync_width: 10,

            hsync_position: 0,
        },
        vsync_idle_level: Level::High,
        hsync_idle_level: Level::High,
        de_idle_level: Level::Low,
        disable_black_region: false,
        ..Default::default()
    };

    let mut dpi = Dpi::new(lcd_cam.lcd, channel.tx, 16.MHz(), config)
        .with_vsync(vsync_pin)
        .with_hsync(peripherals.GPIO46)
        .with_de(peripherals.GPIO17)
        .with_pclk(peripherals.GPIO9)
        // Blue
        .with_data0(peripherals.GPIO10)
        .with_data1(peripherals.GPIO11)
        .with_data2(peripherals.GPIO12)
        .with_data3(peripherals.GPIO13)
        .with_data4(peripherals.GPIO14)
        // Green
        .with_data5(peripherals.GPIO21)
        .with_data6(peripherals.GPIO8)
        .with_data7(peripherals.GPIO18)
        .with_data8(peripherals.GPIO45)
        .with_data9(peripherals.GPIO38)
        .with_data10(peripherals.GPIO39)
        // Red
        .with_data11(peripherals.GPIO40)
        .with_data12(peripherals.GPIO41)
        .with_data13(peripherals.GPIO42)
        .with_data14(peripherals.GPIO2)
        .with_data15(peripherals.GPIO1);

    const MAX_RED: u16 = (1 << 5) - 1;
    const MAX_GREEN: u16 = (1 << 6) - 1;
    const MAX_BLUE: u16 = (1 << 5) - 1;

    fn rgb(r: u16, g: u16, b: u16) -> u16 {
        (r << 11) | (g << 5) | (b << 0)
    }

    let mut colors = empty()
        // Start with red and gradually add green
        .chain((0..=MAX_GREEN).map(|g| rgb(MAX_RED, g, 0)))
        // Then remove the red
        .chain((0..=MAX_RED).rev().map(|r| rgb(r, MAX_GREEN, 0)))
        // Then add blue
        .chain((0..=MAX_BLUE).map(|b| rgb(0, MAX_GREEN, b)))
        // Then remove green
        .chain((0..=MAX_GREEN).rev().map(|g| rgb(0, g, MAX_BLUE)))
        // Then add red
        .chain((0..=MAX_RED).map(|r| rgb(r, 0, MAX_BLUE)))
        // Then remove blue
        .chain((0..=MAX_BLUE).rev().map(|b| rgb(MAX_RED, 0, b)))
        // Once we get we have red, and we can start again.
        .cycle();

    println!("Rendering");
    loop {
        let transfer = dpi.send(false, dma_buf).map_err(|e| e.0).unwrap();
        (_, dpi, dma_buf) = transfer.wait();

        if let Some(color) = colors.next() {
            for chunk in dma_buf.chunks_mut(2) {
                chunk.copy_from_slice(&color.to_le_bytes());
            }
        }
    }
}

struct Tca9554 {
    i2c: I2c<'static, Blocking>,
    address: u8,
}

impl Tca9554 {
    pub fn new(i2c: I2c<'static, Blocking>) -> Self {
        Self { i2c, address: 0x20 }
    }

    pub fn write_direction_reg(&mut self, value: u8) -> Result<(), i2c::master::Error> {
        self.i2c.write(self.address, &[0x03, value])
    }

    pub fn write_output_reg(&mut self, value: u8) -> Result<(), i2c::master::Error> {
        self.i2c.write(self.address, &[0x01, value])
    }
}

#[derive(Copy, Clone)]
enum InitCmd {
    Cmd(u8, &'static [u8]),
    Delay(u8),
}

const INIT_CMDS: &[InitCmd] = &[
    InitCmd::Cmd(0xf0, &[0x55, 0xaa, 0x52, 0x08, 0x00]),
    InitCmd::Cmd(0xf6, &[0x5a, 0x87]),
    InitCmd::Cmd(0xc1, &[0x3f]),
    InitCmd::Cmd(0xc2, &[0x0e]),
    InitCmd::Cmd(0xc6, &[0xf8]),
    InitCmd::Cmd(0xc9, &[0x10]),
    InitCmd::Cmd(0xcd, &[0x25]),
    InitCmd::Cmd(0xf8, &[0x8a]),
    InitCmd::Cmd(0xac, &[0x45]),
    InitCmd::Cmd(0xa0, &[0xdd]),
    InitCmd::Cmd(0xa7, &[0x47]),
    InitCmd::Cmd(0xfa, &[0x00, 0x00, 0x00, 0x04]),
    InitCmd::Cmd(0x86, &[0x99, 0xa3, 0xa3, 0x51]),
    InitCmd::Cmd(0xa3, &[0xee]),
    InitCmd::Cmd(0xfd, &[0x3c, 0x3]),
    InitCmd::Cmd(0x71, &[0x48]),
    InitCmd::Cmd(0x72, &[0x48]),
    InitCmd::Cmd(0x73, &[0x00, 0x44]),
    InitCmd::Cmd(0x97, &[0xee]),
    InitCmd::Cmd(0x83, &[0x93]),
    InitCmd::Cmd(0x9a, &[0x72]),
    InitCmd::Cmd(0x9b, &[0x5a]),
    InitCmd::Cmd(0x82, &[0x2c, 0x2c]),
    InitCmd::Cmd(0xB1, &[0x10]),
    InitCmd::Cmd(
        0x6d,
        &[
            0x00, 0x1f, 0x19, 0x1a, 0x10, 0x0e, 0x0c, 0x0a, 0x02, 0x07, 0x1e, 0x1e, 0x1e, 0x1e,
            0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x08, 0x01, 0x09, 0x0b, 0x0d, 0x0f,
            0x1a, 0x19, 0x1f, 0x00,
        ],
    ),
    InitCmd::Cmd(
        0x64,
        &[
            0x38, 0x05, 0x01, 0xdb, 0x03, 0x03, 0x38, 0x04, 0x01, 0xdc, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x65,
        &[
            0x38, 0x03, 0x01, 0xdd, 0x03, 0x03, 0x38, 0x02, 0x01, 0xde, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x66,
        &[
            0x38, 0x01, 0x01, 0xdf, 0x03, 0x03, 0x38, 0x00, 0x01, 0xe0, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x67,
        &[
            0x30, 0x01, 0x01, 0xe1, 0x03, 0x03, 0x30, 0x02, 0x01, 0xe2, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x68,
        &[
            0x00, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(0x60, &[0x38, 0x08, 0x7a, 0x7a, 0x38, 0x09, 0x7a, 0x7a]),
    InitCmd::Cmd(0x63, &[0x31, 0xe4, 0x7a, 0x7a, 0x31, 0xe5, 0x7a, 0x7a]),
    InitCmd::Cmd(0x69, &[0x04, 0x22, 0x14, 0x22, 0x14, 0x22, 0x08]),
    InitCmd::Cmd(0x6b, &[0x07]),
    InitCmd::Cmd(0x7a, &[0x08, 0x13]),
    InitCmd::Cmd(0x7b, &[0x08, 0x13]),
    InitCmd::Cmd(
        0xd1,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd2,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd3,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd4,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd5,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd6,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(0x3A, &[0x66]),
    InitCmd::Cmd(0x11, &[]),
    InitCmd::Delay(120),
    InitCmd::Cmd(0x29, &[]),
    InitCmd::Delay(20),
];
