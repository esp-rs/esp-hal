//! Drives the 8-bit parallel display on a WT32-SC01 Plus
//!
//! This example clears the screen with red and then blue every second.
//!
//! Pins used:
//!
//! Backlight GPIO45
//! Reset     GPIO4
//! CD        GPIO0
//! WR        GPIO47
//! D0        GPIO9
//! D1        GPIO46
//! D2        GPIO3
//! D3        GPIO8
//! D4        GPIO18
//! D5        GPIO17
//! D6        GPIO16
//! D7        GPIO15

//% CHIPS: esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::IO,
    lcd_cam::{
        lcd::i8080::{Config, TxEightBits, I8080},
        LcdCam,
    },
    peripherals::Peripherals,
    prelude::*,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let lcd_backlight = io.pins.gpio45;
    let lcd_reset = io.pins.gpio4;
    let lcd_rs = io.pins.gpio0; // Command/Data selection
    let lcd_wr = io.pins.gpio47; // Write clock
    let _lcd_te = io.pins.gpio48; // Frame sync

    let dma = Dma::new(peripherals.DMA);
    let channel = dma.channel0;

    let (tx_buffer, mut tx_descriptors, _, mut rx_descriptors) = dma_buffers!(32678, 0);

    let channel = channel.configure(
        false,
        &mut tx_descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    );

    let delay = Delay::new(&clocks);

    let mut backlight = lcd_backlight.into_push_pull_output();
    let mut reset = lcd_reset.into_push_pull_output();

    let tx_pins = TxEightBits::new(
        io.pins.gpio9,
        io.pins.gpio46,
        io.pins.gpio3,
        io.pins.gpio8,
        io.pins.gpio18,
        io.pins.gpio17,
        io.pins.gpio16,
        io.pins.gpio15,
    );

    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
    let mut i8080 = I8080::new(
        lcd_cam.lcd,
        channel.tx,
        tx_pins,
        20.MHz(),
        Config::default(),
        &clocks,
    )
    .with_ctrl_pins(lcd_rs, lcd_wr);

    {
        // https://gist.github.com/sukesh-ak/610508bc84779a26efdcf969bf51a2d1
        // https://github.com/lovyan03/LovyanGFX/blob/302169a6f23e9a2a6451f03311c366d182193831/src/lgfx/v1/panel/Panel_ST7796.hpp#L28

        reset.set_low();
        delay.delay_micros(8_000);
        reset.set_high();
        delay.delay_micros(64_000);

        // const CMD_FRMCTR1: u8 = 0xB1;
        // const CMD_FRMCTR2: u8 = 0xB2;
        // const CMD_FRMCTR3: u8 = 0xB3;
        const CMD_INVCTR: u8 = 0xB4;
        const CMD_DFUNCTR: u8 = 0xB6;
        // const CMD_ETMOD: u8 = 0xB7;
        // const CMD_PWCTR1: u8 = 0xC0;
        const CMD_PWCTR2: u8 = 0xC1;
        const CMD_PWCTR3: u8 = 0xC2;
        // const CMD_PWCTR4: u8 = 0xC3;
        // const CMD_PWCTR5: u8 = 0xC4;
        const CMD_VMCTR: u8 = 0xC5;
        const CMD_GMCTRP1: u8 = 0xE0; // Positive Gamma Correction
        const CMD_GMCTRN1: u8 = 0xE1; // Negative Gamma Correction
        const CMD_DOCA: u8 = 0xE8; // Display Output Ctrl Adjust
        const CMD_CSCON: u8 = 0xF0; // Command Set Control

        i8080.send(CMD_CSCON, 0, &[0xC3]).unwrap(); // Enable extension command 2 part I
        i8080.send(CMD_CSCON, 0, &[0x96]).unwrap(); // Enable extension command 2 part II
        i8080.send(CMD_INVCTR, 0, &[0x01]).unwrap(); // 1-dot inversion
        i8080
            .send(
                CMD_DFUNCTR,
                0,
                &[
                    0x80, // Display Function Control //Bypass
                    0x22, /* Source Output Scan from S1 to S960, Gate Output scan from G1 to
                           * G480, scan cycle=2 */
                    0x3B,
                ],
            )
            .unwrap(); // LCD Drive Line=8*(59+1)
        i8080
            .send(
                CMD_DOCA,
                0,
                &[
                    0x40, 0x8A, 0x00, 0x00, 0x29, // Source eqaulizing period time= 22.5 us
                    0x19, // Timing for "Gate start"=25 (Tclk)
                    0xA5, // Timing for "Gate End"=37 (Tclk), Gate driver EQ function ON
                    0x33,
                ],
            )
            .unwrap();
        i8080.send(CMD_PWCTR2, 0, &[0x06]).unwrap(); // Power control2   //VAP(GVDD)=3.85+( vcom+vcom offset), VAN(GVCL)=-3.85+(
                                                     // vcom+vcom offset)
        i8080.send(CMD_PWCTR3, 0, &[0xA7]).unwrap(); // Power control 3  //Source driving current level=low, Gamma driving current
                                                     // level=High
        i8080.send(CMD_VMCTR, 0, &[0x18]).unwrap(); // VCOM Control    //VCOM=0.9
        delay.delay_micros(120_000);
        i8080
            .send(
                CMD_GMCTRP1,
                0,
                &[
                    0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18,
                    0x1B,
                ],
            )
            .unwrap();
        i8080
            .send(
                CMD_GMCTRN1,
                0,
                &[
                    0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17,
                    0x1B,
                ],
            )
            .unwrap();
        delay.delay_micros(120_000);
        i8080.send(CMD_CSCON, 0, &[0x3C]).unwrap(); // Command Set control // Disable extension command 2 partI
        i8080.send(CMD_CSCON, 0, &[0x69]).unwrap(); // Command Set control // Disable
                                                    // extension command 2 partII

        i8080.send(0x11, 0, &[]).unwrap(); // ExitSleepMode
        delay.delay_micros(130_000);
        i8080.send(0x38, 0, &[]).unwrap(); // ExitIdleMode
        i8080.send(0x29, 0, &[]).unwrap(); // SetDisplayOn

        i8080.send(0x21, 0, &[]).unwrap(); // SetInvertMode(ColorInversion::Inverted)

        // let madctl = SetAddressMode::from(options);
        // i8080.send(madctl)?;

        i8080.send(0x3A, 0, &[0x55]).unwrap(); // RGB565
    }

    let width = 320u16;
    let height = 480u16;
    {
        println!("Set addresses");

        let width_b = width.to_be_bytes();
        let height_b = height.to_be_bytes();
        i8080
            .send(0x2A, 0, &[0, 0, width_b[0], width_b[1]])
            .unwrap(); // CASET
        i8080
            .send(0x2B, 0, &[0, 0, height_b[0], height_b[1]])
            .unwrap(); // PASET
    }

    println!("Drawing");

    const RED: u16 = 0b00000_000000_11111;
    const BLUE: u16 = 0b11111_000000_00000;

    backlight.set_high();

    let total_pixels = width as usize * height as usize;
    let total_bytes = total_pixels * 2;

    let buffer = tx_buffer;

    for color in [RED, BLUE].iter().cycle() {
        let color = color.to_be_bytes();
        for chunk in buffer.chunks_mut(2) {
            chunk.copy_from_slice(&color);
        }

        let mut bytes_left_to_write = total_bytes;

        let transfer = i8080.send_dma(0x2C, 0, &buffer).unwrap();
        transfer.wait().unwrap();

        bytes_left_to_write -= buffer.len();

        while bytes_left_to_write >= buffer.len() {
            let transfer = i8080.send_dma(0x3C, 0, &buffer).unwrap();
            transfer.wait().unwrap();

            bytes_left_to_write -= buffer.len();
        }
        if bytes_left_to_write > 0 {
            let transfer = i8080.send_dma(0x3C, 0, &buffer).unwrap();
            transfer.wait().unwrap();
        }

        delay.delay_millis(1_000);
    }

    loop {
        delay.delay_millis(1_000);
    }
}
