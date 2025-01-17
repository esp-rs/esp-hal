//! Drives the 8-bit parallel display on a WT32-SC01 Plus
//!
//! This example clears the screen with red and then blue every second.
//!
//! The following wiring is assumed:
//! - Backlight => GPIO45
//! - Reset     => GPIO4
//! - TE        => GPIO48
//! - CD        => GPIO0
//! - WR        => GPIO47
//! - D0        => GPIO9
//! - D1        => GPIO46
//! - D2        => GPIO3
//! - D3        => GPIO8
//! - D4        => GPIO18
//! - D5        => GPIO17
//! - D6        => GPIO16
//! - D7        => GPIO15

//% CHIPS: esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::DmaTxBuf,
    dma_tx_buffer,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    lcd_cam::{
        lcd::i8080::{Config, TxEightBits, I8080},
        LcdCam,
    },
    main,
    time::RateExtU32,
    Blocking,
};
use esp_println::println;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let lcd_backlight = peripherals.GPIO45;
    let lcd_reset = peripherals.GPIO4;
    let lcd_rs = peripherals.GPIO0; // Command/Data selection
    let lcd_wr = peripherals.GPIO47; // Write clock
    let lcd_te = peripherals.GPIO48; // Frame sync

    let dma_tx_buf = dma_tx_buffer!(4000).unwrap();

    let delay = Delay::new();

    let config = OutputConfig::default().with_level(Level::Low);
    let mut backlight = Output::new(lcd_backlight, config).unwrap();
    let mut reset = Output::new(lcd_reset, config).unwrap();
    let tear_effect = Input::new(lcd_te, InputConfig::default().with_pull(Pull::None)).unwrap();

    let tx_pins = TxEightBits::new(
        peripherals.GPIO9,
        peripherals.GPIO46,
        peripherals.GPIO3,
        peripherals.GPIO8,
        peripherals.GPIO18,
        peripherals.GPIO17,
        peripherals.GPIO16,
        peripherals.GPIO15,
    );

    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
    let mut i8080_config = Config::default();
    i8080_config.frequency = 20.MHz();
    let i8080 = I8080::new(lcd_cam.lcd, peripherals.DMA_CH0, tx_pins, i8080_config)
        .unwrap()
        .with_ctrl_pins(lcd_rs, lcd_wr);

    // Note: This isn't provided in the HAL since different drivers may require
    // different considerations, like how to manage the CS pin, the CD pin,
    // cancellation semantics, 8 vs 16 bit, non-native primitives like Rgb565,
    // Rgb888, etc. This Bus is just provided as an example of how to implement
    // your own.
    struct Bus<'d> {
        resources: Option<(I8080<'d, Blocking>, DmaTxBuf)>,
    }
    impl<'d> Bus<'d> {
        fn use_resources<T>(
            &mut self,
            func: impl FnOnce(I8080<'d, Blocking>, DmaTxBuf) -> (T, I8080<'d, Blocking>, DmaTxBuf),
        ) -> T {
            let (i8080, buf) = self.resources.take().unwrap();
            let (result, i8080, buf) = func(i8080, buf);
            self.resources = Some((i8080, buf));
            result
        }

        pub fn send(&mut self, cmd: u8, data: &[u8]) {
            self.use_resources(|i8080, mut buf| {
                buf.fill(data);
                match i8080.send(cmd, 0, buf) {
                    Ok(transfer) => transfer.wait(),
                    Err((result, i8080, buf)) => (Err(result), i8080, buf),
                }
            })
            .unwrap();
        }
    }

    let mut bus = Bus {
        resources: Some((i8080, dma_tx_buf)),
    };

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

        bus.send(CMD_CSCON, &[0xC3]); // Enable extension command 2 part I
        bus.send(CMD_CSCON, &[0x96]); // Enable extension command 2 part II
        bus.send(CMD_INVCTR, &[0x01]); // 1-dot inversion
        bus.send(
            CMD_DFUNCTR,
            &[
                0x80, // Display Function Control //Bypass
                0x22, /* Source Output Scan from S1 to S960, Gate Output scan from G1 to
                       * G480, scan cycle=2 */
                0x3B,
            ],
        ); // LCD Drive Line=8*(59+1)
        bus.send(
            CMD_DOCA,
            &[
                0x40, 0x8A, 0x00, 0x00, 0x29, // Source eqaulizing period time= 22.5 us
                0x19, // Timing for "Gate start"=25 (Tclk)
                0xA5, // Timing for "Gate End"=37 (Tclk), Gate driver EQ function ON
                0x33,
            ],
        );
        bus.send(CMD_PWCTR2, &[0x06]); // Power control2   //VAP(GVDD)=3.85+( vcom+vcom offset), VAN(GVCL)=-3.85+(
                                       // vcom+vcom offset)
        bus.send(CMD_PWCTR3, &[0xA7]); // Power control 3  //Source driving current level=low, Gamma driving current
                                       // level=High
        bus.send(CMD_VMCTR, &[0x18]); // VCOM Control    //VCOM=0.9
        delay.delay_micros(120_000);
        bus.send(
            CMD_GMCTRP1,
            &[
                0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B,
            ],
        );
        bus.send(
            CMD_GMCTRN1,
            &[
                0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B,
            ],
        );
        delay.delay_micros(120_000);
        bus.send(CMD_CSCON, &[0x3C]); // Command Set control // Disable extension command 2 partI
        bus.send(CMD_CSCON, &[0x69]); // Command Set control // Disable
                                      // extension command 2 partII

        bus.send(0x11, &[]); // ExitSleepMode
        delay.delay_micros(130_000);
        bus.send(0x38, &[]); // ExitIdleMode
        bus.send(0x29, &[]); // SetDisplayOn

        bus.send(0x21, &[]); // SetInvertMode(ColorInversion::Inverted)

        // let madctl = SetAddressMode::from(options);
        // bus.send(madctl)?;

        bus.send(0x3A, &[0x55]); // RGB565
    }

    // Tearing Effect Line On
    bus.send(0x35, &[0]);

    let width = 320u16;
    let height = 480u16;
    {
        println!("Set addresses");

        let width_b = width.to_be_bytes();
        let height_b = height.to_be_bytes();
        bus.send(0x2A, &[0, 0, width_b[0], width_b[1]]); // CASET
        bus.send(0x2B, &[0, 0, height_b[0], height_b[1]]) // PASET
    }

    println!("Drawing");

    const RED: u16 = 0b00000_000000_11111;
    const BLUE: u16 = 0b11111_000000_00000;

    backlight.set_high();

    let total_pixels = width as usize * height as usize;
    let total_bytes = total_pixels * 2;

    let (mut i8080, mut dma_tx_buf) = bus.resources.take().unwrap();

    dma_tx_buf.set_length(dma_tx_buf.capacity());

    for color in [RED, BLUE].iter().cycle() {
        let color = color.to_be_bytes();
        for chunk in dma_tx_buf.as_mut_slice().chunks_mut(2) {
            chunk.copy_from_slice(&color);
        }

        // Naive implementation of tear prevention. A more robust implementation would
        // use an interrupt handler to start shipping out the next frame the
        // moment the tear effect pin goes high. async/await would be too slow
        // and would risk missing the inter-refresh window.
        {
            // Wait for display to start refreshing.
            while tear_effect.is_high() {}
            // Wait for display to finish refreshing.
            while tear_effect.is_low() {}

            // Now we have the maximum amount of time between each refresh
            // available, for drawing.
        }

        let mut bytes_left_to_write = total_bytes;

        (_, i8080, dma_tx_buf) = i8080.send(0x2Cu8, 0, dma_tx_buf).unwrap().wait();

        bytes_left_to_write -= dma_tx_buf.len();

        while bytes_left_to_write >= dma_tx_buf.len() {
            (_, i8080, dma_tx_buf) = i8080.send(0x3Cu8, 0, dma_tx_buf).unwrap().wait();
            bytes_left_to_write -= dma_tx_buf.len();
        }
        if bytes_left_to_write > 0 {
            dma_tx_buf.set_length(bytes_left_to_write);
            (_, i8080, dma_tx_buf) = i8080.send(0x3Cu8, 0, dma_tx_buf).unwrap().wait();
            dma_tx_buf.set_length(dma_tx_buf.capacity());
        }

        delay.delay_millis(1_000);
    }

    loop {
        delay.delay_millis(1_000);
    }
}
