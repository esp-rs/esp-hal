//! Touch-controlled paint program for the ESP32-S31-Korvo-1 LCD subboard.
//!
//! Draw with a finger, choose a color and brush size from the toolbar, use the
//! eraser, or clear the canvas. The SET button restores the demo picture.

//% CHIP_FILTER: esp32s31
//% TAG: complex

#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics::prelude::Point;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    assign_resources,
    clock::{ClockConfig, CpuClock},
    timer::timg::TimerGroup,
};
use esp_println::println;

use crate::{
    display::Display,
    touch::{Touch, TouchEvent},
    ui::{TOOLBAR_HEIGHT, Toolbar},
};

mod display;
mod framebuf;
mod touch;
mod ui;

esp_bootloader_esp_idf::esp_app_desc!();

assign_resources! {
    pub Resources<'d> {
        display: DisplayResources<'d> {
            lcd_cam: LCD_CAM,
            dma: DMA_AXI_CH0,
            vsync: GPIO45,
            hsync: GPIO44,
            de: GPIO43,
            pclk: GPIO40,
            data0: GPIO8,
            data1: GPIO9,
            data2: GPIO10,
            data3: GPIO11,
            data4: GPIO12,
            data5: GPIO13,
            data6: GPIO14,
            data7: GPIO15,
            data8: GPIO16,
            data9: GPIO17,
            data10: GPIO18,
            data11: GPIO19,
            data12: GPIO33,
            data13: GPIO34,
            data14: GPIO35,
            data15: GPIO36,
        },
        touch: TouchResources<'d> {
            i2c: I2C0,
            sda: GPIO0,
            scl: GPIO1,
        },
    }
}

#[esp_hal::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = esp_hal::init(
        esp_hal::Config::default().with_cpu_clock(ClockConfig::from(CpuClock::max())),
    );
    esp_alloc::heap_allocator!(size: 48 * 1024);
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let timer = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer.timer0);

    let resources = split_resources!(peripherals);

    let mut display = Display::new(resources.display);
    let mut toolbar = Toolbar::new();
    let dirty = {
        let mut frame = framebuf::frame(&mut display);
        ui::clear_canvas(&mut frame);
        toolbar.draw(&mut frame);
        frame.data.dirty_rows()
    };
    display.writeback(dirty);

    let mut touch = Touch::new(resources.touch).await;
    println!("Touch Paint ready (Goodix at 0x{:02x})", touch.address());

    let mut adc_config = AdcConfig::new();
    let mut set_button = adc_config.enable_pin(peripherals.GPIO42, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config).unwrap();

    let mut was_down = false;
    let mut last_point: Option<Point> = None;
    let mut set_debounce = 0u8;
    let mut set_latched = false;

    loop {
        Timer::after(Duration::from_millis(15)).await;

        let raw = adc.read_blocking(&mut set_button).unwrap();
        let millivolts = 2000u32.saturating_sub((raw as u32 * 4000) / 4393) as u16;
        if (1605..1935).contains(&millivolts) {
            set_debounce = set_debounce.saturating_add(1);
            if set_debounce >= 3 && !set_latched {
                let dirty = {
                    let mut frame = framebuf::frame(&mut display);
                    ui::draw_demo_picture(&mut frame);
                    frame.data.dirty_rows()
                };
                display.writeback(dirty);
                last_point = None;
                set_latched = true;
                println!("SET: restored demo picture");
            }
        } else {
            set_debounce = 0;
            set_latched = false;
        }

        match touch.read().await {
            None => {}
            Some(TouchEvent::Up) => {
                toolbar.release();
                was_down = false;
                last_point = None;
            }
            Some(TouchEvent::Down(x, y)) => {
                let dirty = {
                    let mut frame = framebuf::frame(&mut display);
                    if y < TOOLBAR_HEIGHT {
                        if !was_down {
                            if toolbar.press(x, y) {
                                ui::clear_canvas(&mut frame);
                            }
                            toolbar.draw(&mut frame);
                        }
                        last_point = None;
                    } else {
                        let point = Point::new(x, y);
                        let (color, radius) = toolbar.brush();
                        let from = last_point.unwrap_or(point);
                        ui::draw_stroke(&mut frame, from, point, color, radius);
                        last_point = Some(point);
                    }
                    frame.data.dirty_rows()
                };
                display.writeback(dirty);
                was_down = true;
            }
        }
    }
}
