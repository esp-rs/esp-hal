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
    framebuf::FrameBuf,
    touch::{Touch, TouchEvent},
    ui::{CANVAS_COLOR, PALETTE, TOOLBAR_HEIGHT, ToolbarButton},
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

    let mut selected = 1usize;
    let mut radius = 7i32;
    let mut eraser = false;

    let mut display = Display::new(resources.display);
    let dirty = {
        let mut frame = FrameBuf::new(&mut display);
        frame.fill_rect(
            0,
            0,
            display::WIDTH as i32,
            display::HEIGHT as i32,
            CANVAS_COLOR,
        );
        ui::draw_toolbar(&mut frame, selected, radius, eraser);
        frame.dirty_rows()
    };
    display.writeback(dirty);

    let mut touch = Touch::new(resources.touch).await;
    println!("Touch Paint ready (Goodix at 0x{:02x})", touch.address());

    let mut adc_config = AdcConfig::new();
    let mut set_button = adc_config.enable_pin(peripherals.GPIO42, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config).unwrap();

    let mut was_down = false;
    let mut last_point: Option<(i32, i32)> = None;
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
                    let mut frame = FrameBuf::new(&mut display);
                    ui::draw_demo_picture(&mut frame);
                    ui::draw_toolbar(&mut frame, selected, radius, eraser);
                    frame.dirty_rows()
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
                was_down = false;
                last_point = None;
            }
            Some(TouchEvent::Down(x, y)) => {
                let dirty = {
                    let mut frame = FrameBuf::new(&mut display);
                    if y < TOOLBAR_HEIGHT {
                        if !was_down {
                            match ui::toolbar_button(x) {
                                Some(ToolbarButton::Color(index)) => {
                                    selected = index;
                                    eraser = false;
                                }
                                Some(ToolbarButton::BrushSize(size)) => radius = size,
                                Some(ToolbarButton::Eraser) => eraser = true,
                                Some(ToolbarButton::Clear) => ui::clear_canvas(&mut frame),
                                None => {}
                            }
                            ui::draw_toolbar(&mut frame, selected, radius, eraser);
                        }
                        last_point = None;
                    } else {
                        let point = (x, y);
                        let color = if eraser {
                            CANVAS_COLOR
                        } else {
                            PALETTE[selected]
                        };
                        frame.draw_line(last_point.unwrap_or(point), point, radius, color);
                        last_point = Some(point);
                    }
                    frame.dirty_rows()
                };
                display.writeback(dirty);
                was_down = true;
            }
        }
    }
}
