//! Voice assistant for the ESP32-S31-Korvo-1 LCD sub-board.
//!
//! This runs on the ESP32-S31-Korvo-1 with the 4.3" LCD sub-board. Tap the talk button on the
//! screen, say one of the phrases that the phrases button shows, then tap the talk button again.
//! The second core recognizes the phrase offline with MultiNet from Espressif's ESP-SR, through
//! the `esp-sr` crate, which provides its prebuilt libraries and model. The phrase sets the color
//! of the board's RGB LED and is shown on the screen.
//!
//! Hardware setup:
//! - Connect the LCD sub-board.
//! - Connect the USB-UART port to the computer and switch the board on.

//% CHIP_FILTER: esp32s31
//% TAG: complex

#![no_std]
#![no_main]

extern crate alloc;

mod audio;
mod display;
mod es8389;
mod led;
mod speech;
mod touch;

use alloc::{format, string::String, vec::Vec};
use core::ffi::CStr;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, with_timeout};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::Level,
    i2c::{self, master::I2c},
    interrupt::Priority,
    lcd_cam::{
        LcdCam,
        lcd::{
            ClockMode,
            Phase,
            Polarity,
            dpi::{Config as DpiConfig, Dpi, Format, FrameTiming},
        },
    },
    ram,
    system::{self, CpuControl},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_rtos::embassy::InterruptExecutor;
use static_cell::{ConstStaticCell, StaticCell};

use crate::{
    audio::Samples,
    display::{Button, Screen},
    led::{Color, Led},
    touch::Tap,
};

esp_bootloader_esp_idf::esp_app_desc!();

/// The phrases the assistant recognizes, see `speech`, and the RGB LED colors they set.
const PHRASES: &[(&CStr, Color)] = &[
    (c"turn the light red", Color::RED),
    (c"turn the light green", Color::GREEN),
    (c"turn the light blue", Color::BLUE),
    (c"turn the light yellow", Color::YELLOW),
    (c"turn the light purple", Color::PURPLE),
    (c"turn the light white", Color::WHITE),
    (c"turn off the light", Color::OFF),
];

/// Extra pronunciations of phrases in ESP-SR's phonemes, see [`esp_sr::Phrase`]. The t of "light"
/// runs into the next word, and a final consonant is often barely released, so MultiNet hears
/// `Li Wi` instead of `LiT WiT`, or `Li Rf` instead of `LiT RfD`.
const PRONUNCIATIONS: &[(&CStr, &[&CStr])] = &[
    (c"turn the light red", &[c"TkN jc Li RfD", c"TkN jc Li Rf"]),
    (
        c"turn the light white",
        &[c"TkN jc Li WiT", c"TkN jc Li Wi"],
    ),
];

/// How long the recording goes on after the tap that stops it.
const RECORDING_TAIL: Duration = Duration::from_millis(300);

// Full `.dram2_uninit` region for this chip.
const DRAM2_UNINIT_SIZE: usize = {
    let range: core::ops::Range<usize> = esp_metadata_generated::memory_range!("DRAM2_UNINIT");
    range.end - range.start
};

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Log on UART0's default TX pin.
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(peripherals.GPIO58);
    static LOG_WRITER: StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    // Large buffers are allocated explicitly in PSRAM, everything else in internal RAM.
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: DRAM2_UNINIT_SIZE);
    esp_alloc::heap_allocator!(size: 96 * 1024);
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // Speech recognition gets the second core to itself, see `speech`.
    static SPEECH_STACK: ConstStaticCell<system::Stack<{ 32 * 1024 }>> =
        ConstStaticCell::new(system::Stack::new());
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let _speech_core = cpu_control
        .start_app_core(SPEECH_STACK.take(), || speech::run(recognized_phrases()))
        .unwrap();

    // The timings of the 800x480 panel refresh it at about 35 Hz with an 18 MHz pixel clock.
    // See <https://github.com/espressif/esp-bsp/blob/67f41a169401/bsp/esp32_s3_lcd_ev_board/include/bsp/esp32_s3_lcd_ev_board.h#L378-L390>.
    let dpi_config = DpiConfig::default()
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftHigh,
        })
        .with_frequency(Rate::from_mhz(18))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: display::WIDTH as usize,
            horizontal_total_width: display::WIDTH as usize + 40 + 40 + 48,
            horizontal_blank_front_porch: 40 + 40,
            vertical_active_height: display::HEIGHT as usize,
            vertical_total_height: display::HEIGHT as usize + 23 + 32 + 13,
            vertical_blank_front_porch: 23 + 32,
            hsync_width: 40,
            vsync_width: 23,
            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
    let dpi = Dpi::new(lcd_cam.lcd, peripherals.DMA_AXI_CH0, dpi_config)
        .unwrap()
        .with_vsync(peripherals.GPIO45)
        .with_hsync(peripherals.GPIO44)
        .with_de(peripherals.GPIO43)
        .with_pclk(peripherals.GPIO40)
        // Blue
        .with_data0(peripherals.GPIO8)
        .with_data1(peripherals.GPIO9)
        .with_data2(peripherals.GPIO10)
        .with_data3(peripherals.GPIO11)
        .with_data4(peripherals.GPIO12)
        // Green
        .with_data5(peripherals.GPIO13)
        .with_data6(peripherals.GPIO14)
        .with_data7(peripherals.GPIO15)
        .with_data8(peripherals.GPIO16)
        .with_data9(peripherals.GPIO17)
        .with_data10(peripherals.GPIO18)
        // Red
        .with_data11(peripherals.GPIO19)
        .with_data12(peripherals.GPIO33)
        .with_data13(peripherals.GPIO34)
        .with_data14(peripherals.GPIO35)
        .with_data15(peripherals.GPIO36);
    spawner.spawn(display::run(dpi).unwrap());

    // Audio. The audio task must keep the DMA fed while other tasks block the thread executor, for
    // example while the display task draws a screen.
    static AUDIO_EXECUTOR: StaticCell<InterruptExecutor<2>> = StaticCell::new();
    let audio_spawner = AUDIO_EXECUTOR
        .init(InterruptExecutor::new(peripherals.FROM_CPU_INTR2))
        .start(Priority::Priority3);
    audio_spawner.spawn(
        audio::run(
            peripherals.I2S0,
            peripherals.DMA_CH0,
            peripherals.GPIO3.into(),
            peripherals.GPIO4.into(),
            peripherals.GPIO5.into(),
            peripherals.GPIO6.into(),
        )
        .unwrap(),
    );

    // The codec needs BCLK, which the audio task provides from now on.
    let mut i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO0)
        .with_scl(peripherals.GPIO1)
        .into_async();
    es8389::init(&mut i2c).await.unwrap();

    spawner.spawn(touch::run(i2c).unwrap());

    let mut led = Led::new(peripherals.RMT, peripherals.RMT_CH0, peripherals.GPIO37);
    led.set(Color::OFF).await;

    assistant(led).await
}

/// Runs conversation turns, one per tap.
async fn assistant(mut led: Led) -> ! {
    let mut screen = Screen {
        status: String::from("Loading speech model"),
        button: Button::Busy,
        you: String::new(),
        assistant: String::from(PHRASES_HINT),
    };
    display::show(&screen);
    // Loading the model takes so much PSRAM bandwidth that the display refresh can slip. The next
    // screen restarts the refresh, so it must come after the loading.
    speech::wait_until_ready().await;

    loop {
        screen.status = String::from("Ready");
        screen.button = Button::Talk;
        display::show(&screen);
        if touch::wait_for_tap().await == Tap::Phrases {
            screen.you.clear();
            screen.assistant = phrases();
            continue;
        }

        audio::start_recording().await;
        screen.status = String::from("Listening");
        screen.button = Button::Stop;
        display::show(&screen);
        let stop = async { while touch::wait_for_tap().await != Tap::Talk {} };
        let _ = with_timeout(audio::MAX_RECORDING, stop).await;
        // The tap often comes while the last word still ends, and MultiNet misses a phrase whose
        // final consonant is cut off.
        Timer::after(RECORDING_TAIL).await;
        let speech = audio::stop_recording().await;
        log::info!("Recorded {} samples", speech.len());

        screen.button = Button::Busy;
        converse(&mut led, &mut screen, speech).await;
    }
}

async fn converse(led: &mut Led, screen: &mut Screen, speech: Samples) {
    screen.status = String::from("Recognizing");
    display::show(screen);
    let Some(index) = speech::recognize(speech).await else {
        screen.you.clear();
        screen.assistant = format!("I did not recognize that. {PHRASES_HINT}");
        return;
    };
    let &(phrase, color) = &PHRASES[index];
    screen.you = String::from(phrase.to_str().unwrap());
    log::info!("You: {}", screen.you);
    led.set(color).await;
    screen.assistant = String::from("Done.");
}

const PHRASES_HINT: &str = "Tap Phrases to see what I understand.";

/// The phrases for the recognizer, with their extra pronunciations.
fn recognized_phrases() -> Vec<esp_sr::Phrase<'static>> {
    PHRASES
        .iter()
        .map(|&(text, _)| esp_sr::Phrase {
            text,
            pronunciations: PRONUNCIATIONS
                .iter()
                .find(|(phrase, _)| *phrase == text)
                .map_or(&[], |(_, pronunciations)| pronunciations),
        })
        .collect()
}

/// The phrases the assistant recognizes, for the screen.
fn phrases() -> String {
    let phrases: Vec<&str> = PHRASES
        .iter()
        .map(|(phrase, _)| phrase.to_str().unwrap())
        .collect();
    format!("Say one of: {}.", phrases.join(", "))
}
