//! Embassy Voice Assistant Example
//!
//! Set SSID, PASSWORD, ANTHROPIC_API_KEY and ESP_SR_COMPONENTS env variables before running this
//! example.
//!
//! This runs on the ESP32-S31-Korvo-1 with the 4.3" LCD sub-board. Tap the talk button on the
//! screen, say one of the phrases that the phrases button shows, then tap the talk button again.
//! The second core recognizes the phrase with MultiNet from Espressif's ESP-SR. Claude answers
//! questions, and light commands set the color of the board's RGB LED. The phrase and the answer
//! are shown on the screen.
//!
//! The API key is stored in the firmware in plain text, and anyone with access to the board can
//! read it. Use a key with a spending limit.
//!
//! Building needs:
//! - Espressif's RISC-V GCC toolchain (`riscv32-esp-elf-gcc` and `riscv32-esp-elf-ld`, 15.2 or
//!   later, with picolibc) in `PATH`. The ESP-SR libraries need GNU ld.
//! - ESP_SR_COMPONENTS set to a directory with the ESP-IDF components `espressif__esp-sr`,
//!   `espressif__dl_fft` and `espressif__cjson`, for example the `managed_components` directory of
//!   an ESP-IDF project that depends on `espressif/esp-sr`. `src/speech.rs` mirrors C types of
//!   esp-sr 2.6.0.
//!
//! Hardware setup:
//! - Connect the LCD sub-board.
//! - Connect the USB-UART port to the computer and switch the board on.

//% CHIP_FILTER: esp32s31

#![no_std]
#![no_main]

extern crate alloc;

mod api;
mod audio;
mod display;
mod es8389;
mod idf;
mod led;
mod speech;
mod touch;

use alloc::{format, string::String, vec::Vec};
use core::ffi::CStr;

use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
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
    rng::Rng,
    system::{self, CpuControl},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_radio::wifi::{
    AuthenticationMethodConfig,
    Config,
    ControllerConfig,
    Interface,
    WifiController,
    sta::StationConfig,
};
use esp_rtos::embassy::InterruptExecutor;
use static_cell::{ConstStaticCell, StaticCell};

use crate::{
    api::Api,
    audio::Samples,
    display::{Button, Screen},
    led::{Color, Led},
    touch::Tap,
};

esp_bootloader_esp_idf::esp_app_desc!();

/// What the assistant does with a recognized phrase.
#[derive(Clone, Copy)]
enum Action {
    /// Claude answers the phrase.
    Ask,
    /// The RGB LED shows the color.
    Light(Color),
}

/// The phrases the assistant recognizes, see `speech`.
const PHRASES: &[(&CStr, Action)] = &[
    (c"tell me a joke", Action::Ask),
    (c"tell me a fun fact", Action::Ask),
    (c"tell me a short story", Action::Ask),
    (c"say something nice", Action::Ask),
    (c"who are you", Action::Ask),
    (c"what can you do", Action::Ask),
    (c"why is the sky blue", Action::Ask),
    (c"how do airplanes fly", Action::Ask),
    (c"how far away is the moon", Action::Ask),
    (c"how hot is the sun", Action::Ask),
    (c"what is the speed of light", Action::Ask),
    (c"what is the tallest mountain", Action::Ask),
    (c"what is the capital of france", Action::Ask),
    (c"who invented the telephone", Action::Ask),
    (c"how many people live on earth", Action::Ask),
    (c"recommend a good book", Action::Ask),
    (c"give me a recipe for pancakes", Action::Ask),
    (c"what is a microcontroller", Action::Ask),
    (c"turn the light red", Action::Light(Color::RED)),
    (c"turn the light green", Action::Light(Color::GREEN)),
    (c"turn the light blue", Action::Light(Color::BLUE)),
    (c"turn the light yellow", Action::Light(Color::YELLOW)),
    (c"turn the light purple", Action::Light(Color::PURPLE)),
    (c"turn the light white", Action::Light(Color::WHITE)),
    (c"turn off the light", Action::Light(Color::OFF)),
];

/// Extra pronunciations of phrases in ESP-SR's phonemes, see `speech::run`. The t of "light" runs
/// into the next word, and a final consonant is often barely released, so MultiNet hears `Li Wi`
/// instead of `LiT WiT`, or `Li Rf` instead of `LiT RfD`.
const PRONUNCIATIONS: &[(&CStr, &CStr)] = &[
    (c"turn the light red", c"TkN jc Li RfD"),
    (c"turn the light red", c"TkN jc Li Rf"),
    (c"turn the light white", c"TkN jc Li WiT"),
    (c"turn the light white", c"TkN jc Li Wi"),
];

/// How long the recording goes on after the tap that stops it.
const RECORDING_TAIL: Duration = Duration::from_millis(300);

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

// Full `.dram2_uninit` region for this chip.
const DRAM2_UNINIT_SIZE: usize = {
    let range: core::ops::Range<usize> = esp_metadata_generated::memory_range!("DRAM2_UNINIT");
    range.end - range.start
};

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

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
        .start_app_core(SPEECH_STACK.take(), || {
            speech::run(PHRASES.iter().map(|(phrase, _)| *phrase), PRONUNCIATIONS)
        })
        .unwrap();

    // Display, with the sub-board timings from Espressif's `esp32_s3_lcd_ev_board` BSP
    // (`SUB_BOARD3_800_480_PANEL_35HZ_RGB_TIMING`).
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
    // example while a TLS connection is set up.
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

    // Wi-Fi
    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().unwrap())
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().unwrap(),
            )),
    );
    let controller = WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();

    let rng = Rng::new();
    let seed = u64::from(rng.random()) << 32 | u64::from(rng.random());
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        Interface::station(),
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner.spawn(connection(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());

    let mut led = Led::new(peripherals.RMT, peripherals.GPIO37);
    led.set(Color::OFF).await;

    assistant(stack, led).await
}

/// Runs conversation turns, one per tap.
async fn assistant(stack: Stack<'static>, mut led: Led) -> ! {
    let mut screen = Screen {
        status: String::from("Connecting to Wi-Fi"),
        button: Button::Busy,
        you: String::new(),
        assistant: String::from(PHRASES_HINT),
    };
    display::show(&screen);
    stack.wait_config_up().await;

    let mut api = Api::new(stack);

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
        if let Err(error) = converse(&mut api, &mut led, &mut screen, speech).await {
            log::error!("{error}");
            screen.status = String::from("Error");
            screen.assistant = format!("{error}");
        }
    }
}

async fn converse(
    api: &mut Api,
    led: &mut Led,
    screen: &mut Screen,
    speech: Samples,
) -> Result<(), api::Error> {
    screen.status = String::from("Recognizing");
    display::show(screen);
    let Some(index) = speech::recognize(speech).await else {
        screen.you.clear();
        screen.assistant = format!("I did not recognize that. {PHRASES_HINT}");
        return Ok(());
    };
    let (phrase, action) = PHRASES[index];
    screen.you = String::from(phrase.to_str().unwrap());
    log::info!("You: {}", screen.you);
    if let Action::Light(color) = action {
        led.set(color).await;
        screen.assistant = String::from("Done.");
        return Ok(());
    }

    screen.assistant.clear();
    screen.status = String::from("Thinking");
    display::show(screen);
    screen.assistant = api.chat(&screen.you).await?;
    log::info!("Assistant: {}", screen.assistant);
    if screen.assistant.trim().is_empty() {
        screen.assistant = String::from("Claude did not answer.");
    }
    Ok(())
}

const PHRASES_HINT: &str = "Tap Phrases to see what I understand.";

/// The phrases the assistant recognizes, for the screen.
fn phrases() -> String {
    let list = |light: bool| {
        let phrases: Vec<&str> = PHRASES
            .iter()
            .filter(|(_, action)| matches!(action, Action::Light(_)) == light)
            .map(|(phrase, _)| phrase.to_str().unwrap())
            .collect();
        phrases.join(", ")
    };
    format!("Ask one of: {}. Or say: {}.", list(false), list(true))
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    loop {
        match controller.connect_async().await {
            Ok(info) => {
                log::info!("Wi-Fi connected to {info:?}");
                let info = controller.wait_for_disconnect_async().await.ok();
                log::info!("Wi-Fi disconnected: {info:?}");
            }
            Err(error) => log::error!("Failed to connect to Wi-Fi: {error:?}"),
        }
        Timer::after(Duration::from_secs(5)).await
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}
