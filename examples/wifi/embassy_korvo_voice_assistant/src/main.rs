//! Voice assistant for the ESP32-S31-Korvo-1 with the 4.3" LCD sub-board.
//!
//! Tap the button on the screen and speak, then tap it again. The recording is transcribed by the
//! OpenAI API, answered by a chat model, and the answer is shown on the screen and spoken on the
//! speakers.
//!
//! Set the `SSID`, `PASSWORD` and `OPENAI_API_KEY` environment variables before building. The API
//! key is stored in the firmware in plain text, and anyone with access to the board can read it.
//! Use a key with a spending limit.
//!
//! The server certificate is not verified yet.
//!
//! Hardware setup:
//! - Connect the LCD sub-board and one or two speakers.
//! - Connect the USB-UART port to the computer and switch the board on.

//% CHIP_FILTER: esp32s31

#![no_std]
#![no_main]

extern crate alloc;

mod audio;
mod display;
mod es8389;
mod openai;
mod touch;

use alloc::{format, string::String};

use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::{Duration, Timer, with_timeout};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    i2c::{self, master::I2c},
    i2s::master::{Channels, DataFormat, I2s, TdmConfig},
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
use static_cell::StaticCell;

use crate::{
    display::{Button, Screen},
    openai::{OpenAi, TcpState},
};

esp_bootloader_esp_idf::esp_app_desc!();

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

    // Display
    let dpi_config = DpiConfig::default()
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftLow,
        })
        .with_frequency(Rate::from_mhz(16))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: display::WIDTH as usize,
            horizontal_total_width: display::WIDTH as usize + 40,
            horizontal_blank_front_porch: 10,
            vertical_active_height: display::HEIGHT as usize,
            vertical_total_height: display::HEIGHT as usize + 30,
            vertical_blank_front_porch: 10,
            hsync_width: 10,
            vsync_width: 10,
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
    spawner.spawn(display::run(dpi, display::frame_buffer()).unwrap());

    // Audio: TX drives BCLK and WS, and RX shares them (`signal_loopback`).
    let i2s_config = TdmConfig::new_tdm_philips()
        .with_signal_loopback(true)
        .with_sample_rate(Rate::from_hz(audio::I2S_SAMPLE_RATE))
        .with_data_format(DataFormat::Data16Channel16)
        .with_channels(Channels::STEREO);
    let i2s = I2s::new(peripherals.I2S0, peripherals.DMA_CH0, i2s_config)
        .unwrap()
        .into_async();
    let speakers = i2s
        .i2s_tx
        .with_bclk(peripherals.GPIO3)
        .with_ws(peripherals.GPIO4)
        .with_dout(peripherals.GPIO5)
        .build()
        .write(esp_hal::dma_tx_stream_buffer!(16 * 1024, 2048))
        .unwrap();
    let microphones = i2s
        .i2s_rx
        .with_din(peripherals.GPIO6)
        .build()
        .read(esp_hal::dma_rx_stream_buffer!(16 * 1024, 2048))
        .unwrap();

    // The Korvo-1 needs a slow I2C clock for reliable codec access, see the `esp_codec_dev`
    // ES8389 test board.
    let mut i2c = I2c::new(
        peripherals.I2C0,
        i2c::master::Config::default().with_frequency(Rate::from_khz(10)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO0)
    .with_scl(peripherals.GPIO1);
    es8389::init(&mut i2c).unwrap();
    let _amplifiers = Output::new(peripherals.GPIO7, Level::High, OutputConfig::default());
    spawner.spawn(audio::run(speakers, microphones).unwrap());

    spawner.spawn(touch::run(i2c.into_async()).unwrap());

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

    assistant(stack).await
}

/// Runs conversation turns, one per tap.
async fn assistant(stack: Stack<'static>) -> ! {
    let mut screen = Screen {
        status: String::from("Connecting to Wi-Fi"),
        button: Button::Busy,
        you: String::new(),
        assistant: String::new(),
    };
    display::show(&screen);
    stack.wait_config_up().await;

    static TCP_STATE: StaticCell<TcpState> = StaticCell::new();
    let mut openai = OpenAi::new(stack, TCP_STATE.init(TcpState::new()));

    loop {
        screen.status = String::from("Ready");
        screen.button = Button::Talk;
        display::show(&screen);
        touch::wait_for_tap().await;

        audio::start_recording().await;
        screen.status = String::from("Listening");
        screen.button = Button::Stop;
        display::show(&screen);
        let _ = with_timeout(audio::MAX_RECORDING, touch::wait_for_tap()).await;
        let speech = audio::stop_recording().await;
        log::info!("Recorded {} samples", speech.len());

        screen.button = Button::Busy;
        if let Err(error) = converse(&mut openai, &mut screen, &speech).await {
            log::error!("{error}");
            screen.status = String::from("Error");
            screen.assistant = format!("{error}");
        }
    }
}

async fn converse(
    openai: &mut OpenAi,
    screen: &mut Screen,
    speech: &[i16],
) -> Result<(), openai::Error> {
    screen.status = String::from("Transcribing");
    display::show(screen);
    screen.you = openai.transcribe(speech).await?;
    screen.assistant.clear();
    log::info!("You: {}", screen.you);
    if screen.you.trim().is_empty() {
        screen.assistant = String::from("I did not hear anything.");
        return Ok(());
    }

    screen.status = String::from("Thinking");
    display::show(screen);
    screen.assistant = openai.chat(&screen.you).await?;
    log::info!("Assistant: {}", screen.assistant);

    screen.status = String::from("Speaking");
    display::show(screen);
    let answer = openai.speak(&screen.assistant).await?;
    audio::play(answer).await;
    Ok(())
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
