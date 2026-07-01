//! PDM RX from a PDM microphone, similar to ESP-IDF `i2s_pdm_rx.c`.
//!
//! Connect a PDM microphone with `SEL` tied to GND (left channel):
//!
//! ```text
//!   ESP                         PDM mic
//!   GPIO4 (CLK)  ─────────────► CLK
//!   GPIO5 (DIN)  ◄───────────── DATA
//!   GND          ────────────── GND / SEL
//!   3V3          ────────────── VCC
//! ```
//!
//! Reads 16 kHz mono PCM on chips with a hardware PDM2PCM filter (ESP32, ESP32-S3, …),
//! or 2.048 MHz raw PDM oversampling on chips without one (ESP32-C6, …), matching ESP-IDF.
//! Values are printed as `i16` like IDF's example — on raw-PDM targets that means line
//! bitstream words, not decoded PCM.

//% CHIP_FILTER: i2s_supports_pdm_rx
//% FEATURES: unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};
use esp_backtrace as _;
use esp_hal::{
    dma_rx_stream_buffer,
    i2s::master::{I2s, PdmConfig, PdmRxConfig, PdmSlotMode},
    interrupt::software::SoftwareInterruptControl,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

/// Bytes shown per log line (matches IDF `EXAMPLE_BUFF_SIZE`).
const PRINT_BYTES: usize = 2048;
const POP_BUF_SIZE: usize = 8192;

fn sample_i16(data: &[u8], index: usize) -> i16 {
    let offset = index * 2;
    i16::from_le_bytes([data[offset], data[offset + 1]])
}

fn print_head_samples(data: &[u8], count: usize) {
    let samples = count / 2;
    if samples >= 8 {
        println!(
            "[0] {} [1] {} [2] {} [3] {}\n[4] {} [5] {} [6] {} [7] {}",
            sample_i16(data, 0),
            sample_i16(data, 1),
            sample_i16(data, 2),
            sample_i16(data, 3),
            sample_i16(data, 4),
            sample_i16(data, 5),
            sample_i16(data, 6),
            sample_i16(data, 7),
        );
    } else if count >= 2 {
        println!("[0] {}", sample_i16(data, 0));
    } else if count > 0 {
        println!("bytes: {:x?}", &data[..count]);
    }
}

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    println!("embassy_i2s_pdm_rx: init");

    let peripherals = esp_hal::init(esp_hal::Config::default());
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let dma_channel = cfg_select! {
        any(feature = "esp32", feature = "esp32s2") => peripherals.DMA_I2S0,
        _ => peripherals.DMA_CH0,
    };

    cfg_select! {
        any(feature = "esp32", feature = "esp32s3", feature = "esp32p4") => {
            println!("embassy_i2s_pdm_rx: PDM RX in PCM format");
        }
        _ => {
            println!("embassy_i2s_pdm_rx: PDM RX in raw PDM format");
        }
    }

    let rx_cfg = cfg_select! {
        any(feature = "esp32", feature = "esp32s3", feature = "esp32p4") => {
            PdmRxConfig::new_pcm_default(Rate::from_hz(16_000), PdmSlotMode::Mono)
        }
        _ => {
            PdmRxConfig::new_raw_default(Rate::from_hz(2_048_000), PdmSlotMode::Mono)
        }
    };
    let pdm_cfg = PdmConfig::rx_only(rx_cfg);

    let i2s = match I2s::new_pdm(peripherals.I2S0, dma_channel, pdm_cfg) {
        Ok(i2s) => i2s.into_async(),
        Err(err) => {
            println!("I2S PDM RX init failed: {err:?}");
            loop {
                core::hint::spin_loop();
            }
        }
    };

    let i2s_rx = i2s
        .i2s_rx
        .with_clk(peripherals.GPIO4)
        .with_din(peripherals.GPIO5)
        .build();

    println!(
        "embassy_i2s_pdm_rx: PDM RX @ {} Hz on GPIO4 (CLK) / GPIO5 (DIN)",
        rx_cfg.clock.sample_rate.as_hz()
    );

    let buffer = dma_rx_stream_buffer!(4092 * 8, 2048);
    let mut pop_buf = [0u8; POP_BUF_SIZE];
    let mut transaction = match i2s_rx.read(buffer) {
        Ok(tx) => tx,
        Err((err, _, _)) => {
            println!("I2S DMA read failed: {err:?}");
            loop {
                core::hint::spin_loop();
            }
        }
    };

    let mut last_print = Instant::now();
    let print_interval = Duration::from_millis(200);

    loop {
        transaction.wait_for_available_async().await.unwrap();

        while transaction.available_bytes() > 0 {
            let avail = transaction.available_bytes();
            let count = transaction.pop(&mut pop_buf[..avail.min(POP_BUF_SIZE)]);
            if count == 0 {
                break;
            }

            if last_print.elapsed() >= print_interval {
                let show = count.min(PRINT_BYTES);
                println!("PDM RX: read {count} bytes");
                println!("-----------------------------------");
                print_head_samples(&pop_buf, show);
                println!();
                last_print = Instant::now();
            }
        }
    }
}
