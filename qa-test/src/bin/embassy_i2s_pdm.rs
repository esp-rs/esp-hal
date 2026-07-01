//! PDM TX for an NS4150,LM386 or similar amplifier connected via low-pass filter,
//! playing ESP-IDF's "Twinkle Twinkle Little Star" demo.
//!
//! e.g. ESP32-C3-LCDKit (on-board NS4150), PDM data goes to the on-board NS4150 on **GPIO3**
//! (`AUDIO_PA`). No separate clock pin. (DAC case)

//% CHIP_FILTER: i2s_supports_pdm_tx
//% FEATURES: unstable

#![no_std]
#![no_main]

use core::f32::consts::PI;

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    dma_tx_stream_buffer,
    i2s::master::{I2s, PdmConfig, PdmSlotMode, PdmTxConfig},
    interrupt::software::SoftwareInterruptControl,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const SAMPLE_RATE_HZ: u32 = 16_000;
const AMPLITUDE: f32 = 1000.0;
const TONE_MS: u32 = 500;
/// IDF `EXAMPLE_BYTE_NUM_EVERY_TONE` — bytes written, not samples.
const BYTES_PER_TONE_UNIT: usize = TONE_MS as usize * SAMPLE_RATE_HZ as usize / 1000;
const NOTE_GAP_SAMPLES: usize = (15 * SAMPLE_RATE_HZ as usize) / 1000;
const MAX_TONE_POINT: usize = 61;

const TONE: [[u32; 7]; 3] = [
    [262, 294, 330, 349, 392, 440, 494],
    [523, 587, 659, 698, 784, 880, 988],
    [1046, 1175, 1318, 1397, 1568, 1760, 1976],
];

const SONG: [u8; 28] = [
    1, 1, 5, 5, 6, 6, 5, 4, 4, 3, 3, 2, 2, 1, 5, 5, 4, 4, 3, 3, 2, 5, 5, 4, 4, 3, 3, 2,
];

const RHYTHM: [u8; 7] = [1, 1, 1, 1, 1, 1, 2];

const TONE_NAME: [&str; 3] = ["bass", "alto", "treble"];

fn sine_wave_len(freq: u32) -> usize {
    (SAMPLE_RATE_HZ as f32 / freq as f32 + 0.5) as usize
}

enum PlayerPhase {
    Note {
        tone_point: usize,
        sample_idx: usize,
        bytes_left: usize,
    },
    Gap {
        samples_left: usize,
    },
}

struct TwinklePlayer {
    song_idx: usize,
    tone_select: usize,
    tone_buf: [i16; MAX_TONE_POINT],
    phase: PlayerPhase,
}

impl TwinklePlayer {
    fn new() -> Self {
        let mut player = Self {
            song_idx: 0,
            tone_select: 0,
            tone_buf: [0; MAX_TONE_POINT],
            phase: PlayerPhase::Gap { samples_left: 0 },
        };
        println!(
            "Playing {} `twinkle twinkle little star`",
            TONE_NAME[player.tone_select]
        );
        player.start_note();
        player
    }

    fn start_note(&mut self) {
        let freq = TONE[self.tone_select][SONG[self.song_idx] as usize - 1];
        let tone_point = sine_wave_len(freq);
        for i in 0..tone_point {
            self.tone_buf[i] =
                (libm::sinf(2.0 * PI * i as f32 / tone_point as f32) * AMPLITUDE) as i16;
        }
        self.phase = PlayerPhase::Note {
            tone_point,
            sample_idx: 0,
            bytes_left: BYTES_PER_TONE_UNIT * RHYTHM[self.song_idx % 7] as usize,
        };
    }

    fn start_gap(&mut self) {
        self.song_idx += 1;
        if self.song_idx >= SONG.len() {
            self.song_idx = 0;
            self.tone_select = (self.tone_select + 1) % 3;
            println!(
                "Playing {} `twinkle twinkle little star`",
                TONE_NAME[self.tone_select]
            );
        }
        self.phase = PlayerPhase::Gap {
            samples_left: NOTE_GAP_SAMPLES,
        };
    }

    fn next_sample(&mut self) -> i16 {
        loop {
            match self.phase {
                PlayerPhase::Note {
                    tone_point,
                    sample_idx,
                    bytes_left,
                } => {
                    if bytes_left == 0 {
                        self.start_gap();
                        continue;
                    }
                    let sample = self.tone_buf[sample_idx % tone_point];
                    self.phase = PlayerPhase::Note {
                        tone_point,
                        sample_idx: sample_idx + 1,
                        bytes_left: bytes_left - 2,
                    };
                    return sample;
                }
                PlayerPhase::Gap { samples_left } => {
                    if samples_left == 0 {
                        self.start_note();
                        continue;
                    }
                    self.phase = PlayerPhase::Gap {
                        samples_left: samples_left - 1,
                    };
                    return 0;
                }
            }
        }
    }

    fn fill_buffer(&mut self, buf: &mut [u8]) -> usize {
        let mut written = 0;
        while written + 1 < buf.len() {
            let sample = self.next_sample();
            buf[written] = sample as u8;
            buf[written + 1] = (sample >> 8) as u8;
            written += 2;
        }
        written
    }
}

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    println!("embassy_i2s_pdm: init");

    let peripherals = esp_hal::init(esp_hal::Config::default());
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let dma_channel = cfg_select! {
        any(feature = "esp32", feature = "esp32s2") => peripherals.DMA_I2S0,
        _ => peripherals.DMA_CH0,
    };

    let tx_cfg = PdmTxConfig::new_codec_default(Rate::from_hz(SAMPLE_RATE_HZ), PdmSlotMode::Mono);
    let pdm_cfg = PdmConfig::tx_only(tx_cfg);

    let i2s = match I2s::new_pdm(peripherals.I2S0, dma_channel, pdm_cfg) {
        Ok(i2s) => i2s.into_async(),
        Err(err) => {
            println!("I2S PDM init failed: {err:?}");
            loop {
                core::hint::spin_loop();
            }
        }
    };

    let dout = cfg_select! {
        feature = "esp32" => peripherals.GPIO4,
        _ => peripherals.GPIO3,
    };
    let i2s_tx = i2s.i2s_tx.with_dout(dout).build();
    cfg_select! {
        feature = "esp32" => {
            println!("embassy_i2s_pdm: PDM TX on GPIO4 @ {} Hz", SAMPLE_RATE_HZ);
        }
        _ => {
            println!("embassy_i2s_pdm: PDM TX on GPIO3 @ {} Hz", SAMPLE_RATE_HZ);
        }
    }

    let mut player = TwinklePlayer::new();
    let mut buffer = dma_tx_stream_buffer!(4092 * 4, 2048);
    buffer.push_with(|buf| player.fill_buffer(buf));

    let mut transaction = match i2s_tx.write(buffer) {
        Ok(tx) => tx,
        Err((err, _, _)) => {
            println!("I2S DMA write failed: {err:?}");
            loop {
                core::hint::spin_loop();
            }
        }
    };

    loop {
        transaction.wait_for_available_async().await.unwrap();
        transaction.push_with(|buf| player.fill_buffer(buf));
    }
}
