//! Uses PSRAM as heap memory and measures CPU-driven PSRAM throughput.
//!
//! You need a supported target with at least 2 MB of PSRAM memory.

//% CHIP_FILTER: psram_driver_supported
//% FEATURES: esp-alloc/internal-heap-stats

#![no_std]
#![no_main]

extern crate alloc;

use alloc::{string::String, vec::Vec};

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, main, ram, time::Instant};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(is_not_release)]
compile_error!("PSRAM example must be built in release mode!");

/// PSRAM buffer size for the functional check and the throughput measurements.
const PSRAM_BENCH_BYTES: usize = 512 * 1024;
/// Internal-RAM bounce buffer for SRAM <-> PSRAM copies.
const SRAM_CHUNK_BYTES: usize = 16 * 1024;
/// Repeats each timed transfer to reduce timer quantization error.
const ITERATIONS: u32 = 4;

fn print_throughput(label: &str, bytes: u64, elapsed_us: u64) {
    let us = elapsed_us.max(1);
    let mib_x100 = bytes * 100_000_000 / us / (1024 * 1024);
    println!(
        "{label}: {bytes} bytes in {us} us ({}.{:02} MiB/s)",
        mib_x100 / 100,
        mib_x100 % 100
    );
}

#[ram]
fn write_u32(buf: &mut [u32], value: u32) {
    for slot in buf.iter_mut() {
        unsafe {
            core::ptr::write_volatile(slot, value);
        }
    }
}

#[ram]
fn read_u32(buf: &[u32]) -> u32 {
    let mut acc = 0u32;
    for slot in buf {
        acc ^= unsafe { core::ptr::read_volatile(slot) };
    }
    acc
}

#[ram]
fn copy_sram_to_psram(dst: &mut [u8], src: &[u8]) {
    for chunk in dst.chunks_exact_mut(src.len()) {
        chunk.copy_from_slice(src);
    }
}

#[ram]
fn copy_psram_to_sram(dst: &mut [u8], src: &[u8]) {
    for chunk in src.chunks_exact(dst.len()) {
        dst.copy_from_slice(chunk);
    }
}

#[ram]
fn copy_psram_to_psram(dst: &mut [u8], src: &[u8]) {
    dst.copy_from_slice(src);
}

fn measure_write_u32(buf: &mut [u32]) {
    write_u32(buf, 0xa5a5_5a5a);
    let start = Instant::now();
    for i in 0..ITERATIONS {
        write_u32(buf, 0x5a5a_a5a5 ^ i);
    }
    print_throughput(
        "seq write u32 (PSRAM)",
        PSRAM_BENCH_BYTES as u64 * u64::from(ITERATIONS),
        start.elapsed().as_micros(),
    );
}

fn measure_read_u32(buf: &[u32]) {
    core::hint::black_box(read_u32(buf));
    let start = Instant::now();
    let mut acc = 0u32;
    for _ in 0..ITERATIONS {
        acc ^= read_u32(buf);
    }
    core::hint::black_box(acc);
    print_throughput(
        "seq read u32 (PSRAM)",
        PSRAM_BENCH_BYTES as u64 * u64::from(ITERATIONS),
        start.elapsed().as_micros(),
    );
}

fn measure_sram_to_psram(dst: &mut [u8], src: &[u8]) {
    copy_sram_to_psram(dst, src);
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        copy_sram_to_psram(dst, src);
    }
    print_throughput(
        "memcpy SRAM -> PSRAM",
        PSRAM_BENCH_BYTES as u64 * u64::from(ITERATIONS),
        start.elapsed().as_micros(),
    );
}

fn measure_psram_to_sram(dst: &mut [u8], src: &[u8]) {
    copy_psram_to_sram(dst, src);
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        copy_psram_to_sram(dst, src);
    }
    core::hint::black_box(dst);
    print_throughput(
        "memcpy PSRAM -> SRAM",
        PSRAM_BENCH_BYTES as u64 * u64::from(ITERATIONS),
        start.elapsed().as_micros(),
    );
}

fn measure_psram_to_psram(dst: &mut [u8], src: &[u8]) {
    copy_psram_to_psram(dst, src);
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        copy_psram_to_psram(dst, src);
    }
    print_throughput(
        "memcpy PSRAM -> PSRAM",
        PSRAM_BENCH_BYTES as u64 * u64::from(ITERATIONS),
        start.elapsed().as_micros(),
    );
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    println!("Going to access PSRAM");
    let mut psram_a = Vec::<u8>::with_capacity(PSRAM_BENCH_BYTES);
    psram_a.resize(PSRAM_BENCH_BYTES, 0);
    let mut psram_b = Vec::<u8>::with_capacity(PSRAM_BENCH_BYTES);
    psram_b.resize(PSRAM_BENCH_BYTES, 0);

    for (i, byte) in psram_a.iter_mut().enumerate() {
        *byte = (i & 0xff) as u8;
    }

    println!("vec size = {} bytes", psram_a.len());
    println!("vec address = {:p}", psram_a.as_ptr());
    println!("vec[..100] = {:?}", &psram_a[..100]);

    let string = String::from("A string allocated in PSRAM");
    println!("'{}' allocated at {:p}", &string, string.as_ptr());

    println!("{}", esp_alloc::HEAP.stats());

    // Keep the bounce buffer in internal RAM, not on the stack.
    let sram = {
        static mut SRAM: [u8; SRAM_CHUNK_BYTES] = [0; SRAM_CHUNK_BYTES];
        // SAFETY: `main` is the only user of this buffer.
        unsafe { &mut *core::ptr::addr_of_mut!(SRAM) }
    };
    for (i, byte) in sram.iter_mut().enumerate() {
        *byte = (i & 0xff) as u8;
    }

    println!("PSRAM throughput (CPU, {ITERATIONS} iterations):");

    let word_count = PSRAM_BENCH_BYTES / 4;
    // SAFETY: The allocator returns a word-aligned heap block.
    let psram_words =
        unsafe { core::slice::from_raw_parts_mut(psram_a.as_mut_ptr().cast::<u32>(), word_count) };
    measure_write_u32(psram_words);
    measure_read_u32(psram_words);

    measure_sram_to_psram(&mut psram_a, sram);
    measure_psram_to_sram(sram, &psram_a);
    measure_psram_to_psram(&mut psram_b, &psram_a);

    println!("done");

    loop {}
}
