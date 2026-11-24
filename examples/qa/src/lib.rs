//! Shared parts of the qa examples: the console, the progress across resets, the free flash,
//! whether the build places code in PSRAM, and esp-hal's flash value.

#![no_std]

use core::ops::Range;

use esp_bootloader_esp_idf::partitions;
use esp_hal::{
    Blocking,
    flash::Flash,
    rtc_cntl::{SocResetReason, reset_reason},
    system::Cpu,
};

/// Bytes in `.dram2_uninit` for the selected chip.
pub const DRAM2_UNINIT_SIZE: usize = {
    let range: core::ops::Range<usize> = esp_metadata_generated::memory_range!("DRAM2_UNINIT");
    range.end - range.start
};

/// Returns whether the build places code in PSRAM. On some chips, flash and PSRAM share one
/// address range, so the address of the code does not show it.
pub fn code_in_psram() -> bool {
    cfg!(code_in_psram)
}

#[cfg(code_in_psram)]
unsafe extern "Rust" {
    /// esp-hal's flash value. Every copy of esp-hal finds it by this name.
    static ESP_HAL_FLASH_OPERATION: portable_atomic::AtomicU32;
}

/// Returns esp-hal's flash value: 0 while no flash operation is in progress, otherwise one more
/// than the number of the core that started it. Bit 7 marks a stop of the APP core that waits for
/// the end of the APP core's operation.
pub fn flash_operation() -> u32 {
    cfg_select! {
        // SAFETY: esp-hal defines the static.
        code_in_psram => unsafe {
            ESP_HAL_FLASH_OPERATION.load(core::sync::atomic::Ordering::Acquire)
        },
        _ => 0,
    }
}

#[doc(hidden)]
pub mod __private {
    pub use esp_hal::{
        time::{Duration, Instant},
        uart::UartTx,
    };
    pub use log::info;
}

/// Logs a line and waits 20 ms for the UART to send it. A light sleep or a reset loses unsent
/// UART data, and 20 ms sends about 230 bytes at 115200 baud.
#[macro_export]
macro_rules! report {
    ($($arg:tt)*) => {{
        $crate::__private::info!($($arg)*);
        let end = $crate::__private::Instant::now() + $crate::__private::Duration::from_millis(20);
        while $crate::__private::Instant::now() < end {}
    }};
}

/// Returns a `UartTx` on UART0's default TX pin. Takes the `Peripherals` returned by
/// `esp_hal::init`, and moves the fields it needs out of it.
#[macro_export]
macro_rules! console_uart {
    ($peripherals:ident) => {
        $crate::__private::UartTx::new($peripherals.UART0, ::core::default::Default::default())
            .unwrap()
            .with_tx(::core::cfg_select! {
                feature = "esp32" => $peripherals.GPIO1,
                feature = "esp32c2" => $peripherals.GPIO20,
                feature = "esp32c3" => $peripherals.GPIO21,
                any(feature = "esp32c5", feature = "esp32c61") => $peripherals.GPIO11,
                feature = "esp32c6" => $peripherals.GPIO16,
                any(feature = "esp32h2", feature = "esp32h4") => $peripherals.GPIO24,
                feature = "esp32p4" => $peripherals.GPIO37,
                feature = "esp32s31" => $peripherals.GPIO58,
                any(feature = "esp32s2", feature = "esp32s3") => $peripherals.GPIO43,
            })
    };
}

/// The progress of a run of checks or phases, kept across resets in a [`ProgressWord`].
#[derive(Clone, Copy)]
pub struct Progress {
    /// The index of the entry that runs next, or that ran when the chip reset.
    pub next: usize,
    /// The number of entries that passed.
    pub passed: u8,
    /// Whether the program reset the chip itself. Any other reset is unexpected.
    pub reset_requested: bool,
}

/// The word of memory that a reset does not clear, which holds the [`Progress`] of one program.
///
/// The top four bits of the word hold the program's marker, so that a word that another program
/// left does not read as progress.
pub struct ProgressWord {
    marker: u32,
}

impl ProgressWord {
    /// Creates a new progress word for a program. `marker` must be below 16, because it fills the
    /// top four bits of the word.
    pub const fn new(marker: u32) -> Self {
        assert!(marker < 16);
        Self { marker }
    }

    /// Returns the stored progress of a run of `count` entries. Returns `None` after a power-on
    /// reset, and when the word does not hold progress of this program.
    pub fn load(&self, count: usize) -> Option<Progress> {
        if reset_reason(Cpu::ProCpu) == Some(SocResetReason::ChipPowerOn) {
            return None;
        }
        let word = load_word();
        if word >> 28 != self.marker {
            return None;
        }
        let reset_requested = match (word >> 24) & 0xF {
            0 => false,
            1 => true,
            _ => return None,
        };
        let next = (word & 0xFF) as usize;
        if next >= count {
            return None;
        }
        Some(Progress {
            next,
            passed: (word >> 8) as u8,
            reset_requested,
        })
    }

    /// Stores `progress` in the word.
    pub fn store(&self, progress: Progress) {
        store_word(
            (self.marker << 28)
                | ((progress.reset_requested as u32) << 24)
                | ((progress.passed as u32) << 8)
                | progress.next as u32,
        );
    }
}

// The C2, C61, and H4 do not have RTC fast memory, so a store register of the always-on domain
// holds the word there.
#[cfg(not(any(feature = "esp32c2", feature = "esp32c61", feature = "esp32h4")))]
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut PROGRESS: u32 = 0;

fn load_word() -> u32 {
    cfg_select! {
        feature = "esp32c2" => esp_hal::peripherals::LPWR::regs().store6().read().bits(),
        any(feature = "esp32c61", feature = "esp32h4") => {
            esp_hal::peripherals::LP_AON::regs().store6().read().bits()
        }
        _ => unsafe { PROGRESS },
    }
}

fn store_word(value: u32) {
    cfg_select! {
        feature = "esp32c2" => {
            esp_hal::peripherals::LPWR::regs()
                .store6()
                .write(|w| unsafe { w.bits(value) });
        }
        any(feature = "esp32c61", feature = "esp32h4") => {
            esp_hal::peripherals::LP_AON::regs()
                .store6()
                .write(|w| unsafe { w.bits(value) });
        }
        _ => unsafe { PROGRESS = value },
    }
}

/// Returns the free space after the image in the booted partition, as a range of flash addresses.
///
/// The range starts at a 64 KiB boundary and ends at a 4 KiB boundary.
///
/// # Panics
///
/// Panics when the partition table cannot be read, and when the image does not leave free space in
/// its partition.
pub fn free_flash(flash: &mut Flash<'_, Blocking>) -> Range<u32> {
    const SECTOR: u32 = 4096;
    const BLOCK: u32 = 0x1_0000;

    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(flash, &mut pt_mem).unwrap();
    let app = pt.booted_partition().unwrap().unwrap();

    // ESP-IDF image: a 24-byte header, then segments of load address, length, and data. The
    // checksum and the hash that follow fit in the next block.
    let mut header = [0u32; 6];
    flash.read(app.offset(), &mut header).unwrap();
    let mut end = app.offset() + 24;
    for _ in 0..(header[0] >> 8) & 0xFF {
        let mut segment = [0u32; 2];
        flash.read(end, &mut segment).unwrap();
        end += 8 + segment[1];
    }
    let first = (end + BLOCK).div_ceil(BLOCK) * BLOCK;
    let last = (app.offset() + app.len()) / SECTOR * SECTOR;
    assert!(
        first < last,
        "no free space after the image in the booted partition"
    );
    first..last
}

/// Returns the flash offset of `count` 64 KiB blocks in a row, at a random place in [`free_flash`],
/// so that each run wears other blocks.
///
/// # Panics
///
/// Panics as [`free_flash`] does, and when the free space holds fewer than `count` blocks.
pub fn scratch_blocks(flash: &mut Flash<'_, Blocking>, count: u32) -> u32 {
    const BLOCK: u32 = 0x1_0000;

    let free = free_flash(flash);
    let blocks = free.len() as u32 / BLOCK;
    assert!(
        blocks >= count,
        "the free flash holds fewer than {count} blocks of 64 KiB"
    );
    free.start + esp_hal::rng::Rng::new().random() % (blocks - count + 1) * BLOCK
}
