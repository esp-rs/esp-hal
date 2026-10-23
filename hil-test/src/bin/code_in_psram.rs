//! Code in PSRAM: the firmware's code and read-only data run from the PSRAM window.

//% CHIP_FILTER(at_200mhz, at_250mhz): psram_code_in_psram
//% ENV: ESP_HAL_CONFIG_CODE_IN_PSRAM=true
//% ENV(at_250mhz): ESP_HAL_CONFIG_PSRAM_FREQUENCY=250MHz
//% FEATURES: unstable

#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use critical_section::Mutex;
use esp_hal::interrupt::software::SoftwareInterrupt;
use hil_test as _;

#[macro_use]
extern crate esp_metadata_generated;

static RODATA: [u32; 4] = [0x5053_524D; 4];
static mut DATA: u32 = 0x4441_5441;
static mut BSS: u32 = 0;

static SWINT: Mutex<RefCell<Option<SoftwareInterrupt<'static, 2>>>> =
    Mutex::new(RefCell::new(None));
static INTERRUPTS: AtomicU32 = AtomicU32::new(0);

#[inline(never)]
fn in_text() -> u32 {
    unsafe { core::ptr::read_volatile(&RODATA[0]) }
}

extern "C" fn swint_handler() {
    INTERRUPTS.fetch_add(1, Ordering::Relaxed);
    critical_section::with(|cs| {
        if let Some(swint) = SWINT.borrow_ref(cs).as_ref() {
            swint.reset();
        }
    });
}

#[embedded_test::tests(default_timeout = 5)]
mod tests {
    use core::ptr::{addr_of, addr_of_mut};

    use esp_bootloader_esp_idf::partitions::{self, AppPartitionSubType, PartitionType};
    #[cfg(soc_has_clock_node_mpll_clk)]
    use esp_hal::{
        clock::{
            ClockConfig,
            ll::{ClockTree, mpll_clk_frequency, request_mpll_clk},
        },
        psram::PsramTimingParams,
    };
    use esp_hal::{
        flash::{Config, Flash},
        interrupt::{InterruptHandler, Priority},
        psram::{Psram, PsramConfig},
    };

    use super::*;

    const WINDOW: u32 = property!("psram.extmem_origin");
    const WINDOW_SIZE: u32 = property!("mmu.entry_num") * property!("mmu.page_size");
    const PATTERN: u32 = 0xA5C3_5A3C;

    fn in_window(addr: u32) -> bool {
        (WINDOW..WINDOW + WINDOW_SIZE).contains(&addr)
    }

    fn booted_app_offset(flash: &mut Flash<'_, esp_hal::Blocking>) -> u32 {
        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let pt = partitions::read_partition_table(flash, &mut pt_mem).unwrap();
        pt.booted_partition().unwrap().unwrap().offset()
    }

    #[test]
    fn test_code_in_window() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        assert!(in_window(in_text as *const () as u32));
        assert_eq!(in_text(), 0x5053_524D);
    }

    #[test]
    fn test_rodata_in_window() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        assert!(in_window(addr_of!(RODATA) as u32));
        assert_eq!(RODATA, [0x5053_524D; 4]);
    }

    #[test]
    fn test_data_and_bss_are_initialized() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        assert_eq!(unsafe { addr_of!(DATA).read_volatile() }, 0x4441_5441);
        assert_eq!(unsafe { addr_of!(BSS).read_volatile() }, 0);
        unsafe { addr_of_mut!(BSS).write_volatile(PATTERN) };
        assert_eq!(unsafe { addr_of!(BSS).read_volatile() }, PATTERN);
    }

    // Reads go through the data cache, so this test shows the content and bounds of the copy, not
    // the write-back.
    #[test]
    #[timeout(30)]
    fn test_segments_match_image() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();
        let app = booted_app_offset(&mut flash);

        // ESP-IDF image: a 24-byte header, then segments of load address, length, and data.
        let mut header = [0u32; 6];
        flash.read(app, &mut header).unwrap();
        assert_eq!(header[0] & 0xFF, 0xE9);
        let segments = (header[0] >> 8) & 0xFF;

        let mut offset = app + 24;
        let mut compared = 0;
        for _ in 0..segments {
            let mut segment = [0u32; 2];
            flash.read(offset, &mut segment).unwrap();
            let [load, len] = segment;
            offset += 8;

            if in_window(load) {
                let mut chunk = [0u32; 256];
                let mut done = 0;
                while done < len {
                    let words = ((len - done) / 4).min(chunk.len() as u32) as usize;
                    flash.read(offset + done, &mut chunk[..words]).unwrap();
                    let mapped =
                        unsafe { core::slice::from_raw_parts((load + done) as *const u32, words) };
                    let first_difference =
                        chunk[..words].iter().zip(mapped).position(|(a, b)| a != b);
                    assert_eq!(first_difference, None, "in the 1 KiB at {:#x}", load + done);
                    done += words as u32 * 4;
                }
                compared += len;
            }
            offset += len;
        }
        assert!(compared > 0);
    }

    #[test]
    fn test_heap_after_code() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());

        let (start, size) = psram.raw_parts();
        assert!(start as u32 > in_text as *const () as u32);
        assert!(start as u32 > addr_of!(RODATA) as u32);
        assert_eq!(psram.size(), start as usize - WINDOW as usize + size);

        let words = size / 4;
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, words) };
        let probes = [0, words / 2, words - 1];
        for (n, &i) in probes.iter().enumerate() {
            heap[i] = PATTERN.wrapping_add(n as u32);
        }
        for (n, &i) in probes.iter().enumerate() {
            assert_eq!(heap[i], PATTERN.wrapping_add(n as u32));
        }
    }

    #[test]
    fn test_heap_survives_flash_read() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());
        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();
        let app = booted_app_offset(&mut flash);

        let (start, _) = psram.raw_parts();
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, 1024) };
        for (i, word) in heap.iter_mut().enumerate() {
            *word = (i as u32).wrapping_mul(0x9E37_79B9);
        }

        let mut buffer = [0u32; 4096];
        flash.read(app, &mut buffer).unwrap();

        for (i, word) in heap.iter().enumerate() {
            assert_eq!(*word, (i as u32).wrapping_mul(0x9E37_79B9));
        }
    }

    #[test]
    fn test_interrupt_handler_in_window() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut swint = SoftwareInterrupt::new(peripherals.FROM_CPU_INTR2);
        critical_section::with(|cs| {
            swint.set_interrupt_handler(InterruptHandler::new(swint_handler, Priority::Priority1));
            SWINT.borrow_ref_mut(cs).replace(swint);
        });
        critical_section::with(|cs| SWINT.borrow_ref(cs).as_ref().unwrap().raise());

        while INTERRUPTS.load(Ordering::Relaxed) == 0 {}
        assert!(in_window(swint_handler as *const () as u32));
    }

    // The ESP32-S31 clocks PSRAM from the MPLL, at 400 MHz for 200 MHz PSRAM and at 500 MHz for
    // 250 MHz PSRAM.
    #[cfg(soc_has_clock_node_mpll_clk)]
    #[test]
    fn test_mpll_rate() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        let expected = match option_env!("ESP_HAL_CONFIG_PSRAM_FREQUENCY") {
            Some("250MHz") => 500_000_000,
            _ => 400_000_000,
        };
        assert_eq!(mpll_clk_frequency(), expected);
    }

    #[cfg(psram_separate_window)]
    #[test]
    fn test_no_flash_page_mapped() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());
        let spi0 = esp_hal::peripherals::SPI0::regs();

        for i in 0..1024 {
            spi0.mmu_item_index().write(|w| unsafe { w.bits(i) });
            assert!(
                !spi0.mmu_item_content().read().valid().bit_is_set(),
                "entry {} is mapped",
                i
            );
        }
    }

    // The start-up's MPLL rate stays when the configuration leaves the MPLL out.
    #[cfg(soc_has_clock_node_mpll_clk)]
    #[test]
    fn test_mpll_rate_without_mpll_config() {
        let mut clocks = ClockConfig::from(esp_hal::clock::CpuClock::default());
        clocks.mpll_clk = None;
        let _peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(clocks));

        let expected = match option_env!("ESP_HAL_CONFIG_PSRAM_FREQUENCY") {
            Some("250MHz") => 500_000_000,
            _ => 400_000_000,
        };
        assert_eq!(mpll_clk_frequency(), expected);
    }

    #[cfg(soc_has_clock_node_mpll_clk)]
    #[test]
    fn test_mpll_request_after_init() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());
        let (start, _) = psram.raw_parts();
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, 1024) };
        for (i, word) in heap.iter_mut().enumerate() {
            *word = (i as u32).wrapping_mul(0x9E37_79B9);
        }
        let before = mpll_clk_frequency();

        ClockTree::with(request_mpll_clk);

        assert_eq!(mpll_clk_frequency(), before);
        assert_eq!(in_text(), 0x5053_524D);
        for (i, word) in heap.iter().enumerate() {
            assert_eq!(*word, (i as u32).wrapping_mul(0x9E37_79B9));
        }
    }

    #[cfg(soc_has_clock_node_mpll_clk)]
    #[test]
    fn test_psram_new_with_another_frequency() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let before = mpll_clk_frequency();
        let timing = match option_env!("ESP_HAL_CONFIG_PSRAM_FREQUENCY") {
            Some("250MHz") => PsramTimingParams::MHZ_200,
            _ => PsramTimingParams::MHZ_250,
        };

        let psram = Psram::new(
            peripherals.PSRAM,
            PsramConfig {
                timing,
                ..PsramConfig::default()
            },
        );

        let (start, size) = psram.raw_parts();
        assert!(size > 0);
        let word = start as *mut u32;
        unsafe { word.write_volatile(PATTERN) };
        assert_eq!(unsafe { word.read_volatile() }, PATTERN);
        assert_eq!(mpll_clk_frequency(), before);
        assert_eq!(in_text(), 0x5053_524D);
    }

    #[test]
    fn test_booted_partition() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();

        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let pt = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();

        let booted = pt.booted_partition().unwrap().unwrap();
        assert_eq!(
            booted.partition_type(),
            PartitionType::App(AppPartitionSubType::Factory)
        );
    }
}
