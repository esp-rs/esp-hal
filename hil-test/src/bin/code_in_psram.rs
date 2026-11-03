//! Code in PSRAM: the firmware's code and read-only data run from the PSRAM window.

//% CHIP_FILTER(at_200mhz, at_250mhz): psram_code_in_psram
//% ENV: ESP_HAL_CONFIG_CODE_IN_PSRAM=true
//% ENV(at_250mhz): ESP_HAL_CONFIG_PSRAM_FREQUENCY=250MHz
//% FEATURES: unstable esp-alloc embassy

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

// 256 KiB, so the static spans several MMU pages.
#[esp_hal::psram(zeroed)]
static mut PSRAM_BSS: [u32; 0x1_0000] = [0; 0x1_0000];

struct PsramState {
    waker: Mutex<RefCell<Option<core::task::Waker>>>,
    flag: core::sync::atomic::AtomicBool,
}

#[esp_hal::psram(zeroed)]
static PSRAM_STATE: PsramState = PsramState {
    waker: Mutex::new(RefCell::new(None)),
    flag: core::sync::atomic::AtomicBool::new(false),
};

#[repr(C, align(64))]
struct DmaBlock([u8; 4096]);

#[esp_hal::psram(zeroed)]
static mut DMA_SOURCE: DmaBlock = DmaBlock([0; 4096]);

unsafe extern "C" {
    static _psram_code_end: u8;
    static _psram_bss_end: u8;
}

/// A random 64 KiB block after the image in the booted partition, so that each run wears another.
fn scratch_block(flash: &mut esp_hal::flash::Flash<'_, esp_hal::Blocking>) -> u32 {
    use esp_bootloader_esp_idf::partitions;

    const BLOCK: u32 = 0x1_0000;
    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(flash, &mut pt_mem).unwrap();
    let app = pt.booted_partition().unwrap().unwrap();

    // ESP-IDF image: a 24-byte header, then segments of load address, length, and data, then a
    // checksum and a hash, which fit in the next block.
    let mut header = [0u32; 6];
    flash.read(app.offset(), &mut header).unwrap();
    let mut end = app.offset() + 24;
    for _ in 0..(header[0] >> 8) & 0xFF {
        let mut segment = [0u32; 2];
        flash.read(end, &mut segment).unwrap();
        end += 8 + segment[1];
    }
    let first = (end + BLOCK).div_ceil(BLOCK);
    let last = (app.offset() + app.len()) / BLOCK;
    assert!(
        first < last,
        "no free 64 KiB block after the image in the booted partition"
    );
    (first + esp_hal::rng::Rng::new().random() % (last - first)) * BLOCK
}

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

    // PSRAM keeps its content across a reset, so the zeros come from the start-up, at every boot.
    #[test]
    fn test_psram_static_is_zeroed_above_the_code() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        let start = addr_of!(PSRAM_BSS) as u32;
        assert!(in_window(start));
        assert!(start >= addr_of!(_psram_code_end) as u32);

        let words = unsafe { &mut *addr_of_mut!(PSRAM_BSS) };
        assert_eq!(words.iter().position(|&w| w != 0), None);
        let probes = [0, words.len() / 2, words.len() - 1];
        for (n, &i) in probes.iter().enumerate() {
            words[i] = PATTERN.wrapping_add(n as u32);
        }
        for (n, &i) in probes.iter().enumerate() {
            assert_eq!(words[i], PATTERN.wrapping_add(n as u32));
        }
    }

    #[test]
    fn test_psram_static_holds_none_and_false() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        assert!(in_window(addr_of!(PSRAM_STATE) as u32));
        assert!(critical_section::with(|cs| PSRAM_STATE
            .waker
            .borrow_ref(cs)
            .is_none()));
        assert!(!PSRAM_STATE.flag.load(Ordering::Relaxed));
        PSRAM_STATE.flag.store(true, Ordering::Relaxed);
        assert!(PSRAM_STATE.flag.load(Ordering::Relaxed));
    }

    // The CPU writes the source through the cache just before the transfer, so the DMA reads the
    // written bytes only if the driver treats the static as PSRAM and writes the cache back.
    // Outside `psram_range`, the driver rejects the buffer.
    #[cfg(all(dma_can_access_psram, dma_supports_mem2mem))]
    #[test]
    fn test_dma_reads_psram_static() {
        use esp_hal::dma::{BurstConfig, ExternalBurstConfig, Mem2Mem};

        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mem2mem = cfg_select! {
            feature = "esp32s31" => Mem2Mem::new(peripherals.DMA_AXI_CH1),
            _ => Mem2Mem::new(peripherals.DMA_CH0),
        };
        let (rx_buffer, rx_descriptors, _, tx_descriptors) = esp_hal::dma_buffers!(4096);
        let mut mem2mem = mem2mem
            .with_descriptors(
                rx_descriptors,
                tx_descriptors,
                BurstConfig {
                    external_memory: ExternalBurstConfig::Size64,
                    internal_memory: Default::default(),
                },
            )
            .unwrap();

        let source = unsafe { &mut (*addr_of_mut!(DMA_SOURCE)).0 };
        assert!(in_window(source.as_ptr() as u32));
        for (i, byte) in source.iter_mut().enumerate() {
            *byte = (i as u8).wrapping_mul(31).wrapping_add(7);
        }

        mem2mem
            .start_transfer(rx_buffer, source)
            .unwrap()
            .wait()
            .unwrap();

        for (i, &byte) in rx_buffer.iter().enumerate() {
            assert_eq!(byte, (i as u8).wrapping_mul(31).wrapping_add(7));
        }
    }

    #[test]
    fn test_heap_after_psram_statics() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());

        let (start, _) = psram.raw_parts();
        assert!(start as u32 >= addr_of!(_psram_bss_end) as u32);
        assert!(start as u32 >= addr_of!(PSRAM_BSS) as u32 + 0x4_0000);
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

/// Flash operations while code runs from PSRAM: the other core and interrupts keep running.
///
/// The probe uses the ESP32-S31's ROM range and its cross-core interrupt register.
#[cfg(feature = "esp32s31")]
#[embedded_test::tests(default_timeout = 30)]
mod flash_operations {
    use core::sync::atomic::AtomicBool;

    use allocator_api2::boxed::Box;
    use esp_alloc::ExternalMemory;
    use esp_bootloader_esp_idf::partitions;
    use esp_hal::{
        flash::{Config, Error, Flash, MultiCoreStrategy},
        interrupt::{InterruptHandler, Priority},
        peripherals::{CPU_CTRL, FROM_CPU_INTR2, Peripherals, SYSTEM},
        system::{CpuControl, Stack},
    };
    use hil_test::mk_static;

    use super::*;

    /// The ESP32-S31's ROM, where the flash routines run.
    ///
    /// See <https://github.com/espressif/esp-idf/blob/383dda6c182/components/soc/esp32s31/include/soc/soc.h#L147-L148>.
    const ROM: core::ops::Range<u32> = 0x2F80_0000..0x2F85_0000;
    const BLOCK: u32 = 0x1_0000;

    static STARTED: AtomicBool = AtomicBool::new(false);
    static RUNNING: AtomicBool = AtomicBool::new(false);
    static DONE: AtomicBool = AtomicBool::new(false);
    static HANDLER_RUNS: AtomicU32 = AtomicU32::new(0);
    static IN_ROM: AtomicU32 = AtomicU32::new(0);
    static CORE1_STORES: AtomicU32 = AtomicU32::new(0);

    // An interrupt taken while core 0 was in the ROM routine is what the operation must allow.
    extern "C" fn probe_handler() {
        let mepc: u32;
        unsafe { core::arch::asm!("csrr {0}, mepc", out(reg) mepc) };
        if ROM.contains(&mepc) {
            IN_ROM.fetch_add(1, Ordering::Relaxed);
        }
        SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();
        HANDLER_RUNS.fetch_add(1, Ordering::Release);
    }

    /// Starts core 1, which raises the probe interrupt and stores to `page` while `RUNNING`, and
    /// returns once it runs.
    fn start_core1(
        cpu_ctrl: CPU_CTRL<'static>,
        intr: FROM_CPU_INTR2<'static>,
        page: Option<usize>,
    ) {
        let mut swint = SoftwareInterrupt::new(intr);
        swint.set_interrupt_handler(InterruptHandler::new(probe_handler, Priority::max()));
        core::mem::forget(swint);

        let mut cpu_control = CpuControl::new(cpu_ctrl);
        let stack = mk_static!(Stack<8192>, Stack::new());
        let guard = cpu_control
            .start_app_core(stack, move || {
                STARTED.store(true, Ordering::Release);
                loop {
                    while !RUNNING.load(Ordering::Acquire) {}
                    while RUNNING.load(Ordering::Acquire) {
                        let before = HANDLER_RUNS.load(Ordering::Acquire);
                        // `SoftwareInterrupt::raise` waits until the interrupt reads as pending,
                        // and core 0's handler can take and clear it first. So this core only
                        // sets it.
                        SYSTEM::regs()
                            .cpu_intr_from_cpu(2)
                            .write(|w| w.cpu_intr().set_bit());
                        while HANDLER_RUNS.load(Ordering::Acquire) == before
                            && RUNNING.load(Ordering::Acquire)
                        {}
                        if let Some(page) = page {
                            let n = CORE1_STORES.load(Ordering::Relaxed).wrapping_add(1);
                            unsafe { (page as *mut u32).write_volatile(n ^ in_text()) };
                            CORE1_STORES.store(n, Ordering::Release);
                        }
                        // Raising again at once starves core 0's ROM routine. The count sets how
                        // much of the time core 0 runs, and nothing depends on its value.
                        for _ in 0..1000 {
                            core::hint::spin_loop();
                        }
                    }
                    DONE.store(true, Ordering::Release);
                }
            })
            .unwrap();
        core::mem::forget(guard);
        while !STARTED.load(Ordering::Acquire) {}
    }

    /// Runs `operation` while core 1 runs, and returns how many probe interrupts it took in ROM.
    fn while_core1_runs(operation: impl FnOnce()) -> u32 {
        let before = IN_ROM.load(Ordering::Relaxed);
        DONE.store(false, Ordering::Relaxed);
        RUNNING.store(true, Ordering::Release);
        operation();
        RUNNING.store(false, Ordering::Release);
        // The test's timeout already fails a hang. This assertion turns the hang into a failure
        // that prints the counters.
        let deadline = esp_hal::time::Instant::now() + esp_hal::time::Duration::from_secs(1);
        while !DONE.load(Ordering::Acquire) {
            assert!(
                esp_hal::time::Instant::now() < deadline,
                "core 1 did not stop: {} handler runs, {} in ROM, {} stores",
                HANDLER_RUNS.load(Ordering::Relaxed),
                IN_ROM.load(Ordering::Relaxed),
                CORE1_STORES.load(Ordering::Relaxed),
            );
        }
        IN_ROM.load(Ordering::Relaxed) - before
    }

    struct Context {
        flash: Flash<'static, esp_hal::Blocking>,
        heap: Box<[u32; 1024], ExternalMemory>,
    }

    fn setup() -> Context {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);
        let core1_page = Box::into_raw(Box::new_in(0u32, ExternalMemory)) as usize;
        start_core1(p.CPU_CTRL, p.FROM_CPU_INTR2, Some(core1_page));
        Context {
            flash: Flash::new(p.FLASH, Config::default()).unwrap(),
            heap: Box::new_in([PATTERN_WORD; 1024], ExternalMemory),
        }
    }

    const PATTERN_WORD: u32 = 0x5AA5_C33C;

    #[test]
    fn test_erase_and_write_run_with_interrupts_and_the_other_core() {
        let mut ctx = setup();
        let block = scratch_block(&mut ctx.flash);
        let data = mk_static!([u32; 4096], [PATTERN_WORD; 4096]);

        let flash = &mut ctx.flash;
        // SAFETY: the block is in the booted partition, after the image.
        unsafe { flash.erase(block, block + BLOCK).unwrap() };
        let in_write = while_core1_runs(|| unsafe { flash.write(block, data).unwrap() });
        let in_erase = while_core1_runs(|| unsafe { flash.erase(block, block + BLOCK).unwrap() });

        assert!(in_write > 0, "no interrupt ran during the write");
        assert!(in_erase > 0, "no interrupt ran during the erase");
        assert!(ctx.heap.iter().all(|&w| w == PATTERN_WORD));
        assert!(CORE1_STORES.load(Ordering::Acquire) > 0);
    }

    #[test]
    fn test_reads_run_with_interrupts_and_the_other_core() {
        let mut ctx = setup();
        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let app = partitions::read_partition_table(&mut ctx.flash, &mut pt_mem)
            .unwrap()
            .booted_partition()
            .unwrap()
            .unwrap()
            .offset();
        let flash = &mut ctx.flash;
        let buffer = mk_static!([u32; 4096], [0; 4096]);

        let in_reads = while_core1_runs(|| {
            for _ in 0..200 {
                flash.read(app, buffer).unwrap();
            }
        });

        assert!(in_reads > 0, "no interrupt ran during 200 reads");
        assert!(ctx.heap.iter().all(|&w| w == PATTERN_WORD));
    }

    // A page left mapped with its old cache lines would return the old words to the second
    // `read_encrypted`.
    #[test]
    fn test_read_encrypted_sees_new_data() {
        let mut ctx = setup();
        let block = scratch_block(&mut ctx.flash);
        let flash = &mut ctx.flash;
        let mut plain = [0u32; 256];
        let mut through_mmu = [0u32; 256];

        flash.read(block, &mut plain).unwrap();
        flash.read_encrypted(block, &mut through_mmu).unwrap();
        assert_eq!(plain, through_mmu);

        let data = [PATTERN_WORD; 256];
        // SAFETY: the block is in the booted partition, after the image.
        unsafe {
            flash.erase(block, block + BLOCK).unwrap();
            flash.write(block, &data).unwrap();
        }
        flash.read_encrypted(block, &mut through_mmu).unwrap();
        assert_eq!(through_mmu, data);
    }

    #[test]
    fn test_error_strategy_allows_the_other_core() {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        let config = Config::default().with_multi_core_strategy(MultiCoreStrategy::Error);
        let mut flash = Flash::new(p.FLASH, config).unwrap();
        start_core1(p.CPU_CTRL, p.FROM_CPU_INTR2, None);

        let mut buffer = [0u32; 4];
        assert_eq!(flash.read(0x1_0000, &mut buffer), Ok(()));
    }

    #[test]
    fn test_read_into_psram_is_refused() {
        let mut ctx = setup();
        let mut buffer = Box::new_in([0u32; 4], ExternalMemory);
        assert_eq!(
            ctx.flash.read(0x1_0000, &mut buffer[..]),
            Err(Error::NotSupported)
        );
    }
}

/// Flash operations under esp-rtos, with code in PSRAM: a higher-priority thread keeps running on
/// the core that runs the operation.
///
/// The scheduler starts in this module only, because its tick interrupt would run under every other
/// test.
#[embedded_test::tests(default_timeout = 30, executor = esp_rtos::embassy::Executor::new())]
mod rtos_flash {
    use core::sync::atomic::AtomicBool;

    use esp_hal::{
        Blocking,
        flash::{Config, Flash},
        peripherals::Peripherals,
        time::Duration,
        timer::timg::TimerGroup,
    };
    use esp_rtos::{
        CurrentThreadHandle,
        thread::{Stack, ThreadSpawner},
    };
    use hil_test::mk_static;
    use static_cell::ConstStaticCell;

    use super::*;

    const BLOCK: u32 = 0x1_0000;
    const PATTERN_WORD: u32 = 0x5AA5_C33C;

    static IN_OPERATION: AtomicBool = AtomicBool::new(false);
    static TICKS: AtomicU32 = AtomicU32::new(0);
    static TICKER_STACK: ConstStaticCell<Stack<8192>> = ConstStaticCell::new(Stack::new());

    struct Context {
        flash: Flash<'static, Blocking>,
    }

    #[init]
    fn init() -> Context {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);
        // The ticker blocks for 1 ms between counts, so this thread runs in between.
        CurrentThreadHandle::get().set_priority(1);
        ThreadSpawner::from_static(TICKER_STACK.take())
            .with_priority(2)
            .spawn(|| {
                loop {
                    if IN_OPERATION.load(Ordering::Acquire) {
                        TICKS.fetch_add(1, Ordering::Relaxed);
                    }
                    CurrentThreadHandle::get().delay(Duration::from_millis(1));
                }
            })
            .detach();
        Context {
            flash: Flash::new(p.FLASH, Config::default()).unwrap(),
        }
    }

    /// Runs `operation`, which makes one ROM call, and returns how often the ticker ran during it.
    fn ticks_during(operation: impl FnOnce()) -> u32 {
        let before = TICKS.load(Ordering::Relaxed);
        IN_OPERATION.store(true, Ordering::Release);
        operation();
        IN_OPERATION.store(false, Ordering::Release);
        TICKS.load(Ordering::Relaxed) - before
    }

    // One tick can fall just before or just after the ROM call. More than one shows that the
    // ticker preempted this thread inside the call.
    #[test]
    fn test_a_higher_priority_thread_runs_during_an_erase(mut ctx: Context) {
        let block = scratch_block(&mut ctx.flash);
        let flash = &mut ctx.flash;
        // SAFETY: the block is in the booted partition, after the image.
        let ticks = ticks_during(|| unsafe { flash.erase(block, block + BLOCK).unwrap() });
        assert!(ticks > 1, "the ticker ran {ticks} times during the erase");
    }

    #[test]
    fn test_a_higher_priority_thread_runs_during_a_write(mut ctx: Context) {
        let block = scratch_block(&mut ctx.flash);
        let data = mk_static!([u32; 4096], [PATTERN_WORD; 4096]);
        let flash = &mut ctx.flash;
        // SAFETY: the block is in the booted partition, after the image.
        unsafe { flash.erase(block, block + BLOCK).unwrap() };
        // SAFETY: as above.
        let ticks = ticks_during(|| unsafe { flash.write(block, data).unwrap() });
        assert!(ticks > 1, "the ticker ran {ticks} times during the write");
    }
}

/// A stop or a park of the APP core while it erases blocks back to back. Both wait for the APP
/// core's ROM call, so afterwards the flash chip is free, and the APP core's last erase is
/// complete.
#[cfg(feature = "esp32s31")]
#[embedded_test::tests(default_timeout = 30)]
mod stop_and_park_during_erase {
    use core::sync::atomic::AtomicBool;

    use esp_hal::{
        flash::{Config, Flash},
        peripherals::{FLASH, Peripherals},
        system::{AppCoreGuard, Cpu, CpuControl, Stack},
        time::{Duration, Instant},
    };
    use hil_test::mk_static;

    use super::*;

    const BLOCK: u32 = 0x1_0000;
    /// The stop and the park wait for one block erase, which lasts a few hundred milliseconds.
    const LIMIT: Duration = Duration::from_secs(3);

    static ERASES: AtomicU32 = AtomicU32::new(0);
    static ERASE_FAILED: AtomicBool = AtomicBool::new(false);

    /// Starts the APP core, which erases `block` again and again with its own `Flash`, and returns
    /// once it has finished its first erase.
    fn start_erasing<'a>(
        cpu_control: &mut CpuControl<'static>,
        flash: FLASH<'static>,
        block: u32,
    ) -> AppCoreGuard<'a> {
        let stack = mk_static!(Stack<8192>, Stack::new());
        let guard = cpu_control
            .start_app_core(stack, move || {
                let mut flash = Flash::new(flash, Config::default()).unwrap();
                loop {
                    // SAFETY: the block is in the booted partition, after the image.
                    if unsafe { flash.erase(block, block + BLOCK) }.is_err() {
                        ERASE_FAILED.store(true, Ordering::Relaxed);
                    }
                    ERASES.fetch_add(1, Ordering::Release);
                }
            })
            .unwrap();
        while ERASES.load(Ordering::Acquire) == 0 {}
        guard
    }

    /// Checks that the block reads back erased, through the PRO core's `Flash`.
    fn assert_erased(flash: &mut Flash<'static, esp_hal::Blocking>, block: u32) {
        let mut words = [0u32; 256];
        for offset in (0..BLOCK).step_by(1024) {
            flash.read(block + offset, &mut words).unwrap();
            assert!(
                words.iter().all(|&word| word == u32::MAX),
                "the block is not erased at {:#x}",
                block + offset
            );
        }
    }

    #[test]
    fn test_stop_waits_for_the_app_core_erase() {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        // SAFETY: each core uses its own `Flash`, and the flash chip lock keeps their ROM calls
        // apart.
        let app_flash = unsafe { p.FLASH.clone_unchecked() };
        let mut flash = Flash::new(p.FLASH, Config::default()).unwrap();
        let block = scratch_block(&mut flash);
        let mut cpu_control = CpuControl::new(p.CPU_CTRL);

        let guard = start_erasing(&mut cpu_control, app_flash, block);
        let before = Instant::now();
        drop(guard);
        let took = Instant::now() - before;

        assert!(took < LIMIT, "the stop took {took}");
        assert!(!ERASE_FAILED.load(Ordering::Relaxed));
        assert_erased(&mut flash, block);
    }

    #[test]
    fn test_park_waits_for_the_app_core_erase() {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        // SAFETY: as above.
        let app_flash = unsafe { p.FLASH.clone_unchecked() };
        let mut flash = Flash::new(p.FLASH, Config::default()).unwrap();
        let block = scratch_block(&mut flash);
        let mut cpu_control = CpuControl::new(p.CPU_CTRL);

        let _running = start_erasing(&mut cpu_control, app_flash, block);
        let before = Instant::now();
        let parked = cpu_control.park_core(Cpu::AppCpu);
        let took = Instant::now() - before;
        let erases = ERASES.load(Ordering::Acquire);

        assert!(took < LIMIT, "the park took {took}");
        assert!(!ERASE_FAILED.load(Ordering::Relaxed));
        assert_erased(&mut flash, block);
        assert_eq!(
            ERASES.load(Ordering::Acquire),
            erases,
            "the APP core erased while parked"
        );

        drop(parked);
        let resumed = Instant::now();
        while ERASES.load(Ordering::Acquire) == erases {
            assert!(
                Instant::now() - resumed < LIMIT,
                "the APP core did not erase after the park"
            );
        }
    }
}

/// A light sleep that a PRO-core interrupt requests during the PRO core's erase, while the APP core
/// waits for the flash with its interrupts masked. The sleep parks the APP core before it checks
/// the flash, so the APP core must answer the park while it waits, and the sleep then returns
/// `Refused`.
#[cfg(feature = "esp32s31")]
#[embedded_test::tests(default_timeout = 30)]
mod sleep_during_erase_with_masked_reader {
    use core::{ptr::addr_of_mut, sync::atomic::AtomicBool};

    use esp_hal::{
        Blocking,
        flash::{Config, Flash},
        handler,
        peripherals::{LPWR, Peripherals},
        rtc_cntl::sleep::{LightSleep, LowPower, RtcSleepConfig},
        system::{CpuControl, Stack},
        time::{Duration, Instant},
        timer::{OneShotTimer, timg::TimerGroup},
    };
    use hil_test::mk_static;

    use super::*;

    const BLOCK: u32 = 0x1_0000;
    /// After this delay the erase of a block still runs.
    const SLEEP_AFTER: Duration = Duration::from_millis(20);

    static ERASING: AtomicBool = AtomicBool::new(false);
    static STOP_READS: AtomicBool = AtomicBool::new(false);
    static READS: AtomicU32 = AtomicU32::new(0);
    static READ_FAILED: AtomicBool = AtomicBool::new(false);
    /// 0 before the sleep, 1 for `Refused`, 2 for `Ended`.
    static SLEEP_RESULT: AtomicU32 = AtomicU32::new(0);
    static SLEPT_IN_ERASE: AtomicBool = AtomicBool::new(false);
    /// Whether the APP core is inside its critical section, and whether it was at the sleep.
    static IN_CRITICAL_SECTION: AtomicBool = AtomicBool::new(false);
    static WAITER_MASKED: AtomicBool = AtomicBool::new(false);

    /// The timer of the sleep. The test stores it before it arms it, and then only the handler uses
    /// it. The handler takes no critical section, because the APP core holds one while it waits.
    static mut TIMER: Option<OneShotTimer<'static, Blocking>> = None;

    #[handler]
    fn sleep_in_erase() {
        // SAFETY: after the test arms the timer, only this handler uses `TIMER`.
        if let Some(timer) = unsafe { (*addr_of_mut!(TIMER)).as_mut() } {
            timer.clear_interrupt();
        }
        SLEPT_IN_ERASE.store(ERASING.load(Ordering::Acquire), Ordering::Relaxed);
        WAITER_MASKED.store(
            IN_CRITICAL_SECTION.load(Ordering::Acquire),
            Ordering::Relaxed,
        );
        let mut lpwr = LowPower::new(unsafe { LPWR::steal() });
        lpwr.set_wakeup_deadline(Instant::now() + Duration::from_millis(20));
        let result = lpwr.sleep_light(RtcSleepConfig::default());
        let code = if result == LightSleep::Refused { 1 } else { 2 };
        SLEEP_RESULT.store(code, Ordering::Release);
    }

    #[test]
    fn test_sleep_is_refused_while_the_other_core_waits_masked() {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        // SAFETY: each core uses its own `Flash`, and the flash chip lock keeps their ROM calls
        // apart.
        let app_flash = unsafe { p.FLASH.clone_unchecked() };
        let mut flash = Flash::new(p.FLASH, Config::default()).unwrap();
        let block = scratch_block(&mut flash);

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let stack = mk_static!(Stack<8192>, Stack::new());
        let _app = cpu_control
            .start_app_core(stack, move || {
                let mut flash = Flash::new(app_flash, Config::default()).unwrap();
                let mut words = [0u32; 64];
                while !STOP_READS.load(Ordering::Acquire) {
                    // The read waits for the PRO core's erase with interrupts masked.
                    let ok = critical_section::with(|_| {
                        IN_CRITICAL_SECTION.store(true, Ordering::Release);
                        let ok = flash.read(block, &mut words).is_ok();
                        IN_CRITICAL_SECTION.store(false, Ordering::Release);
                        ok
                    });
                    if !ok {
                        READ_FAILED.store(true, Ordering::Relaxed);
                    }
                    READS.fetch_add(1, Ordering::Release);
                }
            })
            .unwrap();
        while READS.load(Ordering::Acquire) == 0 {}

        let timg0 = TimerGroup::new(p.TIMG0);
        let mut timer = OneShotTimer::new(timg0.timer0);
        timer.set_interrupt_handler(sleep_in_erase);
        // SAFETY: the timer is not armed yet, so the handler does not use `TIMER`.
        let timer = unsafe { (*addr_of_mut!(TIMER)).insert(timer) };
        timer.listen();

        ERASING.store(true, Ordering::Release);
        timer.schedule(SLEEP_AFTER).unwrap();
        let started = Instant::now();
        // SAFETY: the block is in the booted partition, after the image.
        let erased = unsafe { flash.erase(block, block + BLOCK) }.is_ok();
        ERASING.store(false, Ordering::Release);
        let took = Instant::now() - started;

        let reads = READS.load(Ordering::Acquire);
        let deadline = Instant::now() + Duration::from_secs(1);
        while READS.load(Ordering::Acquire) == reads {
            assert!(
                Instant::now() < deadline,
                "the APP core read nothing after the erase"
            );
        }
        STOP_READS.store(true, Ordering::Release);

        assert!(erased, "the erase failed");
        assert!(
            !READ_FAILED.load(Ordering::Relaxed),
            "a read on the APP core failed"
        );
        assert!(
            took > SLEEP_AFTER,
            "the erase ended before the sleep, after {took}"
        );
        assert!(
            SLEPT_IN_ERASE.load(Ordering::Relaxed),
            "the sleep came after the erase"
        );
        assert!(
            WAITER_MASKED.load(Ordering::Relaxed),
            "the APP core was not in its critical section at the sleep"
        );
        assert_eq!(
            SLEEP_RESULT.load(Ordering::Acquire),
            1,
            "the sleep was not refused"
        );
    }
}
