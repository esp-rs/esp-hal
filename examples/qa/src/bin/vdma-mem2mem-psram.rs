//! ESP32-P4 DW-GDMA mem2mem diagnostic (SRAM->PSRAM and SRAM->SRAM).
//!
//! This is a standalone QA binary to validate whether the DW-GDMA (VDMA)
//! destination path to PSRAM works independently of MIPI-CSI.
//!
//! The HAL drivers never run memory-to-memory transfers, so this binary
//! programs one contiguous block transfer directly through the VDMA registers.
//! It must not run while a MIPI-CSI or MIPI-DSI driver owns VDMA.
//% CHIP_FILTER: mipi_dsi_driver_supported && dma_can_access_psram && psram_driver_supported

#![no_std]
#![no_main]

extern crate alloc;

use alloc::alloc::{alloc, handle_alloc_error};
use core::alloc::Layout;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    Config,
    dma::aligned::{DmaAlignedMut, DmaAlignedRef},
    main,
    peripherals::{HP_SYS_CLKRST, IOMUX_MSPI_PIN, MEMSPI2, VDMA},
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const TRANSFER_BYTES: usize = 64 * 1024;
const GUARD_BYTES: usize = 4 * 1024;
const DEST_TOTAL_BYTES: usize = TRANSFER_BYTES + GUARD_BYTES;
const TRANSFER_UNITS_64BIT: u32 = (TRANSFER_BYTES / 8) as u32;
const DMA_TIMEOUT_MS: u64 = 50;
const DEST_SENTINEL: u8 = 0xCC;
const PSRAM_CACHE_ALIGNMENT: usize = 128;
const PSRAM_DST_OFFSET_ALIGNMENT: usize = 64;
const INTERNAL_RAM_LOW: usize = 0x4ff0_0000;
const INTERNAL_RAM_HIGH: usize = 0x4ffc_0000;
const PSRAM_LOW: usize = 0x4800_0000;
const PSRAM_HIGH: usize = 0x4c00_0000;

// DW-AXI-DMAC register encodings.
const TT_FC_M2M_DMAC: u8 = 0;
const WIDTH_64_BITS: u8 = 3;
const BURST_SIZE_512_ITEMS: u8 = 8;
const AXI_BURST_LEN_16: u8 = 16;
const CHANNEL_PRIORITY: u8 = 1;
const OSR_LIMIT_5_REQUESTS: u8 = 4;
const MAX_BLOCK_TRANSFER_UNITS: u32 = 0x0040_0000;
const DISABLE_TIMEOUT_MS: u64 = 20;
const SUSPEND_TIMEOUT_MS: u64 = 5;
const ABORT_TIMEOUT_MS: u64 = 5;
// Channel-0 bits of `CHEN0`/`CHEN1`: each field pairs a value bit with a
// write-enable bit 8 positions higher. Shift left by the channel number.
const CHANNEL_ENABLE_WRITE: u32 = 0x0100;
const CHANNEL_ENABLE_SET: u32 = 0x0101;
const CHANNEL_SUSPEND_SET: u32 = 0x0101_0000;
const CHANNEL_ABORT_SET: u32 = 0x0101;
const STATUS_DMA_TRANSFER_DONE: u32 = 1 << 1;
const STATUS_CH_SUSPENDED: u32 = 1 << 29;
const STATUS_ERROR_MASK: u32 = (1 << 5)
    | (1 << 6)
    | (1 << 7)
    | (1 << 8)
    | (1 << 9)
    | (1 << 10)
    | (1 << 11)
    | (1 << 12)
    | (1 << 13)
    | (1 << 14)
    | (1 << 16)
    | (1 << 17)
    | (1 << 18)
    | (1 << 19)
    | (1 << 20)
    | (1 << 21)
    | (1 << 25);
const STATUS_ENABLE_MASK: u32 = (1 << 0)
    | STATUS_DMA_TRANSFER_DONE
    | (1 << 3)
    | (1 << 4)
    | STATUS_ERROR_MASK
    | (1 << 28)
    | STATUS_CH_SUSPENDED
    | (1 << 30)
    | (1 << 31);

/// One mem2mem transfer setup. Unset `VDMA_MEM2MEM_DMA_PROFILE` uses [`Self::BASELINE`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct DmaProfile {
    /// `CTL0.DST_MSIZE` code. 8 is 512 items.
    dst_msize: u8,
    /// `CTL1.AWLEN` value written with `AWLEN_EN=1`.
    dst_awlen: u8,
    /// `CFG1.SRC_OSR_LMT` field. Outstanding reads are this value plus 1.
    src_osr_lmt: u8,
    /// `CFG1.DST_OSR_LMT` field. Outstanding writes are this value plus 1.
    dst_osr_lmt: u8,
}

impl DmaProfile {
    const BASELINE: Self = Self {
        dst_msize: BURST_SIZE_512_ITEMS,
        dst_awlen: AXI_BURST_LEN_16,
        src_osr_lmt: OSR_LIMIT_5_REQUESTS,
        dst_osr_lmt: OSR_LIMIT_5_REQUESTS,
    };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StopOutcome {
    AlreadyStopped,
    Disabled,
    Aborted,
    AbortTimedOut,
}

#[derive(Clone, Copy, Debug)]
struct TransferReport {
    src_addr: u32,
    dst_addr: u32,
    dar_end: u32,
    transfer_units: u32,
    transfer_bytes: u32,
    bytes_written: u32,
    width: u8,
    burst_size: u8,
    axi_burst_len: u8,
    transfer_type_flow_control: u8,
    src_outstanding_limit: u8,
    dst_outstanding_limit: u8,
    intstatus_enable0: u32,
    intstatus_before: u32,
    intstatus_after: u32,
    cfg1_bits: u32,
    ctl0_bits: u32,
    ctl1_bits: u32,
    block_ts0_bits: u32,
    status0_after: u32,
    status1_after: u32,
    chen0_after_stop: u32,
    chen1_after_stop: u32,
    completed: bool,
    dma_error: bool,
    timed_out: bool,
    poll_count: u32,
    elapsed_us: u64,
    stop_outcome: StopOutcome,
    channel_enabled_after_stop: bool,
    abort_in_progress_after_stop: bool,
    buffers_safe_for_cpu: bool,
}

#[repr(align(128))]
struct Align128<const N: usize>([u8; N]);

static mut SRC_INTERNAL: Align128<TRANSFER_BYTES> = Align128([0; TRANSFER_BYTES]);
static mut DST_INTERNAL: Align128<DEST_TOTAL_BYTES> = Align128([0; DEST_TOTAL_BYTES]);

#[derive(Clone, Copy, Debug)]
struct VerificationResult {
    data_mismatch: Option<(usize, u8, u8)>,
    guard_mismatch: Option<(usize, u8)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CaseSelection {
    Both,
    PsramOnly,
    SramOnly,
    /// CPU stores and loads only. No DMA channel is enabled.
    CpuPsram,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CaseOutcome {
    pass: bool,
    buffers_safe_for_cpu: bool,
}

fn step(step: u32, msg: &str) {
    println!("STEP {step}: {msg}");
}

/// Read back the PSRAM function-clock mux and the MSPI SRAM divider after init.
fn log_psram_clock_readback() {
    let mpll_hz = esp_hal::clock::ll::mpll_clk_frequency();
    let source_hz = esp_hal::clock::ll::PsramInstance::Psram.function_clock_frequency();
    let clk_ctrl = esp_hal::peripherals::HP_SYS_CLKRST::regs()
        .peri_clk_ctrl00()
        .read();
    let sram_clk = esp_hal::peripherals::MEMSPI2::regs().sram_clk().read();
    // `src_sel`: 0 = XTAL, 1 = MPLL, 2 = SPLL, 3 = CPLL.
    let src_sel = clk_ctrl.psram_clk_src_sel().bits();
    // Register value N means divide by N + 1. Reset and IDF both use 0 (divide by 1).
    let core_div_num = clk_ctrl.psram_core_clk_div_num().bits();
    let core_div = u32::from(core_div_num) + 1;
    let bus_div = if sram_clk.sclk_equ_sysclk().bit_is_set() {
        1
    } else {
        u32::from(sram_clk.sclkcnt_n().bits()) + 1
    };
    let bus_mhz = source_hz / 1_000_000 / core_div / bus_div;
    println!(
        "psram-clock: mpll_hz={mpll_hz} source_hz={source_hz} src_sel={src_sel} core_div_num={core_div_num} sram_clk=0x{:08x} n={} h={} l={} equ={} bus_div={bus_div} bus_mhz={bus_mhz}",
        sram_clk.bits(),
        sram_clk.sclkcnt_n().bits(),
        sram_clk.sclkcnt_h().bits(),
        sram_clk.sclkcnt_l().bits(),
        u8::from(sram_clk.sclk_equ_sysclk().bit_is_set()),
    );
}

fn selected_case() -> CaseSelection {
    match option_env!("VDMA_MEM2MEM_CASE") {
        Some(mode) if mode.eq_ignore_ascii_case("psram") => CaseSelection::PsramOnly,
        Some(mode) if mode.eq_ignore_ascii_case("sram") => CaseSelection::SramOnly,
        Some(mode) if mode.eq_ignore_ascii_case("cpu-psram") => CaseSelection::CpuPsram,
        _ => CaseSelection::Both,
    }
}

fn parse_channel_env(value: Option<&'static str>, default: u8, name: &str) -> u8 {
    match value {
        None => default,
        Some("0") => 0,
        Some("1") => 1,
        Some(other) => panic!("{name} must be '0' or '1', got '{other}'"),
    }
}

fn parse_usize_env(value: &'static str, name: &str) -> usize {
    if let Some(stripped) = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
    {
        usize::from_str_radix(stripped, 16)
            .unwrap_or_else(|_| panic!("{name} must be a valid usize literal, got '{value}'"))
    } else {
        value
            .parse::<usize>()
            .unwrap_or_else(|_| panic!("{name} must be a valid usize literal, got '{value}'"))
    }
}

fn selected_psram_channel() -> u8 {
    parse_channel_env(
        option_env!("VDMA_MEM2MEM_PSRAM_CH"),
        1,
        "VDMA_MEM2MEM_PSRAM_CH",
    )
}

fn selected_sram_channel() -> u8 {
    parse_channel_env(
        option_env!("VDMA_MEM2MEM_SRAM_CH"),
        0,
        "VDMA_MEM2MEM_SRAM_CH",
    )
}

fn selected_dst_awlen_auto() -> bool {
    match option_env!("VDMA_MEM2MEM_DST_AWLEN_AUTO") {
        None => false,
        Some("0") => false,
        Some("1") => true,
        Some(other) => panic!("VDMA_MEM2MEM_DST_AWLEN_AUTO must be '0' or '1', got '{other}'"),
    }
}

/// One DMA parameter away from the current helper baseline.
///
/// `msize` codes follow IDF `dw_gdma_burst_items_t`: 3 is 16 items, 8 is 512 items.
/// `OSR` fields follow IDF `limit - 1`: 0 is 1 outstanding request, 4 is 5.
/// `AWLEN` is the value written into `CTL1.AWLEN` with `AWLEN_EN=1`.
fn selected_dma_profile() -> (&'static str, DmaProfile) {
    let baseline = DmaProfile::BASELINE;
    match option_env!("VDMA_MEM2MEM_DMA_PROFILE") {
        None | Some("baseline") => ("baseline", baseline),
        Some("dst-msize-16") => (
            "dst-msize-16",
            DmaProfile {
                dst_msize: 3,
                ..baseline
            },
        ),
        Some("dst-awlen-4") => (
            "dst-awlen-4",
            DmaProfile {
                dst_awlen: 4,
                ..baseline
            },
        ),
        Some("dst-osr-1") => (
            "dst-osr-1",
            DmaProfile {
                dst_osr_lmt: 0,
                ..baseline
            },
        ),
        Some("src-osr-1") => (
            "src-osr-1",
            DmaProfile {
                src_osr_lmt: 0,
                ..baseline
            },
        ),
        Some(other) => panic!(
            "VDMA_MEM2MEM_DMA_PROFILE must be baseline, dst-msize-16, dst-awlen-4, dst-osr-1, or src-osr-1, got '{other}'"
        ),
    }
}

fn msize_items(code: u32) -> u32 {
    match code {
        0 => 1,
        1..=9 => 1u32 << (code + 1),
        _ => 0,
    }
}

fn selected_psram_dst_offset() -> usize {
    let offset = match option_env!("VDMA_MEM2MEM_PSRAM_DST_OFFSET") {
        None => 0,
        Some(raw) => parse_usize_env(raw, "VDMA_MEM2MEM_PSRAM_DST_OFFSET"),
    };
    assert!(
        offset % PSRAM_DST_OFFSET_ALIGNMENT == 0,
        "VDMA_MEM2MEM_PSRAM_DST_OFFSET must be {}-byte aligned",
        PSRAM_DST_OFFSET_ALIGNMENT
    );
    offset
}

fn align_up(value: usize, alignment: usize) -> usize {
    value.div_ceil(alignment) * alignment
}

fn alloc_aligned_bytes(size: usize, alignment: usize) -> &'static mut [u8] {
    let layout = Layout::from_size_align(size, alignment).expect("invalid PSRAM allocation layout");
    let ptr = unsafe { alloc(layout) };
    if ptr.is_null() {
        handle_alloc_error(layout);
    }
    unsafe { core::slice::from_raw_parts_mut(ptr, size) }
}

fn fill_pattern(buf: &mut [u8]) {
    let mut x = 0x1357_9bdfu32;
    for byte in buf.iter_mut() {
        x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *byte = (x >> 24) as u8;
    }
}

fn first_data_mismatch(expected: &[u8], actual: &[u8]) -> Option<(usize, u8, u8)> {
    expected
        .iter()
        .zip(actual.iter())
        .enumerate()
        .find_map(|(i, (&e, &a))| (e != a).then_some((i, e, a)))
}

fn first_guard_mismatch(guard: &[u8]) -> Option<(usize, u8)> {
    guard
        .iter()
        .enumerate()
        .find_map(|(i, &v)| (v != DEST_SENTINEL).then_some((i, v)))
}

fn range_of(buf: &[u8]) -> (usize, usize) {
    let start = buf.as_ptr() as usize;
    (start, start + buf.len())
}

fn ranges_overlap(a: (usize, usize), b: (usize, usize)) -> bool {
    a.0 < b.1 && b.0 < a.1
}

fn in_internal_ram(range: (usize, usize)) -> bool {
    range.0 >= INTERNAL_RAM_LOW && range.1 <= INTERNAL_RAM_HIGH
}

fn in_psram(range: (usize, usize)) -> bool {
    range.0 >= PSRAM_LOW && range.1 <= PSRAM_HIGH
}

fn expected_alignment(range: (usize, usize)) -> Option<usize> {
    if in_internal_ram(range) {
        Some(64)
    } else if in_psram(range) {
        Some(128)
    } else {
        None
    }
}

fn print_layout_checks(
    src: &[u8],
    dst_internal: &[u8],
    dst_psram_cache: &[u8],
    dst_psram_offset: usize,
) {
    assert!(
        dst_psram_offset + DEST_TOTAL_BYTES <= dst_psram_cache.len(),
        "PSRAM destination offset exceeds allocated cache region"
    );
    let dst_psram = &dst_psram_cache[dst_psram_offset..dst_psram_offset + DEST_TOTAL_BYTES];
    let src_range = range_of(src);
    let dst_internal_range = range_of(dst_internal);
    let dst_psram_cache_range = range_of(dst_psram_cache);
    let dst_psram_range = range_of(dst_psram);

    println!(
        "layout: src_internal=[0x{:08x}..0x{:08x}) len={}",
        src_range.0,
        src_range.1,
        src.len()
    );
    println!(
        "layout: dst_internal=[0x{:08x}..0x{:08x}) len={}",
        dst_internal_range.0,
        dst_internal_range.1,
        dst_internal.len()
    );
    println!(
        "layout: dst_psram_cache=[0x{:08x}..0x{:08x}) len={}",
        dst_psram_cache_range.0,
        dst_psram_cache_range.1,
        dst_psram_cache.len()
    );
    println!(
        "layout: dst_psram_transfer=[0x{:08x}..0x{:08x}) len={} offset={}",
        dst_psram_range.0,
        dst_psram_range.1,
        dst_psram.len(),
        dst_psram_offset
    );

    let src_internal_ok = in_internal_ram(src_range);
    let dst_internal_ok = in_internal_ram(dst_internal_range);
    let dst_psram_cache_ok = in_psram(dst_psram_cache_range);
    let dst_psram_ok = in_psram(dst_psram_range);
    let src_dst_internal_overlap = ranges_overlap(src_range, dst_internal_range);
    let src_dst_psram_overlap = ranges_overlap(src_range, dst_psram_range);
    let dst_dst_overlap = ranges_overlap(dst_internal_range, dst_psram_range);

    println!(
        "layout-check: src_in_internal={} dst_internal_in_internal={} dst_psram_cache_in_psram={} dst_psram_transfer_in_psram={}",
        src_internal_ok, dst_internal_ok, dst_psram_cache_ok, dst_psram_ok
    );
    println!(
        "layout-check: overlap(src,dst_internal)={} overlap(src,dst_psram)={} overlap(dst_internal,dst_psram)={}",
        src_dst_internal_overlap, src_dst_psram_overlap, dst_dst_overlap
    );

    assert!(src_internal_ok, "source buffer is outside internal SRAM");
    assert!(
        dst_internal_ok,
        "internal destination buffer is outside internal SRAM"
    );
    assert!(dst_psram_cache_ok, "PSRAM cache region is outside PSRAM");
    assert!(dst_psram_ok, "PSRAM destination buffer is outside PSRAM");
    assert!(
        !src_dst_internal_overlap,
        "source and internal destination overlap"
    );
    assert!(
        !src_dst_psram_overlap,
        "source and PSRAM destination overlap"
    );
    assert!(
        !dst_dst_overlap,
        "internal and PSRAM destinations overlap unexpectedly"
    );
}

fn print_sram_layout_checks(src: &[u8], dst_internal: &[u8]) {
    let src_range = range_of(src);
    let dst_internal_range = range_of(dst_internal);
    let src_internal_ok = in_internal_ram(src_range);
    let dst_internal_ok = in_internal_ram(dst_internal_range);
    let overlap = ranges_overlap(src_range, dst_internal_range);

    println!(
        "layout: src_internal=[0x{:08x}..0x{:08x}) len={}",
        src_range.0,
        src_range.1,
        src.len()
    );
    println!(
        "layout: dst_internal=[0x{:08x}..0x{:08x}) len={}",
        dst_internal_range.0,
        dst_internal_range.1,
        dst_internal.len()
    );
    println!(
        "layout-check: src_in_internal={} dst_internal_in_internal={} overlap(src,dst_internal)={}",
        src_internal_ok, dst_internal_ok, overlap
    );

    assert!(src_internal_ok, "source buffer is outside internal SRAM");
    assert!(
        dst_internal_ok,
        "internal destination buffer is outside internal SRAM"
    );
    assert!(!overlap, "source and internal destination overlap");
}

fn check_cache_region(label: &str, buf: &[u8]) {
    let range = range_of(buf);
    let alignment =
        expected_alignment(range).expect("cache-check buffer is outside internal SRAM/PSRAM");
    let addr_aligned = range.0 % alignment == 0;
    let size_aligned = buf.len() % alignment == 0;
    println!(
        "cache-check({label}): range=[0x{:08x}..0x{:08x}) align_req={} addr_aligned={} size_aligned={}",
        range.0, range.1, alignment, addr_aligned, size_aligned
    );
    assert!(addr_aligned, "{label} address alignment is invalid");
    assert!(size_aligned, "{label} size alignment is invalid");
}

fn prepare_buffers_for_dma(label: &str, src: &[u8], dst_with_guard: &mut [u8]) {
    step(14, "before cache-region validation");
    check_cache_region("src", src);
    check_cache_region("dst", dst_with_guard);
    step(15, "after cache-region validation");

    step(16, "before src cache writeback");
    DmaAlignedRef::new(src)
        .expect("source buffer is not DMA/cacheline aligned")
        .writeback();
    step(17, "after src cache writeback");

    step(18, "before dst cache writeback/invalidate");
    let mut dst = DmaAlignedMut::new(dst_with_guard)
        .expect("destination buffer is not DMA/cacheline aligned");
    dst.writeback();
    dst.invalidate();
    step(19, "after dst cache writeback/invalidate");
    println!("cache-ready: {label}");
}

/// CPU-only PSRAM check. Does not enable a DMA channel.
///
/// `0x4800_0000` is the cached PSRAM window. Stores update cache lines and do
/// not, by themselves, prove that PSRAM captured the data. After each
/// pattern this writes the lines back and then invalidates them, using the
/// same `DmaAlignedMut` cache ops as the DMA cases. The following loads miss
/// and are fetched from PSRAM through MSPI2.
fn run_cpu_psram(buf: &mut [u8], offset: usize) -> bool {
    let payload_end = offset + TRANSFER_BYTES;
    let guarded_end = offset + DEST_TOTAL_BYTES;
    assert!(
        guarded_end <= buf.len(),
        "CPU PSRAM buffer is smaller than the 64 KiB payload plus guard"
    );
    let base = buf.as_ptr() as usize;
    println!(
        "cpu-psram: base=0x{base:08x} payload=[0x{:08x}..0x{:08x}) guard=[0x{:08x}..0x{:08x})",
        base + offset,
        base + payload_end,
        base + payload_end,
        base + guarded_end
    );

    buf[payload_end..guarded_end].fill(DEST_SENTINEL);

    let pattern_a = cpu_pattern_pass(
        buf,
        offset,
        payload_end,
        guarded_end,
        0x1357_9bdf,
        "pattern-a",
    );
    let pattern_b = cpu_pattern_pass(
        buf,
        offset,
        payload_end,
        guarded_end,
        0xa5a5_f00d,
        "pattern-b",
    );
    pattern_a && pattern_b
}

fn cpu_pattern_pass(
    buf: &mut [u8],
    offset: usize,
    payload_end: usize,
    guarded_end: usize,
    seed: u32,
    name: &str,
) -> bool {
    write_lcg(&mut buf[offset..payload_end], seed);
    flush_psram_lines(&mut buf[..guarded_end]);

    if let Some((index, expected, actual)) = first_lcg_mismatch(&buf[offset..payload_end], seed) {
        let addr = buf.as_ptr() as usize + offset + index;
        println!(
            "cpu-psram {name}: mismatch addr=0x{addr:08x} offset={index} expected=0x{expected:02x} actual=0x{actual:02x}"
        );
        return false;
    }
    if let Some((index, actual)) = first_guard_mismatch(&buf[payload_end..guarded_end]) {
        let addr = buf.as_ptr() as usize + payload_end + index;
        println!(
            "cpu-psram {name}: guard mismatch addr=0x{addr:08x} offset={index} expected=0x{DEST_SENTINEL:02x} actual=0x{actual:02x}"
        );
        return false;
    }
    println!("cpu-psram {name}: PASS");
    true
}

fn write_lcg(dst: &mut [u8], seed: u32) {
    let mut state = seed;
    for slot in dst.iter_mut() {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *slot = (state >> 24) as u8;
    }
}

fn first_lcg_mismatch(actual: &[u8], seed: u32) -> Option<(usize, u8, u8)> {
    let mut state = seed;
    for (index, &byte) in actual.iter().enumerate() {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let expected = (state >> 24) as u8;
        if byte != expected {
            return Some((index, expected, byte));
        }
    }
    None
}

fn flush_psram_lines(buf: &mut [u8]) {
    let mut aligned = DmaAlignedMut::new(buf).expect("PSRAM buffer is not cache-line aligned");
    aligned.writeback();
    aligned.invalidate();
}

fn invalidate_after_dma(label: &str, buf: &[u8]) {
    step(22, "before destination cache invalidate after DMA");
    DmaAlignedRef::new(buf)
        .expect("destination buffer is not DMA/cacheline aligned")
        .invalidate();
    step(23, "after destination cache invalidate after DMA");
    println!("cache-ready-after-dma: {label}");
}

fn wait_until(timeout_ms: u64, mut condition: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    while Instant::now() <= deadline {
        if condition() {
            return true;
        }
    }
    condition()
}

/// Gates the GDMA clock on and pulses its reset. Nothing else owns VDMA in
/// this binary, so the clock is left enabled.
fn enable_vdma_controller() {
    let clkrst = HP_SYS_CLKRST::regs();
    clkrst
        .soc_clk_ctrl1()
        .modify(|_, w| w.gdma_sys_clk_en().set_bit());
    clkrst.hp_rst_en0().modify(|_, w| w.rst_en_gdma().set_bit());
    clkrst
        .hp_rst_en0()
        .modify(|_, w| w.rst_en_gdma().clear_bit());
    VDMA::regs().cfg0().modify(|_, w| {
        w.dmac_en().set_bit();
        w.int_en().set_bit()
    });
}

fn channel_enabled(channel: usize) -> bool {
    (VDMA::regs().chen0().read().bits() >> channel) & 1 != 0
}

fn abort_in_progress(channel: usize) -> bool {
    VDMA::regs().chen1().read().bits() & (CHANNEL_ABORT_SET << channel) != 0
}

fn set_channel_enabled(channel: usize, enabled: bool) {
    let value = if enabled {
        CHANNEL_ENABLE_SET
    } else {
        CHANNEL_ENABLE_WRITE
    };
    VDMA::regs()
        .chen0()
        .write(|w| unsafe { w.bits(value << channel) });
}

/// Disables the channel, then falls back to suspend and abort.
fn stop_channel(channel: usize) -> StopOutcome {
    if !channel_enabled(channel) {
        return StopOutcome::AlreadyStopped;
    }
    set_channel_enabled(channel, false);
    if wait_until(DISABLE_TIMEOUT_MS, || !channel_enabled(channel)) {
        return StopOutcome::Disabled;
    }

    let regs = VDMA::regs();
    regs.chen0()
        .write(|w| unsafe { w.bits(CHANNEL_SUSPEND_SET << channel) });
    let _ = wait_until(SUSPEND_TIMEOUT_MS, || {
        !channel_enabled(channel)
            || regs.ch(channel).intstatus0().read().bits() & STATUS_CH_SUSPENDED != 0
    });
    regs.chen1()
        .write(|w| unsafe { w.bits(CHANNEL_ABORT_SET << channel) });
    if wait_until(ABORT_TIMEOUT_MS, || {
        !channel_enabled(channel) && !abort_in_progress(channel)
    }) {
        StopOutcome::Aborted
    } else {
        StopOutcome::AbortTimedOut
    }
}

fn print_psram_stall_snapshot() {
    let mspi = MEMSPI2::regs();
    let cmd = mspi.cmd().read();
    let ecc = mspi.smem_ecc_ctrl().read();
    let iomux = IOMUX_MSPI_PIN::regs();
    let dqs0 = iomux.psram_dqs_0_pin0().read();
    let dqs1 = iomux.psram_dqs_1_pin0().read();
    // IDF `REG_PSRAM_DQS_*_DELAY_270` is bits [20:17]. The PAC `delay_270()`
    // accessor reads bits [21:18], so take this field from the raw value.
    let dqs_delay_270 = |bits: u32| (bits >> 17) & 0x0f;
    println!(
        "PSRAM stall snapshot: cmd=0x{:08x} mst_st={} slv_st={} axi=0x{:08x} ecc=0x{:08x} page_size={} ac=0x{:08x} ddr=0x{:08x} timing_cali=0x{:08x} smem_timing_cali=0x{:08x} dqs0=0x{:08x} dqs0_phase={} dqs0_dli={} dqs0_delay90={} dqs0_delay270={} dqs1=0x{:08x} dqs1_phase={} dqs1_dli={} dqs1_delay90={} dqs1_delay270={}",
        cmd.bits(),
        cmd.mst_st().bits(),
        cmd.slv_st().bits(),
        mspi.smem_axi_addr_ctrl().read().bits(),
        ecc.bits(),
        ecc.page_size().bits(),
        mspi.smem_ac().read().bits(),
        mspi.smem_ddr().read().bits(),
        mspi.timing_cali().read().bits(),
        mspi.smem_timing_cali().read().bits(),
        dqs0.bits(),
        dqs0.phase().bits(),
        dqs0.dli().bits(),
        dqs0.delay_90().bits(),
        dqs_delay_270(dqs0.bits()),
        dqs1.bits(),
        dqs1.phase().bits(),
        dqs1.dli().bits(),
        dqs1.delay_90().bits(),
        dqs_delay_270(dqs1.bits()),
    );
}

/// Runs one contiguous SRAM-source block transfer on `channel` and polls for
/// `DMA_TFR_DONE`, an error, or the timeout.
fn mem2mem_transfer(
    channel: usize,
    src_addr: u32,
    dst_addr: u32,
    transfer_units: u32,
    timeout_ms: u64,
    dst_awlen_auto: bool,
    profile: DmaProfile,
) -> TransferReport {
    assert!(transfer_units > 0 && transfer_units <= MAX_BLOCK_TRANSFER_UNITS);

    enable_vdma_controller();
    let regs = VDMA::regs();
    let ch = regs.ch(channel);

    set_channel_enabled(channel, false);
    ch.intclear0().write(|w| unsafe { w.bits(u32::MAX) });
    ch.intstatus_enable0()
        .write(|w| unsafe { w.bits(STATUS_ENABLE_MASK) });
    ch.intsignal_enable0().write(|w| unsafe { w.bits(0) });

    // Contiguous source and destination, software handshake on both sides.
    ch.cfg0().write(|w| unsafe {
        w.ch1_src_multblk_type().bits(0);
        w.ch1_dst_multblk_type().bits(0)
    });
    ch.cfg1().write(|w| unsafe {
        w.ch1_tt_fc().bits(TT_FC_M2M_DMAC);
        w.ch1_hs_sel_src().set_bit();
        w.ch1_hs_sel_dst().set_bit();
        w.ch1_src_per().bits(0);
        w.ch1_dst_per().bits(0);
        w.ch1_ch_prior().bits(CHANNEL_PRIORITY);
        w.ch1_src_osr_lmt().bits(profile.src_osr_lmt);
        w.ch1_dst_osr_lmt().bits(profile.dst_osr_lmt)
    });
    ch.sstatar0().write(|w| unsafe { w.bits(0) });
    ch.dstatar0().write(|w| unsafe { w.bits(0) });

    ch.sar0().write(|w| unsafe { w.ch1_sar0().bits(src_addr) });
    unsafe { ch.sar1().write_with_zero(|w| w) };
    ch.dar0().write(|w| unsafe { w.ch1_dar0().bits(dst_addr) });
    unsafe { ch.dar1().write_with_zero(|w| w) };
    ch.block_ts0()
        .write(|w| unsafe { w.ch1_block_ts().bits(transfer_units - 1) });
    ch.ctl0().write(|w| unsafe {
        w.ch1_sms().set_bit();
        w.ch1_dms().set_bit();
        w.ch1_sinc().clear_bit();
        w.ch1_dinc().clear_bit();
        w.ch1_src_tr_width().bits(WIDTH_64_BITS);
        w.ch1_dst_tr_width().bits(WIDTH_64_BITS);
        w.ch1_src_msize().bits(BURST_SIZE_512_ITEMS);
        w.ch1_dst_msize().bits(profile.dst_msize)
    });
    ch.ctl1().write(|w| unsafe {
        w.ch1_arlen_en().set_bit();
        w.ch1_arlen().bits(AXI_BURST_LEN_16);
        w.ch1_awlen_en().bit(!dst_awlen_auto);
        w.ch1_awlen()
            .bits(if dst_awlen_auto { 0 } else { profile.dst_awlen });
        w.ch1_ioc_blktfr().clear_bit();
        w.ch1_shadowreg_or_lli_last().clear_bit();
        w.ch1_shadowreg_or_lli_valid().clear_bit()
    });

    let cfg1_bits = ch.cfg1().read().bits();
    let ctl0_bits = ch.ctl0().read().bits();
    let ctl1_bits = ch.ctl1().read().bits();
    let block_ts0_bits = ch.block_ts0().read().bits();

    let intstatus_before = ch.intstatus0().read().bits();
    println!(
        "PSRAM AXI before DMA: 0x{:08x}",
        MEMSPI2::regs().smem_axi_addr_ctrl().read().bits()
    );
    let start = Instant::now();
    set_channel_enabled(channel, true);

    let mut completed = false;
    let mut dma_error = false;
    let mut timed_out = false;
    let mut poll_count = 0u32;
    let deadline = start + Duration::from_millis(timeout_ms);
    let intstatus_after = loop {
        poll_count = poll_count.saturating_add(1);
        let status = ch.intstatus0().read().bits();
        if status & STATUS_DMA_TRANSFER_DONE != 0 {
            completed = true;
            break status;
        }
        if status & STATUS_ERROR_MASK != 0 {
            dma_error = true;
            break status;
        }
        if Instant::now() >= deadline {
            timed_out = true;
            break status;
        }
    };
    let elapsed_us = start.elapsed().as_micros();
    let dar_end = ch.dar0().read().bits();

    println!(
        "PSRAM AXI after DMA wait: 0x{:08x}",
        MEMSPI2::regs().smem_axi_addr_ctrl().read().bits()
    );
    if timed_out {
        print_psram_stall_snapshot();
    }

    let stop_outcome = stop_channel(channel);
    let channel_enabled_after_stop = channel_enabled(channel);
    let abort_in_progress_after_stop = abort_in_progress(channel);
    let buffers_safe_for_cpu = !channel_enabled_after_stop
        && !abort_in_progress_after_stop
        && stop_outcome != StopOutcome::AbortTimedOut;
    let report = TransferReport {
        src_addr,
        dst_addr,
        dar_end,
        transfer_units,
        transfer_bytes: transfer_units * 8,
        bytes_written: dar_end.saturating_sub(dst_addr),
        width: WIDTH_64_BITS,
        burst_size: profile.dst_msize,
        axi_burst_len: profile.dst_awlen,
        transfer_type_flow_control: TT_FC_M2M_DMAC,
        src_outstanding_limit: profile.src_osr_lmt,
        dst_outstanding_limit: profile.dst_osr_lmt,
        intstatus_enable0: ch.intstatus_enable0().read().bits(),
        intstatus_before,
        intstatus_after,
        cfg1_bits,
        ctl0_bits,
        ctl1_bits,
        block_ts0_bits,
        status0_after: ch.status0().read().bits(),
        status1_after: ch.status1().read().bits(),
        chen0_after_stop: regs.chen0().read().bits(),
        chen1_after_stop: regs.chen1().read().bits(),
        completed,
        dma_error,
        timed_out,
        poll_count,
        elapsed_us,
        stop_outcome,
        channel_enabled_after_stop,
        abort_in_progress_after_stop,
        buffers_safe_for_cpu,
    };
    if buffers_safe_for_cpu {
        ch.intclear0().write(|w| unsafe { w.bits(u32::MAX) });
    }
    report
}

fn print_report(label: &str, report: &TransferReport, verify: Option<&VerificationResult>) {
    println!("--- {label} ---");
    println!(
        "addr: src=0x{:08x} dst=0x{:08x} dar_end=0x{:08x}",
        report.src_addr, report.dst_addr, report.dar_end
    );
    println!(
        "cfg: units={} bytes={} width={} burst={} axi_len={} tt_fc={} osr(src,dst)=({}, {})",
        report.transfer_units,
        report.transfer_bytes,
        report.width,
        report.burst_size,
        report.axi_burst_len,
        report.transfer_type_flow_control,
        report.src_outstanding_limit,
        report.dst_outstanding_limit
    );
    println!(
        "status: int_ena=0x{:08x} int_before=0x{:08x} int_after=0x{:08x} status0=0x{:08x} status1=0x{:08x}",
        report.intstatus_enable0,
        report.intstatus_before,
        report.intstatus_after,
        report.status0_after,
        report.status1_after
    );
    println!(
        "regs: cfg1=0x{:08x} ctl0=0x{:08x} ctl1=0x{:08x} block_ts0=0x{:08x}",
        report.cfg1_bits, report.ctl0_bits, report.ctl1_bits, report.block_ts0_bits
    );
    println!(
        "regs-decode: tt_fc={} sinc={} dinc={} sms={} dms={} src_width={} dst_width={} src_msize={} src_items={} dst_msize={} dst_items={} arlen_en={} arlen={} awlen_en={} awlen={} src_osr={} src_outstanding={} dst_osr={} dst_outstanding={}",
        report.cfg1_bits & 0x7,
        (report.ctl0_bits >> 4) & 0x1,
        (report.ctl0_bits >> 6) & 0x1,
        report.ctl0_bits & 0x1,
        (report.ctl0_bits >> 2) & 0x1,
        (report.ctl0_bits >> 8) & 0x7,
        (report.ctl0_bits >> 11) & 0x7,
        (report.ctl0_bits >> 14) & 0xf,
        msize_items((report.ctl0_bits >> 14) & 0xf),
        (report.ctl0_bits >> 18) & 0xf,
        msize_items((report.ctl0_bits >> 18) & 0xf),
        (report.ctl1_bits >> 6) & 0x1,
        (report.ctl1_bits >> 7) & 0xff,
        (report.ctl1_bits >> 15) & 0x1,
        (report.ctl1_bits >> 16) & 0xff,
        (report.cfg1_bits >> 23) & 0xf,
        ((report.cfg1_bits >> 23) & 0xf) + 1,
        (report.cfg1_bits >> 27) & 0xf,
        ((report.cfg1_bits >> 27) & 0xf) + 1,
    );
    println!(
        "result: completed={} timed_out={} dma_error={} bytes_written={} elapsed_us={}",
        report.completed,
        report.timed_out,
        report.dma_error,
        report.bytes_written,
        report.elapsed_us
    );
    println!(
        "shutdown: outcome={:?} ch_enabled_after={} abort_after={} chen0=0x{:08x} chen1=0x{:08x}",
        report.stop_outcome,
        report.channel_enabled_after_stop,
        report.abort_in_progress_after_stop,
        report.chen0_after_stop,
        report.chen1_after_stop
    );
    match verify {
        Some(v) => println!(
            "verify: data_mismatch={:?} guard_mismatch={:?}",
            v.data_mismatch, v.guard_mismatch
        ),
        None => println!("verify: SKIPPED (DMA buffer ownership not confirmed safe)"),
    }
    println!("buffers-safe-for-cpu={}", report.buffers_safe_for_cpu);
}

fn run_case(
    label: &str,
    channel: usize,
    src: &mut [u8],
    dst_cache_region: &mut [u8],
    dst_transfer_offset: usize,
    dst_awlen_auto: bool,
) -> CaseOutcome {
    assert!(
        dst_transfer_offset + DEST_TOTAL_BYTES <= dst_cache_region.len(),
        "destination transfer range exceeds cache region"
    );
    println!("STEP 10: case-start {label}");
    step(11, "before source pattern fill");
    fill_pattern(src);
    step(12, "after source pattern fill");
    dst_cache_region[dst_transfer_offset..dst_transfer_offset + DEST_TOTAL_BYTES]
        .fill(DEST_SENTINEL);
    step(13, "after destination sentinel initialization");
    prepare_buffers_for_dma(label, src, dst_cache_region);

    step(20, "before DMA transfer");
    let (_profile_name, profile) = selected_dma_profile();
    let dst_addr = {
        let dst_with_guard =
            &mut dst_cache_region[dst_transfer_offset..dst_transfer_offset + DEST_TOTAL_BYTES];
        dst_with_guard.as_mut_ptr() as u32
    };
    let report = mem2mem_transfer(
        channel,
        src.as_ptr() as u32,
        dst_addr,
        TRANSFER_UNITS_64BIT,
        DMA_TIMEOUT_MS,
        dst_awlen_auto,
        profile,
    );
    step(21, "after DMA transfer");

    if !report.buffers_safe_for_cpu {
        println!(
            "ownership-guard: DMA stop outcome {:?} leaves channel ownership uncertain; skipping invalidate/verify/reuse. This boot will not start another DMA transfer.",
            report.stop_outcome
        );
        print_report(label, &report, None);
        println!(
            "wait-observable: completed={} dma_error={} timed_out={} poll_count={}",
            report.completed, report.dma_error, report.timed_out, report.poll_count
        );
        println!("{label} RESULT: FAIL");
        return CaseOutcome {
            pass: false,
            buffers_safe_for_cpu: false,
        };
    }

    invalidate_after_dma(label, dst_cache_region);

    step(24, "before data/guard verification");
    let dst_with_guard =
        &dst_cache_region[dst_transfer_offset..dst_transfer_offset + DEST_TOTAL_BYTES];
    let transferred = &dst_with_guard[..TRANSFER_BYTES];
    let guard = &dst_with_guard[TRANSFER_BYTES..];
    let verify = VerificationResult {
        data_mismatch: first_data_mismatch(src, transferred),
        guard_mismatch: first_guard_mismatch(guard),
    };
    step(25, "after data/guard verification");

    print_report(label, &report, Some(&verify));

    let transfer_ok = report.completed
        && !report.timed_out
        && !report.dma_error
        && report.bytes_written == TRANSFER_BYTES as u32;
    let shutdown_ok = !report.channel_enabled_after_stop && !report.abort_in_progress_after_stop;
    let verify_ok = verify.data_mismatch.is_none() && verify.guard_mismatch.is_none();
    let pass = transfer_ok && shutdown_ok && verify_ok;

    println!(
        "wait-observable: completed={} dma_error={} timed_out={} poll_count={}",
        report.completed, report.dma_error, report.timed_out, report.poll_count
    );
    println!("{label} RESULT: {}", if pass { "PASS" } else { "FAIL" });
    CaseOutcome {
        pass,
        buffers_safe_for_cpu: true,
    }
}

#[main]
fn main() -> ! {
    let case_selection = selected_case();
    let psram_channel = selected_psram_channel();
    let sram_channel = selected_sram_channel();
    let psram_dst_offset = selected_psram_dst_offset();
    let dst_awlen_auto = selected_dst_awlen_auto();
    let (dma_profile_name, dma_profile) = selected_dma_profile();
    let peripherals = esp_hal::init(Config::default());
    esp_println::logger::init_logger_from_env();
    // Held so that nothing else can use the channels this binary programs.
    let _vdma_channels = (peripherals.VDMA_CH0, peripherals.VDMA_CH1);
    if case_selection != CaseSelection::SramOnly {
        // 250 MHz on the default 500 MHz MPLL. Tuning itself follows the
        // active preset for every PSRAM user; this line is only the QA speed.
        let timing = esp_hal::psram::PsramTimingParams::MHZ_250;
        println!(
            "psram-config: requested_mhz={} clock_source={:?} mr0_rl={} mr4_wl={} rd_dummy={} wr_dummy={} reg_dummy={}",
            timing.clock,
            timing.clock_source,
            timing.mr0_rl,
            timing.mr4_wl,
            timing.rd_dummy_bits,
            timing.wr_dummy_bits,
            timing.reg_dummy_bits
        );
        esp_alloc::psram_allocator!(
            peripherals.PSRAM,
            esp_hal::psram,
            esp_hal::psram::PsramConfig {
                timing,
                ..Default::default()
            }
        );
        log_psram_clock_readback();
    }

    println!("DW-GDMA mem2mem diagnostic (ESP32-P4)");
    println!(
        "transfer={} bytes guard={} bytes timeout={} ms",
        TRANSFER_BYTES, GUARD_BYTES, DMA_TIMEOUT_MS
    );
    println!(
        "selection: {:?} psram_channel={} sram_channel={} dst_awlen_auto={} psram_dst_offset={} dma_profile={}",
        case_selection,
        psram_channel,
        sram_channel,
        dst_awlen_auto,
        psram_dst_offset,
        dma_profile_name
    );
    println!(
        "dma-profile: name={dma_profile_name} dst_msize={} dst_items={} dst_awlen={} src_osr={} src_outstanding={} dst_osr={} dst_outstanding={}",
        dma_profile.dst_msize,
        msize_items(u32::from(dma_profile.dst_msize)),
        dma_profile.dst_awlen,
        dma_profile.src_osr_lmt,
        u32::from(dma_profile.src_osr_lmt) + 1,
        dma_profile.dst_osr_lmt,
        u32::from(dma_profile.dst_osr_lmt) + 1,
    );

    step(1, "before internal SRAM buffer acquisition");
    let src_internal = unsafe { &mut (&mut (*core::ptr::addr_of_mut!(SRC_INTERNAL)).0)[..] };
    let dst_internal = unsafe { &mut (&mut (*core::ptr::addr_of_mut!(DST_INTERNAL)).0)[..] };
    step(2, "after internal SRAM buffer acquisition");

    step(3, "before PSRAM destination allocation");
    let dst_psram_cache = if case_selection != CaseSelection::SramOnly {
        let required = psram_dst_offset + DEST_TOTAL_BYTES;
        let cache_region_len = align_up(required, PSRAM_CACHE_ALIGNMENT);
        Some(alloc_aligned_bytes(cache_region_len, PSRAM_CACHE_ALIGNMENT))
    } else {
        None
    };
    step(4, "after PSRAM destination allocation");

    step(5, "before memory layout checks");
    if let Some(dst_psram_cache) = dst_psram_cache.as_deref() {
        print_layout_checks(
            src_internal,
            dst_internal,
            dst_psram_cache,
            psram_dst_offset,
        );
    } else {
        print_sram_layout_checks(src_internal, dst_internal);
    }
    step(6, "after memory layout checks");

    let mut overall = true;

    match case_selection {
        CaseSelection::CpuPsram => {
            step(7, "before CPU-only PSRAM test");
            let pass = run_cpu_psram(
                dst_psram_cache.expect("PSRAM destination missing in CPU mode"),
                psram_dst_offset,
            );
            step(8, "after CPU-only PSRAM test");
            overall &= pass;
        }
        CaseSelection::PsramOnly => {
            step(7, "before first DMA case (SRAM->PSRAM)");
            let outcome = match psram_channel {
                0 => run_case(
                    "SRAM->PSRAM (VDMA_CH0)",
                    0,
                    src_internal,
                    dst_psram_cache.expect("PSRAM destination missing in PSRAM mode"),
                    psram_dst_offset,
                    dst_awlen_auto,
                ),
                1 => run_case(
                    "SRAM->PSRAM (VDMA_CH1)",
                    1,
                    src_internal,
                    dst_psram_cache.expect("PSRAM destination missing in PSRAM mode"),
                    psram_dst_offset,
                    dst_awlen_auto,
                ),
                _ => unreachable!(),
            };
            step(8, "after first DMA case (SRAM->PSRAM)");
            overall &= outcome.pass;
        }
        CaseSelection::SramOnly => {
            step(9, "before second DMA case (SRAM->SRAM)");
            let outcome = match sram_channel {
                0 => run_case(
                    "SRAM->SRAM control (VDMA_CH0)",
                    0,
                    src_internal,
                    dst_internal,
                    0,
                    dst_awlen_auto,
                ),
                1 => run_case(
                    "SRAM->SRAM control (VDMA_CH1)",
                    1,
                    src_internal,
                    dst_internal,
                    0,
                    dst_awlen_auto,
                ),
                _ => unreachable!(),
            };
            step(28, "after second DMA case (SRAM->SRAM)");
            overall &= outcome.pass;
        }
        CaseSelection::Both => {
            if psram_channel == sram_channel {
                panic!(
                    "CASE=both requires distinct channels; set VDMA_MEM2MEM_PSRAM_CH and \
VDMA_MEM2MEM_SRAM_CH to different values"
                );
            }

            match (psram_channel, sram_channel) {
                (0, 1) => {
                    step(7, "before first DMA case (SRAM->PSRAM)");
                    let psram_outcome = run_case(
                        "SRAM->PSRAM (VDMA_CH0)",
                        0,
                        src_internal,
                        dst_psram_cache.expect("PSRAM destination missing in both mode"),
                        psram_dst_offset,
                        dst_awlen_auto,
                    );
                    step(8, "after first DMA case (SRAM->PSRAM)");
                    overall &= psram_outcome.pass;

                    step(9, "before second DMA case (SRAM->SRAM)");
                    if psram_outcome.buffers_safe_for_cpu {
                        let sram_outcome = run_case(
                            "SRAM->SRAM control (VDMA_CH1)",
                            1,
                            src_internal,
                            dst_internal,
                            0,
                            dst_awlen_auto,
                        );
                        overall &= sram_outcome.pass;
                    } else {
                        println!(
                            "SRAM control skipped: previous DMA case did not confirm safe buffer ownership"
                        );
                        overall = false;
                    }
                    step(28, "after second DMA case (SRAM->SRAM)");
                }
                (1, 0) => {
                    step(7, "before first DMA case (SRAM->PSRAM)");
                    let psram_outcome = run_case(
                        "SRAM->PSRAM (VDMA_CH1)",
                        1,
                        src_internal,
                        dst_psram_cache.expect("PSRAM destination missing in both mode"),
                        psram_dst_offset,
                        dst_awlen_auto,
                    );
                    step(8, "after first DMA case (SRAM->PSRAM)");
                    overall &= psram_outcome.pass;

                    step(9, "before second DMA case (SRAM->SRAM)");
                    if psram_outcome.buffers_safe_for_cpu {
                        let sram_outcome = run_case(
                            "SRAM->SRAM control (VDMA_CH0)",
                            0,
                            src_internal,
                            dst_internal,
                            0,
                            dst_awlen_auto,
                        );
                        overall &= sram_outcome.pass;
                    } else {
                        println!(
                            "SRAM control skipped: previous DMA case did not confirm safe buffer ownership"
                        );
                        overall = false;
                    }
                    step(28, "after second DMA case (SRAM->SRAM)");
                }
                _ => unreachable!(),
            }
        }
    }

    let result_name = match case_selection {
        CaseSelection::CpuPsram => "CPU-PSRAM",
        _ => "DW-GDMA mem2mem overall",
    };
    println!(
        "{result_name} RESULT: {}",
        if overall { "PASS" } else { "FAIL" }
    );

    if !overall {
        panic!("{result_name} diagnostic failed");
    }

    loop {}
}
