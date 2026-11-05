//! ESP32-P4 DW-GDMA mem2mem diagnostic (SRAM->PSRAM and SRAM->SRAM).
//!
//! This is a standalone QA binary to validate whether the DW-GDMA (VDMA)
//! destination path to PSRAM works independently of MIPI-CSI.
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
    mipi_dsi::{__qa_dw_gdma_mem2mem_transfer, DwGdmaMem2MemReport},
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
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CaseOutcome {
    pass: bool,
    buffers_safe_for_cpu: bool,
}

fn step(step: u32, msg: &str) {
    println!("STEP {step}: {msg}");
}

fn selected_case() -> CaseSelection {
    match option_env!("VDMA_MEM2MEM_CASE") {
        Some(mode) if mode.eq_ignore_ascii_case("psram") => CaseSelection::PsramOnly,
        Some(mode) if mode.eq_ignore_ascii_case("sram") => CaseSelection::SramOnly,
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
        0,
        "VDMA_MEM2MEM_PSRAM_CH",
    )
}

fn selected_sram_channel() -> u8 {
    parse_channel_env(
        option_env!("VDMA_MEM2MEM_SRAM_CH"),
        1,
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

fn invalidate_after_dma(label: &str, buf: &[u8]) {
    step(22, "before destination cache invalidate after DMA");
    DmaAlignedRef::new(buf)
        .expect("destination buffer is not DMA/cacheline aligned")
        .invalidate();
    step(23, "after destination cache invalidate after DMA");
    println!("cache-ready-after-dma: {label}");
}

fn print_report(label: &str, report: &DwGdmaMem2MemReport, verify: Option<&VerificationResult>) {
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
        "regs-decode: dms={} dst_width={} dst_msize={} awlen_en={} awlen={}",
        (report.ctl0_bits >> 2) & 0x1,
        (report.ctl0_bits >> 11) & 0x7,
        (report.ctl0_bits >> 18) & 0xf,
        (report.ctl1_bits >> 15) & 0x1,
        (report.ctl1_bits >> 16) & 0xff
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

fn run_case<CH>(
    label: &str,
    channel: CH,
    src: &mut [u8],
    dst_cache_region: &mut [u8],
    dst_transfer_offset: usize,
    dst_awlen_auto: bool,
) -> CaseOutcome
where
    CH: esp_hal::mipi_dsi::VdmaDmaChannel + 'static,
{
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

    step(20, "before helper call (__qa_dw_gdma_mem2mem_transfer)");
    let dst_addr = {
        let dst_with_guard =
            &mut dst_cache_region[dst_transfer_offset..dst_transfer_offset + DEST_TOTAL_BYTES];
        dst_with_guard.as_mut_ptr() as u32
    };
    let report = __qa_dw_gdma_mem2mem_transfer(
        channel,
        src.as_ptr() as u32,
        dst_addr,
        TRANSFER_UNITS_64BIT,
        DMA_TIMEOUT_MS,
        dst_awlen_auto,
    );
    step(21, "after helper call (__qa_dw_gdma_mem2mem_transfer)");

    if !report.buffers_safe_for_cpu {
        println!(
            "ownership-guard: DMA stop outcome {:?} leaves channel ownership uncertain; skipping invalidate/verify/reuse",
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
    let peripherals = esp_hal::init(Config::default());
    esp_println::logger::init_logger_from_env();
    if case_selection != CaseSelection::SramOnly {
        esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    }

    println!("DW-GDMA mem2mem diagnostic (ESP32-P4)");
    println!(
        "transfer={} bytes guard={} bytes timeout={} ms",
        TRANSFER_BYTES, GUARD_BYTES, DMA_TIMEOUT_MS
    );
    println!(
        "selection: {:?} psram_channel={} sram_channel={} dst_awlen_auto={} psram_dst_offset={}",
        case_selection, psram_channel, sram_channel, dst_awlen_auto, psram_dst_offset
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
        CaseSelection::PsramOnly => {
            step(7, "before first DMA case (SRAM->PSRAM)");
            let outcome = match psram_channel {
                0 => run_case(
                    "SRAM->PSRAM (VDMA_CH0)",
                    peripherals.VDMA_CH0,
                    src_internal,
                    dst_psram_cache.expect("PSRAM destination missing in PSRAM mode"),
                    psram_dst_offset,
                    dst_awlen_auto,
                ),
                1 => run_case(
                    "SRAM->PSRAM (VDMA_CH1)",
                    peripherals.VDMA_CH1,
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
                    peripherals.VDMA_CH0,
                    src_internal,
                    dst_internal,
                    0,
                    dst_awlen_auto,
                ),
                1 => run_case(
                    "SRAM->SRAM control (VDMA_CH1)",
                    peripherals.VDMA_CH1,
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
                        peripherals.VDMA_CH0,
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
                            peripherals.VDMA_CH1,
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
                        peripherals.VDMA_CH1,
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
                            peripherals.VDMA_CH0,
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

    println!(
        "DW-GDMA mem2mem overall RESULT: {}",
        if overall { "PASS" } else { "FAIL" }
    );

    if !overall {
        panic!("DW-GDMA mem2mem diagnostic failed");
    }

    loop {}
}
