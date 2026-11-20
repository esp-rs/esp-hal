//! ESP32-P4 PSRAM DQS timing sweep.
//!
//! Port of ESP-IDF v6.1 `mspi_timing_psram_tuning` for
//! `SOC_MEMSPI_TIMING_TUNING_BY_DQS` (`mspi_timing_by_dqs.c`,
//! `mspi_timing_tuning.c`). The phase list and delayline table are the sweep
//! space IDF uses for every tuned speed. The winner is whichever point the
//! PSRAM readback actually matches. Nothing here is a fixed 250 MHz result.
//!
//! IDF enables the sweep at 80, 200, and 250 MHz and leaves it empty at 20 MHz.
//! 125 MHz is not an IDF PSRAM speed, so this module does not invent a rule for it.

use super::{
    PsramTimingParams,
    oct_hex::{self, mspi3_transfer},
};
use crate::{
    pac::{
        generic::Reg,
        iomux_mspi_pin::{
            psram_d_pin0::PSRAM_D_PIN0_SPEC,
            psram_dqs_0_pin0::PSRAM_DQS_0_PIN0_SPEC,
        },
    },
    peripherals::{HP_SYS_CLKRST, IOMUX_MSPI_PIN, MEMSPI2, MEMSPI3},
};

const LOW_SPEED_MHZ: u32 = 20;
const TEST_ADDR: u32 = 0x80;
const CHUNK_BYTES: usize = 64;
const PHASE_TRIALS: u32 = 1;
const DELAYLINE_TRIALS: u32 = 100;
const SYNC_READ: u16 = 0x0000;
const SYNC_WRITE: u16 = 0x8080;

/// IDF `MSPI_LL_DQS_PHASE_*`, in sweep order. Register values are 0..3.
const PHASES: [u8; 4] = [0, 1, 2, 3];

/// `(data_delayline, dqs_delayline)` from IDF `s_test_delayline_config`.
/// The C struct field order is data, then DQS.
const DELAYLINES: [(u8, u8); 31] = [
    (0, 15),
    (0, 14),
    (0, 13),
    (0, 12),
    (0, 11),
    (0, 10),
    (0, 9),
    (0, 8),
    (0, 7),
    (0, 6),
    (0, 5),
    (0, 4),
    (0, 3),
    (0, 2),
    (0, 1),
    (0, 0),
    (1, 0),
    (2, 0),
    (3, 0),
    (4, 0),
    (5, 0),
    (6, 0),
    (7, 0),
    (8, 0),
    (9, 0),
    (10, 0),
    (11, 0),
    (12, 0),
    (13, 0),
    (14, 0),
    (15, 0),
];

/// First 128 bytes of IDF `s_test_data` (32 little-endian words).
const TEST_WORDS: [u32; 32] = [
    0x7f78_6655,
    0xa5ff_005a,
    0x3f3c_33aa,
    0xa5ff_5a00,
    0x1f1e_9955,
    0xa500_5aff,
    0x0f0f_ccaa,
    0xa55a_00ff,
    0x0787_6655,
    0xffa5_5a00,
    0x03c3_33aa,
    0xff00_a55a,
    0x01e1_9955,
    0xff00_5aa5,
    0x00f0_ccaa,
    0xff5a_00a5,
    0x8078_6655,
    0x00a5_ff5a,
    0xc03c_33aa,
    0x00a5_5aff,
    0xe01e_9355,
    0x00ff_5aa5,
    0xf00f_ccaa,
    0x005a_ffa5,
    0xf887_6655,
    0x5aa5_ff00,
    0xfcc3_33aa,
    0x5aff_a500,
    0xfee1_9955,
    0x5a00_a5ff,
    0x11f0_ccaa,
    0x5a00_ffa5,
];

/// Run the DQS sweep when IDF would, using `timing` for the clock and dummies.
///
/// Returns `false` only when a clock change cannot be restored. A sweep that
/// finds no passing window still returns `true`: IDF keeps going and falls
/// back to table index 0.
pub(super) fn tune_psram(timing: &PsramTimingParams) -> bool {
    if !idf_tunes(timing.clock) {
        info!(
            "PSRAM timing tuning skipped at {} MHz (IDF sweeps 80, 200, and 250 MHz)",
            timing.clock
        );
        return true;
    }

    info!("PSRAM timing tuning at {} MHz", timing.clock);

    // Reference pattern is written at 20 MHz with the delays cleared, then
    // every candidate is read back at the target frequency.
    if !apply_bus_clock(LOW_SPEED_MHZ) {
        error!(
            "PSRAM timing tuning skipped: {} MHz MPLL cannot divide to {} MHz",
            crate::clock::ll::PsramInstance::Psram.function_clock_frequency() / 1_000_000,
            LOW_SPEED_MHZ
        );
        return true;
    }
    clear_timing_regs();

    let reference = reference_bytes();
    if !transfer_pattern(timing, SYNC_WRITE, &reference, &mut []) {
        error!(
            "PSRAM timing tuning reference write failed; restoring {} MHz",
            timing.clock
        );
        clear_timing_regs();
        return apply_bus_clock(timing.clock);
    }

    set_mspi3_variable_dummy(false);
    if !apply_bus_clock(timing.clock) {
        return false;
    }

    let phase_index = sweep_phases(timing, &reference);
    let (data_delay, dqs_delay) = sweep_delaylines(timing, &reference, PHASES[phase_index]);
    apply_timing(PHASES[phase_index], data_delay, dqs_delay);

    if !apply_bus_clock(timing.clock) {
        return false;
    }
    // IDF re-enables variable dummy on both controllers after the sweep.
    // The sweep itself clears only the MSPI3 bit.
    MEMSPI2::regs()
        .smem_ddr()
        .modify(|_, w| w.var_dummy().set_bit());
    set_mspi3_variable_dummy(true);

    info!(
        "PSRAM timing tuning done: phase={} data_delay={} dqs_delay={}",
        PHASES[phase_index], data_delay, dqs_delay
    );
    log_applied_readback("after-tune");
    true
}

/// Read back the pin registers the sweep writes. `delay270` uses IDF bits
/// [20:17]; the PAC accessor for that field is one bit high.
pub(super) fn log_applied_readback(tag: &str) {
    let iomux = IOMUX_MSPI_PIN::regs();
    let dqs0 = iomux.psram_dqs_0_pin0().read();
    let dqs1 = iomux.psram_dqs_1_pin0().read();
    let delay270 = |bits: u32| (bits >> 17) & 0xF;
    info!(
        "PSRAM timing readback {}: dqs0=0x{:08x} phase={} delay90={} delay270={} dqs1=0x{:08x} phase={} delay90={} delay270={} dq0_dlc={} dq15_dlc={} ck_dlc={} cs_dlc={}",
        tag,
        dqs0.bits(),
        dqs0.phase().bits(),
        dqs0.delay_90().bits(),
        delay270(dqs0.bits()),
        dqs1.bits(),
        dqs1.phase().bits(),
        dqs1.delay_90().bits(),
        delay270(dqs1.bits()),
        iomux.psram_d_pin0().read().dlc().bits(),
        iomux.psram_dq15_pin0().read().dlc().bits(),
        iomux.psram_ck_pin0().read().dlc().bits(),
        iomux.psram_cs_pin0().read().dlc().bits(),
    );
}

/// IDF `MSPI_TIMING_PSRAM_NEEDS_TUNING`: on at 80/200/250, off at 20.
/// 125 MHz is a HAL preset with no IDF sweep rule.
fn idf_tunes(clock_mhz: u32) -> bool {
    matches!(clock_mhz, 80 | 200 | 250)
}

fn reference_bytes() -> [u8; 128] {
    let mut bytes = [0u8; 128];
    for (index, word) in TEST_WORDS.iter().enumerate() {
        bytes[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
    }
    bytes
}

fn apply_bus_clock(mhz: u32) -> bool {
    // IDF `mspi_timing_config_set_psram_clock` forces the core divider to 1
    // and then divides the MPLL down to `mhz` on both PSRAM controllers.
    HP_SYS_CLKRST::regs()
        .peri_clk_ctrl00()
        .modify(|_, w| unsafe {
            w.psram_core_clk_en().set_bit();
            w.psram_core_clk_div_num().bits(0)
        });
    oct_hex::set_bus_clock(mhz)
}

fn set_mspi3_variable_dummy(enable: bool) {
    MEMSPI3::regs()
        .ddr()
        .modify(|_, w| w.var_dummy().bit(enable));
}

fn transfer_pattern(timing: &PsramTimingParams, cmd: u16, tx: &[u8], rx: &mut [u8]) -> bool {
    let dummy_bits = if cmd == SYNC_WRITE {
        u32::from(timing.wr_dummy_bits) + 1
    } else {
        u32::from(timing.rd_dummy_bits) + 1
    };
    let len = if cmd == SYNC_WRITE {
        tx.len()
    } else {
        rx.len()
    };
    let mut offset = 0;
    while offset < len {
        let end = (offset + CHUNK_BYTES).min(len);
        let addr = TEST_ADDR + offset as u32;
        let result = if cmd == SYNC_WRITE {
            mspi3_transfer(cmd, addr, dummy_bits, &tx[offset..end], &mut [])
        } else {
            mspi3_transfer(cmd, addr, dummy_bits, &[], &mut rx[offset..end])
        };
        if result.is_err() {
            return false;
        }
        offset = end;
    }
    true
}

fn pattern_matches(timing: &PsramTimingParams, reference: &[u8]) -> bool {
    let mut sample = [0u8; 128];
    transfer_pattern(timing, SYNC_READ, &[], &mut sample) && sample == reference
}

/// Longest run of fully successful points. Returns `(length, end_index)`.
fn longest_success_run(good: &[bool]) -> (u32, u32) {
    let mut best_len = 0u32;
    let mut best_end = 0u32;
    let mut run = 0u32;
    for (index, point) in good.iter().enumerate() {
        if *point {
            run += 1;
            if run > best_len {
                best_len = run;
                best_end = index as u32;
            }
        } else {
            run = 0;
        }
    }
    (best_len, best_end)
}

fn sweep_phases(timing: &PsramTimingParams, reference: &[u8]) -> usize {
    let mut good = [false; PHASES.len()];
    for (index, phase) in PHASES.iter().copied().enumerate() {
        set_dqs_phase(phase);
        good[index] = trials_pass(timing, reference, PHASE_TRIALS);
    }
    let (length, end) = longest_success_run(&good);
    // IDF `mspi_timing_psram_select_best_tuning_phase` keeps the first point
    // of the longest run. An empty run falls back to index 0.
    let best = if length == 0 {
        warn!("PSRAM DQS phase sweep found no passing window; using phase index 0");
        0
    } else {
        (end - length + 1) as usize
    };
    info!(
        "PSRAM DQS phase sweep: window_len={} end={} chosen_index={} phase={}",
        length, end, best, PHASES[best]
    );
    best
}

fn sweep_delaylines(timing: &PsramTimingParams, reference: &[u8], phase: u8) -> (u8, u8) {
    let mut good = [false; DELAYLINES.len()];
    for (index, &(data_delay, dqs_delay)) in DELAYLINES.iter().enumerate() {
        apply_timing(phase, data_delay, dqs_delay);
        good[index] = trials_pass(timing, reference, DELAYLINE_TRIALS);
    }
    let (length, end) = longest_success_run(&good);
    // IDF `mspi_timing_psram_select_best_tuning_delayline` uses the middle of
    // the run. A run of 0 or 1 point falls back to table index 0.
    let best = if length <= 1 {
        warn!("PSRAM delayline sweep found no stable window; using delayline index 0");
        0
    } else {
        (end - length / 2) as usize
    };
    let (data_delay, dqs_delay) = DELAYLINES[best];
    info!(
        "PSRAM delayline sweep: window_len={} end={} chosen_index={} data_delay={} dqs_delay={}",
        length, end, best, data_delay, dqs_delay
    );
    (data_delay, dqs_delay)
}

fn trials_pass(timing: &PsramTimingParams, reference: &[u8], trials: u32) -> bool {
    for _ in 0..trials {
        if !pattern_matches(timing, reference) {
            return false;
        }
    }
    true
}

fn apply_timing(phase: u8, data_delay: u8, dqs_delay: u8) {
    set_dqs_phase(phase);
    set_dqs_delay(dqs_delay);
    set_data_delay(data_delay);
}

fn clear_timing_regs() {
    apply_timing(0, 0, 0);
}

fn set_dqs_phase(phase: u8) {
    let iomux = IOMUX_MSPI_PIN::regs();
    iomux
        .psram_dqs_0_pin0()
        .modify(|_, w| unsafe { w.phase().bits(phase) });
    iomux
        .psram_dqs_1_pin0()
        .modify(|_, w| unsafe { w.phase().bits(phase) });
}

fn set_dqs_delay(delay: u8) {
    let iomux = IOMUX_MSPI_PIN::regs();
    write_dqs_delay(iomux.psram_dqs_0_pin0(), delay);
    write_dqs_delay(iomux.psram_dqs_1_pin0(), delay);
}

/// `DELAY_90` is bits [10:7]. `DELAY_270` is bits [20:17] in the v6.1 header.
/// The PAC `delay_270()` accessor is one bit high, so the 270 field is written
/// from the raw register.
fn write_dqs_delay(reg: &Reg<PSRAM_DQS_0_PIN0_SPEC>, delay: u8) {
    const DELAY_90_SHIFT: u32 = 7;
    const DELAY_270_SHIFT: u32 = 17;
    const DELAY_MASK: u32 = 0xF;
    reg.modify(|r, w| unsafe {
        let mut bits = r.bits();
        bits &= !(DELAY_MASK << DELAY_90_SHIFT);
        bits |= (u32::from(delay) & DELAY_MASK) << DELAY_90_SHIFT;
        bits &= !(DELAY_MASK << DELAY_270_SHIFT);
        bits |= (u32::from(delay) & DELAY_MASK) << DELAY_270_SHIFT;
        w.bits(bits)
    });
}

fn set_data_delay(delay: u8) {
    let iomux = IOMUX_MSPI_PIN::regs();
    let pins: [&Reg<PSRAM_D_PIN0_SPEC>; 18] = [
        iomux.psram_d_pin0(),
        iomux.psram_q_pin0(),
        iomux.psram_wp_pin0(),
        iomux.psram_hold_pin0(),
        iomux.psram_dq4_pin0(),
        iomux.psram_dq5_pin0(),
        iomux.psram_dq6_pin0(),
        iomux.psram_dq7_pin0(),
        iomux.psram_ck_pin0(),
        iomux.psram_cs_pin0(),
        iomux.psram_dq8_pin0(),
        iomux.psram_dq9_pin0(),
        iomux.psram_dq10_pin0(),
        iomux.psram_dq11_pin0(),
        iomux.psram_dq12_pin0(),
        iomux.psram_dq13_pin0(),
        iomux.psram_dq14_pin0(),
        iomux.psram_dq15_pin0(),
    ];
    for pin in pins {
        pin.modify(|_, w| unsafe { w.dlc().bits(delay) });
    }
}
