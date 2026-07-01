//! # Register DMA (regDMA) based register retention
//!
//! ## Overview
//!
//! ESP32-C6 contains a **Power Assist Unit (PAU)** with a **regDMA** engine that
//! can automatically back up and restore peripheral/CPU register state to and
//! from RAM. ESP-IDF uses this engine to retain register contents while a power
//! domain (e.g. the CPU or the digital `TOP` domain) is powered down during
//! light sleep, so that execution can resume seamlessly after wakeup.
//!
//! regDMA walks a linked list of *nodes* stored in RAM. Each node describes one
//! backup/restore operation ([`RegdmaLink`]): CONTINUOUS (a run of registers via
//! a RAM buffer), ADDR_MAP (a run of registers where a bitmap selects which ones
//! to transfer), WRITE (a masked register write) or WAIT (poll a register).
//!
//! The list is executed by the **PMU auto-trigger** over PAU entry link 0: when
//! the digital `TOP` domain is powered down during light sleep the PMU runs the
//! list to back up the registers on the way into sleep and restore them on
//! wakeup, with no CPU involvement. The `sys_periph` module builds the
//! TOP-domain register set to retain, and [`enable_top_retention`] arms it.
//!
//! References (ESP-IDF `v5.4`):
//! - `components/soc/include/soc/regdma.h` (node layout)
//! - `components/hal/esp32c6/include/hal/pau_ll.h`
//! - `components/hal/esp32c6/pau_hal.c`
//! - `components/esp_hw_support/port/pau_regdma.c`

use core::sync::atomic::{Ordering, fence};

use crate::peripherals::{PAU, PCR, PMU};

// Bit layout of `regdma_link_head_t` (see ESP-IDF `regdma.h`):
// https://github.com/espressif/esp-idf/blob/v5.4/components/soc/include/soc/regdma.h#L114-L123
const HEAD_LENGTH_MASK: u32 = 0x3ff; // bits 0..=9: register count (words)
const HEAD_MODE_SHIFT: u32 = 16; // bits 16..=19: link mode
const HEAD_SKIP_R_BIT: u32 = 1 << 29; // skip this node on restore
const HEAD_SKIP_B_BIT: u32 = 1 << 30; // skip this node on backup
const HEAD_EOF_BIT: u32 = 1 << 31; // end of link

/// regDMA link node mode (`regdma_link_mode_t`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u32)]
enum LinkMode {
    /// Back up/restore a run of consecutive registers via a RAM buffer.
    Continuous = 0,
    /// Back up/restore a run of registers via a RAM buffer, where a 4-word
    /// bitmap selects which registers in the window are actually transferred
    /// (skipping e.g. read-only status/FIFO registers interspersed in a block).
    AddrMap = 1,
    /// Unconditionally write a masked value to a register.
    Write = 2,
    /// Poll a register until `(reg & mask) == value`.
    Wait = 3,
}

/// A single regDMA linked-list node.
///
/// The in-memory layout must match what the PAU hardware expects: the hardware
/// link address points at the `head` field, followed by the four body words.
/// The software-only `stat` block that ESP-IDF keeps *before* `head` is not
/// needed here, so it is omitted.
///
/// The CONTINUOUS and WRITE/WAIT node bodies are both four words; the ADDR_MAP
/// body adds a four-word register-selection bitmap. A single struct with a
/// trailing `map` array covers all four modes (the hardware reads only as many
/// body words as the mode requires and then follows `next`, so the unused
/// trailing words are harmless padding for the other modes). Branch nodes are
/// not implemented.
///
/// - CONTINUOUS: `w0 = backup addr`, `w1 = restore addr`, `w2 = RAM buffer`.
/// - ADDR_MAP: as CONTINUOUS, plus `map` selecting which registers to transfer.
/// - WRITE/WAIT: `w0 = target addr`, `w1 = value`, `w2 = mask` (`mem`/`map`
///   unused).
#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub(crate) struct RegdmaLink {
    /// Packed `regdma_link_head_t`.
    head: u32,
    /// Pointer to the next node's `head`, or `0` for the end of the list.
    next: u32,
    w0: u32,
    w1: u32,
    w2: u32,
    /// ADDR_MAP register-selection bitmap; zero (and unread) for other modes.
    map: [u32; 4],
}

impl RegdmaLink {
    const EMPTY: Self = Self {
        head: 0,
        next: 0,
        w0: 0,
        w1: 0,
        w2: 0,
        map: [0; 4],
    };

    fn head(mode: LinkMode, len: u32, skip_b: bool, skip_r: bool) -> u32 {
        let mut head = (len & HEAD_LENGTH_MASK) | ((mode as u32) << HEAD_MODE_SHIFT) | HEAD_EOF_BIT;
        if skip_b {
            head |= HEAD_SKIP_B_BIT;
        }
        if skip_r {
            head |= HEAD_SKIP_R_BIT;
        }
        head
    }

    /// A CONTINUOUS node backing up/restoring `len` words at `reg` via `storage`.
    fn continuous(reg: u32, storage: u32, len: u32) -> Self {
        Self::continuous_split(reg, reg, storage, len)
    }

    /// A CONTINUOUS node whose backup source (`backup`) and restore destination
    /// (`restore`) registers differ, sharing one RAM buffer. Used where hardware
    /// exposes a value register for readback and a separate load register for
    /// restore (e.g. the SysTimer counter).
    fn continuous_split(backup: u32, restore: u32, storage: u32, len: u32) -> Self {
        Self {
            head: Self::head(LinkMode::Continuous, len, false, false),
            next: 0,
            w0: backup,
            w1: restore,
            w2: storage,
            map: [0; 4],
        }
    }

    /// An ADDR_MAP node backing up/restoring the `count` registers selected by
    /// `map` from the register window starting at `reg`, via `storage`.
    ///
    /// `map` is a bitmap over the window (bit `i` = the register at
    /// `reg + i * 4`); the engine transfers `count` registers total (one per
    /// set bit, walking bits from LSB up) into `count` consecutive words of
    /// `storage`. Used to skip read-only/FIFO registers interspersed in a
    /// peripheral's register block (e.g. the console UART).
    fn addr_map(reg: u32, storage: u32, count: u32, map: [u32; 4]) -> Self {
        Self {
            head: Self::head(LinkMode::AddrMap, count, false, false),
            next: 0,
            w0: reg,
            w1: reg,
            w2: storage,
            map,
        }
    }

    /// A WRITE node that writes `value` (under `mask`) to `target`.
    ///
    /// `skip_b`/`skip_r` select whether the write happens during backup and/or
    /// restore (WRITE/WAIT nodes are usually restore-only or backup-only).
    fn write(target: u32, value: u32, mask: u32, skip_b: bool, skip_r: bool) -> Self {
        Self {
            head: Self::head(LinkMode::Write, 0, skip_b, skip_r),
            next: 0,
            w0: target,
            w1: value,
            w2: mask,
            map: [0; 4],
        }
    }

    /// A WAIT node that polls `target` until `(reg & mask) == value`.
    fn wait(target: u32, value: u32, mask: u32, skip_b: bool, skip_r: bool) -> Self {
        Self {
            head: Self::head(LinkMode::Wait, 0, skip_b, skip_r),
            next: 0,
            w0: target,
            w1: value,
            w2: mask,
            map: [0; 4],
        }
    }

    fn addr(&self) -> u32 {
        core::ptr::addr_of!(self.head) as u32
    }
}

/// Chain a slice of nodes into a single linked list: each node's `next` points
/// at the following node's `head`, and only the last node keeps its EOF flag.
/// Returns the head address to program into the PAU.
fn link_nodes(nodes: &mut [RegdmaLink]) -> u32 {
    let len = nodes.len();
    for i in 0..len {
        if i + 1 < len {
            nodes[i].next = nodes[i + 1].addr();
            nodes[i].head &= !HEAD_EOF_BIT;
        } else {
            nodes[i].next = 0;
            nodes[i].head |= HEAD_EOF_BIT;
        }
    }
    nodes[0].addr()
}

/// Arm PMU-driven regDMA retention of the TOP-domain system peripherals for the
/// upcoming light sleep.
///
/// When the PMU powers down the digital `TOP` domain (`pd_top`) it loses the
/// system-peripheral register state, so it must be backed up on the
/// HP_ACTIVE -> HP_SLEEP transition and restored on HP_SLEEP -> HP_ACTIVE. Both
/// transitions run PAU entry link 0 (the direction is chosen by the PMU), so a
/// single combined list (see the `sys_periph` module) serves both.
///
/// This programs the entry-link address and enables the two backup phases. The
/// backup *mode*/direction and clocks are already configured per HP state by the
/// sleep power config; only the enable bits (reset every sleep) are flipped
/// here. Mirrors ESP-IDF `sleep_retention` link setup +
/// `pmu_sleep_enable_regdma_backup()` (active/sleep phases only, as there is no
/// modem state).
///
/// Must be called after the PMU power config has been applied (which rewrites
/// the backup registers) and before the sleep request.
pub(crate) fn enable_top_retention() {
    // pau_ll_enable_bus_clock(true): enable the regDMA bus clock and release its
    // reset before programming the entry link.
    PCR::regs().regdma_conf().modify(|_, w| {
        w.regdma_clk_en().set_bit();
        w.regdma_rst_en().clear_bit()
    });
    // pau_hal_set_regdma_wait_timeout: bound how long a WAIT node polls a
    // register so a never-satisfied condition can't hang the engine. Values
    // match ESP-IDF's PAU_REGDMA_LINK_WAIT_{RETRY_COUNT,READ_INTERNAL}.
    PAU::regs().regdma_bkp_conf().modify(|_, w| unsafe {
        w.link_tout_thres().bits(1000);
        w.read_interval().bits(32)
    });

    // Build the combined SYS_PERIPH list and program it as PAU entry link 0.
    let head = link_nodes(sys_periph::build_link());
    fence(Ordering::SeqCst);
    PAU::regs()
        .regdma_link_0_addr()
        .write(|w| unsafe { w.bits(head) });

    // pmu_sleep_enable_regdma_backup (active <-> sleep only): back up on
    // active->sleep, restore on sleep->active.
    let pmu = PMU::regs();
    pmu.hp_sleep_backup()
        .modify(|_, w| w.hp_active2sleep_backup_en().set_bit());
    pmu.hp_active_backup()
        .modify(|_, w| w.hp_sleep2active_backup_en().set_bit());
}

/// ESP32-C6 TOP-domain system-peripheral retention link.
///
/// When the digital `TOP` power domain is powered down during light sleep, the
/// registers of the core system peripherals are lost and must be regDMA-backed
/// up beforehand and restored on wakeup. This module builds the linked list
/// describing those register regions.
///
/// The set and ordering mirror ESP-IDF's `SLEEP_RETENTION_MODULE_SYS_PERIPH`
/// plus `SLEEP_RETENTION_MODULE_CLOCK_SYSTEM` (both TOP-domain), sorted by the
/// same retention priority (system clock first). Within a link, ESP-IDF keeps
/// the same nodes for backup (entry 0) and restore (entry 2), so a single
/// combined list can be programmed into both PAU entry links.
///
/// References (ESP-IDF `v5.4`):
/// - [`system_retention_periph.c`](https://github.com/espressif/esp-idf/blob/v5.4/components/soc/esp32c6/system_retention_periph.c)
/// - [`sleep_clock.c`](https://github.com/espressif/esp-idf/blob/v5.4/components/esp_hw_support/lowpower/port/esp32c6/sleep_clock.c)
/// - [`sleep_system_peripheral.c`](https://github.com/espressif/esp-idf/blob/v5.4/components/esp_hw_support/sleep_system_peripheral.c)
mod sys_periph {
    use super::{HEAD_LENGTH_MASK, RegdmaLink};

    /// TEE mode-control register, rewritten early on restore to unlock access.
    const TEE_M4_MODE_CTRL_REG: u32 = 0x6009_8010;

    /// A run of `count` consecutive 32-bit registers starting at `base`.
    struct ContRegion {
        base: u32,
        count: u32,
    }

    /// Continuous register regions to retain, in ESP-IDF retention-priority
    /// order (highest priority first). The end registers used to size each
    /// region are noted; counts are `((end - base) / 4) + 1`.
    const CONT_REGIONS: &[ContRegion] = &[
        // PRI_0 - system clock/reset (PCR)
        ContRegion { base: 0x6009_6000, count: 79 }, // PCR base ..= PCR_SRAM_POWER_CONF_REG (+0x138)
        ContRegion { base: 0x6009_6FF0, count: 1 },  // PCR_RESET_EVENT_BYPASS_REG
        // PRI_4 - TEE/APM
        ContRegion { base: 0x6009_9000, count: 68 }, // HP_APM base ..= HP_APM_CLOCK_GATE_REG (+0x10c)
        ContRegion { base: 0x6009_8000, count: 33 }, // TEE base ..= TEE_CLOCK_GATE_REG (+0x80)
        // PRI_5 - interrupt matrix + HP system
        ContRegion { base: 0x6001_0000, count: 81 }, // INTMTX base ..= INTMTX_CORE0_CLOCK_GATE_REG (+0x140)
        ContRegion { base: 0x6009_5000, count: 18 }, // HP_SYSTEM base ..= HP_SYSTEM_MEM_TEST_CONF_REG (+0x44)
        // PRI_6 - IO MUX + GPIO matrix
        ContRegion { base: 0x6009_0000, count: 32 },  // IO_MUX base ..= IO_MUX_GPIO30_REG (+0x7c)
        ContRegion { base: 0x6009_1554, count: 35 },  // GPIO_FUNC0_OUT_SEL ..= GPIO_FUNC34_OUT_SEL
        ContRegion { base: 0x6009_114C, count: 127 }, // GPIO_STATUS_NEXT ..= GPIO_FUNC124_IN_SEL
        ContRegion { base: 0x6009_1000, count: 64 },  // GPIO base ..= GPIO_PIN34_REG (+0xfc)
        // PRI_6 - Flash SPI mem (SPIMEM1 then SPIMEM0). MMU content/index
        // registers are intentionally excluded (see ESP-IDF note).
        ContRegion { base: 0x6000_3000, count: 55 }, // SPIMEM1 base ..= SPI_MEM_SPI_SMEM_DDR (+0xd8)
        ContRegion { base: 0x6000_3100, count: 41 }, // SPIMEM1 FMEM_PMS0_ATTR ..= SMEM_AC (+0x1a0)
        ContRegion { base: 0x6000_3200, count: 1 },  // SPIMEM1 CLOCK_GATE
        ContRegion { base: 0x6000_3384, count: 31 }, // SPIMEM1 MMU_POWER_CTRL ..= DATE (+0x3fc)
        ContRegion { base: 0x6000_2000, count: 55 }, // SPIMEM0 base ..= SPI_MEM_SPI_SMEM_DDR
        ContRegion { base: 0x6000_2100, count: 41 }, // SPIMEM0 FMEM_PMS0_ATTR ..= SMEM_AC
        ContRegion { base: 0x6000_2200, count: 1 },  // SPIMEM0 CLOCK_GATE
        ContRegion { base: 0x6000_2384, count: 31 }, // SPIMEM0 MMU_POWER_CTRL ..= DATE
    ];

    /// Index in [`CONT_REGIONS`] at which the TEE/APM (PRI_4) group starts; the
    /// PRI_2 TEE-critical WRITE node is inserted just before it.
    const TEE_APM_START: usize = 2;

    /// Index in [`CONT_REGIONS`] at which the IO MUX / GPIO (PRI_6) group
    /// starts. The console-UART (PRI_5) nodes are inserted just before it, so
    /// the continuous regions split into a PRI_4/5 prefix (TEE/APM, interrupt
    /// matrix, HP system) and a PRI_6 suffix (IO MUX, GPIO, SPI mem).
    const IOMUX_START: usize = 6;

    const fn total_words() -> usize {
        let mut words = 0;
        let mut i = 0;
        while i < CONT_REGIONS.len() {
            words += CONT_REGIONS[i].count as usize;
            i += 1;
        }
        words
    }

    // Console UART0 (base 0x6000_0000). Retained via an ADDR_MAP node whose
    // bitmap selects the 21 configuration registers out of the 37-register
    // window between UART_INT_ENA_REG (+0x0c) and UART_ID_REG (+0x9c), skipping
    // the interspersed FIFO/status/interrupt-raw registers, followed by a
    // restore-only WRITE+WAIT that pulses UART_REG_UPDATE to load the shadow
    // (`_SYNC`) registers. Values from ESP-IDF v5.4 `uart_periph.c`
    // `UART_SLEEP_RETENTION_ENTRIES` and `uart_reg.h`.
    const UART_INT_ENA_REG: u32 = 0x6000_000C;
    const UART_REG_UPDATE_REG: u32 = 0x6000_0098;
    const UART_REG_UPDATE: u32 = 1 << 0;
    /// Number of registers actually retained (set bits in `UART_REGS_MAP`).
    const UART_RETENTION_REGS_CNT: u32 = 21;
    /// `uart_regs_map[4]` from ESP-IDF: bitmap over the INT_ENA..ID window.
    const UART_REGS_MAP: [u32; 4] = [0x007f_ff6d, 0x0000_0010, 0, 0];
    const UART_NODE_COUNT: usize = 3;

    // SysTimer (base 0x6000_A000). Register offsets and bitfield masks from
    // ESP-IDF v5.4 `systimer_reg.h`; node sequence from
    // `systimer_regs_retention[]`.
    const ST_BASE: u32 = 0x6000_A000;
    const ST_CONF: u32 = ST_BASE; // +0x00
    const ST_UNIT0_OP: u32 = ST_BASE + 0x04;
    const ST_UNIT1_OP: u32 = ST_BASE + 0x08;
    const ST_UNIT0_LOAD_HI: u32 = ST_BASE + 0x0C;
    const ST_UNIT1_LOAD_HI: u32 = ST_BASE + 0x14;
    const ST_TARGET0_HI: u32 = ST_BASE + 0x1C;
    const ST_TARGET0_CONF: u32 = ST_BASE + 0x34;
    const ST_TARGET1_CONF: u32 = ST_BASE + 0x38;
    const ST_TARGET2_CONF: u32 = ST_BASE + 0x3C;
    const ST_UNIT0_VALUE_HI: u32 = ST_BASE + 0x40;
    const ST_UNIT1_VALUE_HI: u32 = ST_BASE + 0x48;
    const ST_COMP0_LOAD: u32 = ST_BASE + 0x50;
    const ST_COMP1_LOAD: u32 = ST_BASE + 0x54;
    const ST_COMP2_LOAD: u32 = ST_BASE + 0x58;
    const ST_UNIT0_LOAD: u32 = ST_BASE + 0x5C;
    const ST_UNIT1_LOAD: u32 = ST_BASE + 0x60;
    const ST_INT_ENA: u32 = ST_BASE + 0x64;
    const ST_UNIT_UPDATE: u32 = 1 << 30;
    const ST_UNIT_VALUE_VALID: u32 = 1 << 29;
    const ST_UNIT_LOAD: u32 = 1 << 0;
    const ST_COMP_LOAD: u32 = 1 << 0;
    const ST_TARGET_PERIOD_MODE: u32 = 1 << 30;
    /// TARGET0_HI ..= TARGET2_CONF, i.e. all three targets' hi/lo/conf.
    const ST_TARGETS_LEN: u32 = 9;

    const SYSTIMER_NODE_COUNT: usize = 19;
    /// SysTimer CONTINUOUS-node words: unit0/1 value (2+2), targets (9),
    /// conf (1) and int_ena (1).
    const SYSTIMER_CONT_WORDS: usize = 2 + 2 + ST_TARGETS_LEN as usize + 1 + 1;

    /// One node per continuous region, the TEE-critical WRITE node, the console
    /// UART sequence, and the SysTimer sequence.
    const NODE_COUNT: usize = CONT_REGIONS.len() + 1 + UART_NODE_COUNT + SYSTIMER_NODE_COUNT;
    const BUF_WORDS: usize =
        total_words() + UART_RETENTION_REGS_CNT as usize + SYSTIMER_CONT_WORDS;

    static mut NODES: [RegdmaLink; NODE_COUNT] = [RegdmaLink::EMPTY; NODE_COUNT];
    static mut BUF: [u32; BUF_WORDS] = [0; BUF_WORDS];

    // Every region count must fit the 10-bit `length` field of a regDMA node.
    const _: () = {
        let mut i = 0;
        while i < CONT_REGIONS.len() {
            assert!(CONT_REGIONS[i].count <= HEAD_LENGTH_MASK);
            i += 1;
        }
    };

    /// (Re)build the SYS_PERIPH retention linked list into static storage and
    /// return the node slice ready to be chained/triggered.
    ///
    /// Rebuilding on each call keeps the nodes' `next`/buffer pointers
    /// self-consistent and is cheap (a few dozen writes).
    pub(super) fn build_link() -> &'static mut [RegdmaLink] {
        // SAFETY: retention is driven from a single context around sleep; there
        // is no concurrent access to these statics.
        let nodes = unsafe { &mut *core::ptr::addr_of_mut!(NODES) };
        let buf_base = core::ptr::addr_of_mut!(BUF) as *mut u32;

        let mut node = 0;
        let mut word = 0;

        // PRI_0: system clock (PCR).
        for region in &CONT_REGIONS[..TEE_APM_START] {
            let mem = unsafe { buf_base.add(word) } as u32;
            nodes[node] = RegdmaLink::continuous(region.base, mem, region.count);
            word += region.count as usize;
            node += 1;
        }

        // PRI_2: TEE-critical WRITE node (restore-only: skip on backup). Clears
        // TEE_M4_MODE_CTRL so the following TEE/APM restore can write freely.
        nodes[node] = RegdmaLink::write(TEE_M4_MODE_CTRL_REG, 0, 0xFFFF_FFFF, true, false);
        node += 1;

        // PRI_4/5: TEE/APM, interrupt matrix, HP system.
        for region in &CONT_REGIONS[TEE_APM_START..IOMUX_START] {
            let mem = unsafe { buf_base.add(word) } as u32;
            nodes[node] = RegdmaLink::continuous(region.base, mem, region.count);
            word += region.count as usize;
            node += 1;
        }

        // PRI_5: console UART0. ADDR_MAP restores the config registers, then a
        // restore-only WRITE+WAIT pulses UART_REG_UPDATE to latch the shadow
        // registers. The WRITE/WAIT skip the backup pass (they only matter on
        // restore), so the backup just reads the selected registers.
        let mem = unsafe { buf_base.add(word) } as u32;
        nodes[node] = RegdmaLink::addr_map(
            UART_INT_ENA_REG,
            mem,
            UART_RETENTION_REGS_CNT,
            UART_REGS_MAP,
        );
        word += UART_RETENTION_REGS_CNT as usize;
        node += 1;
        nodes[node] = RegdmaLink::write(
            UART_REG_UPDATE_REG,
            UART_REG_UPDATE,
            UART_REG_UPDATE,
            true,
            false,
        );
        node += 1;
        nodes[node] = RegdmaLink::wait(UART_REG_UPDATE_REG, 0, UART_REG_UPDATE, true, false);
        node += 1;

        // PRI_6: IO MUX, GPIO matrix, SPI mem.
        for region in &CONT_REGIONS[IOMUX_START..] {
            let mem = unsafe { buf_base.add(word) } as u32;
            nodes[node] = RegdmaLink::continuous(region.base, mem, region.count);
            word += region.count as usize;
            node += 1;
        }

        // PRI_6: SysTimer. Backup latches each unit's counter (UPDATE + wait for
        // VALUE_VALID) and reads the value; restore writes the value into the
        // LOAD registers and triggers a load. The counter value is read from the
        // VALUE_HI/LO registers but restored into the LOAD_HI/LO registers, so
        // those CONTINUOUS nodes use split backup/restore addresses.
        //
        // `alloc` reserves `len` words of buffer and returns their address; it
        // captures only the (copyable) buffer base, leaving `nodes` free to
        // index directly.
        let alloc = |len: u32, word: &mut usize| -> u32 {
            let mem = unsafe { buf_base.add(*word) } as u32;
            *word += len as usize;
            mem
        };

        // Unit 0: latch + read value, restore into load.
        nodes[node] = RegdmaLink::write(ST_UNIT0_OP, ST_UNIT_UPDATE, ST_UNIT_UPDATE, false, true);
        node += 1;
        nodes[node] = RegdmaLink::wait(
            ST_UNIT0_OP,
            ST_UNIT_VALUE_VALID,
            ST_UNIT_VALUE_VALID,
            false,
            true,
        );
        node += 1;
        let mem = alloc(2, &mut word);
        nodes[node] = RegdmaLink::continuous_split(ST_UNIT0_VALUE_HI, ST_UNIT0_LOAD_HI, mem, 2);
        node += 1;
        nodes[node] = RegdmaLink::write(ST_UNIT0_LOAD, ST_UNIT_LOAD, ST_UNIT_LOAD, true, false);
        node += 1;

        // Unit 1.
        nodes[node] = RegdmaLink::write(ST_UNIT1_OP, ST_UNIT_UPDATE, ST_UNIT_UPDATE, false, true);
        node += 1;
        nodes[node] = RegdmaLink::wait(
            ST_UNIT1_OP,
            ST_UNIT_VALUE_VALID,
            ST_UNIT_VALUE_VALID,
            false,
            true,
        );
        node += 1;
        let mem = alloc(2, &mut word);
        nodes[node] = RegdmaLink::continuous_split(ST_UNIT1_VALUE_HI, ST_UNIT1_LOAD_HI, mem, 2);
        node += 1;
        nodes[node] = RegdmaLink::write(ST_UNIT1_LOAD, ST_UNIT_LOAD, ST_UNIT_LOAD, true, false);
        node += 1;

        // Comparator target values & periods.
        let mem = alloc(ST_TARGETS_LEN, &mut word);
        nodes[node] = RegdmaLink::continuous(ST_TARGET0_HI, mem, ST_TARGETS_LEN);
        node += 1;
        for comp in [ST_COMP0_LOAD, ST_COMP1_LOAD, ST_COMP2_LOAD] {
            nodes[node] = RegdmaLink::write(comp, ST_COMP_LOAD, ST_COMP_LOAD, true, false);
            node += 1;
        }
        // Re-arm period mode: clear then set for target0/1, clear for target2
        // (matches ESP-IDF's write sequence).
        for target in [ST_TARGET0_CONF, ST_TARGET1_CONF] {
            nodes[node] = RegdmaLink::write(target, 0, ST_TARGET_PERIOD_MODE, true, false);
            node += 1;
            nodes[node] = RegdmaLink::write(
                target,
                ST_TARGET_PERIOD_MODE,
                ST_TARGET_PERIOD_MODE,
                true,
                false,
            );
            node += 1;
        }
        nodes[node] = RegdmaLink::write(ST_TARGET2_CONF, 0, ST_TARGET_PERIOD_MODE, true, false);
        node += 1;

        // Work-enable and interrupt-enable state.
        let mem = alloc(1, &mut word);
        nodes[node] = RegdmaLink::continuous(ST_CONF, mem, 1);
        node += 1;
        let mem = alloc(1, &mut word);
        nodes[node] = RegdmaLink::continuous(ST_INT_ENA, mem, 1);
        node += 1;

        &mut nodes[..node]
    }
}
