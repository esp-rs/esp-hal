//! Register DMA (regDMA) based register retention.
//!
//! The PAU's regDMA engine backs the `TOP`-domain peripheral registers up to RAM
//! and restores them across a `TOP` power-down in light sleep, walking a linked
//! list of [`RegdmaLink`] nodes on PAU entry link 0. Arming builds the core set
//! into a caller-owned [`SystemRetentionMemory`], chains any opt-in peripheral
//! entries and programs the link.
//!
//! All logic here is chip-agnostic; the only per-chip input is register data
//! (the `OPS` program and the base addresses), which lives in the `chip`
//! submodule - one data file per chip. Adding a chip is a new data file.

// References (ESP-IDF `v5.4`): `soc/regdma.h`, `hal/<chip>/pau_ll.h`,
// `hal/<chip>/pau_hal.c`, `esp_hw_support/port/pau_regdma.c`.

use core::{
    marker::PhantomData,
    ptr::NonNull,
    sync::atomic::{Ordering, fence},
};

use esp_sync::NonReentrantMutex;
use procmacros::ram;

use crate::{
    peripherals::{PAU, PCR, PMU},
    rtc_cntl::{
        cpu_retention::CpuRetentionMemory,
        power_domain::{Domain, PowerDomainLock, can_power_down},
    },
};

// Per-chip register data (base addresses, region sizes, the SYS_PERIPH program
// and the CPU-domain device-register bases). Selected by target; consumed only
// by the chip-agnostic logic here and (via the re-export below) `cpu_retention`.
#[cfg_attr(esp32c6, path = "retention/esp32c6.rs")]
#[cfg_attr(esp32h2, path = "retention/esp32h2.rs")]
mod chip;

// The CPU-domain device-register bases live in the same per-chip data module;
// re-export them so `cpu_retention` can read them while `chip` stays private.
pub(crate) use chip::{
    CACHE_BASE,
    CLINT_MINT_BASE,
    CLINT_UINT_BASE,
    INTPRI_BASE,
    PLIC_MX_BASE,
    PLIC_UX_BASE,
};
// Per-chip opt-in peripheral config-register retention data (register
// offsets/masks/maps/counts). The sequence builders below are chip-agnostic;
// only this register-layout data is device-specific.
use chip::{
    I2C_CONF_UPGATE,
    I2C_CTR_OFF,
    I2C_FSM_RST,
    I2C_REGS_MAP,
    I2C_RETENTION_REGS_CNT,
    I2C_SCL_LOW_PERIOD_OFF,
    SPI_CMD_OFF,
    SPI_REGS_MAP,
    SPI_RETENTION_REGS_CNT,
    UART_INT_ENA_OFF,
    UART_REG_UPDATE,
    UART_REG_UPDATE_OFF,
    UART_REGS_MAP,
    UART_RETENTION_REGS_CNT,
};

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
    /// Like [`Continuous`](Self::Continuous), but a 4-word bitmap selects which
    /// registers in the window to transfer (skipping interspersed read-only
    /// status/FIFO registers).
    AddrMap    = 1,
    /// Unconditionally write a masked value to a register.
    Write      = 2,
    /// Poll a register until `(reg & mask) == value`.
    Wait       = 3,
}

/// A single regDMA linked-list node (matches the PAU `head` + body layout).
///
/// - CONTINUOUS: `w0 = backup addr`, `w1 = restore addr`, `w2 = RAM buffer`.
/// - ADDR_MAP: as CONTINUOUS, plus `map` selecting which registers to transfer.
/// - WRITE/WAIT: `w0 = target addr`, `w1 = value`, `w2 = mask`.
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct RegdmaLink {
    /// Packed `regdma_link_head_t`.
    head: u32,
    /// Next node's `head`, or `0` at the end of the list.
    next: u32,
    w0: u32,
    w1: u32,
    w2: u32,
    /// ADDR_MAP register-selection bitmap; unread for other modes.
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

    /// A CONTINUOUS node with distinct backup/restore registers sharing one RAM
    /// buffer (e.g. the SysTimer value vs. load registers).
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

    /// An ADDR_MAP node: back up/restore the `count` registers selected by `map`
    /// (bit `i` = register at `reg + i * 4`) from `reg` into `storage`.
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

    /// A WRITE node that writes `value` (under `mask`) to `target`. `skip_b`/
    /// `skip_r` gate the write on the backup/restore pass.
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

// Console UART config-register retention, shared by the always-on console and
// the opt-in `UartRetentionMemory`. The register data lives in `chip`.
/// One ADDR_MAP + a restore-only WRITE+WAIT pulsing `UART_REG_UPDATE`.
const UART_NODE_COUNT: usize = 3;

/// Build the UART retention sequence for `base` into `nodes`, backing the
/// registers up into `storage`. The WRITE+WAIT pulse the update bit on restore
/// to latch the shadow (`_SYNC`) registers.
fn build_uart_seq(base: u32, nodes: &mut [RegdmaLink], storage: u32) {
    nodes[0] = RegdmaLink::addr_map(
        base + UART_INT_ENA_OFF,
        storage,
        UART_RETENTION_REGS_CNT,
        UART_REGS_MAP,
    );
    nodes[1] = RegdmaLink::write(
        base + UART_REG_UPDATE_OFF,
        UART_REG_UPDATE,
        UART_REG_UPDATE,
        true,
        false,
    );
    nodes[2] = RegdmaLink::wait(base + UART_REG_UPDATE_OFF, 0, UART_REG_UPDATE, true, false);
}

// I2C config-register retention. Config registers are shadowed, so restore
// pulses the FSM reset then requests a config update and waits for it to latch.
// The register data lives in `chip`.
/// One ADDR_MAP + a restore-only WRITE*3/WAIT pulsing `FSM_RST` then `CONF_UPGATE`.
const I2C_NODE_COUNT: usize = 5;

/// Build the I2C retention sequence for `base` into `nodes`, backing the
/// registers up into `storage`.
fn build_i2c_seq(base: u32, nodes: &mut [RegdmaLink], storage: u32) {
    let ctr = base + I2C_CTR_OFF;
    nodes[0] = RegdmaLink::addr_map(
        base + I2C_SCL_LOW_PERIOD_OFF,
        storage,
        I2C_RETENTION_REGS_CNT,
        I2C_REGS_MAP,
    );
    // Restore-only: pulse FSM reset, request config update, wait for it to latch.
    nodes[1] = RegdmaLink::write(ctr, I2C_FSM_RST, I2C_FSM_RST, true, false);
    nodes[2] = RegdmaLink::write(ctr, 0, I2C_FSM_RST, true, false);
    nodes[3] = RegdmaLink::write(ctr, I2C_CONF_UPGATE, I2C_CONF_UPGATE, true, false);
    nodes[4] = RegdmaLink::wait(ctr, 0, I2C_CONF_UPGATE, true, false);
}

// GPSPI2 config-register retention. The register data lives in `chip`.
/// A single ADDR_MAP over the config registers.
const SPI_NODE_COUNT: usize = 1;

/// Build the SPI retention sequence for `base` into `nodes`, backing the
/// registers up into `storage`.
///
/// The config registers are only reachable while the SPI function clock runs.
/// `Spi::with_retention_memory` holds that clock for the retention lifetime, so
/// they stay accessible at both backup and restore.
// `spi2_regs_retention`)
fn build_spi_seq(base: u32, nodes: &mut [RegdmaLink], storage: u32) {
    nodes[0] = RegdmaLink::addr_map(
        base + SPI_CMD_OFF,
        storage,
        SPI_RETENTION_REGS_CNT,
        SPI_REGS_MAP,
    );
}

/// A node in the intrusive registry of opt-in peripheral retention sequences.
///
/// One lives inside each peripheral's caller-owned retention memory, so any
/// number of peripherals can register without a fixed table or allocation. The
/// pointers are only dereferenced in [`arm_link`] (under [`REGISTRY`], on the
/// single HP core), pointing at borrow-frozen memory until deregistered.
pub(crate) struct RetentionNode {
    next: Option<NonNull<RetentionNode>>,
    head: *mut RegdmaLink,
    len: usize,
}

impl RetentionNode {
    const fn new() -> Self {
        Self {
            next: None,
            head: core::ptr::null_mut(),
            len: 0,
        }
    }
}

impl core::fmt::Debug for RetentionNode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("RetentionNode")
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for RetentionNode {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "RetentionNode")
    }
}

/// Head of the intrusive list of registered peripheral retention sequences.
struct Registry(Option<NonNull<RetentionNode>>);

// SAFETY: the pointers are only followed under the `REGISTRY` lock, and while
// armed they point at borrow-frozen caller memory on the single HP core.
unsafe impl Send for Registry {}

static REGISTRY: NonReentrantMutex<Registry> = NonReentrantMutex::new(Registry(None));

/// Push `node` onto the registry so [`arm_link`] chains its `nodes` on the next
/// TOP power-down.
fn register_node(node: &mut RetentionNode, nodes: &mut [RegdmaLink]) -> NonNull<RetentionNode> {
    node.head = nodes.as_mut_ptr();
    node.len = nodes.len();
    REGISTRY.with(|registry| {
        node.next = registry.0;
        registry.0 = Some(NonNull::from(&mut *node));
    });
    NonNull::from(node)
}

/// Remove a registration previously made with [`register_node`].
fn deregister_node(node: &mut RetentionNode) {
    let target = NonNull::from(&mut *node);
    REGISTRY.with(|registry| {
        let mut link: *mut Option<NonNull<RetentionNode>> = &mut registry.0;
        // SAFETY: every pointer in the list points at a live, registered node;
        // the walk only follows `next` links until it reaches `target`.
        unsafe {
            while let Some(current) = *link {
                if current == target {
                    *link = (*current.as_ptr()).next;
                    break;
                }
                link = &raw mut (*current.as_ptr()).next;
            }
        }
    });
    node.next = None;
}

/// Chain a slice of nodes (clearing each EOF flag), returning the last node so
/// the caller can terminate it or link it to a following segment.
fn link_internal(nodes: &mut [RegdmaLink]) -> *mut RegdmaLink {
    let len = nodes.len();
    for i in 0..len - 1 {
        nodes[i].next = nodes[i + 1].addr();
        nodes[i].head &= !HEAD_EOF_BIT;
    }
    &mut nodes[len - 1]
}

/// Chain the always-retained `core` list plus every registered opt-in entry
/// into one list, terminating only the final node. Returns the head address to
/// program into the PAU.
fn arm_link(core: &mut [RegdmaLink]) -> u32 {
    let head = core[0].addr();
    let mut tail = link_internal(core);

    REGISTRY.with(|registry| {
        let mut current = registry.0;
        while let Some(node) = current {
            // SAFETY: a registered node points at a live, borrow-frozen
            // caller-owned array of `len` nodes, distinct from every other.
            let (seg_head, seg_len, next) = unsafe {
                let node = node.as_ref();
                (node.head, node.len, node.next)
            };
            let seg = unsafe { core::slice::from_raw_parts_mut(seg_head, seg_len) };
            unsafe {
                (*tail).next = seg[0].addr();
                (*tail).head &= !HEAD_EOF_BIT;
            }
            tail = link_internal(seg);
            current = next;
        }
    });

    unsafe {
        (*tail).next = 0;
        (*tail).head |= HEAD_EOF_BIT;
    }
    head
}

/// Generate a per-peripheral caller-owned regDMA backing store, sized to its
/// `nodes` and register `buf`. `$build` is its sequence builder.
macro_rules! peripheral_retention_memory {
    ($name:ident, $nodes:expr, $words:expr, $build:path, $doc:expr) => {
        #[doc = $doc]
        #[instability::unstable]
        #[derive(Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[repr(C, align(4))]
        pub struct $name {
            node: RetentionNode,
            nodes: [RegdmaLink; $nodes],
            buf: [u32; $words],
        }

        #[instability::unstable]
        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }

        impl $name {
            #[doc = concat!("Create an empty [`", stringify!($name), "`].")]
            #[instability::unstable]
            pub const fn new() -> Self {
                Self {
                    node: RetentionNode::new(),
                    nodes: [RegdmaLink::EMPTY; $nodes],
                    buf: [0; $words],
                }
            }
        }

        impl RetentionMemory for $name {
            fn register(&mut self, base: u32) -> NonNull<RetentionNode> {
                let storage = self.buf.as_mut_ptr() as u32;
                $build(base, &mut self.nodes, storage);
                register_node(&mut self.node, &mut self.nodes)
            }
        }
    };
}

peripheral_retention_memory!(
    UartRetentionMemory,
    UART_NODE_COUNT,
    UART_RETENTION_REGS_CNT as usize,
    build_uart_seq,
    "Caller-owned store retaining one UART's config registers across a `TOP` \
power-down, passed to \
[`Uart::with_retention_memory`](crate::uart::Uart::with_retention_memory). The \
console/log UART is retained automatically."
);

peripheral_retention_memory!(
    I2cRetentionMemory,
    I2C_NODE_COUNT,
    I2C_RETENTION_REGS_CNT as usize,
    build_i2c_seq,
    "Caller-owned store retaining one I2C's config registers across a `TOP` \
power-down, passed to \
[`I2c::with_retention_memory`](crate::i2c::master::I2c::with_retention_memory). \
See [`UartRetentionMemory`]."
);

peripheral_retention_memory!(
    SpiRetentionMemory,
    SPI_NODE_COUNT,
    SPI_RETENTION_REGS_CNT as usize,
    build_spi_seq,
    "Caller-owned store retaining one SPI's config registers across a `TOP` \
power-down, passed to \
[`Spi::with_retention_memory`](crate::spi::master::Spi::with_retention_memory). \
See [`UartRetentionMemory`]."
);

/// Caller-owned retention memory that can be registered for TOP-domain
/// retention. Implemented by the generated `*RetentionMemory` types.
pub(crate) trait RetentionMemory {
    /// Build the retention sequence for `base` and register it, returning its
    /// registry node.
    fn register(&mut self, base: u32) -> NonNull<RetentionNode>;
}

/// A `TOP`-domain driver's power state, stored in the driver: either active,
/// holding a [`PowerDomainLock`] that keeps `TOP` powered (without preventing
/// sleep), or retained, with the lock dropped and regDMA saving/restoring its
/// config across a `TOP` power-down from `'d`-borrowed memory.
pub(crate) enum PowerManagement<'d, M: RetentionMemory> {
    /// Active, not retained: the held lock keeps `TOP` powered.
    PowerDomainLock { _lock: PowerDomainLock },
    /// Retained: `node` points into the caller-owned memory borrowed for `'d`.
    Retain {
        node: NonNull<RetentionNode>,
        _mem: PhantomData<&'d mut M>,
    },
}

impl<'d, M: RetentionMemory> PowerManagement<'d, M> {
    /// Active, un-retained: keep `TOP` powered so sleep can't lose its state.
    pub(crate) fn new() -> Self {
        Self::PowerDomainLock {
            _lock: PowerDomainLock::new(Domain::Top),
        }
    }

    /// Opt into retention: register `mem` for `base` and drop the domain lock so
    /// a `TOP` power-down can take effect.
    pub(crate) fn retain(&mut self, mem: &'d mut M, base: u32) {
        let node = mem.register(base);
        *self = Self::Retain {
            node,
            _mem: PhantomData,
        };
    }
}

impl<M: RetentionMemory> Drop for PowerManagement<'_, M> {
    fn drop(&mut self) {
        if let Self::Retain { node, .. } = self {
            // SAFETY: the node lives in caller memory borrowed for `'d`, which
            // outlives `self`, so it is still valid to unlink here.
            unsafe { deregister_node(node.as_mut()) };
        }
    }
}

// SAFETY: the only thread-unsafe state a `Retain` holds is the raw node/link
// pointers, and those are only ever dereferenced on the single HP core under the
// `REGISTRY` mutex (see `arm_link`/`deregister_node`); the owner never follows
// them otherwise.
unsafe impl<M: RetentionMemory> Send for PowerManagement<'_, M> {}
unsafe impl<M: RetentionMemory> Sync for PowerManagement<'_, M> {}

impl<M: RetentionMemory> core::fmt::Debug for PowerManagement<'_, M> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::PowerDomainLock { .. } => f.write_str("PowerManagement::PowerDomainLock"),
            Self::Retain { .. } => f.write_str("PowerManagement::Retain"),
        }
    }
}

#[cfg(feature = "defmt")]
impl<M: RetentionMemory> defmt::Format for PowerManagement<'_, M> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        match self {
            Self::PowerDomainLock { .. } => defmt::write!(fmt, "PowerManagement::PowerDomainLock"),
            Self::Retain { .. } => defmt::write!(fmt, "PowerManagement::Retain"),
        }
    }
}

/// One step of a chip's TOP-domain retention program (`chip::OPS`), expanded
/// into PAU regDMA nodes by [`sys_periph::build_link`]. `Write`/`Wait` are
/// restore-only (they re-apply state the backup can't capture, e.g. unlocking
/// TEE/APM or pulsing a clock-update bit).
// A given chip may not use every variant (only the H2 needs `Wait`).
#[allow(dead_code)]
#[derive(Clone, Copy)]
enum SysOp {
    /// Back up/restore `count` consecutive registers starting at `base`.
    Continuous { base: u32, count: u32 },
    /// Back up `count` words from `backup`, restore them to `restore` (e.g.
    /// GPIO output-enable via the W1TS register).
    ContinuousSplit {
        backup: u32,
        restore: u32,
        count: u32,
    },
    /// Restore-only masked write of `value` to `addr`.
    Write { addr: u32, value: u32, mask: u32 },
    /// Restore-only poll of `addr` until `(reg & mask) == value`.
    Wait { addr: u32, value: u32, mask: u32 },
    /// The shared console-UART config sequence for the UART based at `base`.
    Uart { base: u32 },
    /// The shared SysTimer save/restore sequence based at `base`.
    Systimer { base: u32 },
}

/// PAU nodes emitted for one [`SysOp`].
const fn op_nodes(op: &SysOp) -> usize {
    match op {
        SysOp::Continuous { .. }
        | SysOp::ContinuousSplit { .. }
        | SysOp::Write { .. }
        | SysOp::Wait { .. } => 1,
        SysOp::Uart { .. } => UART_NODE_COUNT,
        SysOp::Systimer { .. } => SYSTIMER_NODE_COUNT,
    }
}

/// RAM buffer words consumed by one [`SysOp`].
const fn op_words(op: &SysOp) -> usize {
    match op {
        SysOp::Continuous { count, .. } | SysOp::ContinuousSplit { count, .. } => *count as usize,
        SysOp::Uart { .. } => UART_RETENTION_REGS_CNT as usize,
        SysOp::Systimer { .. } => SYSTIMER_CONT_WORDS,
        SysOp::Write { .. } | SysOp::Wait { .. } => 0,
    }
}

/// Total PAU nodes an op list expands to (sizes [`SystemRetentionMemory`]).
const fn ops_node_count(ops: &[SysOp]) -> usize {
    let mut n = 0;
    let mut i = 0;
    while i < ops.len() {
        n += op_nodes(&ops[i]);
        i += 1;
    }
    n
}

/// Total RAM buffer words an op list needs (sizes [`SystemRetentionMemory`]).
const fn ops_buf_words(ops: &[SysOp]) -> usize {
    let mut w = 0;
    let mut i = 0;
    while i < ops.len() {
        w += op_words(&ops[i]);
        i += 1;
    }
    w
}

// SysTimer register offsets and bit masks (shared; only the base differs per
// chip). Offsets/masks from ESP-IDF v5.4 `systimer_reg.h`; the save/restore
// sequence mirrors `systimer_regs_retention[]`.
const ST_UNIT_UPDATE: u32 = 1 << 30;
const ST_UNIT_VALUE_VALID: u32 = 1 << 29;
const ST_UNIT_LOAD: u32 = 1 << 0;
const ST_COMP_LOAD: u32 = 1 << 0;
const ST_TARGET_PERIOD_MODE: u32 = 1 << 30;
/// TARGET0_HI ..= TARGET2_CONF, i.e. all three targets' hi/lo/conf.
const ST_TARGETS_LEN: u32 = 9;
/// One node per SysTimer step of `build_systimer_seq`.
const SYSTIMER_NODE_COUNT: usize = 19;
/// SysTimer CONTINUOUS words: unit0/1 value (2+2), targets (9), conf, int_ena.
const SYSTIMER_CONT_WORDS: usize = 2 + 2 + ST_TARGETS_LEN as usize + 1 + 1;

/// Build the SysTimer retention sequence for the timer at `base` into `nodes`,
/// drawing its RAM from `buf_base` starting at word `start_word`. Fills exactly
/// [`SYSTIMER_NODE_COUNT`] nodes and consumes [`SYSTIMER_CONT_WORDS`] words.
///
/// Backup latches each unit's counter (UPDATE + wait for VALUE_VALID) and reads
/// it; restore loads it back and triggers a load. The value is read from
/// VALUE_HI/LO but restored into LOAD_HI/LO, hence the split backup/restore
/// addresses.
fn build_systimer_seq(base: u32, nodes: &mut [RegdmaLink], buf_base: *mut u32, start_word: usize) {
    let st_conf = base;
    let st_unit0_op = base + 0x04;
    let st_unit1_op = base + 0x08;
    let st_unit0_load_hi = base + 0x0C;
    let st_unit1_load_hi = base + 0x14;
    let st_target0_hi = base + 0x1C;
    let st_target0_conf = base + 0x34;
    let st_target1_conf = base + 0x38;
    let st_target2_conf = base + 0x3C;
    let st_unit0_value_hi = base + 0x40;
    let st_unit1_value_hi = base + 0x48;
    let st_comp0_load = base + 0x50;
    let st_comp1_load = base + 0x54;
    let st_comp2_load = base + 0x58;
    let st_unit0_load = base + 0x5C;
    let st_unit1_load = base + 0x60;
    let st_int_ena = base + 0x64;

    let mut word = start_word;
    let mut alloc = |len: u32| -> u32 {
        let mem = unsafe { buf_base.add(word) } as u32;
        word += len as usize;
        mem
    };

    let mut node = 0;

    // Per unit: latch + read value, then restore into load.
    for (op, value_hi, load_hi, load) in [
        (
            st_unit0_op,
            st_unit0_value_hi,
            st_unit0_load_hi,
            st_unit0_load,
        ),
        (
            st_unit1_op,
            st_unit1_value_hi,
            st_unit1_load_hi,
            st_unit1_load,
        ),
    ] {
        nodes[node] = RegdmaLink::write(op, ST_UNIT_UPDATE, ST_UNIT_UPDATE, false, true);
        node += 1;
        nodes[node] = RegdmaLink::wait(op, ST_UNIT_VALUE_VALID, ST_UNIT_VALUE_VALID, false, true);
        node += 1;
        let mem = alloc(2);
        nodes[node] = RegdmaLink::continuous_split(value_hi, load_hi, mem, 2);
        node += 1;
        nodes[node] = RegdmaLink::write(load, ST_UNIT_LOAD, ST_UNIT_LOAD, true, false);
        node += 1;
    }

    // Comparator target values & periods.
    let mem = alloc(ST_TARGETS_LEN);
    nodes[node] = RegdmaLink::continuous(st_target0_hi, mem, ST_TARGETS_LEN);
    node += 1;
    for comp in [st_comp0_load, st_comp1_load, st_comp2_load] {
        nodes[node] = RegdmaLink::write(comp, ST_COMP_LOAD, ST_COMP_LOAD, true, false);
        node += 1;
    }
    // Re-arm period mode: clear+set for target0/1, clear for target2.
    for target in [st_target0_conf, st_target1_conf] {
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
    nodes[node] = RegdmaLink::write(st_target2_conf, 0, ST_TARGET_PERIOD_MODE, true, false);
    node += 1;

    // Work-enable and interrupt-enable state.
    let mem = alloc(1);
    nodes[node] = RegdmaLink::continuous(st_conf, mem, 1);
    node += 1;
    let mem = alloc(1);
    nodes[node] = RegdmaLink::continuous(st_int_ena, mem, 1);
    node += 1;

    debug_assert!(node == SYSTIMER_NODE_COUNT);
    debug_assert!(word - start_word == SYSTIMER_CONT_WORDS);
}

/// Caller-owned backing store for the TOP-domain system-peripheral register set
/// (PCR, interrupt matrix, HP system, TEE/APM, IO MUX, GPIO matrix, flash SPI
/// mem, console UART and SysTimer - see the chip's `OPS` retention program).
///
/// The core state regDMA must retain for the `TOP` domain to power down at all;
/// the caller opts in via [`RtcSleepConfig::with_top_power_down`]. Individual
/// peripherals opt into retaining their own config via `with_retention_memory`.
///
/// [`RtcSleepConfig::with_top_power_down`]: crate::rtc_cntl::sleep::RtcSleepConfig::with_top_power_down
#[instability::unstable]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C, align(4))]
pub struct SystemRetentionMemory {
    nodes: [RegdmaLink; ops_node_count(chip::OPS)],
    buf: [u32; ops_buf_words(chip::OPS)],
}

#[instability::unstable]
impl Default for SystemRetentionMemory {
    fn default() -> Self {
        Self::new()
    }
}

impl SystemRetentionMemory {
    /// Create a new, zeroed system-peripheral retention buffer.
    #[instability::unstable]
    pub const fn new() -> Self {
        Self {
            nodes: [RegdmaLink::EMPTY; ops_node_count(chip::OPS)],
            buf: [0; ops_buf_words(chip::OPS)],
        }
    }
}

/// Enable the regDMA bus clock and release its reset (`pau_ll_enable_bus_clock`),
/// and bound the WAIT polling so a never-satisfied condition can't hang the
/// engine (`pau_hal_set_regdma_wait_timeout`, ESP-IDF `PAU_REGDMA_LINK_WAIT_*`).
// `#[ram]`: also called from the wake path (see `restore_top_retention`).
#[ram]
fn regdma_clock_and_timeout() {
    PCR::regs().regdma_conf().modify(|_, w| {
        w.regdma_clk_en().set_bit();
        w.regdma_rst_en().clear_bit()
    });
    PAU::regs().regdma_bkp_conf().modify(|_, w| unsafe {
        w.link_tout_thres().bits(1000);
        w.read_interval().bits(32)
    });
}

/// Software-trigger a regDMA transfer of system link 0 and wait for it. `backup`
/// copies registers into RAM, else restores RAM back into registers. Used on
/// [`chip::SW_TRIGGER_REGDMA`] chips whose PAU powers off with `TOP`, so the PMU
/// can't drive the transfer.
// Mirrors ESP-IDF `pau_hal_start_regdma_system_link`.
// `#[ram]`: the restore runs on wake before the flash SPI controller is back.
#[ram]
fn sw_trigger_system_link(backup: bool) {
    let pau = PAU::regs();
    // `start` must be asserted in its own write, after link_sel/to_mem are set;
    // a combined write leaves the transfer's `done` never asserting.
    pau.int_clr().write(|w| w.done().clear_bit_by_one());
    pau.regdma_conf()
        .modify(|_, w| unsafe { w.link_sel().bits(0) });
    pau.regdma_conf().modify(|_, w| w.to_mem().bit(backup));
    pau.regdma_conf().modify(|_, w| w.start().set_bit());
    // Bounded safety net; WAIT nodes are separately bounded by the wait timeout.
    for _ in 0..2_000_000u32 {
        if pau.int_raw().read().done().bit_is_set() {
            break;
        }
    }
    pau.regdma_conf().modify(|_, w| w.start().clear_bit());
    pau.regdma_conf()
        .modify(|_, w| unsafe { w.link_sel().bits(0) });
    pau.int_clr().write(|w| w.done().clear_bit_by_one());
}

/// Arm regDMA retention of the TOP-domain peripherals for the upcoming light
/// sleep: program PAU entry link 0 and start the backup. Must be called after
/// the PMU power config (which resets the backup-enable bits) and before the
/// sleep request; rebuilds the chain from the live registry each time.
///
/// Where the PAU survives the `TOP` power-down the PMU drives backup/restore in
/// hardware. On [`chip::SW_TRIGGER_REGDMA`] chips the PAU powers off with `TOP`,
/// so the hardware backup is disabled and the backup is triggered in software
/// here, with [`restore_top_retention`] doing the restore on wake.
fn enable_top_retention(mem: &mut SystemRetentionMemory) {
    regdma_clock_and_timeout();

    let head = arm_link(sys_periph::build_link(&mut mem.nodes, &mut mem.buf));
    fence(Ordering::SeqCst);
    PAU::regs()
        .regdma_link_0_addr()
        .write(|w| unsafe { w.bits(head) });

    let pmu = PMU::regs();
    if chip::SW_TRIGGER_REGDMA {
        // The PAU powers off with TOP: the PMU can't run the backup on the
        // active->sleep transition, so disable it and back up in software now.
        pmu.hp_sleep_backup()
            .modify(|_, w| w.hp_active2sleep_backup_en().clear_bit());
        pmu.hp_active_backup()
            .modify(|_, w| w.hp_sleep2active_backup_en().clear_bit());
        sw_trigger_system_link(true);
    } else {
        // pmu_sleep_enable_regdma_backup: back up active->sleep, restore
        // sleep->active, both driven by the PMU in hardware.
        pmu.hp_sleep_backup()
            .modify(|_, w| w.hp_active2sleep_backup_en().set_bit());
        pmu.hp_active_backup()
            .modify(|_, w| w.hp_sleep2active_backup_en().set_bit());
    }
}

/// Restore the TOP-domain peripherals on wake for [`chip::SW_TRIGGER_REGDMA`]
/// chips, whose PAU lost its own configuration with the `TOP` power-down.
///
/// Re-enables the regDMA bus clock, re-programs entry link 0 (the linked list in
/// caller memory survived in RAM) and software-triggers the restore. Must run
/// before any powered-down `TOP` peripheral is touched. No-op on chips that
/// restore in hardware.
// `#[ram]`: runs on the wake path before the flash SPI controller is restored.
#[ram]
pub(crate) fn restore_top_retention(mem: &mut SystemRetentionMemory) {
    if !chip::SW_TRIGGER_REGDMA {
        return;
    }
    regdma_clock_and_timeout();
    // The chain in `mem`/opt-in memory is intact in RAM; entry link 0 is the
    // first system node, so re-point the PAU at it without rebuilding.
    let head = mem.nodes.as_ptr() as u32;
    fence(Ordering::SeqCst);
    PAU::regs()
        .regdma_link_0_addr()
        .write(|w| unsafe { w.bits(head) });
    sw_trigger_system_link(false);
}

/// Request a plain (non-CPU-retention) sleep and spin until wake or reject. In
/// deep sleep the chip resets on wake, so this never returns.
fn request_sleep_and_wait() {
    let pmu = PMU::regs();
    pmu.slp_wakeup_cntl0().write(|w| w.sleep_req().bit(true));
    loop {
        let int_raw = pmu.int_raw().read();
        if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
            break;
        }
    }
}

/// Disable TIMG0's flashboot watchdog after a `TOP` power-down: TIMG0 is not
/// retained and comes back armed.
// Mirrors ESP-IDF `misc_modules_wake_prepare()`.
pub(crate) fn disable_timg0_flashboot_wdt() {
    let tg0 = crate::peripherals::TIMG0::regs();
    tg0.wdtwprotect().write(|w| unsafe { w.bits(0x50D8_3AA1) });
    tg0.wdtconfig0()
        .modify(|_, w| w.wdt_flashboot_mod_en().bit(false));
    tg0.wdtwprotect().write(|w| unsafe { w.bits(0) });
}

/// Light-sleep power-domain retention state, embedded in each chip's
/// `RtcSleepConfig`. Holds the caller's opt-in choices and retention memory plus
/// the chip-agnostic resolve/enter logic; a chip only maps the resolved decision
/// onto its own `PowerDownFlags`.
///
/// Raw pointers (not borrows) keep the embedding `RtcSleepConfig` `Copy`; the
/// setters take `&'static mut`.
#[derive(Clone, Copy)]
pub(crate) struct SleepRetention {
    cpu_power_down: bool,
    top_power_down: bool,
    cpu_mem: *mut CpuRetentionMemory,
    top_mem: *mut SystemRetentionMemory,
}

impl SleepRetention {
    pub(crate) const fn new() -> Self {
        Self {
            cpu_power_down: false,
            top_power_down: false,
            cpu_mem: core::ptr::null_mut(),
            top_mem: core::ptr::null_mut(),
        }
    }

    /// Opt into CPU power-down, saving CPU state into `mem`.
    pub(crate) fn set_cpu_power_down(&mut self, mem: &'static mut CpuRetentionMemory) {
        self.cpu_power_down = true;
        self.cpu_mem = mem;
    }

    /// Opt into `TOP` power-down (which also powers the CPU down), using `cpu`
    /// for the CPU state and `sys` for the regDMA system-peripheral set.
    pub(crate) fn set_top_power_down(
        &mut self,
        cpu: &'static mut CpuRetentionMemory,
        sys: &'static mut SystemRetentionMemory,
    ) {
        self.top_power_down = true;
        self.cpu_mem = cpu;
        self.top_mem = sys;
    }

    pub(crate) fn cpu_power_down(&self) -> bool {
        self.cpu_power_down
    }

    pub(crate) fn top_power_down(&self) -> bool {
        self.top_power_down
    }

    /// Resolve which domains may actually power down for a light sleep, given the
    /// opt-in choices, caller memory and active power-domain locks. Returns
    /// `(cpu_pd, top_pd)`; `top_pd` implies `cpu_pd`, and a domain only powers
    /// down with its retention memory and no lock (else it clock-gates).
    pub(crate) fn resolve(&self) -> (bool, bool) {
        let have_cpu = !self.cpu_mem.is_null();
        let have_sys = !self.top_mem.is_null();
        let top = self.top_power_down && have_cpu && have_sys && can_power_down(Domain::Top);
        let cpu = self.cpu_power_down && have_cpu && can_power_down(Domain::Cpu);
        (cpu || top, top)
    }

    /// Enter the sleep with the resolved `(cpu_pd, top_pd)` decision: arm TOP
    /// regDMA if powering `TOP` down, then run software CPU retention (if
    /// powering the CPU down) or a plain sleep request. `deep` forces the plain
    /// path (retention is light-sleep only; deep sleep does not return).
    ///
    /// # Safety
    ///
    /// The PMU must already be configured for this sleep and the retention
    /// memory must stay valid across it.
    pub(crate) unsafe fn enter(&self, deep: bool, cpu_pd: bool, top_pd: bool) {
        if !deep && top_pd && !self.top_mem.is_null() {
            // After the PMU power config (which resets the backup-enable bits)
            // and before the sleep request.
            enable_top_retention(unsafe { &mut *self.top_mem });
        }
        if !deep && cpu_pd && !self.cpu_mem.is_null() {
            // Save CPU state, sleep, resume with it restored. `top_mem` is passed
            // only when TOP is powered down so its (software) restore runs in RAM
            // on wake; a no-op on hardware-restore chips.
            let top = if top_pd {
                self.top_mem
            } else {
                core::ptr::null_mut()
            };
            unsafe {
                crate::rtc_cntl::cpu_retention::sleep_with_cpu_retention(&mut *self.cpu_mem, top)
            };
        } else {
            request_sleep_and_wait();
        }
    }
}

/// Chip-agnostic interpreter that expands a chip's `OPS` program into PAU regDMA
/// nodes (in retention-priority order, system clock first).
// Mirrors ESP-IDF's `SLEEP_RETENTION_MODULE_SYS_PERIPH` + `..._CLOCK_SYSTEM`
// (`soc/<chip>/system_retention_periph.c`, `.../sleep_clock.c`).
mod sys_periph {
    use super::{
        HEAD_LENGTH_MASK,
        RegdmaLink,
        SYSTIMER_NODE_COUNT,
        SysOp,
        UART_NODE_COUNT,
        UART_RETENTION_REGS_CNT,
        build_systimer_seq,
        build_uart_seq,
        chip,
    };

    /// Nodes this chip's op list expands to (sizes [`super::SystemRetentionMemory`]).
    pub(super) const NODE_COUNT: usize = super::ops_node_count(chip::OPS);

    // Every CONTINUOUS region count must fit the 10-bit node `length` field.
    const _: () = {
        let mut i = 0;
        while i < chip::OPS.len() {
            if let SysOp::Continuous { count, .. } | SysOp::ContinuousSplit { count, .. } =
                chip::OPS[i]
            {
                core::assert!(count <= HEAD_LENGTH_MASK);
            }
            i += 1;
        }
    };

    /// (Re)build the SYS_PERIPH retention list into `nodes`/`buf` by walking the
    /// chip's `OPS` program, and return the filled node slice.
    pub(super) fn build_link<'a>(
        nodes: &'a mut [RegdmaLink; NODE_COUNT],
        buf: &mut [u32],
    ) -> &'a mut [RegdmaLink] {
        let buf_base = buf.as_mut_ptr();

        let mut node = 0;
        let mut word = 0;

        for op in chip::OPS {
            match *op {
                SysOp::Continuous { base, count } => {
                    let mem = unsafe { buf_base.add(word) } as u32;
                    nodes[node] = RegdmaLink::continuous(base, mem, count);
                    word += count as usize;
                    node += 1;
                }
                SysOp::ContinuousSplit {
                    backup,
                    restore,
                    count,
                } => {
                    let mem = unsafe { buf_base.add(word) } as u32;
                    nodes[node] = RegdmaLink::continuous_split(backup, restore, mem, count);
                    word += count as usize;
                    node += 1;
                }
                SysOp::Write { addr, value, mask } => {
                    nodes[node] = RegdmaLink::write(addr, value, mask, true, false);
                    node += 1;
                }
                SysOp::Wait { addr, value, mask } => {
                    nodes[node] = RegdmaLink::wait(addr, value, mask, true, false);
                    node += 1;
                }
                SysOp::Uart { base } => {
                    let mem = unsafe { buf_base.add(word) } as u32;
                    build_uart_seq(base, &mut nodes[node..node + UART_NODE_COUNT], mem);
                    word += UART_RETENTION_REGS_CNT as usize;
                    node += UART_NODE_COUNT;
                }
                SysOp::Systimer { base } => {
                    build_systimer_seq(
                        base,
                        &mut nodes[node..node + SYSTIMER_NODE_COUNT],
                        buf_base,
                        word,
                    );
                    node += SYSTIMER_NODE_COUNT;
                    word += super::SYSTIMER_CONT_WORDS;
                }
            }
        }

        &mut nodes[..node]
    }
}
