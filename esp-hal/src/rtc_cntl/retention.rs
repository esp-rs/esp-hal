//! Register DMA (regDMA) based register retention.
//!
//! The PAU's regDMA engine backs the `TOP`-domain peripheral registers up to RAM
//! and restores them across a `TOP` power-down in light sleep, walking a linked
//! list of [`RegdmaLink`] nodes on PAU entry link 0. Arming builds the core set
//! into a caller-owned [`SystemRetentionMemory`], chains any opt-in peripheral
//! entries and programs the link.

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

/// Runtime absolute address (as `u32`) of a named PAC register.
macro_rules! reg_addr {
    ($peri:ident, $($path:tt)+) => {
        (unsafe { &*crate::pac::$peri::PTR }.$($path)+.as_ptr()) as u32
    };
}

/// Runtime base address (as `u32`) of a PAC peripheral.
macro_rules! peri_base {
    ($peri:ident) => {
        crate::pac::$peri::ptr() as u32
    };
}

/// `const` byte offset of a named PAC register within its peripheral.
macro_rules! reg_off {
    ($peri:ident, $($path:tt)+) => {{
        let block = core::mem::MaybeUninit::<
            <crate::pac::$peri as core::ops::Deref>::Target,
        >::uninit();
        let base = block.as_ptr();
        let reg = unsafe { (*base).$($path)+ };

        (unsafe { (reg as *const _ as *const u8).offset_from(base as *const u8) }) as u32
    }};
}

macro_rules! reg {
    ($peri:ident, $($path:tt)+) => {
        || reg_addr!($peri, $($path)+)
    };
}

/// `const` width of a named PAC register in 32-bit words: `2` for the 64-bit
/// CLINT counters, `1` for everything else.
macro_rules! reg_words {
    ($peri:ident, $($path:tt)+) => {{
        let block = core::mem::MaybeUninit::<
            <crate::pac::$peri as core::ops::Deref>::Target,
        >::uninit();
        let base = block.as_ptr();
        let reg = unsafe { (*base).$($path)+ };
        (core::mem::size_of_val(reg) / 4) as u32
    }};
}

/// `const` count of registers spanning `[from] ..= [to]` (inclusive).
macro_rules! span {
    ($peri:ident, [$($from:tt)+] ..= [$($to:tt)+]) => {
        (reg_off!($peri, $($to)+) - reg_off!($peri, $($from)+)) / 4
            + reg_words!($peri, $($to)+)
    };
}

// ---------------------------------------------------------------------------
// `SysOp` builders. Each retention step is one self-describing line whose
// address(es) and register count are both derived from the named register(s).
// ---------------------------------------------------------------------------

/// A `Continuous` op. Two forms:
/// - `continuous!(PCR, [uart(0).conf()] ..= [sram_power_conf()])` - back up the inclusive register
///   span; `count` is derived from it.
/// - `continuous!(GPIO, [func_out_sel_cfg(0)], 35)` - named start, explicit count (used where the
///   end register isn't modelled in the PAC).
macro_rules! continuous {
    ($peri:ident, [$($from:tt)+] ..= [$($to:tt)+]) => {
        SysOp::Continuous {
            addr: || reg_addr!($peri, $($from)+),
            count: span!($peri, [$($from)+] ..= [$($to)+]),
        }
    };
    ($peri:ident, [$($from:tt)+], $count:expr) => {
        SysOp::Continuous {
            addr: || reg_addr!($peri, $($from)+),
            count: $count,
        }
    };
}

/// A `ContinuousSplit` op: back up `[backup]`, restore into `[restore]`.
macro_rules! continuous_split {
    ($peri:ident, [$($backup:tt)+] => [$($restore:tt)+], $count:expr) => {
        SysOp::ContinuousSplit {
            backup: || reg_addr!($peri, $($backup)+),
            restore: || reg_addr!($peri, $($restore)+),
            count: $count,
        }
    };
}

/// A restore-only masked `Write`.
macro_rules! write_reg {
    ($peri:ident, [$($path:tt)+], $value:expr, $mask:expr) => {
        SysOp::Write {
            addr: || reg_addr!($peri, $($path)+),
            value: $value,
            mask: $mask,
        }
    };
}

/// A restore-only `Wait` until `(reg & mask) == value`.
#[cfg(sleep_regdma_wait_ops)]
macro_rules! wait_reg {
    ($peri:ident, [$($path:tt)+], $value:expr, $mask:expr) => {
        SysOp::Wait {
            addr: || reg_addr!($peri, $($path)+),
            value: $value,
            mask: $mask,
        }
    };
}

/// The shared console-UART sequence for the given UART peripheral.
macro_rules! uart_seq {
    ($peri:ident) => {
        SysOp::Uart {
            base: || peri_base!($peri),
        }
    };
}

/// The shared SysTimer sequence for the given SysTimer peripheral.
macro_rules! systimer_seq {
    ($peri:ident) => {
        SysOp::Systimer {
            base: || peri_base!($peri),
        }
    };
}

// ---------------------------------------------------------------------------
// `PeriphOp` builders. Opt-in peripheral (UART/I2C/SPI) retention chains are
// per-chip, per-peripheral `&[PeriphOp]` lists.
// ---------------------------------------------------------------------------

/// Back up/restore a run of config registers. Two forms:
/// - `periph_continuous!(UART0, [hwfc_conf()] ..= [tout_conf()])`
/// - `periph_continuous!(UART0, [int_ena()])`
macro_rules! periph_continuous {
    ($peri:ident, [$($from:tt)+] ..= [$($to:tt)+]) => {
        PeriphOp::Continuous {
            off: reg_off!($peri, $($from)+),
            count: span!($peri, [$($from)+] ..= [$($to)+]),
        }
    };
    ($peri:ident, [$($reg:tt)+]) => {
        PeriphOp::Continuous {
            off: reg_off!($peri, $($reg)+),
            count: 1,
        }
    };
}

/// A restore-only masked write to a named config register.
macro_rules! periph_write {
    ($peri:ident, [$($reg:tt)+], $value:expr, $mask:expr) => {
        PeriphOp::Write {
            off: reg_off!($peri, $($reg)+),
            value: $value,
            mask: $mask,
        }
    };
}

/// A restore-only poll of a named config register until `(reg & mask) == value`.
macro_rules! periph_wait {
    ($peri:ident, [$($reg:tt)+], $value:expr, $mask:expr) => {
        PeriphOp::Wait {
            off: reg_off!($peri, $($reg)+),
            value: $value,
            mask: $mask,
        }
    };
}

// ---------------------------------------------------------------------------
// CPU-domain register regions, copied word by word by `cpu_retention` because
// regDMA cannot reach them while the CPU domain is off.
// ---------------------------------------------------------------------------

/// A contiguous run of `words` 32-bit registers starting at `start`.
///
/// `start` is resolved from the PAC at runtime (see [`reg!`]); `words` stays
/// `const` so the retention store can be sized at compile time.
pub(crate) struct Region {
    pub(crate) start: fn() -> u32,
    pub(crate) words: usize,
}

/// Total 32-bit words covered by a set of [`Region`]s, to size their store.
pub(crate) const fn total_words(regions: &[Region]) -> usize {
    let mut words = 0;
    let mut i = 0;
    while i < regions.len() {
        words += regions[i].words;
        i += 1;
    }
    words
}

/// A [`Region`] over named registers. Two forms:
/// - `region!(INTPRI, [cpu_int_enable()] ..= [rnd_eco_low()])`
/// - `region!(PLIC_MX, [mxint_conf()])`
macro_rules! region {
    ($peri:ident, [$($from:tt)+] ..= [$($to:tt)+]) => {
        Region {
            start: reg!($peri, $($from)+),
            words: span!($peri, [$($from)+] ..= [$($to)+]) as usize,
        }
    };
    ($peri:ident, [$($reg:tt)+]) => {
        Region {
            start: reg!($peri, $($reg)+),
            words: reg_words!($peri, $($reg)+) as usize,
        }
    };
}

#[cfg_attr(esp32c6, path = "retention/esp32c6.rs")]
#[cfg_attr(esp32h2, path = "retention/esp32h2.rs")]
mod chip;

pub(crate) use chip::{CACHE_REGIONS, CLINT_REGIONS, INTPRI_REGIONS, PLIC_REGIONS};
use chip::{I2C_OPS, SPI_OPS, UART_OPS};

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
    /// Unconditionally write a masked value to a register.
    Write      = 2,
    /// Poll a register until `(reg & mask) == value`.
    Wait       = 3,
}

/// A single regDMA linked-list node.
///
/// - CONTINUOUS: `w0 = backup addr`, `w1 = restore addr`, `w2 = RAM buffer`.
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
}

impl RegdmaLink {
    const EMPTY: Self = Self {
        head: 0,
        next: 0,
        w0: 0,
        w1: 0,
        w2: 0,
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
    /// buffer.
    fn continuous_split(backup: u32, restore: u32, storage: u32, len: u32) -> Self {
        Self {
            head: Self::head(LinkMode::Continuous, len, false, false),
            next: 0,
            w0: backup,
            w1: restore,
            w2: storage,
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
        }
    }

    fn addr(&self) -> u32 {
        core::ptr::addr_of!(self.head) as u32
    }
}

/// One step of an opt-in peripheral's config-register retention chain.
///
/// Unlike [`SysOp`] (whose peripherals are fixed, so it carries absolute
/// addresses) these apply to whichever instance the caller registers.
#[derive(Clone, Copy)]
enum PeriphOp {
    /// Back up/restore `count` consecutive registers at `base + off`.
    Continuous { off: u32, count: u32 },
    /// Restore-only masked write of `value` to `base + off`.
    Write { off: u32, value: u32, mask: u32 },
    /// Restore-only poll of `base + off` until `(reg & mask) == value`.
    Wait { off: u32, value: u32, mask: u32 },
}

/// PAU nodes an opt-in peripheral op list expands to (one per op).
const fn periph_nodes(ops: &[PeriphOp]) -> usize {
    ops.len()
}

/// RAM buffer words an opt-in peripheral op list needs (its backed-up registers).
const fn periph_words(ops: &[PeriphOp]) -> usize {
    let mut w = 0;
    let mut i = 0;
    while i < ops.len() {
        if let PeriphOp::Continuous { count, .. } = ops[i] {
            w += count as usize;
        }
        i += 1;
    }
    w
}

/// Build a peripheral's retention sequence for `base` into `nodes`, drawing its
/// CONTINUOUS RAM from `storage`. Fills exactly [`periph_nodes`] nodes and
/// consumes [`periph_words`] words.
fn build_periph_seq(base: u32, ops: &[PeriphOp], nodes: &mut [RegdmaLink], storage: *mut u32) {
    let mut word = 0;
    for (node, op) in nodes.iter_mut().zip(ops) {
        *node = match *op {
            PeriphOp::Continuous { off, count } => {
                let mem = unsafe { storage.add(word) } as u32;
                word += count as usize;
                RegdmaLink::continuous(base + off, mem, count)
            }
            PeriphOp::Write { off, value, mask } => {
                RegdmaLink::write(base + off, value, mask, true, false)
            }
            PeriphOp::Wait { off, value, mask } => {
                RegdmaLink::wait(base + off, value, mask, true, false)
            }
        };
    }
}

/// Console UART config-register retention, shared by the always-on console and
/// the opt-in `UartRetentionMemory`.
const UART_NODE_COUNT: usize = periph_nodes(chip::UART_OPS);
/// RAM buffer words the UART chain backs up (see [`chip::UART_OPS`]).
const UART_WORDS: usize = periph_words(chip::UART_OPS);

/// A node in the intrusive registry of opt-in peripheral retention sequences.
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

/// Generate a per-peripheral caller-owned regDMA backing store, sized from and
/// built by the chip's `$ops` op chain.
macro_rules! peripheral_retention_memory {
    ($name:ident, $ops:expr, $doc:expr) => {
        #[doc = $doc]
        #[instability::unstable]
        #[derive(Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[repr(C, align(4))]
        pub struct $name {
            node: RetentionNode,
            nodes: [RegdmaLink; periph_nodes($ops)],
            buf: [u32; periph_words($ops)],
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
                    nodes: [RegdmaLink::EMPTY; periph_nodes($ops)],
                    buf: [0; periph_words($ops)],
                }
            }
        }

        impl RetentionMemory for $name {
            fn register(&mut self, base: u32) -> NonNull<RetentionNode> {
                let storage = self.buf.as_mut_ptr();
                build_periph_seq(base, $ops, &mut self.nodes, storage);
                register_node(&mut self.node, &mut self.nodes)
            }
        }
    };
}

peripheral_retention_memory!(
    UartRetentionMemory,
    UART_OPS,
    "Caller-owned store retaining one UART's config registers across a `TOP` \
power-down, passed to \
[`Uart::with_retention_memory`](crate::uart::Uart::with_retention_memory). The \
console/log UART is retained automatically."
);

peripheral_retention_memory!(
    I2cRetentionMemory,
    I2C_OPS,
    "Caller-owned store retaining one I2C's config registers across a `TOP` \
power-down, passed to \
[`I2c::with_retention_memory`](crate::i2c::master::I2c::with_retention_memory). \
See [`UartRetentionMemory`]."
);

peripheral_retention_memory!(
    SpiRetentionMemory,
    SPI_OPS,
    "Caller-owned store retaining one SPI's config registers across a `TOP` \
power-down, passed to \
[`Spi::with_retention_memory`](crate::spi::master::Spi::with_retention_memory). \
See [`UartRetentionMemory`]."
);

/// Caller-owned retention memory that can be registered for TOP-domain
/// retention.
pub(crate) trait RetentionMemory {
    /// Build the retention sequence for `base` and register it, returning its
    /// registry node.
    fn register(&mut self, base: u32) -> NonNull<RetentionNode>;
}

/// A `TOP`-domain driver's power state, stored in the driver.
pub(crate) enum PowerManagement<'d, M: RetentionMemory> {
    /// Active, not retained: the held lock keeps `TOP` powered.
    PowerDomainLock { _lock: PowerDomainLock },
    /// Retained: `node` points into the caller-owned memory.
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
            unsafe { deregister_node(node.as_mut()) };
        }
    }
}

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
/// into PAU regDMA nodes by [`sys_periph::build_link`].
#[derive(Clone, Copy)]
enum SysOp {
    /// Back up/restore `count` consecutive registers starting at `addr`.
    Continuous { addr: fn() -> u32, count: u32 },
    /// Back up `count` consecutive registers starting at `backup`, restore into
    /// `restore`.
    ContinuousSplit {
        backup: fn() -> u32,
        restore: fn() -> u32,
        count: u32,
    },
    /// Restore-only masked write of `value` to `addr`.
    Write {
        addr: fn() -> u32,
        value: u32,
        mask: u32,
    },
    /// Restore-only poll of `addr` until `(reg & mask) == value`.
    #[cfg(sleep_regdma_wait_ops)]
    Wait {
        addr: fn() -> u32,
        value: u32,
        mask: u32,
    },
    /// The shared console-UART config sequence.
    Uart { base: fn() -> u32 },
    /// The shared SysTimer save/restore sequence.
    Systimer { base: fn() -> u32 },
}

/// PAU nodes emitted for one [`SysOp`].
const fn op_nodes(op: &SysOp) -> usize {
    match op {
        SysOp::Continuous { .. } | SysOp::ContinuousSplit { .. } | SysOp::Write { .. } => 1,
        #[cfg(sleep_regdma_wait_ops)]
        SysOp::Wait { .. } => 1,
        SysOp::Uart { .. } => UART_NODE_COUNT,
        SysOp::Systimer { .. } => SYSTIMER_NODE_COUNT,
    }
}

/// RAM buffer words consumed by one [`SysOp`].
const fn op_words(op: &SysOp) -> usize {
    match op {
        SysOp::Continuous { count, .. } | SysOp::ContinuousSplit { count, .. } => *count as usize,
        SysOp::Uart { .. } => UART_WORDS,
        SysOp::Systimer { .. } => SYSTIMER_CONT_WORDS,
        SysOp::Write { .. } => 0,
        #[cfg(sleep_regdma_wait_ops)]
        SysOp::Wait { .. } => 0,
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

const ST_UNIT_OP_UPDATE: u32 = 1 << 30;
const ST_UNIT_OP_VALUE_VALID: u32 = 1 << 29;
const ST_UNIT_LOAD: u32 = 1 << 0;
const ST_COMP_LOAD: u32 = 1 << 0;
const ST_TARGET_CONF_PERIOD_MODE: u32 = 1 << 30;
const ST_TARGETS_LEN: u32 = span!(SYSTIMER, [trgt(0).hi()]..=[target_conf(2)]);
// Nodes `build_systimer_seq` emits: four per counter unit (latch, poll, read,
// load), the target values, one load per comparator, a clear and set to re-arm
// period mode on target0/1 plus a clear on target2, then `conf` and `int_ena`.
const SYSTIMER_NODE_COUNT: usize = 2 * 4 + 1 + 3 + 2 * 2 + 1 + 1 + 1;
// SysTimer CONTINUOUS words: unit0/1 value (2+2), the targets, conf, int_ena.
const SYSTIMER_CONT_WORDS: usize = 2 + 2 + ST_TARGETS_LEN as usize + 1 + 1;

/// Build the SysTimer retention sequence for the timer at `base` into `nodes`,
/// drawing its RAM from `buf_base`.
fn build_systimer_seq(base: u32, nodes: &mut [RegdmaLink], buf_base: *mut u32, start_word: usize) {
    let st_conf = base + reg_off!(SYSTIMER, conf());
    let st_unit0_op = base + reg_off!(SYSTIMER, unit_op(0));
    let st_unit1_op = base + reg_off!(SYSTIMER, unit_op(1));
    let st_unit0_load_hi = base + reg_off!(SYSTIMER, unitload(0).hi());
    let st_unit1_load_hi = base + reg_off!(SYSTIMER, unitload(1).hi());
    let st_target0_hi = base + reg_off!(SYSTIMER, trgt(0).hi());
    let st_target0_conf = base + reg_off!(SYSTIMER, target_conf(0));
    let st_target1_conf = base + reg_off!(SYSTIMER, target_conf(1));
    let st_target2_conf = base + reg_off!(SYSTIMER, target_conf(2));
    let st_unit0_value_hi = base + reg_off!(SYSTIMER, unit_value(0).hi());
    let st_unit1_value_hi = base + reg_off!(SYSTIMER, unit_value(1).hi());
    let st_comp0_load = base + reg_off!(SYSTIMER, comp_load(0));
    let st_comp1_load = base + reg_off!(SYSTIMER, comp_load(1));
    let st_comp2_load = base + reg_off!(SYSTIMER, comp_load(2));
    let st_unit0_load = base + reg_off!(SYSTIMER, unit_load(0));
    let st_unit1_load = base + reg_off!(SYSTIMER, unit_load(1));
    let st_int_ena = base + reg_off!(SYSTIMER, int_ena());

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
        nodes[node] = RegdmaLink::write(op, ST_UNIT_OP_UPDATE, ST_UNIT_OP_UPDATE, false, true);
        node += 1;
        nodes[node] = RegdmaLink::wait(
            op,
            ST_UNIT_OP_VALUE_VALID,
            ST_UNIT_OP_VALUE_VALID,
            false,
            true,
        );
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
        nodes[node] = RegdmaLink::write(target, 0, ST_TARGET_CONF_PERIOD_MODE, true, false);
        node += 1;
        nodes[node] = RegdmaLink::write(
            target,
            ST_TARGET_CONF_PERIOD_MODE,
            ST_TARGET_CONF_PERIOD_MODE,
            true,
            false,
        );
        node += 1;
    }
    nodes[node] = RegdmaLink::write(st_target2_conf, 0, ST_TARGET_CONF_PERIOD_MODE, true, false);
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

/// Caller-owned backing store for the TOP-domain system-peripheral register set.
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
/// engine.
// `pau_hal_set_regdma_wait_timeout`, ESP-IDF `PAU_REGDMA_LINK_WAIT_*`
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
/// copies registers into RAM, else restores RAM back into registers.
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
fn enable_top_retention(mem: &mut SystemRetentionMemory) {
    regdma_clock_and_timeout();

    let head = arm_link(sys_periph::build_link(&mut mem.nodes, &mut mem.buf));
    fence(Ordering::SeqCst);
    PAU::regs()
        .regdma_link_0_addr()
        .write(|w| unsafe { w.bits(head) });

    let pmu = PMU::regs();
    if cfg!(sleep_regdma_sw_trigger) {
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

/// Restore the TOP-domain peripherals on wake for `sleep_regdma_sw_trigger`
/// chips, whose PAU lost its own configuration with the `TOP` power-down.
///
/// Re-enables the regDMA bus clock, re-programs entry link 0 (the linked list in
/// caller memory survived in RAM) and software-triggers the restore. Must run
/// before any powered-down `TOP` peripheral is touched. No-op on chips that
/// restore in hardware.
// `#[ram]`: runs on the wake path before the flash SPI controller is restored.
#[ram]
pub(crate) fn restore_top_retention(mem: &mut SystemRetentionMemory) {
    if !cfg!(sleep_regdma_sw_trigger) {
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
    /// opt-in choices, caller memory and active power-domain locks.
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

// Chip-agnostic interpreter that expands a chip's `OPS` program into PAU regDMA
// nodes (in retention-priority order, system clock first).
// Mirrors ESP-IDF's `SLEEP_RETENTION_MODULE_SYS_PERIPH` + `..._CLOCK_SYSTEM`
// (`soc/<chip>/system_retention_periph.c`, `.../sleep_clock.c`).
mod sys_periph {
    use super::{
        HEAD_LENGTH_MASK,
        RegdmaLink,
        SYSTIMER_NODE_COUNT,
        SysOp,
        UART_NODE_COUNT,
        UART_OPS,
        UART_WORDS,
        build_periph_seq,
        build_systimer_seq,
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
                SysOp::Continuous { addr, count } => {
                    let mem = unsafe { buf_base.add(word) } as u32;
                    nodes[node] = RegdmaLink::continuous(addr(), mem, count);
                    word += count as usize;
                    node += 1;
                }
                SysOp::ContinuousSplit {
                    backup,
                    restore,
                    count,
                } => {
                    let mem = unsafe { buf_base.add(word) } as u32;
                    nodes[node] = RegdmaLink::continuous_split(backup(), restore(), mem, count);
                    word += count as usize;
                    node += 1;
                }
                SysOp::Write { addr, value, mask } => {
                    nodes[node] = RegdmaLink::write(addr(), value, mask, true, false);
                    node += 1;
                }
                #[cfg(sleep_regdma_wait_ops)]
                SysOp::Wait { addr, value, mask } => {
                    nodes[node] = RegdmaLink::wait(addr(), value, mask, true, false);
                    node += 1;
                }
                SysOp::Uart { base } => {
                    let mem = unsafe { buf_base.add(word) };
                    build_periph_seq(
                        base(),
                        UART_OPS,
                        &mut nodes[node..node + UART_NODE_COUNT],
                        mem,
                    );
                    word += UART_WORDS;
                    node += UART_NODE_COUNT;
                }
                SysOp::Systimer { base } => {
                    build_systimer_seq(
                        base(),
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
