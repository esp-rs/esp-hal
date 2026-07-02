//! Register DMA (regDMA) based register retention (ESP32-C6).
//!
//! The PAU's regDMA engine backs peripheral registers up to RAM and restores
//! them while the `TOP` domain is powered down in light sleep. It walks a linked
//! list of [`RegdmaLink`] nodes over PAU entry link 0 with no CPU involvement.
//! `sys_periph` builds the core register set into a caller-owned
//! [`SystemRetentionMemory`]; [`enable_top_retention`] chains it with any opt-in
//! peripheral entries before arming the link. Without a `SystemRetentionMemory`
//! the `TOP` domain is only clock-gated.
//!
//! References (ESP-IDF `v5.4`): `soc/regdma.h`, `hal/esp32c6/pau_ll.h`,
//! `hal/esp32c6/pau_hal.c`, `esp_hw_support/port/pau_regdma.c`.

use core::{
    marker::PhantomData,
    ptr::NonNull,
    sync::atomic::{Ordering, fence},
};

use esp_sync::NonReentrantMutex;

use crate::{
    peripherals::{PAU, PCR, PMU},
    rtc_cntl::power_domain::{Domain, PowerDomainLock},
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
    /// Back up/restore a run of registers via a RAM buffer, where a 4-word
    /// bitmap selects which registers in the window are actually transferred
    /// (skipping e.g. read-only status/FIFO registers interspersed in a block).
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
    /// (bit `i` = register at `reg + i * 4`) from `reg` into `storage`, skipping
    /// interspersed read-only/FIFO registers.
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
// the opt-in `UartRetentionMemory`. ESP-IDF v5.4 `uart_periph.c`
// `UART_SLEEP_RETENTION_ENTRIES`, `uart_reg.h`.
const UART_INT_ENA_OFF: u32 = 0x0C; // UART_INT_ENA_REG
const UART_REG_UPDATE_OFF: u32 = 0x98; // UART_REG_UPDATE_REG
const UART_REG_UPDATE: u32 = 1 << 0;
/// Registers retained (set bits in [`UART_REGS_MAP`]).
const UART_RETENTION_REGS_CNT: u32 = 21;
/// `uart_regs_map[4]`: config registers in the INT_ENA..ID window.
const UART_REGS_MAP: [u32; 4] = [0x007f_ff6d, 0x0000_0010, 0, 0];
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

// I2C config-register retention. ESP-IDF v5.4 `i2c_periph.c`
// `i2c0_regs_retention`, `i2c_reg.h`. Config registers are shadowed, so restore
// pulses the FSM reset then requests a config update and waits for it to latch.
const I2C_SCL_LOW_PERIOD_OFF: u32 = 0x00; // I2C_SCL_LOW_PERIOD_REG: ADDR_MAP window base
const I2C_CTR_OFF: u32 = 0x04; // I2C_CTR_REG
const I2C_FSM_RST: u32 = 1 << 10; // I2C_FSM_RST (value == mask)
const I2C_CONF_UPGATE: u32 = 1 << 11; // I2C_CONF_UPGATE (value == mask)
/// Registers retained (set bits in [`I2C_REGS_MAP`]).
const I2C_RETENTION_REGS_CNT: u32 = 18;
/// `i2c0_regs_map[4]`: config registers in the `SCL_LOW_PERIOD..SCL_STRETCH_CONF` window.
const I2C_REGS_MAP: [u32; 4] = [0xc03f_345b, 0x3, 0, 0];
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

// GPSPI2 config-register retention. ESP-IDF v5.4 `spi_periph.c`
// `spi2_regs_retention`, `spi_reg.h`. IDF's restore-time re-set of the
// TRANS_DONE/DMA_SEG_TRANS_DONE interrupt bits is omitted: esp-hal only powers
// `TOP` down on an idle bus, so it would only inject a spurious completion IRQ.
const SPI_CMD_OFF: u32 = 0x00; // SPI_CMD_REG: ADDR_MAP window base
/// Registers retained (set bits in [`SPI_REGS_MAP`]).
const SPI_RETENTION_REGS_CNT: u32 = 12;
/// `spi_regs_map[4]`: config registers in the `CMD..SLAVE` window.
const SPI_REGS_MAP: [u32; 4] = [0x0000_31ff, 0x0100_0000, 0, 0];
/// A single ADDR_MAP over the config registers.
const SPI_NODE_COUNT: usize = 1;

/// Build the SPI retention sequence for `base` into `nodes`, backing the
/// registers up into `storage`.
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
            // SAFETY: `tail` is the last node of the previous segment.
            unsafe {
                (*tail).next = seg[0].addr();
                (*tail).head &= !HEAD_EOF_BIT;
            }
            tail = link_internal(seg);
            current = next;
        }
    });

    // Terminate the final node.
    // SAFETY: `tail` points at the last node of the last segment.
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
    "Caller-owned backing store retaining one UART's config registers across a \
`TOP` power-down. Passed to \
[`Uart::with_retention_memory`](crate::uart::Uart::with_retention_memory); the \
console/log UART is retained automatically."
);

peripheral_retention_memory!(
    I2cRetentionMemory,
    I2C_NODE_COUNT,
    I2C_RETENTION_REGS_CNT as usize,
    build_i2c_seq,
    "Caller-owned backing store retaining one I2C's config registers across a \
`TOP` power-down. Passed to \
[`I2c::with_retention_memory`](crate::i2c::master::I2c::with_retention_memory). \
See [`UartRetentionMemory`]."
);

peripheral_retention_memory!(
    SpiRetentionMemory,
    SPI_NODE_COUNT,
    SPI_RETENTION_REGS_CNT as usize,
    build_spi_seq,
    "Caller-owned backing store retaining one SPI's config registers across a \
`TOP` power-down. Passed to \
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

/// A `TOP`-domain peripheral driver's power-management state: either active and
/// holding a [`PowerDomainLock`] (keeping `TOP` powered so it can't lose state,
/// without preventing sleep), or retained (the lock is dropped and regDMA
/// save/restores its config across a `TOP` power-down from `'d`-borrowed memory).
/// Stored in the driver, so the user never juggles a separate guard.
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

/// Caller-owned backing store for the ESP32-C6 TOP-domain system-peripheral
/// register set (PCR, interrupt matrix, HP system, TEE/APM, IO MUX, GPIO matrix,
/// flash SPI mem, console UART and SysTimer).
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
    nodes: [RegdmaLink; sys_periph::NODE_COUNT],
    buf: [u32; sys_periph::BUF_WORDS],
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
            nodes: [RegdmaLink::EMPTY; sys_periph::NODE_COUNT],
            buf: [0; sys_periph::BUF_WORDS],
        }
    }
}

/// Arm regDMA retention of the TOP-domain peripherals for the upcoming light
/// sleep: program PAU entry link 0 and enable the backup phases.
///
/// Must be called after the PMU power config (which resets the backup-enable
/// bits) and before the sleep request. Rebuilds the chain from the live registry
/// every time, so the PAU never walks a deregistered entry. Mirrors ESP-IDF's
/// link setup + `pmu_sleep_enable_regdma_backup()` (active/sleep phases only).
pub(crate) fn enable_top_retention(mem: &mut SystemRetentionMemory) {
    // pau_ll_enable_bus_clock: enable the regDMA bus clock and release its reset.
    PCR::regs().regdma_conf().modify(|_, w| {
        w.regdma_clk_en().set_bit();
        w.regdma_rst_en().clear_bit()
    });
    // pau_hal_set_regdma_wait_timeout: bound WAIT polling so a never-satisfied
    // condition can't hang the engine (ESP-IDF PAU_REGDMA_LINK_WAIT_*).
    PAU::regs().regdma_bkp_conf().modify(|_, w| unsafe {
        w.link_tout_thres().bits(1000);
        w.read_interval().bits(32)
    });

    // Build the SYS_PERIPH list plus opt-in entries and program it as link 0.
    let head = arm_link(sys_periph::build_link(&mut mem.nodes, &mut mem.buf));
    fence(Ordering::SeqCst);
    PAU::regs()
        .regdma_link_0_addr()
        .write(|w| unsafe { w.bits(head) });

    // pmu_sleep_enable_regdma_backup: back up active->sleep, restore sleep->active.
    let pmu = PMU::regs();
    pmu.hp_sleep_backup()
        .modify(|_, w| w.hp_active2sleep_backup_en().set_bit());
    pmu.hp_active_backup()
        .modify(|_, w| w.hp_sleep2active_backup_en().set_bit());
}

/// ESP32-C6 TOP-domain system-peripheral retention link: the register regions
/// for the core system peripherals lost when `TOP` powers down, mirroring
/// ESP-IDF's `SLEEP_RETENTION_MODULE_SYS_PERIPH` + `..._CLOCK_SYSTEM` in
/// retention-priority order (system clock first).
///
/// References (ESP-IDF `v5.4`): `soc/esp32c6/system_retention_periph.c`,
/// `esp_hw_support/.../sleep_clock.c`, `esp_hw_support/sleep_system_peripheral.c`.
mod sys_periph {
    use super::{
        HEAD_LENGTH_MASK,
        RegdmaLink,
        UART_NODE_COUNT,
        UART_RETENTION_REGS_CNT,
        build_uart_seq,
    };

    /// TEE mode-control register, rewritten early on restore to unlock access.
    const TEE_M4_MODE_CTRL_REG: u32 = 0x6009_8010;

    /// A run of `count` consecutive 32-bit registers starting at `base`.
    struct ContRegion {
        base: u32,
        count: u32,
    }

    /// Continuous register regions to retain, in retention-priority order. The
    /// sizing end register is noted per region; `count = ((end - base) / 4) + 1`.
    const CONT_REGIONS: &[ContRegion] = &[
        // PRI_0 - system clock/reset (PCR)
        ContRegion {
            base: 0x6009_6000,
            count: 79,
        }, // PCR base ..= PCR_SRAM_POWER_CONF_REG (+0x138)
        ContRegion {
            base: 0x6009_6FF0,
            count: 1,
        }, // PCR_RESET_EVENT_BYPASS_REG
        // PRI_4 - TEE/APM
        ContRegion {
            base: 0x6009_9000,
            count: 68,
        }, // HP_APM base ..= HP_APM_CLOCK_GATE_REG (+0x10c)
        ContRegion {
            base: 0x6009_8000,
            count: 33,
        }, // TEE base ..= TEE_CLOCK_GATE_REG (+0x80)
        // PRI_5 - interrupt matrix + HP system
        ContRegion {
            base: 0x6001_0000,
            count: 81,
        }, // INTMTX base ..= INTMTX_CORE0_CLOCK_GATE_REG (+0x140)
        ContRegion {
            base: 0x6009_5000,
            count: 18,
        }, // HP_SYSTEM base ..= HP_SYSTEM_MEM_TEST_CONF_REG (+0x44)
        // PRI_6 - IO MUX + GPIO matrix
        ContRegion {
            base: 0x6009_0000,
            count: 32,
        }, // IO_MUX base ..= IO_MUX_GPIO30_REG (+0x7c)
        ContRegion {
            base: 0x6009_1554,
            count: 35,
        }, // GPIO_FUNC0_OUT_SEL ..= GPIO_FUNC34_OUT_SEL
        ContRegion {
            base: 0x6009_114C,
            count: 127,
        }, // GPIO_STATUS_NEXT ..= GPIO_FUNC124_IN_SEL
        ContRegion {
            base: 0x6009_1000,
            count: 64,
        }, // GPIO base ..= GPIO_PIN34_REG (+0xfc)
        // PRI_6 - Flash SPI mem (SPIMEM1 then SPIMEM0). MMU content/index
        // registers are intentionally excluded (see ESP-IDF note).
        ContRegion {
            base: 0x6000_3000,
            count: 55,
        }, // SPIMEM1 base ..= SPI_MEM_SPI_SMEM_DDR (+0xd8)
        ContRegion {
            base: 0x6000_3100,
            count: 41,
        }, // SPIMEM1 FMEM_PMS0_ATTR ..= SMEM_AC (+0x1a0)
        ContRegion {
            base: 0x6000_3200,
            count: 1,
        }, // SPIMEM1 CLOCK_GATE
        ContRegion {
            base: 0x6000_3384,
            count: 31,
        }, // SPIMEM1 MMU_POWER_CTRL ..= DATE (+0x3fc)
        ContRegion {
            base: 0x6000_2000,
            count: 55,
        }, // SPIMEM0 base ..= SPI_MEM_SPI_SMEM_DDR
        ContRegion {
            base: 0x6000_2100,
            count: 41,
        }, // SPIMEM0 FMEM_PMS0_ATTR ..= SMEM_AC
        ContRegion {
            base: 0x6000_2200,
            count: 1,
        }, // SPIMEM0 CLOCK_GATE
        ContRegion {
            base: 0x6000_2384,
            count: 31,
        }, // SPIMEM0 MMU_POWER_CTRL ..= DATE
    ];

    /// [`CONT_REGIONS`] index where TEE/APM (PRI_4) starts; the PRI_2 TEE WRITE
    /// node is inserted just before it.
    const TEE_APM_START: usize = 2;

    /// [`CONT_REGIONS`] index where IO MUX/GPIO (PRI_6) starts; the console-UART
    /// (PRI_5) nodes are inserted just before it.
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

    /// Console UART0 base, retained automatically via [`build_uart_seq`].
    const UART0_BASE: u32 = 0x6000_0000;

    // SysTimer. Offsets/masks from ESP-IDF v5.4 `systimer_reg.h`; sequence from
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
    /// SysTimer CONTINUOUS words: unit0/1 value (2+2), targets (9), conf, int_ena.
    const SYSTIMER_CONT_WORDS: usize = 2 + 2 + ST_TARGETS_LEN as usize + 1 + 1;

    /// One node per continuous region + TEE WRITE + console UART + SysTimer.
    pub(super) const NODE_COUNT: usize =
        CONT_REGIONS.len() + 1 + UART_NODE_COUNT + SYSTIMER_NODE_COUNT;
    pub(super) const BUF_WORDS: usize =
        total_words() + UART_RETENTION_REGS_CNT as usize + SYSTIMER_CONT_WORDS;

    // Every region count must fit the 10-bit node `length` field.
    const _: () = {
        let mut i = 0;
        while i < CONT_REGIONS.len() {
            core::assert!(CONT_REGIONS[i].count <= HEAD_LENGTH_MASK);
            i += 1;
        }
    };

    /// (Re)build the SYS_PERIPH retention list into `nodes`/`buf` and return the
    /// filled node slice.
    pub(super) fn build_link<'a>(
        nodes: &'a mut [RegdmaLink; NODE_COUNT],
        buf: &mut [u32; BUF_WORDS],
    ) -> &'a mut [RegdmaLink] {
        let buf_base = buf.as_mut_ptr();

        let mut node = 0;
        let mut word = 0;

        // PRI_0: system clock (PCR).
        for region in &CONT_REGIONS[..TEE_APM_START] {
            let mem = unsafe { buf_base.add(word) } as u32;
            nodes[node] = RegdmaLink::continuous(region.base, mem, region.count);
            word += region.count as usize;
            node += 1;
        }

        // PRI_2: restore-only WRITE clearing TEE_M4_MODE_CTRL so the TEE/APM
        // restore can write freely.
        nodes[node] = RegdmaLink::write(TEE_M4_MODE_CTRL_REG, 0, 0xFFFF_FFFF, true, false);
        node += 1;

        // PRI_4/5: TEE/APM, interrupt matrix, HP system.
        for region in &CONT_REGIONS[TEE_APM_START..IOMUX_START] {
            let mem = unsafe { buf_base.add(word) } as u32;
            nodes[node] = RegdmaLink::continuous(region.base, mem, region.count);
            word += region.count as usize;
            node += 1;
        }

        // PRI_5: console UART0 (same sequence as the opt-in path).
        let mem = unsafe { buf_base.add(word) } as u32;
        build_uart_seq(UART0_BASE, &mut nodes[node..node + UART_NODE_COUNT], mem);
        word += UART_RETENTION_REGS_CNT as usize;
        node += UART_NODE_COUNT;

        // PRI_6: IO MUX, GPIO matrix, SPI mem.
        for region in &CONT_REGIONS[IOMUX_START..] {
            let mem = unsafe { buf_base.add(word) } as u32;
            nodes[node] = RegdmaLink::continuous(region.base, mem, region.count);
            word += region.count as usize;
            node += 1;
        }

        // PRI_6: SysTimer. Backup latches each unit's counter (UPDATE + wait for
        // VALUE_VALID) and reads it; restore loads it back and triggers a load.
        // The value is read from VALUE_HI/LO but restored into LOAD_HI/LO, hence
        // the split backup/restore addresses.
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
        // Re-arm period mode: clear+set for target0/1, clear for target2.
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
