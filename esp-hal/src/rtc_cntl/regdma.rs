//! REGDMA link lists, which save registers before light sleep and restore them after it.
//!
//! A power domain that loses power in light sleep also loses its registers. On the way into the
//! sleep and out of it, the PMU starts the REGDMA, which walks a list of nodes. On the way in, each
//! node copies registers to memory, and on the way out, back from memory.
//!
//! A driver describes its registers with [`LinkConfig`]s, builds them into the nodes of a
//! [`Module`] with [`create`], and joins the module into the list with [`attach`]. The nodes stay
//! in the memory of the driver.
//!
//! The REGDMA has several entries, and a node belongs to the entries in its owner mask. The PMU
//! runs [`SLEEP_ENTRY`] between the active and the sleep state. The list starts at a branch node,
//! which has a `next` pointer for each entry.

use core::{cell::UnsafeCell, ptr::NonNull};

use esp_sync::NonReentrantMutex;
use portable_atomic::{AtomicBool, Ordering};

use crate::peripherals::PMU;

/// The number of REGDMA entries.
const ENTRIES: usize = property!("sleep.regdma_link_count");

/// The entry that the PMU runs between the active and the sleep state.
pub const SLEEP_ENTRY: u32 = 1 << 0;

#[cfg(regdma_bt_needs_pll)]
const EXTRA_ENTRY_INDEX: usize = 3;

/// The entry that software runs with [`run_extra_link`].
#[cfg(regdma_bt_needs_pll)]
pub const EXTRA_ENTRY: u32 = 1 << EXTRA_ENTRY_INDEX;

/// The entries that [`create`] builds nodes for.
const USED_ENTRIES: &[usize] = &[
    0,
    #[cfg(regdma_bt_needs_pll)]
    EXTRA_ENTRY_INDEX,
];

/// The owner bits of [`USED_ENTRIES`].
const USED_OWNERS: u32 = cfg_select! {
    regdma_bt_needs_pll => SLEEP_ENTRY | EXTRA_ENTRY,
    _ => SLEEP_ENTRY,
};

// A node is a statistics word, the head, then the body of its mode. Every link points to the head.
// See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/soc/include/soc/regdma.h#L138-L204>.
const MODE_CONTINUOUS: u32 = 0;
const MODE_ADDR_MAP: u32 = 1;
const MODE_WRITE: u32 = 2;
const MODE_WAIT: u32 = 3;

const HEAD_LENGTH_MASK: u32 = 0x3ff;
const HEAD_MODE_SHIFT: u32 = 16;
const HEAD_BRANCH: u32 = 1 << 28;
const HEAD_SKIP_RESTORE: u32 = 1 << 29;
const HEAD_SKIP_BACKUP: u32 = 1 << 30;
const HEAD_EOF: u32 = 1 << 31;

const fn head(length: u32, mode: u32) -> u32 {
    (length & HEAD_LENGTH_MASK) | (mode << HEAD_MODE_SHIFT)
}

const fn mode_of(head: u32) -> u32 {
    (head >> HEAD_MODE_SHIFT) & 0xf
}

/// The head of a wait that the REGDMA skips in both directions.
const SKIPPED_WAIT: u32 = head(0, MODE_WAIT) | HEAD_SKIP_BACKUP | HEAD_SKIP_RESTORE;

/// One node of a REGDMA link list, and the entries that it belongs to.
///
/// The layout is the `sleep_retention_entries_config_t` of ESP-IDF, so the radio blobs can hand
/// out their tables in this form.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LinkConfig {
    head: u32,
    /// `next`, then the fields of the mode: `backup`, `restore`, `mem` and the address map for a
    /// copy, `backup`, `value` and `mask` for a write or a wait.
    body: [u32; 8],
    id: u32,
    owner: u32,
}

const _: () = assert!(core::mem::size_of::<LinkConfig>() == 44);

impl LinkConfig {
    /// Copies `length` words from `backup` to memory before the sleep, and from memory to
    /// `restore` after it.
    pub const fn continuous(id: u16, backup: u32, restore: u32, length: u16, owner: u32) -> Self {
        Self {
            head: head(length as u32, MODE_CONTINUOUS),
            body: [0, backup, restore, 0, 0, 0, 0, 0],
            id: id as u32,
            owner,
        }
    }

    /// Writes `value` to the bits in `mask` of the register at `address` after the sleep.
    pub const fn write(id: u16, address: u32, value: u32, mask: u32, owner: u32) -> Self {
        Self {
            head: head(0, MODE_WRITE) | HEAD_SKIP_BACKUP,
            body: [0, address, value, mask, 0, 0, 0, 0],
            id: id as u32,
            owner,
        }
    }

    const fn mode(&self) -> u32 {
        mode_of(self.head)
    }

    const fn length(&self) -> usize {
        (self.head & HEAD_LENGTH_MASK) as usize
    }

    /// The words of the node, with the buffer that it copies to.
    const fn words(&self) -> usize {
        // The statistics word, the head, then the body.
        match self.mode() {
            MODE_CONTINUOUS => 2 + 4 + self.length(),
            MODE_ADDR_MAP => 2 + 8 + self.length(),
            _ => 2 + 4,
        }
    }
}

/// Returns the bytes that [`create`] takes for the nodes of `configs`.
pub const fn links_size(configs: &[LinkConfig]) -> usize {
    let mut words = 0;
    let mut i = 0;
    while i < configs.len() {
        let entries = (configs[i].owner & USED_OWNERS).count_ones() as usize;
        words += configs[i].words() * entries;
        i += 1;
    }
    words * 4
}

/// A part of the modem domain, with its place in the list.
///
/// The REGDMA restores the modules in the order of this enum.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Module {
    /// The clock and reset registers of the modem.
    ClockModem,
    /// The Wi-Fi MAC.
    WifiMac,
    /// The Wi-Fi baseband.
    WifiBb,
    /// The PHY front end.
    PhyFe,
    /// The Bluetooth baseband.
    BtBb,
    /// The Bluetooth LE MAC.
    BleMac,
}

const MODULES: usize = Module::BleMac as usize + 1;

/// Memory that [`create`] builds nodes in.
pub struct LinkMemory {
    next: *mut u32,
    end: *mut u32,
}

impl LinkMemory {
    /// Takes `size` bytes from `start` for nodes.
    ///
    /// # Safety
    ///
    /// The memory must be 4-byte aligned, in internal RAM that keeps its contents in light sleep,
    /// and valid for writes. Nothing else may use it while a module in it exists.
    pub unsafe fn new(start: NonNull<u8>, size: usize) -> Self {
        let next = start.as_ptr().cast::<u32>();
        Self {
            next,
            end: unsafe { next.add(size / 4) },
        }
    }

    fn take(&mut self, words: usize) -> Option<*mut u32> {
        let available = (self.end as usize - self.next as usize) / 4;
        if words > available {
            return None;
        }
        let node = self.next;
        self.next = unsafe { node.add(words) };
        Some(node)
    }
}

/// The memory has no room for the nodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OutOfMemory;

/// The first and the last node of a module in one entry.
#[derive(Clone, Copy)]
struct Chain {
    first: NonNull<u32>,
    last: NonNull<u32>,
}

// SAFETY: The nodes are plain memory, which only the registry touches, under its lock.
unsafe impl Send for Chain {}

/// The chains of a module, one for each of [`USED_ENTRIES`].
type Links = [Option<Chain>; USED_ENTRIES.len()];

struct Registry {
    links: [Option<Links>; MODULES],
    attached: u8,
}

static REGISTRY: NonReentrantMutex<Registry> = NonReentrantMutex::new(Registry {
    links: [None; MODULES],
    attached: 0,
});

/// A node that the REGDMA reads, outside the memory of any driver.
#[repr(C, align(4))]
struct StaticNode<const WORDS: usize>(UnsafeCell<[u32; WORDS]>);

// SAFETY: Only the registry writes the node, under its lock.
unsafe impl<const WORDS: usize> Sync for StaticNode<WORDS> {}

impl<const WORDS: usize> StaticNode<WORDS> {
    /// The address that the REGDMA and the `next` pointers use, the head word.
    fn link(&self) -> *mut u32 {
        unsafe { self.0.get().cast::<u32>().add(1) }
    }
}

/// The first node: a wait that the REGDMA skips, with a `next` pointer for each entry.
static HEAD: StaticNode<{ 2 + ENTRIES + 3 }> = StaticNode(UnsafeCell::new({
    let mut words = [0; 2 + ENTRIES + 3];
    words[1] = SKIPPED_WAIT | HEAD_BRANCH;
    words
}));

/// The last node of every entry: a wait that the REGDMA skips, and the end of the list.
static TAIL: StaticNode<6> = StaticNode(UnsafeCell::new([0, SKIPPED_WAIT | HEAD_EOF, 0, 0, 0, 0]));

/// Whether the modem domain loses power in light sleep.
static MODEM_POWER_DOWN: AtomicBool = AtomicBool::new(false);

/// Builds the nodes of `module` from `configs` in `memory`, without joining them into the list.
///
/// The nodes belong to `module` until [`destroy`].
///
/// # Errors
///
/// Returns [`OutOfMemory`] if `memory` has no room for the nodes.
///
/// # Panics
///
/// Panics if `module` exists already.
///
/// # Safety
///
/// `configs` must describe registers that the REGDMA can read and write.
pub unsafe fn create(
    module: Module,
    configs: &[&[LinkConfig]],
    memory: &mut LinkMemory,
) -> Result<(), OutOfMemory> {
    let mut links: Links = [None; USED_ENTRIES.len()];
    for (slot, &entry) in USED_ENTRIES.iter().enumerate() {
        let mut chain: Option<Chain> = None;
        for config in configs.iter().flat_map(|configs| configs.iter()) {
            if config.owner & (1 << entry) == 0 {
                continue;
            }
            let link = build(config, memory)?;
            append(&mut chain, link);
        }
        links[slot] = chain;
    }

    REGISTRY.with(|registry| {
        let links_of_module = &mut registry.links[module as usize];
        assert!(links_of_module.is_none(), "{module:?} exists already");
        *links_of_module = Some(links);
    });
    Ok(())
}

/// Builds one node, and returns its link, the address of its head.
fn build(config: &LinkConfig, memory: &mut LinkMemory) -> Result<NonNull<u32>, OutOfMemory> {
    let words = config.words();
    let node = memory.take(words).ok_or(OutOfMemory)?;
    unsafe {
        let link = node.add(1);
        node.write_volatile(config.id << 16);
        link.write_volatile(config.head & !(HEAD_BRANCH | HEAD_EOF));
        let body = link.add(1);
        let body_words = match config.mode() {
            MODE_ADDR_MAP => 8,
            _ => 4,
        };
        for (i, &word) in config.body[..body_words].iter().enumerate() {
            body.add(i).write_volatile(word);
        }
        if matches!(config.mode(), MODE_CONTINUOUS | MODE_ADDR_MAP) {
            let buffer = body.add(body_words);
            body.add(3).write_volatile(buffer as u32);
        }
        Ok(NonNull::new_unchecked(link))
    }
}

fn append(chain: &mut Option<Chain>, link: NonNull<u32>) {
    match chain {
        Some(chain) => {
            set_next(chain.last.as_ptr(), link.as_ptr());
            chain.last = link;
        }
        None => {
            *chain = Some(Chain {
                first: link,
                last: link,
            })
        }
    }
}

/// Points a node that is not a branch to `next`.
fn set_next(link: *mut u32, next: *mut u32) {
    unsafe { link.add(1).write_volatile(next as u32) };
}

fn next_of(link: *mut u32) -> *mut u32 {
    unsafe { link.add(1).read_volatile() as *mut u32 }
}

/// Removes the nodes of `module`, and takes them out of the list. The memory is free afterwards.
pub fn destroy(module: Module) {
    REGISTRY.with(|registry| {
        registry.links[module as usize] = None;
        registry.attached &= !(1 << module as u8);
        join(registry);
    });
}

/// Joins the nodes of `module` into the list.
///
/// # Panics
///
/// Panics if `module` does not exist.
pub fn attach(module: Module) {
    REGISTRY.with(|registry| {
        assert!(
            registry.links[module as usize].is_some(),
            "{module:?} does not exist"
        );
        registry.attached |= 1 << module as u8;
        join(registry);
    });
}

/// Takes the nodes of `module` out of the list, and keeps them.
pub fn detach(module: Module) {
    REGISTRY.with(|registry| {
        registry.attached &= !(1 << module as u8);
        join(registry);
    });
}

/// Returns whether the nodes of `module` are in the list.
pub fn is_attached(module: Module) -> bool {
    REGISTRY.with(|registry| registry.attached & (1 << module as u8) != 0)
}

/// Links the attached modules in the order of [`Module`], for each entry.
fn join(registry: &Registry) {
    let head = HEAD.link();
    let tail = TAIL.link();
    for entry in 0..ENTRIES {
        unsafe { head.add(1 + entry).write_volatile(tail as u32) };
    }

    for (slot, &entry) in USED_ENTRIES.iter().enumerate() {
        let mut next = tail;
        for module in (0..MODULES).rev() {
            if registry.attached & (1 << module) == 0 {
                continue;
            }
            let Some(chain) = registry.links[module].and_then(|links| links[slot]) else {
                continue;
            };
            set_next(chain.last.as_ptr(), next);
            next = chain.first.as_ptr();
        }
        unsafe { head.add(1 + entry).write_volatile(next as u32) };
    }
}

/// Returns the first node with `id`, from every module, attached or not.
pub fn find_link_by_id(id: u16) -> Option<NonNull<u32>> {
    REGISTRY.with(|registry| {
        for links in registry.links.iter().flatten() {
            for chain in links.iter().flatten() {
                let mut link = chain.first.as_ptr();
                loop {
                    let stat = unsafe { link.sub(1).read_volatile() };
                    if stat >> 16 == id as u32 {
                        return NonNull::new(link);
                    }
                    if link == chain.last.as_ptr() {
                        break;
                    }
                    link = next_of(link);
                }
            }
        }
        None
    })
}

/// Changes the value and the mask of a write or wait node.
///
/// # Safety
///
/// `link` must come from [`find_link_by_id`], and its module must still exist.
pub unsafe fn set_write_wait_content(link: NonNull<u32>, value: u32, mask: u32) {
    let link = link.as_ptr();
    unsafe {
        if matches!(mode_of(link.read_volatile()), MODE_WRITE | MODE_WAIT) {
            link.add(3).write_volatile(value);
            link.add(4).write_volatile(mask);
        }
    }
}

/// Lets light sleep power the modem domain down, with the REGDMA that saves and restores the
/// attached modules.
///
/// Call this with `true` only while the modules hold every register of the modem domain that
/// a radio needs after the sleep.
pub fn set_modem_power_down(enabled: bool) {
    REGISTRY.with(|registry| {
        if enabled {
            // The `next` pointers of the head stay 0 until the first join.
            join(registry);
            init_pau();
        } else {
            set_backup(false);
        }
        MODEM_POWER_DOWN.store(enabled, Ordering::Relaxed);
    });
}

/// Returns whether light sleep powers the modem domain down.
pub(crate) fn modem_power_down() -> bool {
    MODEM_POWER_DOWN.load(Ordering::Relaxed)
}

/// Lets the PMU start the REGDMA around the next sleep, only if the sleep powers the modem domain
/// down.
pub(crate) fn prepare_sleep(light: bool) {
    set_backup(light && modem_power_down());
}

/// Lets the PMU start the REGDMA between the active and the sleep state.
fn set_backup(enabled: bool) {
    let pmu = PMU::regs();
    pmu.hp_active_backup()
        .modify(|_, w| w.hp_sleep2active_backup_en().bit(enabled));
    pmu.hp_sleep_backup()
        .modify(|_, w| w.hp_active2sleep_backup_en().bit(enabled));
}

// A wait node reads its register up to `WAIT_RETRY_COUNT` times, `WAIT_READ_INTERVAL` apart.
// See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_hw_support/port/pau_regdma.c#L21-L24>.
const WAIT_RETRY_COUNT: u16 = 1000;
const WAIT_READ_INTERVAL: u8 = 32;

cfg_select! {
    regdma_link_configurable => {
        const WORK_TIMEOUT: u16 = 0x3ff;
        const REGISTER_ACCESS_TIMEOUT: u16 = 0x3ff;
    }
    _ => {
        // The reset values of the PAU.
        const REGISTER_ACCESS_TIMEOUT: u16 = 500;
        const BURST_LIMIT: u8 = 8;
    }
}

/// Enables the REGDMA, and gives it the head of the list.
fn init_pau() {
    cfg_select! {
        esp32s31 => {
            crate::peripherals::HP_SYS_CLKRST::regs()
                .regdma_ctrl0()
                .modify(|_, w| w.sys_clk_en().set_bit().rst_en().clear_bit());
        }
        _ => {
            crate::peripherals::PCR::regs()
                .regdma_conf()
                .modify(|_, w| w.regdma_clk_en().set_bit().regdma_rst_en().clear_bit());
        }
    }

    // The PMU init of the other chips sets the same retention parameters.
    // See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_hw_support/port/esp32h2/pmu_param.c#L278-L322>.
    #[cfg(esp32h2)]
    {
        const MODEM_CLK_CODE: u8 = 3;
        const RESTORE_ENTRY_0: u8 = 0;
        const BACKUP_ENTRY_0: u8 = 1 << 2;

        let pmu = PMU::regs();
        pmu.hp_active_backup().modify(|_, w| unsafe {
            w.hp_sleep2active_backup_modem_clk_code()
                .bits(MODEM_CLK_CODE)
                .hp_sleep2active_backup_clk_sel()
                .bits(0)
                .hp_sleep2active_backup_mode()
                .bits(RESTORE_ENTRY_0)
        });
        pmu.hp_sleep_backup().modify(|_, w| unsafe {
            w.hp_active2sleep_backup_modem_clk_code()
                .bits(MODEM_CLK_CODE)
                .hp_active2sleep_backup_clk_sel()
                .bits(0)
                .hp_active2sleep_backup_mode()
                .bits(BACKUP_ENTRY_0)
        });
        pmu.hp_active_backup_clk()
            .write(|w| unsafe { w.bits(u32::MAX) });
        pmu.hp_sleep_backup_clk()
            .write(|w| unsafe { w.bits(u32::MAX) });
    }

    let head = HEAD.link() as u32;

    cfg_select! {
        regdma_link_configurable => {
            let lp = cfg_select! {
                esp32s31 => crate::peripherals::LP_SYS::regs(),
                _ => crate::peripherals::LP_AON::regs(),
            };
            cfg_select! {
                esp32s31 => {
                    lp.backup_dma_cfg0().modify(|_, w| unsafe {
                        w.read_interval_aon()
                            .bits(WAIT_READ_INTERVAL)
                            .link_backup_tout_thres_aon()
                            .bits(REGISTER_ACCESS_TIMEOUT)
                    });
                    lp.backup_dma_cfg1().modify(|_, w| unsafe {
                        w.link_wait_tout_thres_aon()
                            .bits(WAIT_RETRY_COUNT)
                            .link_work_tout_thres_aon()
                            .bits(WORK_TIMEOUT)
                            .branch_link_length_aon()
                            .bits(ENTRIES as u8 - 1)
                    });
                }
                _ => {
                    lp.backup_dma_cfg0().modify(|_, w| unsafe {
                        w.read_interval_aon()
                            .bits(WAIT_READ_INTERVAL)
                            .branch_link_length_aon()
                            .bits(ENTRIES as u8 - 1)
                    });
                    lp.backup_dma_cfg1().modify(|_, w| unsafe {
                        w.link_wait_tout_thres_aon()
                            .bits(WAIT_RETRY_COUNT)
                            .link_work_tout_thres_aon()
                            .bits(WORK_TIMEOUT)
                            .link_backup_tout_thres_aon()
                            .bits(REGISTER_ACCESS_TIMEOUT)
                    });
                }
            }
            lp.backup_dma_cfg2()
                .write(|w| unsafe { w.link_addr_aon().bits(head) });
        }
        _ => {
            let pau = crate::peripherals::PAU::regs();
            // The PAU reads back 0 for a moment after its clock starts, so this writes every field
            // instead of keeping the reset values.
            pau.regdma_bkp_conf().write(|w| unsafe {
                w.read_interval()
                    .bits(WAIT_READ_INTERVAL)
                    .link_tout_thres()
                    .bits(WAIT_RETRY_COUNT)
                    .burst_limit()
                    .bits(BURST_LIMIT)
                    .backup_tout_thres()
                    .bits(REGISTER_ACCESS_TIMEOUT)
            });
            pau.regdma_link_0_addr().write(|w| unsafe { w.bits(head) });
            pau.regdma_link_1_addr().write(|w| unsafe { w.bits(head) });
            pau.regdma_link_2_addr().write(|w| unsafe { w.bits(head) });
            pau.regdma_link_3_addr().write(|w| unsafe { w.bits(head) });
        }
    }
}

/// Runs the extra entry, which copies the attached modules of [`EXTRA_ENTRY`] to memory
/// (`backup`), or back. Blocks until the REGDMA finishes. Does nothing while
/// [`set_modem_power_down`] is off.
///
/// Call this while the PLL runs.
#[cfg(regdma_bt_needs_pll)]
pub fn run_extra_link(backup: bool) {
    if !modem_power_down() {
        return;
    }
    // See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_hal_pmu/esp32c6/pau_hal.c#L50-L71>.
    let pau = crate::peripherals::PAU::regs();
    let clear = || {
        pau.int_clr()
            .write(|w| w.done().clear_bit_by_one().error().clear_bit_by_one())
    };
    clear();
    pau.regdma_conf().modify(|_, w| unsafe {
        w.link_sel()
            .bits(EXTRA_ENTRY_INDEX as u8)
            .to_mem()
            .bit(backup)
    });
    pau.regdma_conf().modify(|_, w| w.start().set_bit());
    // After an error, `done` never comes.
    loop {
        let raw = pau.int_raw().read();
        if raw.done().bit_is_set() || raw.error().bit_is_set() {
            break;
        }
    }
    pau.regdma_conf()
        .modify(|_, w| unsafe { w.start().clear_bit().link_sel().bits(0) });
    clear();
}
