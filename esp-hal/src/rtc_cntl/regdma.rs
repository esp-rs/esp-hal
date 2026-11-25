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
//!
//! In the PMU modem state, the CPU sleeps while the Wi-Fi modem receives the beacons of the access
//! point. The PMU runs `MODEM_ENTRY` between the sleep and the modem state, entry 2 from the modem
//! to the active state, and the PHY link of `PHY_ENTRY` to turn the RF on and off.
//! The PHY driver runs the PHY link too, with `run_phy_link`.

use core::{cell::UnsafeCell, ptr::NonNull};

use esp_sync::NonReentrantMutex;
use portable_atomic::{AtomicBool, Ordering};

use crate::peripherals::PMU;

/// The number of REGDMA entries that the head branches to.
const ENTRIES: usize = property!("sleep.regdma_list_count");

/// The entry that the PMU runs between the active and the sleep state.
pub const SLEEP_ENTRY: u32 = 1 << 0;

/// The entry that the PMU runs between the sleep and the modem state.
#[cfg(supports_modem_state)]
pub const MODEM_ENTRY: u32 = 1 << 1;

/// The entry that software runs with [`run_extra_link`].
#[cfg(regdma_bt_needs_pll)]
pub const EXTRA_ENTRY: u32 = 1 << 3;

/// The PHY link, which turns the RF on and off: in the modem state, and with [`run_phy_link`].
///
/// On chips with four entries, the PHY link is a list of its own, which the PAU starts from its
/// Wi-Fi MAC link address.
#[cfg(supports_modem_state)]
pub const PHY_ENTRY: u32 = 1 << 4;

/// The entries that [`create`] builds nodes for. With the modem state, entry 2 runs from the modem
/// to the active state, for the nodes of the Wi-Fi blob.
const USED_ENTRIES: u32 = cfg_select! {
    all(supports_modem_state, regdma_bt_needs_pll) => {
        SLEEP_ENTRY | MODEM_ENTRY | 1 << 2 | EXTRA_ENTRY | PHY_ENTRY
    }
    supports_modem_state => SLEEP_ENTRY | MODEM_ENTRY | 1 << 2 | PHY_ENTRY,
    regdma_bt_needs_pll => SLEEP_ENTRY | EXTRA_ENTRY,
    _ => SLEEP_ENTRY,
};

// The head of a node, the word that every link points to.
// See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/soc/include/soc/regdma.h#L138-L204>.
const HEAD_LENGTH: u32 = 0x3ff;
const HEAD_MODE: u32 = 0xf << 16;
const MODE_CONTINUOUS: u32 = 0;
const MODE_ADDR_MAP: u32 = 1 << 16;
const MODE_WRITE: u32 = 2 << 16;
const MODE_WAIT: u32 = 3 << 16;
const HEAD_BRANCH: u32 = 1 << 28;
const HEAD_SKIP_RESTORE: u32 = 1 << 29;
const HEAD_SKIP_BACKUP: u32 = 1 << 30;
const HEAD_EOF: u32 = 1 << 31;

/// One node of a REGDMA link list, and the entries that it belongs to.
///
/// The layout is the `sleep_retention_entries_config_t` of ESP-IDF, 11 words, so the radio blobs
/// can hand out their tables in this form.
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

impl LinkConfig {
    /// Copies `length` words from `backup` to memory before the sleep, and from memory to
    /// `restore` after it.
    pub const fn continuous(id: u16, backup: u32, restore: u32, length: u16, owner: u32) -> Self {
        Self {
            head: MODE_CONTINUOUS | (length as u32 & HEAD_LENGTH),
            body: [0, backup, restore, 0, 0, 0, 0, 0],
            id: id as u32,
            owner,
        }
    }

    /// Writes `value` to the bits in `mask` of the register at `address` after the sleep.
    ///
    /// [`Self::skip`] makes the write run before the sleep instead, or both times.
    pub const fn write(id: u16, address: u32, value: u32, mask: u32, owner: u32) -> Self {
        Self {
            head: MODE_WRITE | HEAD_SKIP_BACKUP,
            body: [0, address, value, mask, 0, 0, 0, 0],
            id: id as u32,
            owner,
        }
    }

    /// Waits after the sleep until the bits in `mask` of the register at `address` equal `value`.
    ///
    /// [`Self::skip`] makes the wait run before the sleep instead, or both times.
    pub const fn wait(id: u16, address: u32, value: u32, mask: u32, owner: u32) -> Self {
        Self {
            head: MODE_WAIT | HEAD_SKIP_BACKUP,
            body: [0, address, value, mask, 0, 0, 0, 0],
            id: id as u32,
            owner,
        }
    }

    /// Sets whether the REGDMA skips the node when it saves, and when it restores.
    pub const fn skip(mut self, backup: bool, restore: bool) -> Self {
        self.head = with_skip(self.head, backup, restore);
        self
    }

    /// Whether the node copies registers, to a buffer.
    const fn copies(&self) -> bool {
        matches!(self.head & HEAD_MODE, MODE_CONTINUOUS | MODE_ADDR_MAP)
    }

    /// The words of the body, which an address map extends with its map.
    const fn body_words(&self) -> usize {
        if self.head & HEAD_MODE == MODE_ADDR_MAP {
            8
        } else {
            4
        }
    }

    /// The words of the node in memory: the statistics word, the head and the body.
    const fn node_words(&self) -> usize {
        2 + self.body_words()
    }

    /// The words of the buffer of a copy, which the nodes of every entry share.
    const fn buffer_words(&self) -> usize {
        if self.copies() {
            (self.head & HEAD_LENGTH) as usize
        } else {
            0
        }
    }
}

/// Returns `head` with the skip flags for the save and the restore.
const fn with_skip(head: u32, backup: bool, restore: bool) -> u32 {
    let mut head = head & !(HEAD_SKIP_BACKUP | HEAD_SKIP_RESTORE);
    if backup {
        head |= HEAD_SKIP_BACKUP;
    }
    if restore {
        head |= HEAD_SKIP_RESTORE;
    }
    head
}

/// Returns the bytes that [`create`] takes for the nodes of `configs`.
pub const fn links_size(configs: &[LinkConfig]) -> usize {
    let mut words = 0;
    let mut i = 0;
    while i < configs.len() {
        let entries = (configs[i].owner & USED_ENTRIES).count_ones() as usize;
        if entries > 0 {
            words += configs[i].node_words() * entries + configs[i].buffer_words();
        }
        i += 1;
    }
    words * 4
}

/// A part of the modem domain, with its place in the list.
///
/// The REGDMA restores the modules in the order of this enum.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Module {
    /// The system clocks that the modem needs in the PMU modem state.
    #[cfg(supports_modem_state)]
    ModemStateClock,
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
    /// The PHY link, which turns the RF on and off.
    #[cfg(supports_modem_state)]
    PhyLink,
}

const MODULES: usize = cfg_select! {
    supports_modem_state => Module::PhyLink as usize + 1,
    _ => Module::BleMac as usize + 1,
};

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

/// The reason why a REGDMA operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The module has nodes already.
    AlreadyCreated,
    /// The module has no nodes.
    NotCreated,
    /// The memory has no room for the nodes.
    OutOfMemory,
    /// The REGDMA raised its error flag before it ran every node.
    #[cfg(any(regdma_bt_needs_pll, supports_modem_state))]
    Transfer,
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::AlreadyCreated => write!(f, "the module has nodes already"),
            Self::NotCreated => write!(f, "the module has no nodes"),
            Self::OutOfMemory => write!(f, "the memory has no room for the nodes"),
            #[cfg(any(regdma_bt_needs_pll, supports_modem_state))]
            Self::Transfer => write!(f, "the REGDMA stopped on an error"),
        }
    }
}

/// The first and the last node of a module in one entry.
#[derive(Clone, Copy)]
struct Chain {
    first: NonNull<u32>,
    last: NonNull<u32>,
}

// SAFETY: The nodes are plain memory, which only the registry touches after `create`, under its
// lock.
unsafe impl Send for Chain {}

impl Chain {
    /// Returns the links of the nodes, in order.
    fn links(self) -> impl Iterator<Item = *mut u32> {
        let last = self.last.as_ptr();
        let mut next = Some(self.first.as_ptr());
        core::iter::from_fn(move || {
            let link = next?;
            next = (link != last).then(|| next_of(link));
            Some(link)
        })
    }
}

/// The chains of a module, one for each of [`USED_ENTRIES`].
type Links = [Option<Chain>; USED_ENTRIES.count_ones() as usize];

/// Returns the index of each entry in [`USED_ENTRIES`], in order.
fn used_entries() -> impl Iterator<Item = usize> {
    (0..u32::BITS as usize).filter(|entry| USED_ENTRIES & (1 << entry) != 0)
}

struct Registry {
    links: [Option<Links>; MODULES],
    attached: u8,
}

impl Registry {
    /// Returns the links of every node, attached or not.
    fn all_links(&self) -> impl Iterator<Item = *mut u32> {
        self.links
            .iter()
            .flatten()
            .flat_map(|links| links.iter().flatten())
            .flat_map(|chain| chain.links())
    }
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
///
/// Its words are the statistics word, the head, a `next` pointer for each entry, then the address,
/// the value and the mask of the wait.
static HEAD: StaticNode<{ 2 + ENTRIES + 3 }> = StaticNode(UnsafeCell::new({
    let mut words = [0; 2 + ENTRIES + 3];
    words[1] = MODE_WAIT | HEAD_SKIP_BACKUP | HEAD_SKIP_RESTORE | HEAD_BRANCH;
    words
}));

/// The last node of every entry: a wait that the REGDMA skips, and the end of the list.
static TAIL: StaticNode<6> = StaticNode(UnsafeCell::new([
    0,
    MODE_WAIT | HEAD_SKIP_BACKUP | HEAD_SKIP_RESTORE | HEAD_EOF,
    0,
    0,
    0,
    0,
]));

/// The first node of the PHY link on chips where the head has no branch for it: a wait that the
/// REGDMA skips.
#[cfg(all(supports_modem_state, not(regdma_link_configurable)))]
static PHY_HEAD: StaticNode<6> = StaticNode(UnsafeCell::new([
    0,
    MODE_WAIT | HEAD_SKIP_BACKUP | HEAD_SKIP_RESTORE,
    0,
    0,
    0,
    0,
]));

/// Whether the modem domain loses power in light sleep.
static MODEM_POWER_DOWN: AtomicBool = AtomicBool::new(false);

/// Builds the nodes of `module` from `configs` in `memory`, without joining them into the list.
///
/// The nodes belong to `module` until [`destroy`]. On an error, `memory` stays as it was.
///
/// # Errors
///
/// - [`Error::AlreadyCreated`] if `module` has nodes already.
/// - [`Error::OutOfMemory`] if `memory` has no room for the nodes.
///
/// # Safety
///
/// `configs` must describe registers that the REGDMA can read and write.
pub unsafe fn create(
    module: Module,
    configs: &[&[LinkConfig]],
    memory: &mut LinkMemory,
) -> Result<(), Error> {
    let size = configs
        .iter()
        .map(|configs| links_size(configs))
        .sum::<usize>();
    let start = memory.next;
    let mut next = memory.take(size / 4).ok_or(Error::OutOfMemory)?;

    // Nothing reads the nodes before the registry has them, so they are built without its lock.
    let mut links: Links = Default::default();
    for config in configs.iter().flat_map(|configs| configs.iter()) {
        let owner = config.owner & USED_ENTRIES;
        if owner == 0 {
            continue;
        }
        // The nodes of every entry share the buffer, so that an entry restores what another
        // entry saved.
        let buffer = next;
        next = unsafe { next.add(config.buffer_words()) };
        for (chain, entry) in links.iter_mut().zip(used_entries()) {
            if owner & (1 << entry) != 0 {
                append(chain, build(config, buffer, &mut next));
            }
        }
    }

    REGISTRY
        .with(|registry| {
            let links_of_module = &mut registry.links[module as usize];
            if links_of_module.is_some() {
                return Err(Error::AlreadyCreated);
            }
            *links_of_module = Some(links);
            Ok(())
        })
        .inspect_err(|_| memory.next = start)
}

/// Builds one node at `next`, moves `next` past it, and returns its link, the address of its head.
/// A copy keeps its registers in `buffer`.
///
/// [`create`] takes the memory of every node of a module before the first call.
fn build(config: &LinkConfig, buffer: *mut u32, next: &mut *mut u32) -> NonNull<u32> {
    let node = *next;
    let body_words = config.body_words();
    unsafe {
        *next = node.add(config.node_words());
        node.write_volatile(config.id << 16);
        let link = node.add(1);
        link.write_volatile(config.head & !(HEAD_BRANCH | HEAD_EOF));
        let body = link.add(1);
        for (i, &word) in config.body[..body_words].iter().enumerate() {
            body.add(i).write_volatile(word);
        }
        if config.copies() {
            // `mem` points to the buffer.
            body.add(3).write_volatile(buffer as u32);
        }
        NonNull::new_unchecked(link)
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
/// # Errors
///
/// Returns [`Error::NotCreated`] if `module` has no nodes.
pub fn attach(module: Module) -> Result<(), Error> {
    REGISTRY.with(|registry| {
        if registry.links[module as usize].is_none() {
            return Err(Error::NotCreated);
        }
        registry.attached |= 1 << module as u8;
        join(registry);
        Ok(())
    })
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

    for (slot, entry) in used_entries().enumerate() {
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
        set_first(entry, next);
    }
}

/// Points the start of `entry` to `first`.
fn set_first(entry: usize, first: *mut u32) {
    #[cfg(all(supports_modem_state, not(regdma_link_configurable)))]
    if entry == PHY_ENTRY.trailing_zeros() as usize {
        set_next(PHY_HEAD.link(), first);
        return;
    }
    unsafe { HEAD.link().add(1 + entry).write_volatile(first as u32) };
}

fn id_of(link: *mut u32) -> u32 {
    unsafe { link.sub(1).read_volatile() >> 16 }
}

/// Returns the first node with `id`, from every module, attached or not.
pub fn find_link_by_id(id: u16) -> Option<NonNull<u32>> {
    REGISTRY.with(|registry| {
        registry
            .all_links()
            .find(|&link| id_of(link) == id as u32)
            .and_then(NonNull::new)
    })
}

/// Changes the value and the mask of a write or wait node, in every entry that the node belongs
/// to.
///
/// # Safety
///
/// `link` must come from [`find_link_by_id`], and its module must still exist.
pub unsafe fn set_write_wait_content(link: NonNull<u32>, value: u32, mask: u32) {
    let link = link.as_ptr();
    let is_write_or_wait = |link: *mut u32| unsafe {
        matches!(link.read_volatile() & HEAD_MODE, MODE_WRITE | MODE_WAIT)
    };
    if !is_write_or_wait(link) {
        return;
    }
    // The nodes of one config have the same ID and register.
    let id = id_of(link);
    let address = unsafe { link.add(2).read_volatile() };
    REGISTRY.with(|registry| {
        for node in registry.all_links() {
            if id_of(node) == id
                && is_write_or_wait(node)
                && unsafe { node.add(2).read_volatile() } == address
            {
                unsafe {
                    node.add(3).write_volatile(value);
                    node.add(4).write_volatile(mask);
                }
            }
        }
    });
}

/// Sets whether the REGDMA skips the nodes of `module` with `id` when it saves, and when it
/// restores.
pub fn set_skip(module: Module, id: u16, backup: bool, restore: bool) {
    REGISTRY.with(|registry| {
        let chains = registry.links[module as usize].iter().flatten().flatten();
        for link in chains.flat_map(|chain| chain.links()) {
            if id_of(link) == id as u32 {
                unsafe { link.write_volatile(with_skip(link.read_volatile(), backup, restore)) };
            }
        }
    });
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
            init_regdma();
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

/// Lets the PMU start the REGDMA on the transitions of light sleep.
fn set_backup(enabled: bool) {
    let pmu = PMU::regs();
    pmu.hp_active_backup().modify(|_, w| {
        #[cfg(supports_modem_state)]
        w.hp_modem2active_backup_en().bit(enabled);
        w.hp_sleep2active_backup_en().bit(enabled)
    });
    // ESP-IDF leaves the backup off from the modem state to sleep. On that transition, the Wi-Fi
    // MAC starts the REGDMA itself.
    pmu.hp_sleep_backup()
        .modify(|_, w| w.hp_active2sleep_backup_en().bit(enabled));
    #[cfg(supports_modem_state)]
    pmu.hp_modem_backup()
        .modify(|_, w| w.hp_sleep2modem_backup_en().bit(enabled));
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
fn init_regdma() {
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
                        w.read_interval_aon().bits(WAIT_READ_INTERVAL);
                        w.link_backup_tout_thres_aon().bits(REGISTER_ACCESS_TIMEOUT)
                    });
                    lp.backup_dma_cfg1().modify(|_, w| unsafe {
                        w.link_wait_tout_thres_aon().bits(WAIT_RETRY_COUNT);
                        w.link_work_tout_thres_aon().bits(WORK_TIMEOUT);
                        w.branch_link_length_aon().bits(ENTRIES as u8 - 1)
                    });
                }
                _ => {
                    lp.backup_dma_cfg0().modify(|_, w| unsafe {
                        w.read_interval_aon().bits(WAIT_READ_INTERVAL);
                        w.branch_link_length_aon().bits(ENTRIES as u8 - 1)
                    });
                    lp.backup_dma_cfg1().modify(|_, w| unsafe {
                        w.link_wait_tout_thres_aon().bits(WAIT_RETRY_COUNT);
                        w.link_work_tout_thres_aon().bits(WORK_TIMEOUT);
                        w.link_backup_tout_thres_aon().bits(REGISTER_ACCESS_TIMEOUT)
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
                w.read_interval().bits(WAIT_READ_INTERVAL);
                w.link_tout_thres().bits(WAIT_RETRY_COUNT);
                w.burst_limit().bits(BURST_LIMIT);
                w.backup_tout_thres().bits(REGISTER_ACCESS_TIMEOUT)
            });
            pau.regdma_link_0_addr().write(|w| unsafe { w.bits(head) });
            pau.regdma_link_1_addr().write(|w| unsafe { w.bits(head) });
            pau.regdma_link_2_addr().write(|w| unsafe { w.bits(head) });
            pau.regdma_link_3_addr().write(|w| unsafe { w.bits(head) });
            #[cfg(supports_modem_state)]
            pau.regdma_link_mac_addr()
                .write(|w| unsafe { w.bits(PHY_HEAD.link() as u32) });
        }
    }
}

/// Runs the nodes of [`EXTRA_ENTRY`] in the attached modules. With `backup`, they copy the
/// registers to memory, otherwise back from memory. Blocks until the REGDMA finishes. Does nothing
/// while [`set_modem_power_down`] is off.
///
/// Call this while the PLL runs.
///
/// # Errors
///
/// Returns [`Error::Transfer`] if the REGDMA stopped before it ran every node.
#[cfg(regdma_bt_needs_pll)]
pub fn run_extra_link(backup: bool) -> Result<(), Error> {
    if !modem_power_down() {
        return Ok(());
    }
    // See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_hal_pmu/esp32c6/pau_hal.c#L50-L71>.
    run_entry(EXTRA_ENTRY, backup)
}

/// Runs the nodes of [`PHY_ENTRY`] in the attached [`Module::PhyLink`]. With `backup`, the nodes
/// that turn the RF off run, otherwise the nodes that turn it on. Blocks until the REGDMA finishes.
///
/// With `wifi_mac`, the PAU starts the PHY link with its Wi-Fi MAC trigger, which also clears the
/// modem sleep flag of the MAC. Chips where the head has no branch for the PHY link always use
/// this trigger.
///
/// # Errors
///
/// - [`Error::NotCreated`] if [`Module::PhyLink`] is not in the list.
/// - [`Error::Transfer`] if the REGDMA stopped before it ran every node.
#[cfg(supports_modem_state)]
pub fn run_phy_link(backup: bool, wifi_mac: bool) -> Result<(), Error> {
    // The lock keeps the list as it is while the REGDMA walks it.
    REGISTRY.with(|registry| {
        if registry.attached & (1 << Module::PhyLink as u8) == 0 {
            return Err(Error::NotCreated);
        }
        // Wi-Fi turns the RF off before light sleep sets the REGDMA up for the first time.
        init_regdma();

        // See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_hal_pmu/esp32c5/pau_hal.c#L23-L65>.
        if !wifi_mac && cfg!(regdma_link_configurable) {
            return run_entry(PHY_ENTRY, backup);
        }
        let pau = crate::peripherals::PAU::regs();
        run_link(
            || {
                pau.regdma_conf()
                    .modify(|_, w| w.sel_mac().set_bit().to_mem_mac().bit(backup));
                pau.regdma_conf().modify(|_, w| w.start_mac().set_bit());
            },
            || {
                pau.regdma_conf()
                    .modify(|_, w| w.start_mac().clear_bit().sel_mac().clear_bit());
            },
        )
    })
}

/// Runs the nodes of `entry` from software, and blocks until the REGDMA finishes.
#[cfg(any(regdma_bt_needs_pll, supports_modem_state))]
fn run_entry(entry: u32, backup: bool) -> Result<(), Error> {
    let pau = crate::peripherals::PAU::regs();
    run_link(
        || {
            pau.regdma_conf().modify(|_, w| unsafe {
                w.link_sel().bits(entry.trailing_zeros() as u8);
                w.to_mem().bit(backup)
            });
            pau.regdma_conf().modify(|_, w| w.start().set_bit());
        },
        || {
            pau.regdma_conf()
                .modify(|_, w| unsafe { w.start().clear_bit().link_sel().bits(0) });
        },
    )
}

/// Starts the REGDMA with `start`, blocks until it finishes, then stops it with `stop`.
#[cfg(any(regdma_bt_needs_pll, supports_modem_state))]
fn run_link(start: impl FnOnce(), stop: impl FnOnce()) -> Result<(), Error> {
    let pau = crate::peripherals::PAU::regs();
    let clear = || {
        pau.int_clr()
            .write(|w| w.done().clear_bit_by_one().error().clear_bit_by_one())
    };
    clear();
    start();
    // After an error, `done` never comes.
    let failed = loop {
        let raw = pau.int_raw().read();
        if raw.error().bit_is_set() {
            break true;
        }
        if raw.done().bit_is_set() {
            break false;
        }
    };
    stop();
    clear();
    if failed { Err(Error::Transfer) } else { Ok(()) }
}
