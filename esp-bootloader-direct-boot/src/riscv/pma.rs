//! Physical memory attributes.
//!
//! The ROM of these chips leaves some PMA entries enabled (on the ESP32-C5 e.g. all of SRAM as
//! non-executable), which the 2nd-stage bootloader replaces, so the first `#[ram]` code or trap
//! would lock the CPU up. The new entries block the address space gaps, keep ROM RX and flash
//! RWX + cacheable, and leave everything else (SRAM, LP memory, peripherals, CPU subsystem)
//! uncovered. Addresses are `soc.h` constants.
//!
//! See <https://github.com/espressif/esp-idf/blob/fff9895c82d744c7237be8847347bdd1b07c6643/components/esp_hw_support/port/esp32c5/cpu_region_protect.c#L34-L93>.

use core::arch::asm;

const EN: u32 = 1 << 0;
const X: u32 = 1 << 2;
const W: u32 = 1 << 3;
const R: u32 = 1 << 4;
const L: u32 = 1 << 29;
const TOR: u32 = 0x4000_0000;
const NAPOT: u32 = 0xC000_0000;

const NONE: u32 = L | EN;
#[cfg(any(esp32c5, esp32h4, esp32s31))]
const RX: u32 = L | EN | R | X;
#[cfg(esp32s31)]
const RW: u32 = L | EN | R | W;
const RWX: u32 = L | EN | R | W | X;

const fn tor(addr: u32) -> u32 {
    addr >> 2
}

const fn napot(base: u32, size: u32) -> u32 {
    (base | ((size >> 1) - 1)) >> 2
}

#[cfg(esp32h4)]
fn read_csr<const CSR: u16>() -> u32 {
    let value: u32;
    unsafe { asm!("csrr {0}, {csr}", out(reg) value, csr = const CSR) };
    value
}

fn write_csr<const CSR: u16>(value: u32) {
    unsafe { asm!("csrw {csr}, {0}", in(reg) value, csr = const CSR) };
}

/// The 16 consecutive CSRs starting at `$base`, accessed through `$f`.
macro_rules! csr_array {
    ($f:ident, $base:literal) => {
        [
            $f::<{ $base }>,
            $f::<{ $base + 1 }>,
            $f::<{ $base + 2 }>,
            $f::<{ $base + 3 }>,
            $f::<{ $base + 4 }>,
            $f::<{ $base + 5 }>,
            $f::<{ $base + 6 }>,
            $f::<{ $base + 7 }>,
            $f::<{ $base + 8 }>,
            $f::<{ $base + 9 }>,
            $f::<{ $base + 10 }>,
            $f::<{ $base + 11 }>,
            $f::<{ $base + 12 }>,
            $f::<{ $base + 13 }>,
            $f::<{ $base + 14 }>,
            $f::<{ $base + 15 }>,
        ]
    };
}

/// `(pmaaddr, pmacfg)` for entries 0..16.
#[cfg(esp32c5)]
const ENTRIES: [(u32, u32); 16] = {
    const CPU_SUBSYSTEM_LOW: u32 = 0x2000_0000;
    const CPU_SUBSYSTEM_HIGH: u32 = 0x3000_0000;
    const IROM_MASK_LOW: u32 = 0x4000_0000;
    const DROM_MASK_HIGH: u32 = 0x4005_0000;
    const IRAM_LOW: u32 = 0x4080_0000;
    const IRAM_HIGH: u32 = 0x4086_0000;
    const IROM_LOW: u32 = 0x4200_0000;
    const IROM_HIGH: u32 = 0x4400_0000;
    const RTC_IRAM_LOW: u32 = 0x5000_0000;
    const RTC_IRAM_HIGH: u32 = 0x5000_4000;
    const PERIPHERAL_LOW: u32 = 0x6000_0000;
    const PERIPHERAL_HIGH: u32 = 0x6010_0000;
    [
        // gap at the bottom of the address space
        (napot(0, CPU_SUBSYSTEM_LOW), NAPOT | NONE),
        // gap between CPU subsystem and ROM
        (tor(CPU_SUBSYSTEM_HIGH), NONE),
        (tor(IROM_MASK_LOW), TOR | NONE),
        // ROM: RX, cacheable
        (tor(IROM_MASK_LOW), NONE),
        (tor(DROM_MASK_HIGH), TOR | RX),
        // gap between SRAM and flash
        (tor(IRAM_HIGH), NONE),
        (tor(IROM_LOW), TOR | NONE),
        // flash (and PSRAM): RWX, cacheable
        (napot(IROM_LOW, IROM_HIGH - IROM_LOW), NAPOT | RWX),
        // gap between flash and LP memory
        (tor(IROM_HIGH), NONE),
        (tor(RTC_IRAM_LOW), TOR | NONE),
        // end of the address space
        (tor(PERIPHERAL_HIGH), NONE),
        (tor(u32::MAX), TOR | NONE),
        // gap between ROM and SRAM
        (tor(DROM_MASK_HIGH), NONE),
        (tor(IRAM_LOW), TOR | NONE),
        // gap between LP memory and peripherals
        (tor(RTC_IRAM_HIGH), NONE),
        (tor(PERIPHERAL_LOW), TOR | NONE),
    ]
};

#[cfg(esp32c61)]
const ENTRIES: [(u32, u32); 16] = {
    const CPU_SUBSYSTEM_LOW: u32 = 0x2000_0000;
    const CPU_SUBSYSTEM_HIGH: u32 = 0x3000_0000;
    const IROM_MASK_LOW: u32 = 0x4000_0000;
    const DROM_MASK_HIGH: u32 = 0x4004_0000;
    const IRAM_LOW: u32 = 0x4080_0000;
    const IRAM_HIGH: u32 = 0x4085_0000;
    const IROM_LOW: u32 = 0x4200_0000;
    const IROM_HIGH: u32 = 0x4400_0000;
    const PERIPHERAL_LOW: u32 = 0x6000_0000;
    const PERIPHERAL_HIGH: u32 = 0x6010_0000;
    [
        // gap at the bottom of the address space
        (napot(0, CPU_SUBSYSTEM_LOW), NAPOT | NONE),
        // gap between CPU subsystem and ROM
        (tor(CPU_SUBSYSTEM_HIGH), NONE),
        (tor(IROM_MASK_LOW), TOR | NONE),
        // gap between ROM and SRAM
        (tor(DROM_MASK_HIGH), NONE),
        (tor(IRAM_LOW), TOR | NONE),
        // gap between SRAM and flash
        (tor(IRAM_HIGH), NONE),
        (tor(IROM_LOW), TOR | NONE),
        // flash (and PSRAM): RWX, cacheable
        (napot(IROM_LOW, IROM_HIGH - IROM_LOW), NAPOT | RWX),
        // gap between flash and peripherals
        (tor(IROM_HIGH), NONE),
        (tor(PERIPHERAL_LOW), TOR | NONE),
        // end of the address space
        (tor(PERIPHERAL_HIGH), NONE),
        (tor(u32::MAX), TOR | NONE),
        (0, 0),
        (0, 0),
        (0, 0),
        (0, 0),
    ]
};

#[cfg(esp32p4)]
const ENTRIES: [(u32, u32); 16] = {
    const CPU_SUBSYSTEM_LOW: u32 = 0x2000_0000;
    const CPU_SUBSYSTEM_HIGH: u32 = 0x3000_0000;
    const SPM_LOW: u32 = 0x3010_0000;
    const SPM_HIGH: u32 = 0x3010_2000;
    const CPU_PERIPH_LOW: u32 = 0x3FF0_0000;
    const CPU_PERIPH_HIGH: u32 = 0x3FF2_0000;
    const IROM_LOW: u32 = 0x4000_0000;
    const DROM_HIGH: u32 = 0x4400_0000;
    const EXTRAM_LOW: u32 = 0x4800_0000;
    const EXTRAM_HIGH: u32 = 0x4C00_0000;
    const IROM_MASK_LOW: u32 = 0x4FC0_0000;
    const IROM_MASK_HIGH: u32 = 0x4FC2_0000;
    const IRAM_LOW: u32 = 0x4FF0_0000;
    const DRAM_HIGH: u32 = 0x4FFC_0000;
    const PERIPHERAL_LOW: u32 = 0x5000_0000;
    /// `CACHE_LL_L2MEM_NON_CACHE_ADDR`: L2MEM + `SOC_NON_CACHEABLE_OFFSET_SRAM`.
    const L2MEM_NON_CACHE_OFFSET: u32 = 0x4000_0000;
    [
        // gap at the bottom of the address space
        (napot(0, CPU_SUBSYSTEM_LOW), NAPOT | NONE),
        // gap between CPU subsystem and HP SPM
        (tor(CPU_SUBSYSTEM_HIGH), NONE),
        (tor(SPM_LOW), TOR | NONE),
        // gap between HP SPM and CPU peripherals
        (tor(SPM_HIGH), NONE),
        (tor(CPU_PERIPH_LOW), TOR | NONE),
        // gap between CPU peripherals and flash
        (tor(CPU_PERIPH_HIGH), NONE),
        (tor(IROM_LOW), TOR | NONE),
        // gap between flash and PSRAM
        (napot(DROM_HIGH, EXTRAM_LOW - DROM_HIGH), NAPOT | NONE),
        // gap between PSRAM and ROM
        (tor(EXTRAM_HIGH), NONE),
        (tor(IROM_MASK_LOW), TOR | NONE),
        // gap between ROM and L2MEM
        (tor(IROM_MASK_HIGH), NONE),
        (tor(IRAM_LOW), TOR | NONE),
        // gap between L2MEM and HP peripherals
        (napot(DRAM_HIGH, PERIPHERAL_LOW - DRAM_HIGH), NAPOT | NONE),
        // flash, PSRAM, ROM and L2MEM: RWX, cacheable (ROM uses entry 15 for this)
        (napot(IROM_LOW, PERIPHERAL_LOW - IROM_LOW), NAPOT | RWX),
        // gap between the uncached L2MEM alias and the end of the address space
        (tor(DRAM_HIGH + L2MEM_NON_CACHE_OFFSET), NONE),
        (tor(u32::MAX), TOR | NONE),
    ]
};

#[cfg(esp32s31)]
const ENTRIES: [(u32, u32); 16] = {
    const CPU_SUBSYSTEM_LOW: u32 = 0x1000_0000;
    const RTC_IRAM_HIGH: u32 = 0x2E00_8000;
    const IRAM_LOW: u32 = 0x2F00_0000;
    const IRAM_HIGH: u32 = 0x2F08_0000;
    const IROM_MASK_LOW: u32 = 0x2F80_0000;
    const DROM_MASK_HIGH: u32 = 0x2F85_0000;
    const IROM_LOW: u32 = 0x4000_0000;
    const IROM_HIGH: u32 = 0x4400_0000;
    const EXTRAM_LOW: u32 = 0x5000_0000;
    const EXTRAM_HIGH: u32 = 0x5400_0000;
    const NON_CACHEABLE_OFFSET_FLASH: u32 = 0x6000_0000;
    const NON_CACHEABLE_OFFSET_PSRAM: u32 = 0x7000_0000;
    [
        // gap at the bottom of the address space
        (napot(0, CPU_SUBSYSTEM_LOW), NAPOT | NONE),
        // gap between LP TCM and HP TCM (internal SRAM)
        (tor(RTC_IRAM_HIGH), NONE),
        (tor(IRAM_LOW), TOR | NONE),
        // gap between internal SRAM and ROM
        (tor(IRAM_HIGH), NONE),
        (tor(IROM_MASK_LOW), TOR | NONE),
        // ROM: RX, cacheable
        (tor(DROM_MASK_HIGH), TOR | RX),
        // gap between ROM and flash
        (tor(IROM_LOW), TOR | NONE),
        // flash: RX, cacheable
        (tor(IROM_HIGH), TOR | RX),
        // gap between flash and PSRAM
        (tor(EXTRAM_LOW), TOR | NONE),
        // PSRAM: RWX, cacheable
        (tor(EXTRAM_HIGH), TOR | RWX),
        // gap between PSRAM and the cache-bypass alias of flash
        (tor(IROM_LOW + NON_CACHEABLE_OFFSET_FLASH), TOR | NONE),
        // cache-bypass alias of flash
        (tor(IROM_HIGH + NON_CACHEABLE_OFFSET_FLASH), TOR | RX),
        // gap between the flash and PSRAM cache-bypass aliases
        (tor(EXTRAM_LOW + NON_CACHEABLE_OFFSET_PSRAM), TOR | NONE),
        // cache-bypass alias of PSRAM
        (tor(EXTRAM_HIGH + NON_CACHEABLE_OFFSET_PSRAM), TOR | RW),
        // end of the address space
        (tor(u32::MAX), TOR | NONE),
        (0, 0),
    ]
};

/// `cpu_region_protect.c` for the ESP32-H4. `HP_RAM_HIGH` is `SOC_IRAM_HIGH`: ICache1
/// (`0x40860000`) stays outside the valid RAM window, as in the multi-core bootloader. SRAM itself
/// is left uncovered so it keeps the default executable attribute.
#[cfg(esp32h4)]
const ENTRIES: [(u32, u32); 16] = {
    const CPU_SUBSYSTEM_LOW: u32 = 0x2000_0000;
    const CPU_SUBSYSTEM_HIGH: u32 = 0x3000_0000;
    const IROM_MASK_LOW: u32 = 0x4000_0000;
    const DROM_MASK_HIGH: u32 = 0x4002_0000;
    const IRAM_LOW: u32 = 0x4081_0000;
    const HP_RAM_HIGH: u32 = 0x4086_0000;
    const IROM_LOW: u32 = 0x4200_0000;
    const IROM_HIGH: u32 = 0x4400_0000;
    const PERIPHERAL_LOW: u32 = 0x6000_0000;
    const PERIPHERAL_HIGH: u32 = 0x6010_0000;
    [
        (napot(0, CPU_SUBSYSTEM_LOW), NAPOT | NONE),
        (tor(CPU_SUBSYSTEM_HIGH), NONE),
        (tor(IROM_MASK_LOW), TOR | NONE),
        (tor(DROM_MASK_HIGH), TOR | RX),
        (tor(IRAM_LOW), TOR | NONE),
        (tor(HP_RAM_HIGH), NONE),
        (tor(IROM_LOW), TOR | NONE),
        (tor(IROM_HIGH), TOR | RWX),
        (tor(PERIPHERAL_LOW), TOR | NONE),
        (tor(PERIPHERAL_HIGH), NONE),
        (tor(u32::MAX), TOR | NONE),
        (0, 0),
        (0, 0),
        (0, 0),
        (0, 0),
        (0, 0),
    ]
};

/// Replaces the ROM's PMA entries with [`ENTRIES`].
///
/// This runs from flash, so flash must stay executable throughout. Each entry is reset just before
/// it's set: flash is covered by the new entries before the ROM's flash entry (one of the last
/// ones) is replaced, and lower entries take priority. The ESP32-H4's ROM layout is not
/// known, so that chip parks its own flash grant on entry 15 before clearing the rest.
pub(super) fn configure() {
    let cfg: [fn(u32); 16] = csr_array!(write_csr, 0xBC0);
    let addr: [fn(u32); 16] = csr_array!(write_csr, 0xBD0);

    #[cfg(esp32h4)]
    configure_h4(&cfg, &addr);

    #[cfg(not(esp32h4))]
    for (i, &(entry_addr, entry_cfg)) in ENTRIES.iter().enumerate() {
        cfg[i](0);
        addr[i](0);
        addr[i](entry_addr);
        cfg[i](entry_cfg);
    }
}

/// Entry 15 is never another entry's TOR base. Unless it is already the entry winning for the
/// PC, it is overwritten with an unlocked flash grant; every other entry is then disabled and the
/// final map is written under that grant.
#[cfg(esp32h4)]
fn configure_h4(cfg_w: &[fn(u32); 16], addr_w: &[fn(u32); 16]) {
    const FLASH_NAPOT: u32 = napot(0x4200_0000, 0x0200_0000);
    const FLASH_X: u32 = NAPOT | EN | R | W | X;

    let cfg_r: [fn() -> u32; 16] = csr_array!(read_csr, 0xBC0);
    let addr_r: [fn() -> u32; 16] = csr_array!(read_csr, 0xBD0);
    let mut cur_addr = [0u32; 16];
    let mut cur_cfg = [0u32; 16];
    for i in 0..16 {
        cur_cfg[i] = cfg_r[i]();
        cur_addr[i] = addr_r[i]();
    }

    let pc: u32;
    unsafe {
        asm!("auipc {0}, 0", out(reg) pc, options(nomem, nostack));
    }

    let mut lowest = None;
    for i in 0..16 {
        let prev = if i == 0 { 0 } else { cur_addr[i - 1] };
        if region_matches(cur_cfg[i], prev, cur_addr[i], pc) {
            lowest = Some(i);
            break;
        }
    }

    if lowest != Some(15) {
        addr_w[15](FLASH_NAPOT);
        cfg_w[15](FLASH_X);
        unsafe {
            asm!("fence", "fence.i", options(nostack));
        }
    }

    // High index first: a lower-priority entry is dropped before the one currently winning, so
    // the parked grant is what fetch falls through to.
    for i in (0..15).rev() {
        cfg_w[i](0);
    }
    unsafe {
        asm!("fence", "fence.i", options(nostack));
    }

    for i in 0..15 {
        let (entry_addr, entry_cfg) = ENTRIES[i];
        cfg_w[i](0);
        addr_w[i](0);
        addr_w[i](entry_addr);
        cfg_w[i](entry_cfg);
    }

    // Entry 7 now covers flash. Final entries 11..15 are off.
    cfg_w[15](0);
    addr_w[15](0);
    unsafe {
        asm!("fence", "fence.i", options(nostack));
    }
}

#[cfg(esp32h4)]
fn region_matches(cfg: u32, prev_addr: u32, addr: u32, pc: u32) -> bool {
    let pc = pc as u64;
    match cfg & 0xC000_0000 {
        TOR => {
            let start = (prev_addr as u64) << 2;
            let end = (addr as u64) << 2;
            pc >= start && pc < end
        }
        0x8000_0000 => {
            let base = (addr as u64) << 2;
            pc >= base && pc < base + 4
        }
        NAPOT => napot_contains(addr, pc),
        _ => false,
    }
}

#[cfg(esp32h4)]
fn napot_contains(pmaaddr: u32, pc: u64) -> bool {
    let mut ones = 0u32;
    let mut bits = pmaaddr;
    while bits & 1 == 1 {
        ones += 1;
        bits >>= 1;
        if ones == 32 {
            break;
        }
    }
    // `2^(ones + 3)` bytes; 29 ones already covers the 32-bit space.
    if ones >= 29 {
        return true;
    }
    let size = 1u64 << (ones + 3);
    let base = ((pmaaddr & !((1u32 << ones) - 1)) as u64) << 2;
    pc >= base && pc < base + size
}
