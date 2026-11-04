//! Entry stub for RISC-V chips.
//!
//! The ROM jumps to `_esp_bootloader_entry` (flash offset of the magic + 8) with nothing loaded
//! into RAM. The stub copies every entry of the linker-generated copy table from flash to RAM
//! without using a stack, sets up the stack, runs [`early_init`] and continues with `_start` of
//! riscv-rt.

#[cfg(rom_pma)]
mod pma;

// The ROM leaves the L2 cache split it booted with. The RAM sections are linked for the split
// `ESP_HAL_CONFIG_L2_CACHE_SIZE` selects, so program it before copying anything there.
// `Cache_Set_L2_Cache_Mode` runs from ROM, on the stack the ROM left behind, while nothing is
// fetched from flash.
#[cfg(esp32p4)]
macro_rules! l2_cache_setup {
    () => {
        r#"
    la   a0, _direct_boot_l2_cache_size
    li   a1, 8 # ways
    li   a2, 3 # CACHE_LINE_SIZE_64B
    call Cache_Set_L2_Cache_Mode
"#
    };
}

#[cfg(not(esp32p4))]
macro_rules! l2_cache_setup {
    () => {
        ""
    };
}

core::arch::global_asm!(
    concat!(
        r#"
    .pushsection .direct_boot.entry, "ax"
    .global _esp_bootloader_entry
    .type _esp_bootloader_entry, @function
    .option push
    .option norelax
_esp_bootloader_entry:
"#,
        l2_cache_setup!(),
        r#"
    la   t0, _direct_boot_copy_table_start
    la   t1, _direct_boot_copy_table_end
1:
    bgeu t0, t1, 3f
    lw   t2, 0(t0)
    lw   t3, 4(t0)
    lw   t4, 8(t0)
    addi t0, t0, 12
2:
    bgeu t3, t4, 1b
    lw   t5, 0(t2)
    sw   t5, 0(t3)
    addi t2, t2, 4
    addi t3, t3, 4
    j    2b
3:
    fence
    fence.i
    la   sp, _stack_start
    call {early_init}
    la   t0, _start
    jr   t0
    .option pop
    .size _esp_bootloader_entry, . - _esp_bootloader_entry
    .popsection
"#
    ),
    early_init = sym early_init,
);

/// Hardware setup the 2nd-stage bootloader would otherwise have done before the runtime starts.
///
/// Runs from flash with a stack, but before riscv-rt initialized anything else: `.bss` is not
/// zeroed yet and `gp` is not set up, so this must not touch any statics.
extern "C" fn early_init() {
    #[cfg(esp32p4)]
    cache::sync_copied_sections();

    #[cfg(rom_pma)]
    pma::configure();
}

/// The internal SRAM of the ESP32-P4 sits behind the L1 caches: the copy loop leaves the code it
/// copied in the write-back data cache, where instruction fetches don't see it.
#[cfg(esp32p4)]
mod cache {
    const CACHE_MAP_L1_ICACHE_0: u32 = 1 << 0;
    const CACHE_MAP_L1_ICACHE_1: u32 = 1 << 1;
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;

    unsafe extern "C" {
        fn Cache_WriteBack_Addr(map: u32, addr: u32, size: u32);
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
        static _direct_boot_copy_table_start: [u32; 3];
        static _direct_boot_copy_table_end: [u32; 3];
    }

    pub(super) fn sync_copied_sections() {
        let mut entry = &raw const _direct_boot_copy_table_start;
        while entry < &raw const _direct_boot_copy_table_end {
            unsafe {
                let [_, start, end] = entry.read();
                if end > start {
                    Cache_WriteBack_Addr(CACHE_MAP_L1_DCACHE, start, end - start);
                    Cache_Invalidate_Addr(
                        CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1,
                        start,
                        end - start,
                    );
                }
                entry = entry.add(1);
            }
        }
    }
}
