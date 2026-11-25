//! Entry stub for the ESP32-S3.
//!
//! The ROM jumps to `_esp_bootloader_entry` (flash offset of the magic + 8) with nothing loaded
//! into RAM. `Reset` (xtensa-lx-rt) and [`early_init`] are windowed functions in `.rwtext`, so
//! they can only be called once the copy is done. The register window state the ROM leaves behind
//! is reset first, so that their `entry` can't trigger a window overflow into ROM frames.

core::arch::global_asm!(
    r#"
    .pushsection .direct_boot.entry, "ax"
    .global _esp_bootloader_entry
    .type _esp_bootloader_entry, @function
_esp_bootloader_entry:
    j    .Ldb_start

    .p2align 2
.Ldb_table_start:
    .word _direct_boot_copy_table_start
.Ldb_table_end:
    .word _direct_boot_copy_table_end
.Ldb_stack:
    .word _stack_start_cpu0
.Ldb_reset:
    .word Reset
.Ldb_early_init:
    .word {early_init}
.Ldb_ps:
    // PS.WOE | PS.UM, INTLEVEL 0
    .word 0x00040020

.Ldb_start:
    movi   a0, 0
    wsr.intenable a0
    rsync

    l32r   a2, .Ldb_table_start
    l32r   a3, .Ldb_table_end
.Ldb_next_entry:
    bgeu   a2, a3, .Ldb_copy_done
    l32i   a4, a2, 0
    l32i   a5, a2, 4
    l32i   a6, a2, 8
    addi   a2, a2, 12
.Ldb_copy_word:
    bgeu   a5, a6, .Ldb_next_entry
    l32i   a7, a4, 0
    s32i   a7, a5, 0
    addi   a4, a4, 4
    addi   a5, a5, 4
    j      .Ldb_copy_word

.Ldb_copy_done:
    memw

    movi   a0, 0
    wsr.windowbase a0
    rsync
    movi   a0, 1
    wsr.windowstart a0
    rsync
    l32r   a0, .Ldb_ps
    wsr.ps a0
    rsync

    l32r   a1, .Ldb_stack
    movi   a0, 0
    l32r   a8, .Ldb_early_init
    callx4 a8

    movi   a0, 0
    l32r   a8, .Ldb_reset
    callx4 a8
    .size _esp_bootloader_entry, . - _esp_bootloader_entry
    .popsection
"#,
    early_init = sym early_init,
);

/// The ROM jumps to the entry point with cache operations still in flight. esp-hal reconfigures
/// the caches in `__pre_init`, before `__esp_bootloader_init` runs, through the ROM's
/// `Cache_Suspend_DCache`. That returns before the cache is idle, and the data cache then hangs on
/// the next miss. Once the caches are idle, this loads RTC memory.
///
/// Runs from RAM while the caches may still be busy, and before `.bss` is zeroed, so this must
/// not touch any statics.
///
/// See <https://github.com/espressif/esp-idf/blob/fff9895c82d744c7237be8847347bdd1b07c6643/components/esp_rom/patches/esp_rom_cache_esp32s2_esp32s3.c#L37-L67>.
#[unsafe(link_section = ".rwtext")]
extern "C" fn early_init() {
    let extmem = esp_hal::peripherals::EXTMEM::regs();
    loop {
        let state = extmem.cache_state().read();
        if state.icache_state().bits() == 1 && state.dcache_state().bits() == 1 {
            break;
        }
    }

    crate::load_rtc_memory();
}
