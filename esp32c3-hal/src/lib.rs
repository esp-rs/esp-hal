#![no_std]

use core::arch::{asm, global_asm};
#[cfg(feature = "mcu-boot")]
use core::mem::size_of;

pub use embedded_hal as ehal;
#[doc(inline)]
pub use esp_hal_common::{
    analog::adc::implementation as adc,
    clock,
    dma,
    dma::gdma,
    efuse,
    gpio,
    i2c,
    i2s,
    interrupt,
    ledc,
    macros,
    peripherals,
    prelude,
    pulse_control,
    spi,
    system,
    systimer,
    timer,
    uart,
    utils,
    Cpu,
    Delay,
    PulseControl,
    Rng,
    Rtc,
    Rwdt,
    Uart,
    UsbSerialJtag,
    sha
};

#[cfg(feature = "embassy")]
pub use esp_hal_common::embassy;

#[cfg(feature = "direct-boot")]
use riscv_rt::pre_init;

pub use self::gpio::IO;

/// Common module for analog functions
pub mod analog {
    pub use esp_hal_common::analog::{AvailableAnalog, SarAdcExt};
}

extern "C" {
    cfg_if::cfg_if! {
        if #[cfg(feature = "mcu-boot")] {
            // Functions from internal ROM
            fn cache_suspend_icache() -> u32;
            fn cache_resume_icache(val: u32);
            fn cache_invalidate_icache_all();
            fn cache_dbus_mmu_set(
                ext_ram: u32,
                vaddr: u32,
                paddr: u32,
                psize: u32,
                num: u32,
                fixed: u32,
            ) -> i32;
            fn cache_ibus_mmu_set(
                ext_ram: u32,
                vaddr: u32,
                paddr: u32,
                psize: u32,
                num: u32,
                fixed: u32,
            ) -> i32;

            /* IROM metadata:
             * - Destination address (VMA) for IROM region
             * - Flash offset (LMA) for start of IROM region
             * - Size of IROM region
             */
            static mut _image_irom_vma: u32;
            static mut _image_irom_lma: u32;
            static mut _image_irom_size: u32;

            /* DROM metadata:
             * - Destination address (VMA) for DROM region
             * - Flash offset (LMA) for start of DROM region
             * - Size of DROM region
             */
            static mut _image_drom_vma: u32;
            static mut _image_drom_lma: u32;
            static mut _image_drom_size: u32;
        }
    }

    // Boundaries of the .iram section
    static mut _srwtext: u32;
    static mut _erwtext: u32;
    static mut _irwtext: u32;

    // Boundaries of the .bss section
    static mut _ebss: u32;
    static mut _sbss: u32;

    // Boundaries of the rtc .bss section
    static mut _rtc_fast_bss_start: u32;
    static mut _rtc_fast_bss_end: u32;

    // Boundaries of the .rtc_fast.text section
    static mut _srtc_fast_text: u32;
    static mut _ertc_fast_text: u32;
    static mut _irtc_fast_text: u32;

    // Boundaries of the .rtc_fast.data section
    static mut _rtc_fast_data_start: u32;
    static mut _rtc_fast_data_end: u32;
    static mut _irtc_fast_data: u32;
}

global_asm!(
    r#"
.section .trap, "ax"
.balign 0x100
.global _vector_table_hal
.type _vector_table_hal, @function
.option norelax

_vector_table_hal:
    .option push
    .option norvc
    .rept 31
    j _start_trap_hal
    .endr
"#
);

global_asm!(
    r#"
    /*
    Trap entry point (_start_trap_hal)
    Saves registers and calls _start_trap_rust_hal,
    restores registers and then returns.
*/
.section .trap, "ax"
.global _start_trap_hal
.option norelax
.align 6

_start_trap_hal:
    addi sp, sp, -40*4

    sw ra, 0*4(sp)
    sw t0, 1*4(sp)
    sw t1, 2*4(sp)
    sw t2, 3*4(sp)
    sw t3, 4*4(sp)
    sw t4, 5*4(sp)
    sw t5, 6*4(sp)
    sw t6, 7*4(sp)
    sw a0, 8*4(sp)
    sw a1, 9*4(sp)
    sw a2, 10*4(sp)
    sw a3, 11*4(sp)
    sw a4, 12*4(sp)
    sw a5, 13*4(sp)
    sw a6, 14*4(sp)
    sw a7, 15*4(sp)
    sw s0, 16*4(sp)
    sw s1, 17*4(sp)
    sw s2, 18*4(sp)
    sw s3, 19*4(sp)
    sw s4, 20*4(sp)
    sw s5, 21*4(sp)
    sw s6, 22*4(sp)
    sw s7, 23*4(sp)
    sw s8, 24*4(sp)
    sw s9, 25*4(sp)
    sw s10, 26*4(sp)
    sw s11, 27*4(sp)
    sw gp, 28*4(sp)
    sw tp, 29*4(sp)
    csrrs t1, mepc, x0
    sw t1, 31*4(sp)
    csrrs t1, mstatus, x0
    sw t1, 32*4(sp)
    csrrs t1, mcause, x0
    sw t1, 33*4(sp)
    csrrs t1, mtval, x0
    sw t1, 34*4(sp)

    addi s0, sp, 40*4
    sw s0, 30*4(sp)

    add a0, sp, zero
    jal ra, _start_trap_rust_hal

    lw t1, 31*4(sp)
    csrrw x0, mepc, t1

    lw t1, 32*4(sp)
    csrrw x0, mstatus, t1

    lw ra, 0*4(sp)
    lw t0, 1*4(sp)
    lw t1, 2*4(sp)
    lw t2, 3*4(sp)
    lw t3, 4*4(sp)
    lw t4, 5*4(sp)
    lw t5, 6*4(sp)
    lw t6, 7*4(sp)
    lw a0, 8*4(sp)
    lw a1, 9*4(sp)
    lw a2, 10*4(sp)
    lw a3, 11*4(sp)
    lw a4, 12*4(sp)
    lw a5, 13*4(sp)
    lw a6, 14*4(sp)
    lw a7, 15*4(sp)
    lw s0, 16*4(sp)
    lw s1, 17*4(sp)
    lw s2, 18*4(sp)
    lw s3, 19*4(sp)
    lw s4, 20*4(sp)
    lw s5, 21*4(sp)
    lw s6, 22*4(sp)
    lw s7, 23*4(sp)
    lw s8, 24*4(sp)
    lw s9, 25*4(sp)
    lw s10, 26*4(sp)
    lw s11, 27*4(sp)
    lw gp, 28*4(sp)
    lw tp, 29*4(sp)
    lw sp, 30*4(sp)

    # SP was restored from the original SP
    mret

"#
);

#[cfg(feature = "mcu-boot")]
#[link_section = ".entry_addr"]
#[no_mangle]
#[used]
// Entry point address for the MCUboot image header
static ENTRY_POINT: unsafe fn() -> ! = start_hal;

#[link_section = ".init"]
#[export_name = "_start_hal"]
unsafe fn start_hal() -> ! {
    asm!(
        r#"
        .option norelax

        // unsupported on ESP32-C3
        // csrw mie, 0
        // csrw mip, 0

        li  x1, 0
        li  x2, 0
        li  x3, 0
        li  x4, 0
        li  x5, 0
        li  x6, 0
        li  x7, 0
        li  x8, 0
        li  x9, 0
        li  x10,0
        li  x11,0
        li  x12,0
        li  x13,0
        li  x14,0
        li  x15,0
        li  x16,0
        li  x17,0
        li  x18,0
        li  x19,0
        li  x20,0
        li  x21,0
        li  x22,0
        li  x23,0
        li  x24,0
        li  x25,0
        li  x26,0
        li  x27,0
        li  x28,0
        li  x29,0
        li  x30,0
        li  x31,0

        .option push
        .option norelax
        la gp, __global_pointer$
        .option pop

        // Check hart id
        csrr a2, mhartid
        lui t0, %hi(_max_hart_id)
        add t0, t0, %lo(_max_hart_id)
        bgtu a2, t0, abort_hal

        // Allocate stacks
        la sp, _stack_start
        lui t0, %hi(_hart_stack_size)
        add t0, t0, %lo(_hart_stack_size)

        beqz a2, 2f  // Jump if single-hart
        mv t1, a2
        mv t2, t0
    1:
        add t0, t0, t2
        addi t1, t1, -1
        bnez t1, 1b
    2:
        sub sp, sp, t0

        // Set frame pointer
        add s0, sp, zero

        jal zero, _start_rust
    "#
    );

    unreachable!()
}

global_asm!(
    r#"
/* Make sure there is an abort when linking */
.globl abort_hal
abort_hal:
    j abort_hal
"#
);

#[cfg(feature = "direct-boot")]
#[doc(hidden)]
#[pre_init]
unsafe fn init() {
    r0::init_data(&mut _srwtext, &mut _erwtext, &_irwtext);

    r0::init_data(
        &mut _rtc_fast_data_start,
        &mut _rtc_fast_data_end,
        &_irtc_fast_data,
    );

    r0::init_data(&mut _srtc_fast_text, &mut _ertc_fast_text, &_irtc_fast_text);
}

#[cfg(feature = "mcu-boot")]
#[link_section = ".rwtext"]
unsafe fn configure_mmu() {
    const PARTITION_OFFSET: u32 = 0x10000;
    let app_irom_lma = PARTITION_OFFSET + ((&_image_irom_lma as *const u32) as u32);
    let app_irom_size = (&_image_irom_size as *const u32) as u32;
    let app_irom_vma = (&_image_irom_vma as *const u32) as u32;
    let app_drom_lma = PARTITION_OFFSET + ((&_image_drom_lma as *const u32) as u32);
    let app_drom_size = (&_image_drom_size as *const u32) as u32;
    let app_drom_vma = (&_image_drom_vma as *const u32) as u32;

    let autoload = cache_suspend_icache();
    cache_invalidate_icache_all();

    // Clear the MMU entries that are already set up, so the new app only has
    // the mappings it creates.

    const FLASH_MMU_TABLE: *mut u32 = 0x600c_5000 as *mut u32;
    const ICACHE_MMU_SIZE: usize = 0x200;
    const FLASH_MMU_TABLE_SIZE: usize = ICACHE_MMU_SIZE / size_of::<u32>();
    const MMU_TABLE_INVALID_VAL: u32 = 0x100;

    for i in 0..FLASH_MMU_TABLE_SIZE {
        FLASH_MMU_TABLE.add(i).write_volatile(MMU_TABLE_INVALID_VAL);
    }

    const MMU_BLOCK_SIZE: u32 = 0x0001_0000;
    const MMU_FLASH_MASK: u32 = !(MMU_BLOCK_SIZE - 1);

    let calc_mmu_pages = |size, vaddr| {
        (size + (vaddr - (vaddr & MMU_FLASH_MASK)) + MMU_BLOCK_SIZE - 1) / MMU_BLOCK_SIZE
    };

    let drom_lma_aligned = app_drom_lma & MMU_FLASH_MASK;
    let drom_vma_aligned = app_drom_vma & MMU_FLASH_MASK;
    let drom_page_count = calc_mmu_pages(app_drom_size, app_drom_vma);
    cache_dbus_mmu_set(
        0,
        drom_vma_aligned,
        drom_lma_aligned,
        64,
        drom_page_count,
        0,
    );

    let irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
    let irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
    let irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);
    cache_ibus_mmu_set(
        0,
        irom_vma_aligned,
        irom_lma_aligned,
        64,
        irom_page_count,
        0,
    );

    let peripherals = pac::Peripherals::steal();
    peripherals.EXTMEM.icache_ctrl1.modify(|_, w| {
        w.icache_shut_ibus()
            .clear_bit()
            .icache_shut_dbus()
            .clear_bit()
    });

    cache_resume_icache(autoload);
}

#[allow(unreachable_code)]
#[export_name = "_mp_hook"]
#[doc(hidden)]
#[cfg_attr(feature = "mcu-boot", link_section = ".rwtext")]
pub fn mp_hook() -> bool {
    #[cfg(feature = "mcu-boot")]
    unsafe {
        configure_mmu();
    }

    unsafe {
        r0::zero_bss(&mut _rtc_fast_bss_start, &mut _rtc_fast_bss_end);
    }

    #[cfg(feature = "direct-boot")]
    return true;

    // no init data when using normal boot - but we need to zero out BSS
    unsafe {
        r0::zero_bss(&mut _sbss, &mut _ebss);
    }

    false
}

#[no_mangle]
extern "C" fn EspDefaultHandler(_interrupt: peripherals::Interrupt) {}
