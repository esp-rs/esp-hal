#![no_std]

use core::arch::global_asm;

pub use embedded_hal as ehal;
pub use esp_hal_common::{
    clock,
    efuse,
    gpio as gpio_types,
    i2c,
    interrupt,
    ledc,
    macros,
    pac,
    prelude,
    pulse_control,
    serial,
    spi,
    system,
    systimer,
    timer,
    utils,
    Cpu,
    Delay,
    PulseControl,
    Rng,
    Rtc,
    Serial,
    UsbSerialJtag,
};
#[cfg(feature = "direct-boot")]
use riscv_rt::pre_init;

pub use self::gpio::IO;

pub mod adc;
pub mod gpio;

/// Common module for analog functions
pub mod analog {
    pub use esp_hal_common::analog::{AvailableAnalog, SarAdcExt};
}

extern "C" {
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
    addi sp, sp, -32*4

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

    addi s0, sp, 32*4
    sw s0, 30*4(sp)

    add a0, sp, zero
    jal ra, _start_trap_rust_hal

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

global_asm!(
    r#"
.section .init, "ax"
.global _start_hal

_start_hal:
    /* Jump to the absolute address defined by the linker script. */
    lui ra, %hi(_abs_start_hal)
    jr %lo(_abs_start_hal)(ra)
"#
);

global_asm!(
    r#"
.section .text

_abs_start_hal:
    .option norelax
    .cfi_startproc
    .cfi_undefined ra

    // unsupported on ESP32C3
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

    .cfi_endproc
"#
);

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

#[allow(unreachable_code)]
#[export_name = "_mp_hook"]
#[doc(hidden)]
pub fn mp_hook() -> bool {
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

fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    int_enable as u8 | ((nmi_enable as u8) << 1)
}

#[no_mangle]
extern "C" fn EspDefaultHandler(_interrupt: pac::Interrupt) {}
