#![cfg_attr(
    all(docsrs, not(not_really_docsrs)),
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.espressif.com/projects/rust/'>browse the <code>esp-lp-hal</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C6, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs change depending on the chip.</p></div>\n\n<br/>\n\n"
)]
//! Bare-metal (`no_std`) HAL for the low power and ultra-low power cores found
//! in some Espressif devices. Where applicable, drivers implement the
//! [embedded-hal] traits.
//!
//! ## Choosing a device
//!
//! Depending on your target device, you need to enable the chip feature
//! for that device.
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(asm_sub_register)]
#![deny(missing_docs)]
#![no_std]

#[allow(unused_imports, reason = "Only used for some MCUs currently")]
#[macro_use]
extern crate esp_metadata_generated;

use core::arch::global_asm;

pub mod delay;
pub mod gpio;
#[cfg(lp_i2c_master_driver_supported)]
pub mod i2c;
#[cfg(lp_uart_driver_supported)]
pub mod uart;

#[cfg(esp32c6)]
pub use esp32c6_lp as pac;
#[cfg(esp32s2)]
pub use esp32s2_ulp as pac;
#[cfg(esp32s3)]
pub use esp32s3_ulp as pac;

/// The prelude
pub mod prelude {
    pub use procmacros::entry;
}

cfg_if::cfg_if! {
    if #[cfg(esp32c6)] {
        // LP_FAST_CLK is not very accurate, for now use a rough estimate
        const LP_FAST_CLK_HZ: u32 = 16_000_000;
        const XTAL_D2_CLK_HZ: u32 = 20_000_000;
    } else if #[cfg(esp32s2)] {
        const LP_FAST_CLK_HZ: u32 = 8_000_000;
    } else if #[cfg(esp32s3)] {
        const LP_FAST_CLK_HZ: u32 = 17_500_000;
    }
}

// ULP interrupt bitflags from:
// https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L16
cfg_if::cfg_if! {
    if #[cfg(any(esp32s2,esp32s3))]
    {
        #[allow(unused)]
        const ULP_RISCV_TIMER_INT                         : u32 = 1 << 0;   /* Internal Timer Interrupt */
        #[allow(unused)]
        const ULP_RISCV_EBREAK_ECALL_ILLEGAL_INSN_INT     : u32 = 1 << 1;   /* EBREAK, ECALL or Illegal instruction */
        #[allow(unused)]
        const ULP_RISCV_BUS_ERROR_INT                     : u32 = 1 << 2;   /* Bus Error (Unaligned Memory Access) */
        #[allow(unused)]
        const ULP_RISCV_PERIPHERAL_INTERRUPT              : u32 = 1 << 31;  /* RTC Peripheral Interrupt */
        #[allow(unused)]
        const ULP_RISCV_INTERNAL_INTERRUPT                : u32 = ULP_RISCV_TIMER_INT | ULP_RISCV_EBREAK_ECALL_ILLEGAL_INSN_INT | ULP_RISCV_BUS_ERROR_INT;
    }
}

pub(crate) static mut CPU_CLOCK: u32 = LP_FAST_CLK_HZ;

/// Wake up the HP core
pub fn wake_hp_core() {
    #[cfg(esp32c6)]
    unsafe { &*esp32c6_lp::PMU::PTR }
        .hp_lp_cpu_comm()
        .write(|w| w.lp_trigger_hp().set_bit());
    #[cfg(esp32s2)]
    unsafe { &*esp32s2_ulp::RTC_CNTL::PTR }
        .state0()
        .write(|w| w.rtc_sw_cpu_int().set_bit());
    #[cfg(esp32s3)]
    unsafe { &*esp32s3_ulp::RTC_CNTL::PTR }
        .rtc_state0()
        .write(|w| w.rtc_sw_cpu_int().set_bit());
}

#[cfg(esp32c6)]
global_asm!(
    r#"
    .section    .init.vector, "ax"
    /* This is the vector table. It is currently empty, but will be populated
     * with exception and interrupt handlers when this is supported
     */

    .align  0x4, 0xff
    .global _vector_table
    .type _vector_table, @function
_vector_table:
    .option push
    .option norvc

    .rept 32
    nop
    .endr

    .option pop
    .size _vector_table, .-_vector_table

    .section .init, "ax"
    .global reset_vector

/* The reset vector, jumps to startup code */
reset_vector:
    j __start

__start:
    /* setup the stack pointer */
    la sp, __stack_top
    call rust_main
loop:
    j loop
"#
);

// Include macros which define custom RISCV R-Type instructions, used for interrupt handling.
#[cfg(any(esp32s2, esp32s3))]
global_asm!(include_str!("./ulp_riscv_interrupt_ops.S"));
// Assembly containing the reset, trap, and startup assembly procedures..
#[cfg(any(esp32s2, esp32s3))]
global_asm!(include_str!("./ulp_riscv_vectors.S"));

/// Called prior to main(), to prepare ULP core
/// for execution, and enable interrupts.
#[cfg(any(esp32s2, esp32s3))]
#[unsafe(link_section = ".init.rust")]
#[inline(always)]
pub fn lp_core_pre_init() {
    // Exit 'monitor mode'
    unsafe { ulp_riscv_rescue_from_monitor() };
    // Unmask the interrupts
    ulp_interrupts_enable();
}

/// Entry point to the ULP program
#[unsafe(link_section = ".init.rust")]
#[unsafe(export_name = "_start_rust")]
unsafe extern "C" fn lp_core_startup() -> ! {
    unsafe {
        unsafe extern "Rust" {
            // This symbol will be provided by the user via `#[entry]`
            fn main();
        }

        #[cfg(any(esp32s2, esp32s3))]
        lp_core_pre_init();

        #[cfg(esp32c6)]
        if (*pac::LP_CLKRST::PTR)
            .lp_clk_conf()
            .read()
            .fast_clk_sel()
            .bit_is_set()
        {
            CPU_CLOCK = XTAL_D2_CLK_HZ;
        }

        main();
        ulp_riscv_halt();
    }
}

#[cfg(any(esp32s2, esp32s3))]
#[unsafe(link_section = ".init.rust")]
#[unsafe(no_mangle)]
unsafe extern "C" fn ulp_riscv_rescue_from_monitor() {
    // Rescue RISC-V core from monitor state.
    unsafe { &*pac::RTC_CNTL::PTR }
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_done().clear_bit().cocpu_shut_reset_en().clear_bit());
}

/// Stops the ULP core, called from itself.
#[unsafe(link_section = ".init.rust")]
fn ulp_riscv_halt() -> ! {
    #[cfg(any(esp32s2, esp32s3))]
    {
        unsafe { &*pac::RTC_CNTL::PTR }
            .cocpu_ctrl()
            .modify(|_, w| unsafe {
                w.cocpu_shut_2_clk_dis().bits(0x3F);
                w.cocpu_done().set_bit();
                w.cocpu_shut_reset_en().set_bit()
            });
    }

    // All chips will enter a no-op loop, when halting.
    loop {
        unsafe {
            core::arch::asm!("addi x0, x0, 0"); // no-op
        }
    }
}

/// TODO: Move custom ULP instructions into their own module,
///       and cleanup the assembly files at the same time.
/// Disable all interrupts
#[cfg(any(esp32s2, esp32s3))]
#[inline(always)]
pub fn ulp_interrupts_disable() {
    // Enter a critical section by disabling all interrupts
    // This inline assembly construct uses the t0 register and is equivalent to:
    // > li t0, 0x80000007
    // > maskirq_insn(zero, t0) // Mask all interrupt bits
    // The mask 0x80000007 represents:
    //   Bit 31 - RTC peripheral interrupt
    //   Bit 2  - Bus error
    //   Bit 1  - Ebreak / Ecall / Illegal Instruction
    //   Bit 0  - Internal Timer
    //
    unsafe {
        core::arch::asm!("li t0, 0x80000007", ".word 0x0602e00b");
    }
}

/// Enable all interrupts
#[cfg(any(esp32s2, esp32s3))]
#[inline(always)]
pub fn ulp_interrupts_enable() {
    // Exit a critical section by enabling all interrupts
    // This inline assembly construct is equivalent to:
    // > maskirq_insn(zero, zero)
    unsafe {
        core::arch::asm!(".word 0x0600600b");
    }
}

/// Wait for any (masked or unmasked) interrupt
#[cfg(any(esp32s2, esp32s3))]
pub fn ulp_waitirq() -> u32 {
    // Wait for pending interrupt, return pending interrupt mask
    // waitirq a0
    let result: u32;
    unsafe {
        core::arch::asm!(".word 0x0800400B", out("a0") result);
    }
    result
}

/// TODO: Write store_trap / load_trap functions, to generate the context saving assembly code,
//        which is currently hand-written in ulp_riscv_vectors.S
/// Registers saved in trap handler
#[cfg(any(esp32s2, esp32s3))]
#[repr(C)]
#[derive(Debug)]
pub struct TrapFrame {
    /// `x1`: return address, stores the address to return to after a function call or interrupt.
    pub ra: usize,
    /// `x5`: temporary register `t0`, used for intermediate values.
    pub t0: usize,
    /// `x6`: temporary register `t1`, used for intermediate values.
    pub t1: usize,
    /// `x7`: temporary register `t2`, used for intermediate values.
    pub t2: usize,
    /// `x28`: temporary register `t3`, used for intermediate values.
    pub t3: usize,
    /// `x29`: temporary register `t4`, used for intermediate values.
    pub t4: usize,
    /// `x30`: temporary register `t5`, used for intermediate values.
    pub t5: usize,
    /// `x31`: temporary register `t6`, used for intermediate values.
    pub t6: usize,
    /// `x10`: argument register `a0`. Used to pass the first argument to a function.
    pub a0: usize,
    /// `x11`: argument register `a1`. Used to pass the second argument to a function.
    pub a1: usize,
    /// `x12`: argument register `a2`. Used to pass the third argument to a function.
    pub a2: usize,
    /// `x13`: argument register `a3`. Used to pass the fourth argument to a function.
    pub a3: usize,
    /// `x14`: argument register `a4`. Used to pass the fifth argument to a function.
    pub a4: usize,
    /// `x15`: argument register `a5`. Used to pass the sixth argument to a function.
    pub a5: usize,
    /// `x16`: argument register `a6`. Used to pass the seventh argument to a function.
    pub a6: usize,
    /// `x17`: argument register `a7`. Used to pass the eighth argument to a function.
    pub a7: usize,
}

/// Trap entry point rust (_start_trap_rust)
/// `irqs` is a bitmask of IRQs to handle.
#[cfg(any(esp32s2, esp32s3))]
#[unsafe(link_section = ".trap.rust")]
#[unsafe(export_name = "_start_trap_rust")]
pub extern "C" fn ulp_start_trap_rust(trap_frame: *const TrapFrame, irqs: u32) {
    unsafe extern "C" {
        fn trap_handler(regs: &TrapFrame, pending_irqs: u32);
    }

    unsafe {
        // dispatch trap to handler
        trap_handler(&*trap_frame, irqs);
    }
}

/// Creates the trap_handler() function.
/// This is macro is used later in this file.
/// This style of macro is usually provided
/// for users to hook their own handlers, but here it's just used
/// as a quick way to generate the bit-mask code :)
#[cfg(any(esp32s2, esp32s3))]
macro_rules! ulp_interrupts {
    (@interrupt ($n:literal, $pending_irqs:expr, $regs:expr, $handler:ident)) => {
        if $pending_irqs & (1 << $n) != 0 {
            #[allow(unused_unsafe)]
            unsafe { $handler($regs); }
        }
    };
    ( $( $irq:literal : $handler:ident ),* ) => {
        /// Called by _start_trap_rust, this trap handler will call other interrupt handling
        /// functions depending on the bits set in pending_irqs.
        #[unsafe(no_mangle)]
        pub extern "C" fn trap_handler(regs: *const TrapFrame, pending_irqs: u32) {
            let regs = unsafe { regs.as_ref().unwrap() };
            $(
                ulp_interrupts!(@interrupt($irq, pending_irqs, regs, $handler));
            )*
        }
    };
}

/// Default timer interrupt handler.
/// Users may override TimerInterrupt to change this behaviour.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[unsafe(no_mangle)]
pub fn default_timer_interrupt(_regs: &TrapFrame) {
    // Does nothing.
}

/// Default illegal instruction or bus error exception handler.
/// Users may override IllegalInstructionException, and/or BusErrorException, to change this
/// behaviour.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[unsafe(no_mangle)]
pub fn default_exception_handler(_regs: &TrapFrame) {
    panic!("Unhandled exception!");
}

/// Default interrupt handler for RTC Peripheral interrupts (SENS).
/// Users may override SensInterrupt to change this behaviour.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[unsafe(no_mangle)]
pub fn default_sens_interrupt(_sar_cocpu_int_st: u32) {
    // does nothing
}

/// Default interrupt handler for RTC IO interrupts (GPIO).
/// Users may override GpioInterrupt to change this behaviour.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[unsafe(no_mangle)]
pub fn default_gpio_interrupt(_pin_status: u32) {
    // pin status contains the GPIO irq bits.
    // it has already been right-shifted by 10,
    // so Bit 0 of pin_status == GPIO0  :)
}

unsafe extern "Rust" {
    fn TimerInterrupt(regs: &TrapFrame);
    fn IllegalInstructionException(regs: &TrapFrame);
    fn BusErrorException(regs: &TrapFrame);
}

/// Peripheral interrupt handler for the IRQ bit 31.
/// Checks the peripheral interrupt flags, and calls further interrupt handlers as appropriate.
/// Clears interrupt flags automatically.
#[cfg(any(esp32s2, esp32s3))]
#[unsafe(no_mangle)]
fn _dispatch_peripheral_interrupt(_regs: &TrapFrame) {
    // This function is based on the ESP-IDF implementation found here:
    // https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L110

    unsafe extern "Rust" {
        fn SensInterrupt(sar_cocpu_int_st: u32);
        fn GpioInterrupt(pin_status: u32);
    }

    // RTC Peripheral interrupts
    let cocpu_int_st: u32 = unsafe { &*pac::SENS::PTR }.sar_cocpu_int_st().read().bits();
    // RTC IO interrupts
    let rtcio_int_st: u32 = unsafe { &*pac::RTC_IO::PTR }.status().read().bits();

    // Call handler, and clear the interrupt flags, if they were raised at the start of this ISR
    // call.

    if cocpu_int_st > 0 {
        unsafe {
            SensInterrupt(cocpu_int_st);
        }

        // Clear the interrupt
        unsafe { &*pac::SENS::PTR }
            .sar_cocpu_int_clr()
            .write(|w| unsafe { w.bits(cocpu_int_st) });
    }

    if rtcio_int_st > 0 {
        // Right-shift by 10, to remove the offset, and make it so GPIO0 == Bit 0.
        unsafe {
            GpioInterrupt(rtcio_int_st >> 10);
        }

        // Clear the interrupt
        unsafe { &*pac::RTC_IO::PTR }
            .status_w1tc()
            .write(|w| unsafe { w.bits(rtcio_int_st) });
    }
}

// Define the trap_handler function,
// which calls interrupt symbol names,
// which may be overriden by the user at link-time.
ulp_interrupts!(
    0: TimerInterrupt,
    1: IllegalInstructionException,
    2: BusErrorException,
    31: _dispatch_peripheral_interrupt
);
