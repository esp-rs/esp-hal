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
#[cfg(any(esp32s2, esp32s3))]
pub use pac::interrupt as sens_interrupt;

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

/// Entry point to the ULP program
#[unsafe(link_section = ".init.rust")]
#[unsafe(export_name = "rust_main")]
unsafe extern "C" fn lp_core_startup() -> ! {
    unsafe {
        unsafe extern "Rust" {
            // This symbol will be provided by the user via `#[entry]`
            fn main();
        }

        #[cfg(any(esp32s2, esp32s3))]
        {
            ulp_riscv_rescue_from_monitor();
            ulp_interrupts_enable();
        }

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
#[doc(hidden)]
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
#[doc(hidden)]
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
        #[doc(hidden)]
        #[unsafe(no_mangle)]
        pub extern "C" fn trap_handler(regs: *const TrapFrame, pending_irqs: u32) {
            let regs = unsafe { regs.as_ref().unwrap() };
            $(
                ulp_interrupts!(@interrupt($irq, pending_irqs, regs, $handler));
            )*
        }
    };
}

/// Peripheral interrupt handler for the IRQ bit 31.
/// Checks the peripheral interrupt flags, and calls further interrupt handlers as appropriate.
/// Clears interrupt flags automatically.
#[cfg(any(esp32s2, esp32s3))]
#[doc(hidden)]
#[unsafe(no_mangle)]
fn dispatch_peripheral_interrupt(_regs: &TrapFrame) {
    // This function is based on the ESP-IDF implementation found here:
    // https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L110

    // RTC Peripheral interrupts
    // let cocpu_int_st: u32 = unsafe { &*pac::SENS::PTR }.sar_cocpu_int_st().read().bits();
    let cocpu_int_st = unsafe { &*pac::SENS::PTR }.sar_cocpu_int_st().read();
    let cocpu_int_st_bits = cocpu_int_st.bits();

    // RTC IO interrupts
    let rtcio_int_st: u32 = unsafe { &*pac::RTC_IO::PTR }.status().read().bits();

    // Call handler, and clear the interrupt flags, if they
    // were raised at the start of this ISR call.

    if cocpu_int_st_bits > 0 {
        dispatch_sens_interrupt(cocpu_int_st);

        // Clear the interrupt
        unsafe { &*pac::SENS::PTR }
            .sar_cocpu_int_clr()
            .write(|w| unsafe { w.bits(cocpu_int_st_bits) });
    }

    if rtcio_int_st > 0 {
        dispatch_rtcio_interrupt(rtcio_int_st >> 10);

        // Clear the interrupt
        unsafe { &*pac::RTC_IO::PTR }
            .status_w1tc()
            .write(|w| unsafe { w.bits(rtcio_int_st) });
    }
}

/// Dispatcher for RTC Peripheral interrupts (SENS).
/// This function is called from dispatch_peripheral_interrupt() function,
/// which is itself called from the trap_handler() function.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[doc(hidden)]
#[unsafe(no_mangle)]
pub fn dispatch_sens_interrupt(sar_cocpu_int_st: pac::sens::sar_cocpu_int_st::R) {
    // by default, this will attempt to service the __EXTERNAL_INTERRUPTS vectors.
    use pac::__EXTERNAL_INTERRUPTS;
    let bitflags = sar_cocpu_int_st.bits();
    // iterate over bits set and call the handler vectors
    for i in 0..__EXTERNAL_INTERRUPTS.len() {
        // Check bit set
        if (bitflags & (1 << i)) != 0 {
            unsafe { (__EXTERNAL_INTERRUPTS[i]._handler)() };
        }
    }
}

/// Dispatcher for RTC IO (GPIO) interrupts.
/// This function is called from dispatch_peripheral_interrupt() function,
/// which is itself called from the trap_handler() function.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[doc(hidden)]
#[unsafe(no_mangle)]
pub fn dispatch_rtcio_interrupt(pin_status: u32) {
    // iterate over bits set and call the handler vectors
    for i in 0..crate::rtcio_interrupt::__RTC_IO_INTERRUPTS.len() {
        // Check bit set
        if (pin_status & (1 << i)) != 0 {
            unsafe { (crate::rtcio_interrupt::__RTC_IO_INTERRUPTS[i]._handler)() };
        }
    }
}

/// Default interrupt handler, does nothing.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[doc(hidden)]
#[unsafe(no_mangle)]
pub fn noop_interrupt_handler() {}

// ULP interrupt bitflags from:
// https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L16
#[cfg(any(esp32s2, esp32s3))]
unsafe extern "Rust" {
    fn IllegalInstructionException(regs: &TrapFrame);
    fn BusErrorException(regs: &TrapFrame);
}

// Create the trap_handler function
#[cfg(any(esp32s2, esp32s3))]
ulp_interrupts!(
    1: IllegalInstructionException,
    2: BusErrorException,
    31: dispatch_peripheral_interrupt
);

/// Default illegal instruction or bus error exception handler.
/// Users may override IllegalInstructionException, and/or BusErrorException,
/// to change this behaviour.
/// This handler is dispatched by the trap_handler() function.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[doc(hidden)]
#[unsafe(no_mangle)]
pub fn default_exception_handler(_regs: &TrapFrame) {
    panic!("Unhandled exception!");
}

/// TODO: Clean this up and merge into GPIO probably.
/// Hacky GPIO interrupt vector table, similar to how it's done in ulp-pacs.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
mod rtcio_interrupt {
    unsafe extern "C" {
        fn GPIO0();
        fn GPIO1();
        fn GPIO2();
        fn GPIO3();
        fn GPIO4();
        fn GPIO5();
        fn GPIO6();
        fn GPIO7();
        fn GPIO8();
        fn GPIO9();
        fn GPIO10();
        fn GPIO11();
        fn GPIO12();
        fn GPIO13();
        fn GPIO14();
        fn GPIO15();
        fn GPIO16();
        fn GPIO17();
        fn GPIO18();
        fn GPIO19();
        fn GPIO20();
        fn GPIO21();
    }

    use crate::{gpio::MAX_GPIO_PIN, pac::Vector};

    #[doc(hidden)]
    #[unsafe(link_section = ".rwtext")]
    #[unsafe(no_mangle)]
    pub static __RTC_IO_INTERRUPTS: [Vector; (MAX_GPIO_PIN + 1) as usize] = [
        Vector { _handler: GPIO0 },
        Vector { _handler: GPIO1 },
        Vector { _handler: GPIO2 },
        Vector { _handler: GPIO3 },
        Vector { _handler: GPIO4 },
        Vector { _handler: GPIO5 },
        Vector { _handler: GPIO6 },
        Vector { _handler: GPIO7 },
        Vector { _handler: GPIO8 },
        Vector { _handler: GPIO9 },
        Vector { _handler: GPIO10 },
        Vector { _handler: GPIO11 },
        Vector { _handler: GPIO12 },
        Vector { _handler: GPIO13 },
        Vector { _handler: GPIO14 },
        Vector { _handler: GPIO15 },
        Vector { _handler: GPIO16 },
        Vector { _handler: GPIO17 },
        Vector { _handler: GPIO18 },
        Vector { _handler: GPIO19 },
        Vector { _handler: GPIO20 },
        Vector { _handler: GPIO21 },
    ];

    #[macro_export]
    #[doc = r" Assigns a handler to an interrupt"]
    #[doc = r""]
    #[doc = r" This macro takes two arguments: the name of an interrupt and the path to the"]
    #[doc = r" function that will be used as the handler of that interrupt. That function"]
    #[doc = r" must have signature `fn()`."]
    #[doc = r""]
    #[doc = r" # Example"]
    #[doc = r""]
    #[doc = r" ``` ignore"]
    #[doc = r" interrupt!(GPIO0, button);"]
    #[doc = r""]
    #[doc = r" fn button() {"]
    #[doc = r#"     print!("Pressed");"#]
    #[doc = r" }"]
    #[doc = r""]
    macro_rules! gpio_interrupt {
        ($ NAME : ident , $ path : path) => {
            #[allow(non_snake_case)]
            #[unsafe(no_mangle)]
            pub extern "C" fn $NAME() {
                let f: fn() = $path;
                f();
            }
        };
    }

    #[allow(unused)]
    pub(crate) use gpio_interrupt;
}
