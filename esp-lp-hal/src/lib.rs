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

// This crate needs to bring in some other global assembly
// extern crate riscv_rt;

// /// Re-exported interrupt APIs from riscv-rt / riscv crates.
// pub use riscv_rt::{TrapFrame};
pub use riscv_rt::{external_interrupt,core_interrupt,exception,TrapFrame};
use riscv_rt::{InterruptNumber,ExceptionNumber,CoreInterruptNumber};

/// Custom CoreInterrupts for ULP
#[riscv::pac_enum(unsafe CoreInterruptNumber)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(usize)]
#[allow(dead_code)]
pub enum UlpCoreInterrupt {
    /// Interrupt caused by ULP Timer
    Timer = 0,
    /// Interrupt caused by external peripheral (RTC_IO or SENS)
    Peripheral = 1,
}

unsafe impl InterruptNumber for UlpCoreInterrupt {
    const MAX_INTERRUPT_NUMBER: usize = Self::Peripheral as usize;

    #[inline]
    fn number(self) -> usize {
        self as usize
    }

    #[inline]
    fn from_number(value: usize) -> riscv_rt::result::Result<Self> {
        match value {
            0 => Ok(Self::Timer),
            1 => Ok(Self::Peripheral),
            _ => Err(riscv_rt::result::Error::InvalidVariant(value)),
        }
    }
}

unsafe impl CoreInterruptNumber for UlpCoreInterrupt {}

/// CPU Exceptions for ULP
#[riscv::pac_enum(unsafe ExceptionNumber)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(usize)]
#[allow(dead_code)]
pub enum UlpException {
    /// EBREAK, ECALL, or Illegal Instruction
    Breakpoint = 0,
    /// Bus Error / Misaligned Load or Store
    MisalignedAccess = 1,
}

unsafe impl ExceptionNumber for UlpException {
    const MAX_EXCEPTION_NUMBER: usize = Self::MisalignedAccess as usize;

    #[inline]
    fn number(self) -> usize {
        self as usize
    }

    #[inline]
    fn from_number(value: usize) -> riscv_rt::result::Result<Self> {
        match value {
            0 => Ok(Self::Breakpoint),
            1 => Ok(Self::MisalignedAccess),
            _ => Err(riscv_rt::result::Error::InvalidVariant(value)),
        }
    }
}

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

/// ULP version of _riscv_trap_rust handler, adapted from riscv-rt.
/// This is necessary, because the ULP core does not have an xcause register.
// #[cfg(any(esp32s2, esp32s3))]
// #[unsafe(link_section = ".trap.rust")]
#[unsafe(no_mangle)]
unsafe extern "C" fn _ulp_trap_rust(trap_frame: *const TrapFrame, irq_bitmask : u32) {
    // This interrupt handler is placeholder - it simply checks the interrupt flags, and clears
    // them. This function is based on the ESP-IDF implementation found here:
    // https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L110

    // // Provided by riscv-rt
    // unsafe extern "C" {
    //     fn _dispatch_core_interrupt(code: usize);
    //     fn _dispatch_exception(trap_frame: &TrapFrame, code: usize);
    // }

    // ULP Internal Interrupts & Exceptions
    if (irq_bitmask & ULP_RISCV_TIMER_INT) > 0 {
        // // ULP_RISCV_TIMER_INT /* Internal Timer Interrupt */
        // unsafe {
        //     _dispatch_core_interrupt(UlpCoreInterrupt::Timer.number());
        // }
    }
    if (irq_bitmask & ULP_RISCV_EBREAK_ECALL_ILLEGAL_INSN_INT) > 0 {
        // // ULP_RISCV_EBREAK_ECALL_ILLEGAL_INSN_INT /* EBREAK, ECALL or Illegal instruction */
        // unsafe {
        //     _dispatch_exception(&*trap_frame, UlpException::Breakpoint.number())
        // };
    }
    if (irq_bitmask & ULP_RISCV_BUS_ERROR_INT) > 0 {
        // // ULP_RISCV_BUS_ERROR_INT /* Bus Error (Unaligned Memory Access) */
        // unsafe {
        //     // TODO: Inspect instruction to determine if it was instruction, load, or store misalignment.
        //     // for now, lets assume it was just instruction misaligned.
        //     _dispatch_exception(&*trap_frame, UlpException::MisalignedAccess.number())
        // };
    }

    // External/Peripheral interrupts - marked as MachineExternal.
    // This core interrupt will need to delegate further to do other interrupt handling I think??? Unsure???
    if (irq_bitmask & ULP_RISCV_PERIPHERAL_INTERRUPT) > 0 {
        // RTC Peripheral interrupts
        let cocpu_int_st: u32 = unsafe { &*pac::SENS::PTR }.sar_cocpu_int_st().read().bits();
        // RTC IO interrupts
        let rtcio_int_st: u32 = unsafe { &*pac::RTC_IO::PTR }.status().read().bits();

        const ADDRESS: u32 = 0x1000;
        let ptr = ADDRESS as *mut u32;
        unsafe { ptr.write_volatile(0xABABABAB) };

        // // External interrupt, handler will need to decode it.
        // unsafe {
        //     _dispatch_core_interrupt(UlpCoreInterrupt::Peripheral.number());
        // }

        // Clear the interrupt flags, if they were raised at the start of this ISR call.
        if cocpu_int_st > 0 {
            // Clear the interrupt
            unsafe { &*pac::SENS::PTR }
                .sar_cocpu_int_clr()
                .write(|w| unsafe { w.bits(cocpu_int_st) });
        }

        if rtcio_int_st > 0 {
            // Clear the interrupt
            unsafe { &*pac::RTC_IO::PTR }
                .status_w1tc()
                .write(|w| unsafe { w.bits(rtcio_int_st) });
        }
    }
}


#[cfg(any(esp32s2, esp32s3))]
// Include macros which define custom RISCV R-Type instructions, used for interrupt handling.
global_asm!(include_str!("./ulp_riscv_interrupt_ops.S"));

// Assembly containing the reset_vector and irq_vector instructions.
#[cfg(any(esp32s2, esp32s3))]
global_asm!(include_str!("./ulp_riscv_vectors.S"));

/// riscv-rt redefinition which unmasks the interrupts.
/// Called during _start_rust, which is prior to rust_main,
/// which is prior to main().
#[cfg(any(esp32s2, esp32s3))]
#[unsafe(export_name = "_setup_interrupts")]
#[unsafe(link_section = ".init.rust")]
pub fn setup_interrupts() {
    // disable interrupt handling for now...
    ulp_enable_interrupts();
    unsafe { ulp_riscv_rescue_from_monitor() };
}

/// Enter a critical section (disable interrupts)
#[cfg(any(esp32s2, esp32s3))]
#[inline(always)]
pub fn ulp_disable_interrupts() {
    // Enter a critical section by disabling all interrupts
    // This inline assembly construct uses the t0 register and is equivalent to:
    // > li t0, 0x80000007
    // > maskirq_insn(zero, t0) // Mask all interrupt bits
    //
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

/// Exit a critical section (re-enable interrupts)
#[cfg(any(esp32s2, esp32s3))]
#[inline(always)]
pub fn ulp_enable_interrupts() {
    // Exit a critical section by enabling all interrupts
    // This inline assembly construct is equivalent to:
    // > maskirq_insn(zero, zero)
    unsafe {
        core::arch::asm!(".word 0x0600600b");
    }
}

/// Wait for any (even unmasked) interrupt
#[cfg(any(esp32s2, esp32s3))]
#[inline(always)]
pub fn ulp_waitirq() -> u32 {
    // Wait for pending interrupt, return pending interrupt mask
    // waitirq a0
    let result: u32;
    unsafe {
        core::arch::asm!(".word 0x0800400B", out("a0") result);
    }
    result
}

/// Entry point to the ULP program
#[unsafe(link_section = ".init.rust")]
#[unsafe(export_name = "rust_main")]
unsafe extern "C" fn lp_core_startup() -> ! {
    unsafe {
        unsafe extern "Rust" {
            fn main();
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
