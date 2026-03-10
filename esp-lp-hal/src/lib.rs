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
    if #[cfg(any(esp32s2,esp32s3))] {
        const ULP_RISCV_TIMER_INT                         : u32 = 1 << 0;   /* Internal Timer Interrupt */
        const ULP_RISCV_EBREAK_ECALL_ILLEGAL_INSN_INT     : u32 = 1 << 1;   /* EBREAK, ECALL or Illegal instruction */
        const ULP_RISCV_BUS_ERROR_INT                     : u32 = 1 << 2;   /* Bus Error (Unaligned Memory Access) */
        const ULP_RISCV_PERIPHERAL_INTERRUPT              : u32 = 1 << 31;  /* RTC Peripheral Interrupt */
        const ULP_RISCV_INTERNAL_INTERRUPT                : u32 = ULP_RISCV_TIMER_INT | ULP_RISCV_EBREAK_ECALL_ILLEGAL_INSN_INT | ULP_RISCV_BUS_ERROR_INT;
    }
}
// ULP interrupt registers, for RTC_GPIO and SENS.
cfg_if::cfg_if! {
    if #[cfg(any(esp32s2,esp32s3))] {
        const ULP_DR_REG_RTC_IO_BASE : u32 = 0xA400;
        const ULP_DR_REG_SENS_BASE   : u32 = 0xC800;
        const RTC_GPIO_STATUS_REG        : u32 = 0xA418;
        const RTC_GPIO_STATUS_W1TC_REG   : u32 = 0xA420;
        const SENS_SAR_COCPU_INT_ST_REG  : u32 = 0xC8F0;
        const SENS_SAR_COCPU_INT_CLR_REG : u32 = 0xC8F4;
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

#[cfg(any(esp32s2,esp32s3))]
// Include the following macros which define custom RISCV R-Type instructions...
// 
// # getq_insn rd, qs
// Copies the value of Qx into a general purpose register rd
// 
// # setq_insn qd, rs
// Copies the value of general purpose register rs to Qx
// 
// # retirq_insn
// Copies the value of Q0 to CPU PC, and renables interrupts
// 
// # maskirq_insn rd, rs
// Copies the value of the register IRQ Mask to the register rd, and copies the value of register rs to to IRQ mask.
// 
// # waitirq_insn rd
// Pause execution until any interrupt (masked or unmasked) becomes pending. Stores the pending IRQ bitmask into register rd.
global_asm!(
    include_str!("./ulp_riscv_interrupt_ops.S")
);

// Assembly containing the reset_vector and irq_vector instructions.
#[cfg(any(esp32s2, esp32s3))]
global_asm!(
    include_str!("./ulp_riscv_vectors.S")
);

#[cfg(any(esp32s2, esp32s3))]
global_asm!(
    r#"
    .balign 0x10
    .section .init
    __start:
        /* setup the stack pointer */
        la sp, __stack_top

        /* Custom instruction to un-mask the interrupts */
        /* waitirq_insn zero */
        maskirq_insn zero, zero

        call ulp_riscv_rescue_from_monitor
        call rust_main

    loop:
        j loop
    "#
);


#[cfg(esp32s2)]
global_asm!(
    r#"
    /* Weakly-linked IRQ handler.
    *  Simply clears the IRQ and returns.
    */
    ulp_interrupt_handler:
        /* Register a0 == q1 == the bitmask of IRQs to be handled.
        * 'bltz a0,1f' is checking if the MSB is set for an i32,
        * indiciating that this is a ULP_RISCV_PERIPHERAL_INTERRUPT.
        * All other interrupt types ('internal' interrupts) are ignored.
        */
        bltz    a0,1f
        ret
        1:
        /* 1. Check for SENS_SAR peripheral interrupts.
        * Load 0xD000 into a0,
        * which will be used to
        * read/write SENS_SAR registers.
        */
        lui     a0,0xd
        /* Read interrupt flags from  
        * SENS_SAR_COCPU_INT_ST_REG
        * at address -1808(a0) = 0xC8F0.
        */
        lw      a1,-1808(a0)
        /* If equal to zero then no
        * interrupts have been raised,
        * and we can skip clearing them.
        */
        beqz    a1,2f
        /* Clear interrupts by writing
        * to SENS_SAR_COCPU_INT_CLR_REG 
        * at address -1804(a0) = 0xC8F4
        */
        sw      a1,-1804(a0)
        2:
        /* 2. Check for RTC_GPIO peripheral interrupts.
        * Load 0xA000 into a0,
        * which will be used to 
        * read/write RTC_GPIO registers.
        */
        lui     a0,0xa
        /* Read RTC_GPIO_STATUS_REG from +1048(a0) = 0xA418 */
        lw      a1,1048(a0)
        /* If no interrupt is raised, skip clearing them. */
        beqz    a1,3f
        /* Clear interrupts by writing
        * to RTC_GPIO_STATUS_W1TC_REG
        * at address +1056(a0) = 0xA420
        */
        sw      a1,1056(a0)
        3:
        ret 
    "#
);

#[cfg(any(esp32s2, esp32s3))]
#[unsafe(no_mangle)]
unsafe extern "C" fn _ulp_riscv_interrupt_handler(q1: u32) {
    // This interrupt handler is placeholder - it simply checks the interrupt flags, and clears
    // them. This function is based on the ESP-IDF implementation found here:
    // https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L110

    // ULP Internal Interrupts
    if (q1 & ULP_RISCV_INTERNAL_INTERRUPT) > 0 {
        // TODO
    }

    // External/Peripheral interrupts
    if (q1 & ULP_RISCV_PERIPHERAL_INTERRUPT) > 0 {
        // RTC Peripheral interrupts
        let cocpu_int_st: u32 = unsafe { &*pac::SENS::PTR }.sar_cocpu_int_st().read().bits();

        if cocpu_int_st > 0 {
            // Clear the interrupt
            unsafe { &*pac::SENS::PTR }
                .sar_cocpu_int_clr()
                .write(|w| unsafe { w.bits(cocpu_int_st) });
        }

        // RTC IO interrupts
        let rtcio_int_st: u32 = unsafe { &*pac::RTC_IO::PTR }.status().read().bits();

        if rtcio_int_st > 0 {
            // Clear the interrupt
            unsafe { &*pac::RTC_IO::PTR }
                .status_w1tc()
                .write(|w| unsafe { w.bits(rtcio_int_st) });
        }
    }
}

/// Entry point to the ULP program
#[unsafe(export_name = "rust_main")]
#[unsafe(link_section = ".init")]
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
#[unsafe(no_mangle)]
#[unsafe(link_section = ".init")]
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
            .modify(|r, w| unsafe {
                // Read the existing register value
                let mut cocpu_ctl = r.bits();
                // Set the value of COCPU_SHUT_2_CLK_DIS to 0x3F,
                // as per the ESP-IDF source: https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_utils.c#L29
                cocpu_ctl &= 0xFFC03FFF;
                cocpu_ctl |= 0x3F << 14;
                // Write to the register.
                w.bits(cocpu_ctl);
                // Re-read the register
                cocpu_ctl = r.bits();
                // Set RTC_CNTL_COCPU_DONE and RTC_CNTL_COCPU_DONE
                cocpu_ctl |= (1 << 22) | (1 << 25);
                // Write to the register.
                w.bits(cocpu_ctl);
                w
            });
    }

    // All chips will enter a no-op loop, when halting.
    loop {
        unsafe {
            core::arch::asm!("addi x0, x0, 0"); // no-op
        }
    }
}