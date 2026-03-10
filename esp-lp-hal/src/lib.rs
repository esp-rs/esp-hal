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

pub(crate) static mut CPU_CLOCK: u32 = LP_FAST_CLK_HZ;

/// Wake up the HP core
#[cfg(esp32c6)]
pub fn wake_hp_core() {
    unsafe { &*esp32c6_lp::PMU::PTR }
        .hp_lp_cpu_comm()
        .write(|w| w.lp_trigger_hp().set_bit());
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

#[cfg(any(esp32s2, esp32s3))]
global_asm!(
    r#"
  .equ SAVE_REGS, 17
  .equ CONTEXT_SIZE, (SAVE_REGS * 4)

  /* Much of this assembly was sourced from the following ESP-IDF files...
  *  ...irq handler macros:
  *    https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_vectors.S
  *
  *  ...critical section assembly
  *    https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/include/ulp_riscv_utils.h
  *
  *  ...riscv halt code
  *    https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_utils.c
  */

  /* Macro which first allocates space on the stack to save general
   * purpose registers, and then save them. GP register is excluded.
   * The default size allocated on the stack is CONTEXT_SIZE, but it
   * can be overridden.
   *
   * Note: We don't save the callee-saved s0-s11 registers to save space
   */
  .macro save_general_regs cxt_size=CONTEXT_SIZE
      addi sp, sp, -\cxt_size
      sw   ra, 0(sp)
      sw   tp, 4(sp)
      sw   t0, 8(sp)
      sw   t1, 12(sp)
      sw   t2, 16(sp)
      sw   a0, 20(sp)
      sw   a1, 24(sp)
      sw   a2, 28(sp)
      sw   a3, 32(sp)
      sw   a4, 36(sp)
      sw   a5, 40(sp)
      sw   a6, 44(sp)
      sw   a7, 48(sp)
      sw   t3, 52(sp)
      sw   t4, 56(sp)
      sw   t5, 60(sp)
      sw   t6, 64(sp)
  .endm

  /* Restore the general purpose registers (excluding gp) from the context on
   * the stack. The context is then deallocated. The default size is CONTEXT_SIZE
   * but it can be overridden. */
  .macro restore_general_regs cxt_size=CONTEXT_SIZE
      lw   ra, 0(sp)
      lw   tp, 4(sp)
      lw   t0, 8(sp)
      lw   t1, 12(sp)
      lw   t2, 16(sp)
      lw   a0, 20(sp)
      lw   a1, 24(sp)
      lw   a2, 28(sp)
      lw   a3, 32(sp)
      lw   a4, 36(sp)
      lw   a5, 40(sp)
      lw   a6, 44(sp)
      lw   a7, 48(sp)
      lw   t3, 52(sp)
      lw   t4, 56(sp)
      lw   t5, 60(sp)
      lw   t6, 64(sp)
      addi sp,sp, \cxt_size
  .endm

  .section .text.vectors
  .global irq_vector
  .global reset_vector
  .weak ulp_irq_handler
  .type ulp_irq_handler, @function
  
  /* The reset vector, jumps to startup code */
  reset_vector:
    j __start

  /* Interrupt handler */
  .balign 0x10 
  irq_vector:
    /* Save the general gurpose register context before handling the interrupt */
    save_general_regs
    /* Fetch the interrupt status from the custom q1 register into a0 */
    /* Equivalent to getq_insn(a0, q1) */
    .word 0x0000C50B

    /* Call the global C interrupt handler. The interrupt status is passed as the argument in a0.
     * We do not re-enable interrupts before calling the C handler as ULP RISC-V does not
     * support nested interrupts.
     */
    jal ulp_irq_handler

    /* Restore the register context after returning from the C interrupt handler */
    restore_general_regs

    /* Exit interrupt handler by executing the custom retirq instruction which will restore pc and re-enable interrupts */
    /* Equivalent to retirq_insn() */
    .word 0x0400000B
 

  .balign 0x10
	.section .init

  /* Weakly-linked IRQ handler.
  *  Simply clears the IRQ and returns.
  */
  ulp_irq_handler:
     bltz    a0,1f
     ret
     1:
     /* Store a base address in a0 (0xD000) */
     lui     a0,0xd
     /* Read interrupt flag from (a0-1808) = 0xC8F0 */
     lw      a1,-1808(a0)
     /* If zero, goto 2 */
     beqz    a1,2f
     /* Clear the interrupt in 0xC8F4 */
     sw      a1,-1804(a0)
     2:
     /* Store a base address in a0 (0xA000) */
     lui     a0,0xa
     /* Read RTC_GPIO_STATUS_REG from (a0+1048) = 0xA418 */
     lw      a1,1048(a0)
     /* If no interrupt is raised (equal to 0), goto 3 */
     beqz    a1,3f
     /* Clear RTC_GPIO_STATUS_W1TC_REG by writing to (a0+1056) = 0xA420 */
     sw      a1,1056(a0)
     3:
     ret 

  .balign 0x10
  __start:
    /* setup the stack pointer */
    la sp, __stack_top
    /* Equivalent to ulp_enable_interrupts() */
    .word 0x0600600b
    call ulp_riscv_rescue_from_monitor
    call rust_main

  loop:
    j loop
  "#
);

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
    // ESP-S2 and ESP-S3 chips need to set the cocpu_shut_2_clk_dis delay field,
    // and the cocpu_done flag, in order to halt.
    #[cfg(any(esp32s2, esp32s3))]
    unsafe {
        // Equivalent Rust code.
        // unsafe { &*pac::RTC_CNTL::PTR }
        //     .cocpu_ctrl()
        //     .modify(|_, w| unsafe { w.cocpu_shut_2_clk_dis().bits(0x3f).cocpu_done().set_bit()
        // });
        core::arch::asm!(
            "lui a5,0x8",
            "addi a5,a5,260",
            // 0x8000 + 260 = 0x8104
            "lw a4,0(a5)",
            // a4 == *0x8104 == contents of RTC_CNTL::cocpu_ctrl register
            "lui a3,0xffc04",
            "addi a3,a3,-1",
            // a3 = 0xFFC03FFF = bitmask
            "and a4,a4,a3",
            // a4 == a4 & a3 == bits [14-21] of a3
            "lui a3,0xfc",
            // a3 == 0xFC000
            "or a4,a4,a3",
            // a4 = a4 | a3, so we are setting bits [14-19] to 1, while keeping the existing values
            // of btis [20,21].
            "sw a4,0(a5)",
            // store it back to cocpu_ctrl
            "lw a4,0(a5)",
            // re-load it back again?
            // a3 = 0x2400000, i.e. bit 22 and 25 are set.
            "lui a3,0x2400",
            // do another or operation! so we are setting bit 22 and 25
            "or a4,a4,a3",
            // save it back!
            "sw a4,0(a5)",
        );
    }

    // All chips will enter a no-op loop, when halting.
    loop {
        unsafe {
            // core::arch::asm!("addi x0, x0, 0");
            core::arch::asm!("nop");
        }
    }
}
