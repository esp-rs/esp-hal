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
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(asm_sub_register)]
#![deny(missing_docs)]
#![no_std]

use core::arch::global_asm;

pub mod delay;
pub mod gpio;
#[cfg(esp32c6)]
pub mod i2c;
#[cfg(esp32c6)]
pub mod uart;

#[cfg(feature = "esp32c6")]
pub use esp32c6_lp as pac;
#[cfg(feature = "esp32s2")]
pub use esp32s2_ulp as pac;
#[cfg(feature = "esp32s3")]
pub use esp32s3_ulp as pac;

/// The prelude
pub mod prelude {
    pub use procmacros::entry;
}

cfg_if::cfg_if! {
    if #[cfg(feature = "esp32c6")] {
        // LP_FAST_CLK is not very accurate, for now use a rough estimate
        const LP_FAST_CLK_HZ: u32 = 16_000_000;
        const XTAL_D2_CLK_HZ: u32 = 20_000_000;
    } else if #[cfg(feature = "esp32s2")] {
        const LP_FAST_CLK_HZ: u32 = 8_000_000;
    } else if #[cfg(feature = "esp32s3")] {
        const LP_FAST_CLK_HZ: u32 = 17_500_000;
    }
}

pub(crate) static mut CPU_CLOCK: u32 = LP_FAST_CLK_HZ;

/// Wake up the HP core
#[cfg(feature = "esp32c6")]
pub fn wake_hp_core() {
    unsafe { &*esp32c6_lp::PMU::PTR }
        .hp_lp_cpu_comm()
        .write(|w| w.lp_trigger_hp().set_bit());
}

#[cfg(feature = "esp32c6")]
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

#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
global_asm!(
    r#"
	.section .text.vectors
	.global irq_vector
	.global reset_vector

/* The reset vector, jumps to startup code */
reset_vector:
	j __start

/* Interrupt handler */
.balign 16
irq_vector:
	ret

	.section .text

__start:
    /* setup the stack pointer */
	la sp, __stack_top

	call ulp_riscv_rescue_from_monitor
	call rust_main
	call ulp_riscv_halt
loop:
	j loop
"#
);

#[unsafe(link_section = ".init.rust")]
#[unsafe(export_name = "rust_main")]
unsafe extern "C" fn lp_core_startup() -> ! {
    unsafe {
        unsafe extern "Rust" {
            fn main() -> !;
        }

        #[cfg(feature = "esp32c6")]
        if (*pac::LP_CLKRST::PTR)
            .lp_clk_conf()
            .read()
            .fast_clk_sel()
            .bit_is_set()
        {
            CPU_CLOCK = XTAL_D2_CLK_HZ;
        }

        main();
    }
}

#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
#[unsafe(link_section = ".init.rust")]
#[unsafe(no_mangle)]
unsafe extern "C" fn ulp_riscv_rescue_from_monitor() {
    // Rescue RISC-V core from monitor state.
    unsafe { &*pac::RTC_CNTL::PTR }
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_done().clear_bit().cocpu_shut_reset_en().clear_bit());
}

#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
#[unsafe(link_section = ".init.rust")]
#[unsafe(no_mangle)]
unsafe extern "C" fn ulp_riscv_halt() {
    unsafe { &*pac::RTC_CNTL::PTR }
        .cocpu_ctrl()
        .modify(|_, w| unsafe { w.cocpu_shut_2_clk_dis().bits(0x3f).cocpu_done().set_bit() });

    loop {
        riscv::asm::wfi();
    }
}
