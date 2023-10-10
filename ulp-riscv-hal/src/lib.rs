#![no_std]

use core::arch::global_asm;

pub mod delay;
pub mod gpio;
pub mod prelude;

#[cfg(feature = "esp32s2")]
use esp32s2_ulp as pac;
#[cfg(feature = "esp32s3")]
use esp32s3_ulp as pac;

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

#[link_section = ".init.rust"]
#[export_name = "rust_main"]
unsafe extern "C" fn lp_core_startup() -> ! {
    extern "Rust" {
        fn main() -> !;
    }

    main();
}

#[link_section = ".init.rust"]
#[export_name = "ulp_riscv_rescue_from_monitor"]
unsafe extern "C" fn ulp_riscv_rescue_from_monitor() {
    // Rescue RISCV from monitor state.
    let rtc_cntl = unsafe { pac::RTC_CNTL::steal() };

    // TODO align naming in PACs
    #[cfg(feature = "esp32s2")]
    rtc_cntl
        .cocpu_ctrl
        .modify(|_, w| w.cocpu_done().clear_bit().cocpu_shut_reset_en().clear_bit());
    #[cfg(feature = "esp32s3")]
    rtc_cntl
        .rtc_cocpu_ctrl
        .modify(|_, w| w.cocpu_done().clear_bit().cocpu_shut_reset_en().clear_bit());
}

#[link_section = ".init.rust"]
#[export_name = "ulp_riscv_halt"]
unsafe extern "C" fn ulp_riscv_halt() {
    let rtc_cntl = unsafe { pac::RTC_CNTL::steal() };

    // TODO align naming in PACs
    #[cfg(feature = "esp32s2")]
    {
        rtc_cntl
            .cocpu_ctrl
            .modify(|_, w| w.cocpu_shut_2_clk_dis().variant(0x3f));

        rtc_cntl.cocpu_ctrl.modify(|_, w| w.cocpu_done().set_bit());
    }
    #[cfg(feature = "esp32s3")]
    {
        rtc_cntl
            .rtc_cocpu_ctrl
            .modify(|_, w| w.cocpu_shut_2_clk_dis().variant(0x3f));

        rtc_cntl
            .rtc_cocpu_ctrl
            .modify(|_, w| w.cocpu_done().set_bit());
    }

    #[allow(clippy::empty_loop)]
    loop {}
}
