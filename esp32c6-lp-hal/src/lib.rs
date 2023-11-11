#![no_std]

use core::arch::global_asm;

pub mod delay;
pub mod gpio;

pub mod riscv {
    //! Low level access to RISC-V processors.
    //!
    //! Re-exports <https://crates.io/crates/riscv>

    pub use riscv::*;
}
pub mod prelude;

// LP_FAST_CLK is not very accurate, for now use a rough estimate
const LP_FAST_CLK_HZ: u32 = 16_000_000;
const XTAL_D2_CLK_HZ: u32 = 20_000_000;

pub static mut CPU_CLOCK: u32 = LP_FAST_CLK_HZ;

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
        call lp_core_startup
    loop:
        j loop
"#
);

#[link_section = ".init.rust"]
#[export_name = "lp_core_startup"]
unsafe extern "C" fn lp_core_startup() -> ! {
    extern "Rust" {
        fn main() -> !;
    }

    let clkrst = &*esp32c6_lp::LP_CLKRST::PTR;
    if clkrst.lp_clk_conf.read().fast_clk_sel().bit_is_set() {
        CPU_CLOCK = XTAL_D2_CLK_HZ;
    }

    main();
}

mod critical_section_impl {
    struct CriticalSection;

    critical_section::set_impl!(CriticalSection);

    unsafe impl critical_section::Impl for CriticalSection {
        unsafe fn acquire() -> critical_section::RawRestoreState {
            let mut mstatus = 0u32;
            core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus);
            let interrupts_active = (mstatus & 0b1000) != 0;
            interrupts_active as _
        }

        unsafe fn release(token: critical_section::RawRestoreState) {
            if token != 0 {
                riscv::interrupt::enable();
            }
        }
    }
}
