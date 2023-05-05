//! SP Monitoring
//!
//! Demonstrates usage of the SP monitoring DEBUG_ASSIST feature to watch for
//! stack overflow / underflow.

#![no_std]
#![no_main]
#![feature(maybe_uninit_uninit_array)]
#![feature(asm_const)]

use core::{arch::global_asm, mem::MaybeUninit};

use esp32c3_hal::{
    clock::ClockControl,
    interrupt::{self},
    peripherals::{self, Peripherals},
    prelude::*,
    riscv,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_hal_common::{assist_debug::DebugAssist, TrapFrame};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clockctrl = system.clock_control;
    let clocks = ClockControl::boot_defaults(clockctrl).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    extern "C" {
        static _estack: *const usize;
        static _sstack: *const usize;
    }

    let mut da = DebugAssist::new(
        peripherals.ASSIST_DEBUG,
        &mut system.peripheral_clock_control,
    );

    let (lower, upper) = {
        (
            unsafe { &_estack as *const _ } as u32,
            unsafe { &_sstack as *const _ } as u32,
        )
    };
    // TODO: where do we go when we run off the end here?
    // let lower = lower + 0xf000; // carve out a whole buncha space there
    da.enable_sp_monitor(lower, upper);

    unsafe {
        interrupt::map(
            esp32c3_hal::get_core(),
            peripherals::Interrupt::ASSIST_DEBUG,
            CPU_IRQ,
        );
        interrupt::set_priority(esp32c3_hal::get_core(), CPU_IRQ, interrupt::Priority::max());
        #[cfg(hulk_smash)]
        {
            // with a large enough allocation, we'll blow the stack so badly we'll get stuck
            // before we can print anything, so catch that condition eagerly
            interrupt::enable_cpu_interrupt(CPU_IRQ);
        }
    }

    // this is an attempt to override a _vector_table slot at runtime; it doesn't
    // work with the _vector_table stored in flash
    #[cfg(off)]
    unsafe {
        extern "C" {
            static _vector_table: *const u32;
            static _start_trap: *const extern "C" fn();
            static _stack_trap: *const extern "C" fn();
        }
        let vec_table = &_vector_table as *const _ as *mut u32;
        let irq_no = cpu_irq as isize;
        let _orig_addr = &_start_trap as *const _ as usize;

        // little baby assembler
        let imm = {
            let target_addr = _stack_trap as *const () as usize;
            let slot_addr = vec_table.offset(irq_no) as usize;
            let off = target_addr.wrapping_sub(slot_addr) as isize;

            if off % 2 != 0 || !(!(0x10_0000 - 1)..0x10_0000).contains(&off) {
                panic!(
                    "offset out of range! {off} ({off:x}) must be in the inclusive range [-1048576, 1048574], inclusive, and cannot be odd (the lsb is always zero)",
                )
            }

            // a 20 bit immediate, following this pattern:
            //   [20][10:1][11][19:12]
            // (NB: the bit 0 is always zero)
            let imm: u32 = [
                off & 0x10_0000,       // [20]    ( 1 bit ) -> bit 20
                (off & 0x7fe) << 9,    // [10:1]  (10 bits) -> bit 10-19
                (off & 0x800) >> 2,    // [11]    ( 1 bit ) -> bit 9
                (off & 0xff000) >> 11, // [19:12] ( 7 bits) -> bit 1-7
            ]
            .into_iter()
            .sum::<isize>() as u32;
            imm >> 1
        };
        // jal x0, <imm>
        let insn = imm << 12 | 0b0000_1101111;
        *vec_table.offset(irq_no) = insn; // TODO oops needs to be in ram

        println!("set {:x} to {insn:x}", vec_table.offset(irq_no) as usize);

        println!("table:");
        for i in 1..32 {
            let ptr = vec_table.offset(i);
            println!("{:x} = {:x}", ptr as usize, *ptr)
        }
    }
    // vs.
    // interrupt::enable(
    //     peripherals::Interrupt::ASSIST_DEBUG,
    //     interrupt::Priority::max(),
    // )
    // .unwrap();

    // TODO: is it possible to separate that out without clobbering the data?
    //   (we're already 4 cycles late b/c it's an async interrupt, so no, but...)
    //   kind of renders the debug assist useless? Or maybe there's a way to not
    // corrupt the stack and still catch the overflow?

    // hmm, seems synchronous actually?
    //          debug pc = 4200140c
    // ```
    // 420013ee:       81010113                addi    sp,sp,-2032
    // 420013f2:       7e112623                sw      ra,2028(sp)
    // 420013f6:       7e812423                sw      s0,2024(sp)
    // 420013fa:       7e912223                sw      s1,2020(sp)
    // 420013fe:       7f010413                addi    s0,sp,2032
    // 42001402:       6541                    lui     a0,0x10
    // 42001404:       88050513                addi    a0,a0,-1920 # f880 <.Lline_table_start0+0x4ffe>
    // 42001408:       40a10133                sub     sp,sp,a0
    // 4200140c:       6541                    lui     a0,0x10
    // 4200140e:       06850513                addi    a0,a0,104 # 10068 <.Lline_table_start0+0x20f>
    // ```
    // which is one call after `sp: 3fc9f600 - 3fc809f0 = 1ec10` false
    // with a buffer of 0xf000

    // TODO: Can we have, like, tp - 4 words of memory set aside to always have a
    // non-stack location to write stuff to? It's too late to write things to the
    // stack when the da fires, because we've already bumped the stack pointer out
    // of bounds. How far? Impossible to say, and we can't catch it before the bump
    // occurs.

    // TODO: what happens if we just read/write out of bounds? i.e. the flip-link
    // model? cf. https://github.com/knurling-rs/flip-link

    unsafe {
        riscv::interrupt::enable();
    }

    #[allow(unconditional_recursion)] // this is intentionally broken, the idea is to catch it before it does damage
    fn recur() -> u32 {
        let alloc = {
            #[cfg(hulk_smash)]
            {
                MaybeUninit::<u8>::uninit_array::<0x1_0000>()
            }

            #[cfg(not(hulk_smash))]
            {
                MaybeUninit::<u8>::uninit_array::<0x400>()
            }
        };
        let _ = core::hint::black_box(alloc);

        let (sp, end) = {
            let end: usize = (unsafe { &_estack as *const _ } as usize);
            let sp: usize;
            unsafe {
                core::arch::asm!("mv {0}, sp", out(reg) sp);
            }

            (sp, end)
        };
        let rem = sp.saturating_sub(end);
        println!(
            "sp: {sp:x} - {end:x} = {rem:05x} {}",
            unsafe { core::mem::transmute::<_, DebugAssist>(()) }.is_sp_monitor_interrupt_set()
        );
        // this is here to demonstrate we can happily smash whatever's next to the
        // stack if interrupts aren't turned on
        if rem == 0 {
            unsafe {
                interrupt::enable_cpu_interrupt(CPU_IRQ);
            }
        }
        recur() + 1
    }

    loop {
        let _ = recur();
    }
}

#[no_mangle]
pub unsafe extern "C" fn DefaultHandler(frame: &mut TrapFrame) -> ! {
    let cause = core::mem::transmute::<_, riscv::register::mcause::Mcause>(frame.mcause);

    panic!("unhandled {:?} {:?}", cause.code(), frame)
}

const CPU_IRQ: interrupt::CpuInterrupt = esp32c3_hal::CpuInterrupt::Interrupt31;
#[export_name = "interrupt31"]
pub extern "C" fn _da_trap() {
    let da = unsafe { core::mem::transmute::<_, DebugAssist>(()) };

    panic!("default flow: 0x{:x}", da.get_sp_monitor_pc());
}

/// # Safety
///
/// Called from assembly, expects the errant sp to be in a0.
///
/// NB: Must be called with a valid stack!
#[link_section = ".trap"]
#[no_mangle]
pub unsafe extern "C" fn _stack_mon_trap_rust(sp: usize) -> ! {
    extern "C" {
        static _estack: *const usize;
        static _sstack: *const usize;
    }

    let da = unsafe { core::mem::transmute::<_, DebugAssist>(()) };

    let (min, max) = {
        (
            unsafe { &_estack as *const _ } as usize,
            unsafe { &_sstack as *const _ } as usize,
        )
    };

    let frag = if sp > max { "underflow" } else { "overflow" };
    panic!(
        "stack {}: sp {:x} exceeded bounds [{:x}, {:x}]: last pc: 0x{:x}",
        frag,
        sp,
        min,
        max,
        da.get_sp_monitor_pc()
    );
}

const SZ: usize = 0x800;

#[repr(align(32))] // needs to be aligned to a 32-bit address boundary so esp-backtrace will see it
                   // as a [`valid_ram_address`](https://github.com/esp-rs/esp-backtrace/blob/097db0297b9198424b41a48488b92af013663e29/src/lib.rs#L127-L129)
pub struct Stack([usize; SZ]);

// #[link_section = ".trap"]
#[no_mangle]
static mut _backup_stack: Stack = Stack([0; SZ]);

global_asm!(
    r#"
.section .trap, "ax"
.global _stack_mon_trap
.type _stack_mon_trap, @function

_stack_mon_trap:
    /* we'll clobber some registers here, but that's fine: we won't be
       coming back */
    mv      a0, sp

    /* pretend we were called normally, for the backtrace */
    // csrrs   ra, mepc, x0
    la      sp, _backup_stack + {backup_stack_sz}

    j _stack_mon_trap_rust

"#,
    backup_stack_sz = const SZ);

global_asm!(
    r#"
/*
    Interrupt vector table (_vector_table)
*/

.section .trap, "ax"
.global _vector_table
.type _vector_table, @function

.option push
.balign 0x100
.option norelax
.option norvc

_vector_table:
    j _start_trap
    .rept 30
    j _start_trap
    .endr
    j _stack_mon_trap

.option pop
"#
);
