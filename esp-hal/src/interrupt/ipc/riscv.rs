//! The IPC of a dual-core RISC-V chip.
//!
//! A CPU reaches its own CLINT through one alias, and the CLINT of the other CPU through
//! another alias. The machine software interrupt therefore carries the call in both directions.

use portable_atomic::Ordering;

use super::{CoreState, STATE};
use crate::{
    interrupt::{CpuInterrupt, Priority},
    peripherals::{CLINT, CLINT_OTHER_CORE},
    system::Cpu,
};

/// The CLIC line that serves the machine software interrupt.
const CLINT_INTERRUPT: CpuInterrupt = CpuInterrupt::Interrupt3;

pub(super) fn raise(core: Cpu) {
    if core == Cpu::current() {
        CLINT::regs()
    } else {
        CLINT_OTHER_CORE::regs()
    }
    .msip()
    .write(|w| w.msip().set_bit());
}

/// Installs the IPC interrupt handler of the PRO CPU.
///
/// [`crate::init`] calls this function.
pub(crate) fn install() {
    install_core(Cpu::ProCpu);
}

/// Installs the IPC interrupt handler of the APP CPU.
///
/// Direct binding writes CPU-local registers. The APP CPU must therefore call this function
/// itself, after it sets up vectoring.
pub(crate) fn install_app() {
    install_core(Cpu::AppCpu);
}

fn install_core(core: Cpu) {
    debug_assert_eq!(core, Cpu::current());

    let handler = match core {
        Cpu::ProCpu => ipc_handler::<0>,
        Cpu::AppCpu => ipc_handler::<1>,
    };

    // The handler clears the request through the alias of its own CLINT.
    STATE[core as usize]
        .request
        .store(CLINT::regs().msip().as_ptr(), Ordering::Relaxed);

    // The CLIC serves the machine software interrupt as interrupt 3, and applies the
    // priority of that line to it. The interrupt is core-local, so it needs no matrix
    // entry.
    crate::interrupt::bind_cpu_interrupt(CLINT_INTERRUPT, handler);
    crate::interrupt::enable_cpu_interrupt(CLINT_INTERRUPT, Priority::min());
}

/// Handles the IPC interrupt of one CPU.
///
/// The handler clears the request, runs the posted function, then jumps to the context-switch
/// handler. It borrows `ra`, `t0`, and `t1` only. `t0` goes to `mscratch`. That CSR is free,
/// because the hardware clears `mstatus.MIE` on trap entry, and nothing here enables
/// interrupts again.
#[cfg_attr(
    not(interrupt_controller = "clic"),
    unsafe(link_section = ".trap.rust")
)]
#[cfg_attr(interrupt_controller = "clic", unsafe(link_section = ".rwtext"))]
#[unsafe(naked)]
unsafe extern "C" fn ipc_handler<const CORE: usize>() {
    core::arch::naked_asm!(
        "
        csrw mscratch, t0
        addi sp, sp, -16
        sw   t1, 0(sp)

        # Load the address of `STATE[CORE]`.
        la t1, {state} + {base}

        # Clear the request before taking the work, so that a request raised for the next
        # run survives.
        lw   t0, {request}(t1)
        sw   zero, 0(t0)

        # The callback returns here, so it runs first. A CPU rarely has one, so the callback
        # runs out of line: a taken branch costs two cycles, and the context switch that the
        # scheduler asks for on every yield must not pay them.
        lw   t0, {callback}(t1)
        bnez t0, 3f

    1:
        lw   t0, {pending_context_switch}(t1)
        sw   zero, {pending_context_switch}(t1)

        lw   t1, 0(sp)
        addi sp, sp, 16

        beqz t0, 2f
        # The context switch handler restores t0 from mscratch and returns with mret.
        jr   t0
    2:
        csrr t0, mscratch
        mret

    3:
        # `callback_handler` takes the callback from `t1`, and may clobber `ra`, `t0` and `t1`.
        sw   ra, 4(sp)
        la   t0, {callback_handler}
        jalr ra, 0(t0)
        lw   ra, 4(sp)
        la   t1, {state} + {base}
        j    1b
        ",
        state = sym STATE,
        base = const CORE * size_of::<CoreState>(),
        callback = const core::mem::offset_of!(CoreState, callback),
        pending_context_switch = const core::mem::offset_of!(CoreState, pending_context_switch),
        request = const core::mem::offset_of!(CoreState, request),
        callback_handler = sym callback_handler,
    )
}

/// Runs the posted function of one CPU, then returns to [`ipc_handler`].
///
/// `t1` holds the address of the `CoreState` of the CPU. This function saves the registers
/// that the RISC-V calling convention lets the callee clobber, except `ra`, `t0`, and `t1`,
/// which [`ipc_handler`] saves. It leaves `mscratch` alone, because the context-switch handler
/// that runs next reads the interrupted `t0` from it.
#[cfg_attr(
    not(interrupt_controller = "clic"),
    unsafe(link_section = ".trap.rust")
)]
#[cfg_attr(interrupt_controller = "clic", unsafe(link_section = ".rwtext"))]
#[unsafe(naked)]
unsafe extern "C" fn callback_handler() {
    core::arch::naked_asm!(
        "
        addi sp, sp, -64
        sw   ra, 0(sp)
        sw   t2, 4(sp)
        sw   t3, 8(sp)
        sw   t4, 12(sp)
        sw   t5, 16(sp)
        sw   t6, 20(sp)
        sw   a0, 24(sp)
        sw   a1, 28(sp)
        sw   a2, 32(sp)
        sw   a3, 36(sp)
        sw   a4, 40(sp)
        sw   a5, 44(sp)
        sw   a6, 48(sp)
        sw   a7, 52(sp)

        lw   t0, {callback}(t1)
        sw   zero, {callback}(t1)
        jalr ra, 0(t0)

        lw   ra, 0(sp)
        lw   t2, 4(sp)
        lw   t3, 8(sp)
        lw   t4, 12(sp)
        lw   t5, 16(sp)
        lw   t6, 20(sp)
        lw   a0, 24(sp)
        lw   a1, 28(sp)
        lw   a2, 32(sp)
        lw   a3, 36(sp)
        lw   a4, 40(sp)
        lw   a5, 44(sp)
        lw   a6, 48(sp)
        lw   a7, 52(sp)
        addi sp, sp, 64
        ret
        ",
        callback = const core::mem::offset_of!(CoreState, callback),
    )
}
