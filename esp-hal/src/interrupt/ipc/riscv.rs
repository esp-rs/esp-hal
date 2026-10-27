//! The IPC of a dual-core RISC-V chip.
//!
//! A CPU reaches its own CLINT through one alias, and the CLINT of the other CPU through
//! another alias. The machine software interrupt therefore carries the call in both directions.

use esp_sync::raw::{RawLock, SingleCoreInterruptLock};

use crate::{
    interrupt::{CpuInterrupt, Priority, isr_stack::machine_software_interrupt_entry},
    peripherals::{CLINT, CLINT_OTHER_CORE},
    ram,
    system::Cpu,
};

/// The CLIC line that serves the machine software interrupt.
const CLINT_INTERRUPT: CpuInterrupt = CpuInterrupt::Interrupt3;

pub(super) fn raise(core: Cpu) {
    // An unpinned task can move to the other CPU between the read of the current CPU and the
    // write to the CLINT alias.
    let token = unsafe { SingleCoreInterruptLock.enter() };
    if core == Cpu::current() {
        CLINT::regs()
    } else {
        CLINT_OTHER_CORE::regs()
    }
    .msip()
    .write(|w| w.msip().set_bit());
    unsafe { SingleCoreInterruptLock.exit(token) };
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

    // The CLIC serves the machine software interrupt as interrupt 3, and applies the
    // priority of that line to it. The interrupt is core-local, so it needs no matrix
    // entry.
    crate::interrupt::bind_cpu_interrupt(CLINT_INTERRUPT, machine_software_interrupt_entry);
    crate::interrupt::enable_cpu_interrupt(CLINT_INTERRUPT, Priority::min());
}

/// Handles the IPC interrupt of the current CPU.
///
/// Clears the request before it runs the queued requests, so that a request raised for the next
/// run survives. A pending context switch runs when the interrupt exits.
#[ram]
pub(crate) extern "C" fn machine_software_interrupt() {
    CLINT::regs().msip().write(|w| w.msip().clear_bit());
    // `mhartid` is 0 or 1.
    let core = crate::system::raw_core() & 1;
    unsafe { super::queue::drain(&super::runtime::STATE[core].lists, core as u32) };
}
