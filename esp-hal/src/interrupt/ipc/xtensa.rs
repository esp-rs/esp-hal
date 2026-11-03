//! The IPC of a dual-core Xtensa chip.
//!
//! The CPUs have no interrupt that reaches the other CPU. Each CPU therefore takes the call in
//! a `FROM_CPU` interrupt of its own.

use crate::{
    interrupt::{Priority, software::SoftwareInterrupt},
    peripherals::{FROM_CPU_INTR0, FROM_CPU_INTR1, Interrupt},
    ram,
    system::Cpu,
    trapframe::TrapFrame,
};

pub(super) fn raise(core: Cpu) {
    match core {
        Cpu::ProCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).raise(),
        Cpu::AppCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR1::steal() }).raise(),
    }
}

fn interrupt(core: Cpu) -> Interrupt {
    match core {
        Cpu::ProCpu => Interrupt::FROM_CPU_INTR0,
        Cpu::AppCpu => Interrupt::FROM_CPU_INTR1,
    }
}

/// Installs the IPC interrupt handler of both CPUs.
///
/// [`crate::init`] calls this function.
pub(crate) fn install() {
    install_core(Cpu::ProCpu);

    // The second CPU uses a different interrupt, so it needs a vector entry of its own.
    // Mapping an interrupt writes the interrupt matrix, which is shared. The startup code
    // of the second CPU only enables CPU interrupts (see `start_core1_init`, which avoids
    // `setup_interrupts` for this reason), so the mapping survives.
    install_core(Cpu::AppCpu);
}

fn install_core(core: Cpu) {
    let handler = crate::interrupt::InterruptHandler::new(
        // The vectored dispatcher calls every handler with the trap frame.
        unsafe {
            core::mem::transmute::<fn(&mut TrapFrame), extern "C" fn()>(
                ipc_handler as fn(&mut TrapFrame),
            )
        },
        Priority::min(),
    );

    crate::interrupt::bind_vector(interrupt(core), handler);
    crate::interrupt::enable_on_cpu(core, interrupt(core), Priority::min());
}

/// A pending context switch runs when the interrupt exits.
#[ram]
fn ipc_handler(_context: &mut TrapFrame) {
    // This is the only place that clears the request. A second clear would drop a request
    // that arrived while a handler was running.
    match Cpu::current() {
        Cpu::ProCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).reset(),
        Cpu::AppCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR1::steal() }).reset(),
    }

    super::run_callback();
}
