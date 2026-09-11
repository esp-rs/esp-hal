//! The IPC of a dual-core Xtensa chip.
//!
//! The CPUs have no interrupt that reaches the other CPU. Each CPU therefore takes the call in
//! a `FROM_CPU` interrupt of its own.

use portable_atomic::Ordering;

use super::STATE;
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

#[ram]
fn ipc_handler(context: &mut TrapFrame) {
    let core = Cpu::current();

    // This is the only place that clears the request. A second clear would drop a request
    // that arrived while a handler was running.
    match core {
        Cpu::ProCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).reset(),
        Cpu::AppCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR1::steal() }).reset(),
    }

    let state = &STATE[core as usize];

    // The callback returns here, so it runs first.
    let callback = state.callback.swap(core::ptr::null_mut(), Ordering::AcqRel);
    if !callback.is_null() {
        let callback = unsafe { core::mem::transmute::<*mut (), fn()>(callback) };
        callback();
    }

    // The context switch chooses the context that the interrupt returns to, so it runs last.
    let handler = state
        .pending_context_switch
        .swap(core::ptr::null_mut(), Ordering::AcqRel);
    if !handler.is_null() {
        let handler =
            unsafe { core::mem::transmute::<*mut (), extern "C" fn(&mut TrapFrame)>(handler) };
        handler(context);
    }
}
