//! Runs a function on a chosen CPU.
//!
//! [`Ipc`] posts a handler to a CPU, then raises that CPU's IPC interrupt. The interrupt
//! handler runs the posted handler at [`Priority::min`].
//!
//! esp-hal reserves `FROM_CPU_INTR0` for this path, and `FROM_CPU_INTR1` on dual-core
//! chips. [`crate::init`] installs the handler on the first CPU. Starting the second CPU
//! installs it there.
//!
//! The interrupt also drives the context switch of the RTOS that owns scheduling. That RTOS
//! registers one handler per CPU through [`__rtos_implementation`], then asks a CPU to run
//! it. The IPC interrupt runs that handler after the callback, because the handler chooses
//! the context that the interrupt returns to. A callback and a context switch never wait for
//! each other, and a callback alone does not make a CPU switch tasks.

use portable_atomic::{AtomicPtr, Ordering};

#[cfg(multi_core)]
use crate::peripherals::FROM_CPU_INTR1;
use crate::{
    interrupt::{Priority, software::SoftwareInterrupt},
    peripherals::{FROM_CPU_INTR0, IPC, Interrupt},
    system::Cpu,
};
#[cfg(xtensa)]
use crate::{ram, trapframe::TrapFrame};

/// The IPC state of one CPU.
///
/// Every CPU has its own handler, which reads the fields at constant addresses.
#[repr(C)]
struct CoreState {
    /// The Rust callback that [`Ipc::call_function`] posts, or null if the CPU has none.
    callback: AtomicPtr<()>,

    /// The context switch handler that the CPU runs, or null if the CPU has none pending.
    ///
    /// [`__rtos_implementation::request_context_switch`] copies `context_switch_handler` here,
    /// so that a callback does not make the CPU switch tasks.
    pending_context_switch: AtomicPtr<()>,

    /// The context switch handler of the CPU, or null if no RTOS registered one.
    ///
    /// This is set once, and holds the only handler that the CPU accepts.
    context_switch_handler: AtomicPtr<()>,

    /// The `FROM_CPU` request register of this CPU, written by [`install_core`].
    #[cfg(riscv)]
    request: AtomicPtr<u32>,
}

impl CoreState {
    const fn new() -> Self {
        Self {
            callback: AtomicPtr::new(core::ptr::null_mut()),
            pending_context_switch: AtomicPtr::new(core::ptr::null_mut()),
            context_switch_handler: AtomicPtr::new(core::ptr::null_mut()),
            #[cfg(riscv)]
            request: AtomicPtr::new(core::ptr::null_mut()),
        }
    }
}

static STATE: [CoreState; Cpu::COUNT] = [const { CoreState::new() }; Cpu::COUNT];

/// Inter-processor call handle.
///
/// Copies of this handle share one state table.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Ipc;

impl Ipc {
    /// Creates a new IPC handle.
    ///
    /// The handle binds no interrupt handler. [`crate::init`] installs the IPC path.
    pub const fn new(_ipc: IPC<'_>) -> Self {
        Self
    }

    /// Runs `handler` on `core`.
    ///
    /// `handler` runs in an interrupt handler at [`Priority::min`]. This function does not wait
    /// for `handler` to return.
    ///
    /// The interrupt handler saves the registers that the calling convention lets the callee
    /// clobber, except the floating-point registers. `handler` must not use those, and must be
    /// callable with the cache disabled.
    ///
    /// Care must be taken if this function is called in an interrupt-free context targeting
    /// the current core:
    ///
    /// - If `core` has the same `handler` pending, that handler has not run yet, and it runs after
    ///   this call. This function coalesces the two calls into that one run, and returns. A caller
    ///   that needs a run of its own must wait for the pending one.
    /// - If `core` has a *different* callback pending, this function waits until `core` takes that
    ///   callback. `core` takes it in a minimum-priority interrupt, which cannot run in an
    ///   interrupt-free context, and cannot run on the second CPU while it is in reset. The wait
    ///   never ends in those cases.
    ///
    /// A context switch does not delay this function.
    pub fn call_function(self, core: Cpu, handler: fn()) {
        let handler = handler as *mut ();
        assert_target_runs(core);

        while let Err(pending) = STATE[core as usize].callback.compare_exchange(
            core::ptr::null_mut(),
            handler,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            // The IPC handler takes the callback before it runs it, so a pending copy of `handler`
            // has not run yet. It runs after this call, which covers this call, too.
            if pending == handler {
                break;
            }

            core::hint::spin_loop();
        }

        raise(core);
    }
}

/// Private implementation details of the RTOS that owns context switching.
///
/// The IPC interrupt is the lowest-priority interrupt of a CPU, which makes it the place to
/// switch tasks. An RTOS registers its context switch handler here, then asks a CPU to run
/// it. Applications have no use for these functions, and must use [`Ipc::call_function`].
#[doc(hidden)]
pub mod __rtos_implementation {
    use super::*;

    /// The signature of a context switch handler.
    ///
    /// On RISC-V the handler runs as the direct-bound IPC interrupt handler, and returns from
    /// the interrupt itself. On Xtensa the vectored dispatcher calls it with the trap frame
    /// that the interrupt returns through.
    pub type ContextSwitchHandler = cfg_select! {
        riscv => {
            unsafe extern "C" fn()
        }
        xtensa => {
             extern "C" fn(&mut TrapFrame)
        }
    };

    /// Registers the context switch handler of `core`.
    ///
    /// A CPU accepts one context switch handler, so that [`request_context_switch`] needs no
    /// handler of its own and never waits. This function panics if `core` already has one.
    ///
    /// # Safety
    ///
    /// On RISC-V, the IPC interrupt jumps to `handler` with the interrupted `t0` in
    /// `mscratch`. `handler` must restore `t0` from that CSR, save and restore every other
    /// register it modifies, and return with `mret`.
    ///
    /// On Xtensa, the interrupt returns to the context that `handler` leaves in the trap
    /// frame, so `handler` must leave a context that the CPU can run.
    pub unsafe fn set_context_switch_handler(core: Cpu, handler: ContextSwitchHandler) {
        unwrap!(
            STATE[core as usize]
                .context_switch_handler
                .compare_exchange(
                    core::ptr::null_mut(),
                    handler as *mut (),
                    Ordering::Release,
                    Ordering::Relaxed,
                ),
            "A CPU accepts one context switch handler"
        );
    }

    /// Asks `core` to run its context switch handler.
    ///
    /// The handler runs at [`Priority::min`], after the callback that [`Ipc::call_function`]
    /// posts, if there is one. This function does not wait, not even for a pending callback.
    ///
    /// # Panics
    ///
    /// Panics in debug builds if `core` has no context switch handler.
    #[inline]
    pub fn request_context_switch(core: Cpu) {
        assert_target_runs(core);

        let state = &STATE[core as usize];

        // A CPU accepts one context switch handler, so this needs no read-modify-write: the
        // store is idempotent, and the IPC handler takes the handler before it runs it, which
        // makes the handler run at least once after this store.
        let handler = state.context_switch_handler.load(Ordering::Relaxed);
        debug_assert!(!handler.is_null(), "The CPU has no context switch handler");
        state
            .pending_context_switch
            .store(handler, Ordering::Release);

        raise(core);
    }
}

fn raise(core: Cpu) {
    match core {
        Cpu::ProCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).raise(),
        #[cfg(multi_core)]
        Cpu::AppCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR1::steal() }).raise(),
    }
}

fn assert_target_runs(_core: Cpu) {
    #[cfg(multi_core)]
    debug_assert!(
        _core != Cpu::AppCpu || crate::soc::cpu_control::is_running(Cpu::AppCpu),
        "IPC target APP core is not running"
    );
}

fn interrupt(core: Cpu) -> Interrupt {
    match core {
        Cpu::ProCpu => Interrupt::FROM_CPU_INTR0,
        #[cfg(multi_core)]
        Cpu::AppCpu => Interrupt::FROM_CPU_INTR1,
    }
}

/// Installs the IPC interrupt handler of the current CPU.
///
/// [`crate::init`] calls this on the first CPU.
pub(crate) fn install() {
    install_core(Cpu::ProCpu);

    // The second CPU uses a different interrupt, so it needs a vector entry of its own.
    // Mapping an interrupt writes the interrupt matrix, which is shared. The startup code
    // of the second CPU only enables CPU interrupts (see `start_core1_init`, which avoids
    // `setup_interrupts` for this reason), so the mapping survives.
    #[cfg(all(xtensa, multi_core))]
    install_core(Cpu::AppCpu);
}

/// Installs the IPC interrupt handler of the second CPU.
///
/// Direct binding writes CPU-local registers, so the second CPU must call this itself,
/// after it sets up vectoring.
#[cfg(all(riscv, multi_core))]
pub(crate) fn install_app() {
    install_core(Cpu::AppCpu);
}

#[cfg(riscv)]
fn install_core(core: Cpu) {
    debug_assert_eq!(core, Cpu::current());

    let regs = cfg_select! {
        soc_has_intpri => crate::peripherals::INTPRI::regs(),
        _ => crate::peripherals::SYSTEM::regs(),
    };
    STATE[core as usize].request.store(
        regs.cpu_intr_from_cpu(core as usize).as_ptr(),
        Ordering::Relaxed,
    );

    crate::interrupt::enable_direct(
        interrupt(core),
        Priority::min(),
        crate::interrupt::DirectBindableCpuInterrupt::Interrupt0,
        match core {
            Cpu::ProCpu => ipc_handler::<0>,
            #[cfg(multi_core)]
            Cpu::AppCpu => ipc_handler::<1>,
        },
    );
}

#[cfg(xtensa)]
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

#[cfg(xtensa)]
#[ram]
fn ipc_handler(context: &mut TrapFrame) {
    let core = Cpu::current();

    // This is the only place that clears the request. A second clear would drop a request
    // that arrived while a handler was running.
    match core {
        Cpu::ProCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).reset(),
        #[cfg(multi_core)]
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

/// The direct-bound IPC interrupt handler of one CPU.
///
/// The handler clears the request, runs the callback, then jumps to the context switch
/// handler. It borrows `ra`, `t0` and `t1` only. `t0` goes to `mscratch`, which is free
/// because the hardware clears `mstatus.MIE` on trap entry, and nothing here enables
/// interrupts again.
#[cfg(riscv)]
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

/// Runs the Rust callback of one CPU, then returns to [`ipc_handler`].
///
/// `t1` holds the address of the `CoreState` of the CPU. This function saves the registers
/// that the RISC-V calling convention lets the callee clobber, except `ra`, `t0` and `t1`,
/// which [`ipc_handler`] saves. It leaves `mscratch` alone, because the context handler that
/// runs next reads the interrupted `t0` from it.
#[cfg(riscv)]
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
