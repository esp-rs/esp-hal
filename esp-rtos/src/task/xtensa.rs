//! Xtensa context switching implementation.
//!
//! Context switching is implemented via interrupt handlers. Servicing the interrupt request
//! saves context to the stack. The OS copies the state to memory, replaces it with the new task's
//! state, then returns from the interrupt handler.
//!
//! To trigger a context switch on the same core, we (where possible) use the Software0 CPU
//! interrupt. To trigger a cross-core context switch, we use the FROM_CPUn CPU interrupts. On
//! ESP32, Software0 is not available, so we use FROM_CPUn there, as well.
//!
//! Context switching must happen at the lower interrupt priority level. This ensures that context
//! switching does not interfere with other interrupts, so we don't leave an interrupt handler only
//! partially executed.

#[cfg(feature = "esp-radio")]
use core::ffi::c_void;
use core::sync::atomic::Ordering;

pub(crate) use esp_hal::trapframe::TrapFrame as CpuContext;
#[cfg(not(esp32))]
use esp_hal::xtensa_lx::interrupt;
use esp_hal::{interrupt::software::SoftwareInterrupt, ram};
#[cfg(multi_core)]
use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    system::Cpu,
};
use portable_atomic::AtomicPtr;

#[cfg(feature = "rtos-trace")]
use crate::TraceEvents;
use crate::{SCHEDULER, task::IdleFn};

static IDLE_HOOK: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

pub(crate) extern "C" fn idle_hook() -> ! {
    loop {
        unsafe { core::arch::asm!("waiti 0") };
    }
}

#[unsafe(naked)]
extern "C" fn idle_entry() -> ! {
    core::arch::naked_asm!("
        .literal idle_hook_fn, {idle_hook_fn}

        l32r   a2, idle_hook_fn
        l32i.n a2, a2, 0
        callx4 a2
    ", idle_hook_fn = sym IDLE_HOOK);
}

// Exception mode. Setting this bit prevents interrupts below EXCMLEVEL. Cleared by `rfe` at the end
// of the Level 1 interrupt handler.
const PS_EXCM: u32 = 1 << 4;
// User mode. This bit doesn't matter for us yet, we don't have separate kernel mode exceptions.
const PS_UM: u32 = 1 << 5;
// Windowed mode.
const PS_WOE: u32 = 1 << 18;
// CALLINC field value for call4 instruction.
#[cfg(feature = "esp-radio")]
const PS_CALLINC_CALL4: u32 = 1 << 16;

pub(crate) fn set_idle_hook_entry(idle_context: &mut CpuContext, hook_fn: IdleFn) {
    IDLE_HOOK.store(hook_fn as *mut (), Ordering::Relaxed);

    // Point idle context PC at the assembly that calls the idle hook. We need a new stack
    // frame for the idle task on the main stack.
    idle_context.PC = idle_entry as usize as u32;
    // Set a valid processor status value, that will not end up spilling registers into the main
    // task's stack. Here we jump to a naked function so we can omit the CALLINC bits.
    idle_context.PS = PS_EXCM | PS_UM | PS_WOE;
}

#[cfg(feature = "esp-radio")]
pub(crate) fn new_task_context(
    task_fn: extern "C" fn(*mut c_void),
    param: *mut c_void,
    stack_top: *mut (),
) -> CpuContext {
    // stack must be aligned by 16
    let stack_top = stack_top as u32;
    let stack_top = stack_top - (stack_top % 16);

    unsafe {
        *((stack_top - 4) as *mut u32) = 0;
        *((stack_top - 8) as *mut u32) = 0;
        *((stack_top - 12) as *mut u32) = stack_top;
        *((stack_top - 16) as *mut u32) = 0;
    }

    CpuContext {
        PC: super::task_wrapper as usize as u32,
        A0: 0,
        A1: stack_top,
        A6: task_fn as usize as u32,
        A7: param as usize as u32,

        // For windowed ABI set WOE, UM, EXCM and CALLINC = call4 (pretend task was 'call4'd)
        // UM and EXCM are important, so that restoring context will correctly restore an exception
        // context (where the context switch happens).
        PS: PS_EXCM | PS_UM | PS_WOE | PS_CALLINC_CALL4,

        ..Default::default()
    }
}

pub(crate) fn task_switch(
    current_context: *mut CpuContext,
    next_context: *mut CpuContext,
    trap_frame: &mut CpuContext,
) {
    if !current_context.is_null() {
        unsafe { core::ptr::copy_nonoverlapping(trap_frame, current_context, 1) };
    }
    unsafe { core::ptr::copy_nonoverlapping(next_context, trap_frame, 1) };
}

// S2 and S3 use Software0 (priority 1) for same-core task switching. This is slightly faster than
// the FROM_CPU0 interrupt.
#[cfg(not(esp32))]
const SW_INTERRUPT: u32 = 1 << 7;

pub(crate) fn setup_multitasking<const IRQ: u8>(mut _irq: SoftwareInterrupt<'static, IRQ>) {
    #[cfg(not(esp32))]
    unsafe {
        // Set up a CPU-internal interrupt, which will be used to trigger a context switch on the
        // same core.
        interrupt::enable_mask(SW_INTERRUPT);
    }

    #[cfg(multi_core)]
    {
        _irq.set_interrupt_handler(InterruptHandler::new(
            unsafe {
                core::mem::transmute::<*const (), extern "C" fn()>(
                    cross_core_yield_handler as *const (),
                )
            },
            Priority::min(),
        ));
    }
}

#[cfg(multi_core)]
pub(crate) fn setup_smp<const IRQ: u8>(irq: SoftwareInterrupt<'static, IRQ>) {
    setup_multitasking(irq);
}

// Non-ESP32 can use Software0 (priority 1) for same-core task switching. This is slightly faster
// than the FROM_CPU0 interrupt. On ESP32, this is not available because the bluetooth driver uses
// Software0.
#[allow(non_snake_case)]
#[ram]
#[cfg(not(esp32))]
#[unsafe(export_name = "Software0")]
fn task_switch_interrupt(context: &mut CpuContext) {
    unsafe { interrupt::clear(SW_INTERRUPT) };

    trigger_task_switch(context);
}

#[inline]
pub(crate) fn yield_task() {
    #[cfg(feature = "rtos-trace")]
    {
        rtos_trace::trace::marker_begin(TraceEvents::YieldTask as u32);
        rtos_trace::trace::marker_end(TraceEvents::YieldTask as u32);
    }

    #[cfg(not(esp32))]
    unsafe {
        interrupt::set(SW_INTERRUPT);
    }

    #[cfg(esp32)]
    match Cpu::current() {
        Cpu::ProCpu => unsafe { SoftwareInterrupt::<'static, 0>::steal() }.raise(),
        Cpu::AppCpu => unsafe { SoftwareInterrupt::<'static, 1>::steal() }.raise(),
    }
}

#[cfg(multi_core)]
#[ram]
extern "C" fn cross_core_yield_handler(context: &mut CpuContext) {
    match Cpu::current() {
        Cpu::ProCpu => unsafe { SoftwareInterrupt::<'static, 0>::steal() }.reset(),
        Cpu::AppCpu => unsafe { SoftwareInterrupt::<'static, 1>::steal() }.reset(),
    }

    trigger_task_switch(context);
}

// Having this function separate ensures there is a single un-inlined copy of the task switch logic
// living in RAM. `ram` is conditional to ensure the function is inlined on ESP32 and S2.
#[cfg_attr(esp32s3, ram)]
fn trigger_task_switch(context: &mut CpuContext) {
    SCHEDULER.with(|scheduler| scheduler.switch_task(context));
}
