#![cfg_attr(
    all(docsrs, not(not_really_docsrs)),
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.espressif.com/projects/rust/'>browse the <code>esp-hal</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C6, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs change depending on the chip.</p></div>\n\n<br/>\n\n"
)]
//! This crate provides RTOS functionality for `esp-radio`, and provides executors to enable
//! running `async` code.
//!
//! ## Setup
//!
//! This crate requires an esp-hal timer to operate, and needs to be started like so:
//!
//! ```rust, no_run
//! use esp_hal::timer::timg::TimerGroup;
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
#![doc = esp_hal::before_snippet!()]
#![cfg_attr(
    any(riscv, multi_core),
    doc = "

use esp_hal::interrupt::software::SoftwareInterruptControl;
let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);"
)]
#![cfg_attr(
    xtensa,
    doc = "

esp_rtos::start(timg0.timer0);"
)]
#![cfg_attr(
    riscv,
    doc = "

esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);"
)]
#![cfg_attr(
    all(xtensa, multi_core),
    doc = "
// Optionally, start the scheduler on the second core
esp_rtos::start_second_core(
    software_interrupt.software_interrupt0,
    software_interrupt.software_interrupt1,
    || {}, // Second core's main function.
);
"
)]
#![cfg_attr(
    all(riscv, multi_core),
    doc = "
// Optionally, start the scheduler on the second core
esp_rtos::start_second_core(
    software_interrupt.software_interrupt1,
    || {}, // Second core's main function.
);
"
)]
#![doc = ""]
//! // You can now start esp-radio:
//! // let esp_radio_controller = esp_radio::init().unwrap();
//! # }
//! ```
//! 
//! To write `async` code, enable the `embassy` feature, and mark the main function with `#[esp_rtos::main]`.
//! Note that, to create async tasks, you will need the `task` macro from the `embassy-executor` crate. Do
//! NOT enable any of the `arch-*` features on `embassy-executor`.
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![no_std]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![deny(missing_docs)]

#[cfg(feature = "alloc")]
extern crate alloc;

// MUST be the first module
mod fmt;

#[cfg(feature = "esp-radio")]
mod esp_radio;
mod run_queue;
mod scheduler;
pub mod semaphore;
mod task;
mod timer;
mod wait_queue;

#[cfg(feature = "embassy")]
#[cfg_attr(docsrs, doc(cfg(feature = "embassy")))]
pub mod embassy;

use core::mem::MaybeUninit;

#[cfg(feature = "alloc")]
pub(crate) use esp_alloc::InternalMemory;
#[cfg(any(multi_core, riscv))]
use esp_hal::interrupt::software::SoftwareInterrupt;
use esp_hal::{
    Blocking,
    system::Cpu,
    time::{Duration, Instant},
    timer::{AnyTimer, OneShotTimer},
};
#[cfg(multi_core)]
use esp_hal::{
    peripherals::CPU_CTRL,
    system::{CpuControl, Stack},
};
#[cfg_attr(docsrs, doc(cfg(feature = "embassy")))]
pub use macros::rtos_main as main;
pub(crate) use scheduler::SCHEDULER;
pub use task::CurrentThreadHandle;

use crate::{task::IdleFn, timer::TimeDriver};

type TimeBase = OneShotTimer<'static, Blocking>;

// Polyfill the InternalMemory allocator
#[cfg(all(feature = "alloc", not(feature = "esp-alloc")))]
mod esp_alloc {
    use core::{alloc::Layout, ptr::NonNull};

    use allocator_api2::alloc::{AllocError, Allocator};

    unsafe extern "C" {
        fn malloc_internal(size: usize) -> *mut u8;

        fn free_internal(ptr: *mut u8);
    }

    /// An allocator that uses internal memory only.
    pub struct InternalMemory;

    unsafe impl Allocator for InternalMemory {
        fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
            let raw_ptr = unsafe { malloc_internal(layout.size()) };
            let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
            Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
        }

        unsafe fn deallocate(&self, ptr: NonNull<u8>, _layout: Layout) {
            unsafe { free_internal(ptr.as_ptr()) };
        }
    }
}

/// A trait to allow better UX for initializing esp-rtos.
///
/// This trait is meant to be used only for the `init` function.
pub trait TimerSource: private::Sealed + 'static {
    /// Returns the timer source.
    fn timer(self) -> TimeBase;
}

mod private {
    pub trait Sealed {}
}

impl private::Sealed for TimeBase {}
impl private::Sealed for AnyTimer<'static> {}
#[cfg(timergroup)]
impl private::Sealed for esp_hal::timer::timg::Timer<'static> {}
#[cfg(systimer)]
impl private::Sealed for esp_hal::timer::systimer::Alarm<'static> {}

impl<T> TimerSource for T
where
    T: esp_hal::timer::any::Degrade + private::Sealed + 'static,
{
    fn timer(self) -> TimeBase {
        let any_timer: AnyTimer<'static> = self.degrade();
        TimeBase::new(any_timer)
    }
}

/// Starts the scheduler.
///
/// The current context will be converted into the main task, and will be pinned to the first core.
///
/// This function is equivalent to [start_with_idle_hook], with the default idle hook. The default
/// idle hook will wait for an interrupt.
///
/// For information about the other arguments, see [start_with_idle_hook].
pub fn start(timer: impl TimerSource, #[cfg(riscv)] int0: SoftwareInterrupt<'static, 0>) {
    start_with_idle_hook(
        timer,
        #[cfg(riscv)]
        int0,
        crate::task::idle_hook,
    )
}

/// Starts the scheduler, with a custom idle hook.
///
/// The current context will be converted into the main task, and will be pinned to the first core.
///
/// The idle hook will be called when no tasks are ready to run. The idle hook's context is not
/// preserved. If you need to execute a longer process to enter a low-power state, make sure to call
/// the relevant code in a critical section.
///
/// The `timer` argument is a timer source that is used by the scheduler to
/// schedule internal tasks. The timer source can be any of the following:
///
/// - A timg `Timer` instance
/// - A systimer `Alarm` instance
/// - An `AnyTimer` instance
#[doc = ""]
#[cfg_attr(
    riscv,
    doc = "The `int0` argument must be `SoftwareInterrupt<0>` which will be used to trigger context switches."
)]
#[doc = ""]
/// For an example, see the [crate-level documentation][self].
pub fn start_with_idle_hook(
    timer: impl TimerSource,
    #[cfg(riscv)] int0: SoftwareInterrupt<'static, 0>,
    idle_hook: IdleFn,
) {
    trace!("Starting scheduler for the first core");
    assert_eq!(Cpu::current(), Cpu::ProCpu);

    SCHEDULER.with(move |scheduler| {
        scheduler.setup(TimeDriver::new(timer.timer()), idle_hook);

        // Allocate the default task.

        unsafe extern "C" {
            static _stack_start_cpu0: u32;
            static _stack_end_cpu0: u32;
            static __stack_chk_guard: u32;
        }
        let stack_top = &raw const _stack_start_cpu0;
        let stack_bottom = (&raw const _stack_end_cpu0).cast::<MaybeUninit<u32>>();
        let stack_slice = core::ptr::slice_from_raw_parts_mut(
            stack_bottom.cast_mut(),
            stack_top as usize - stack_bottom as usize,
        );

        task::allocate_main_task(
            scheduler,
            stack_slice,
            esp_config::esp_config_int!(usize, "ESP_HAL_CONFIG_STACK_GUARD_OFFSET"),
            // For compatibility with -Zstack-protector, we read and use the value of
            // `__stack_chk_guard`.
            unsafe { (&raw const __stack_chk_guard).read_volatile() },
        );

        task::setup_multitasking(
            #[cfg(riscv)]
            int0,
        );

        task::yield_task();
    })
}

/// Starts the scheduler on the second CPU core.
///
/// Note that the scheduler must be started first, before starting the second core.
///
/// The supplied stack and function will be used as the main thread of the second core. The thread
/// will be pinned to the second core.
#[cfg(multi_core)]
pub fn start_second_core<const STACK_SIZE: usize>(
    cpu_control: CPU_CTRL,
    #[cfg(xtensa)] int0: SoftwareInterrupt<'static, 0>,
    int1: SoftwareInterrupt<'static, 1>,
    stack: &'static mut Stack<STACK_SIZE>,
    func: impl FnOnce() + Send + 'static,
) {
    start_second_core_with_stack_guard_offset::<STACK_SIZE>(
        cpu_control,
        #[cfg(xtensa)]
        int0,
        int1,
        stack,
        Some(esp_config::esp_config_int!(
            usize,
            "ESP_HAL_CONFIG_STACK_GUARD_OFFSET"
        )),
        func,
    );
}

/// Starts the scheduler on the second CPU core.
///
/// Note that the scheduler must be started first, before starting the second core.
///
/// The supplied stack and function will be used as the main thread of the second core. The thread
/// will be pinned to the second core.
#[cfg(multi_core)]
pub fn start_second_core_with_stack_guard_offset<const STACK_SIZE: usize>(
    cpu_control: CPU_CTRL,
    #[cfg(xtensa)] int0: SoftwareInterrupt<'static, 0>,
    int1: SoftwareInterrupt<'static, 1>,
    stack: &'static mut Stack<STACK_SIZE>,
    stack_guard_offset: Option<usize>,
    func: impl FnOnce() + Send + 'static,
) {
    trace!("Starting scheduler for the second core");

    #[cfg(xtensa)]
    task::setup_smp(int0);

    struct SecondCoreStack {
        stack: *mut [MaybeUninit<u32>],
    }
    unsafe impl Send for SecondCoreStack {}
    let stack_ptrs = SecondCoreStack {
        stack: core::ptr::slice_from_raw_parts_mut(
            stack.bottom().cast::<MaybeUninit<u32>>(),
            STACK_SIZE,
        ),
    };

    let mut cpu_control = CpuControl::new(cpu_control);
    let guard = cpu_control
        .start_app_core_with_stack_guard_offset(stack, stack_guard_offset, move || {
            trace!("Second core running");
            task::setup_smp(int1);
            SCHEDULER.with(move |scheduler| {
                // Make sure the whole struct is captured, not just a !Send field.
                let ptrs = stack_ptrs;
                assert!(
                    scheduler.time_driver.is_some(),
                    "The scheduler must be started on the first core first."
                );

                // esp-hal may be configured to use a watchpoint. To work around that, we read the
                // memory at the stack guard, and we'll use whatever we find as the main task's
                // stack guard value.
                let stack_guard_offset = stack_guard_offset.unwrap_or(esp_config::esp_config_int!(
                    usize,
                    "ESP_HAL_CONFIG_STACK_GUARD_OFFSET"
                ));

                let stack_bottom = ptrs.stack.cast::<u32>();
                let stack_guard = unsafe { stack_bottom.byte_add(stack_guard_offset) };

                task::allocate_main_task(scheduler, ptrs.stack, stack_guard_offset, unsafe {
                    stack_guard.read()
                });
                task::yield_task();
                trace!("Second core scheduler initialized");
            });

            func();

            loop {
                SCHEDULER.sleep_until(Instant::EPOCH + Duration::MAX);
            }
        })
        .unwrap();

    core::mem::forget(guard);
}

const TICK_RATE: u32 = esp_config::esp_config_int!(u32, "ESP_RTOS_CONFIG_TICK_RATE_HZ");

pub(crate) fn now() -> u64 {
    Instant::now().duration_since_epoch().as_micros()
}

#[cfg(feature = "embassy")]
embassy_time_driver::time_driver_impl!(static TIMER_QUEUE: crate::timer::embassy::TimerQueue = crate::timer::embassy::TimerQueue::new());

/// Waits for a condition to be met or a timeout to occur.
///
/// This function is meant to simplify implementation of blocking primitives. Upon failure the
/// `attempt` function should enqueue the task in a wait queue and put the task to sleep.
fn with_deadline(timeout_us: Option<u32>, attempt: impl Fn(Instant) -> bool) -> bool {
    let deadline = timeout_us
        .map(|us| Instant::now() + Duration::from_micros(us as u64))
        .unwrap_or(Instant::EPOCH + Duration::MAX);

    while !attempt(deadline) {
        // We are here because the operation failed. We've either timed out, or the operation is
        // ready to be attempted again. However, any higher priority task can wake up and
        // preempt us still. Let's just check for the timeout, and try the whole process
        // again.

        if timeout_us.is_some() && deadline < Instant::now() {
            // We have a deadline and we've timed out.
            return false;
        }
        // We can block more, so let's attempt the operation again.
    }

    true
}
