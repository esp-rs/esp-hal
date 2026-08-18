#![cfg_attr(
    all(docsrs, not(not_really_docsrs)),
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.espressif.com/projects/rust/'>browse the <code>esp-hal</code> documentation on the esp-rs website</a> instead.</p><p>The documentation here on <a href='https://docs.rs'>docs.rs</a> is built for a single chip only (ESP32-C6, in particular), while on the esp-rs website you can select your exact chip from the list of supported devices. Available peripherals and their APIs change depending on the chip.</p></div>\n\n<br/>\n\n"
)]
//! An RTOS (Real-Time Operating System) implementation for esp-hal.
//!
//! This crate provides the runtime necessary to run `async` code on top of esp-hal,
//! and implements the necessary capabilities (threads, queues, etc.) required by esp-radio.
//!
//! ## Setup
//!
//! This crate requires an `esp-hal` timer, as well as the `FROM_CPU0` software interrupt to
//! operate, and needs to be started like so:
//!
//! ```rust, no_run
#![doc = esp_hal::before_snippet!()]
//! # struct FakeHeap;
//! # unsafe impl core::alloc::GlobalAlloc for FakeHeap {
//! #     unsafe fn alloc(&self, _: core::alloc::Layout) -> *mut u8 {
//! #         unimplemented!()
//! #     }
//! #     unsafe fn dealloc(&self, _: *mut u8, _: core::alloc::Layout) {
//! #         unimplemented!()
//! #     }
//! # }
//! # #[global_allocator]
//! # static ALLOCATOR: FakeHeap = FakeHeap;
//! use esp_hal::timer::timg::TimerGroup;
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//!
//! esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
#![cfg_attr(
    multi_core,
    doc = "
// Optionally, start the scheduler on the second core
use static_cell::ConstStaticCell;
use esp_hal::system::Stack;

static STACK: ConstStaticCell<Stack<8192>> = ConstStaticCell::new(Stack::new());
esp_rtos::start_second_core(
    peripherals.CPU_CTRL,
    peripherals.FROM_CPU_INTR1,
    STACK.take(),
    || {}, // Second core's main function.
);
"
)]
#![doc = ""]
//! // You can now start esp-radio:
//! // let esp_radio_controller = esp_radio::init().unwrap();
#![doc = esp_hal::after_snippet!()]
//! ```
//! 
//! To write `async` code, enable the `embassy` feature, and make the main function `async`.
//! This will create a thread-mode executor on the main thread. Note that, to create async tasks, you will need
//! the `task` macro from the `embassy-executor` crate. Do NOT enable any of the `arch-*` features on `embassy-executor`.
#![cfg_attr(
    multi_core,
    doc = r"
The scheduler can also run on the second core alone, which keeps the first core free for bare-metal
code. See [`start_on_second_core_only`] for that configuration and its restrictions.
"
)]
#![cfg_attr(
    sleep_light_sleep,
    doc = r"
## Automatic Light Sleep (experimental)

When enabled, the CPU will automatically enter light sleep mode when there are no tasks to run,
and wake up when a task is ready to run. This can help reduce power consumption.

Because waking up from automatic light sleep can increase latency, the minimum expected idle time
can be configured using `ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US`.

To enable automatic light sleep, call the [`sleep::configure`] function and pass the
idle hook from the returned [`Sleep`](sleep::Sleep) object to [`start_with_idle_hook`].

To prevent the CPU from entering light sleep, take a [`WakeLock`]. Drop the lock when you are done
with the critical code.
"
)]
#![cfg_attr(
    all(sleep_light_sleep, multi_core),
    doc = r"

⚠️ If you are using bare-metal code on the second core (i.e. the second core is
not managed by the RTOS), make sure to take a [`WakeLock`], otherwise the automatic
light sleep may cause unexpected behavior.
"
)]
#![cfg_attr(
    sleep_light_sleep,
    doc = r"
### Example

```rust,no_run
#![no_std]
# #[panic_handler]
# fn panic(_: &core::panic::PanicInfo) -> ! {
#     loop {}
# }
# struct FakeHeap;
# unsafe impl core::alloc::GlobalAlloc for FakeHeap {
#     unsafe fn alloc(&self, _: core::alloc::Layout) -> *mut u8 {
#         unimplemented!()
#     }
#     unsafe fn dealloc(&self, _: *mut u8, _: core::alloc::Layout) {
#         unimplemented!()
#     }
# }
# #[global_allocator]
# static ALLOCATOR: FakeHeap = FakeHeap;

use esp_hal::timer::timg::TimerGroup;

# fn main() {
let p = esp_hal::init(esp_hal::Config::default());

let timg0 = TimerGroup::new(p.TIMG0);

let sleep = esp_rtos::sleep::configure(p.LPWR);

esp_rtos::start_with_idle_hook(
    timg0.timer0,
    p.FROM_CPU_INTR0,
    sleep.light_sleep_hook,
);
# }
```

[`WakeLock`]: esp_hal::rtc_cntl::WakeLock
"
)]
//! ## Additional configuration
#![doc = ""]
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_rtos_config_table.md"))]
#![doc = ""]
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://docs.espressif.com/projects/rust/esp-rs-grey-bg.svg")]
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
#[cfg(sleep_light_sleep)]
pub mod sleep;
mod syscall;
mod task;
mod timer;
// TODO: these esp-radio gates will need to be cleaned up once we re-introduce IPC objects.
#[cfg(feature = "esp-radio")]
mod wait_queue;

#[cfg(feature = "embassy")]
#[cfg_attr(docsrs, doc(cfg(feature = "embassy")))]
pub mod embassy;

use core::mem::MaybeUninit;

#[cfg(feature = "alloc")]
pub(crate) use esp_alloc::InternalMemory;
#[cfg(systimer_driver_supported)]
use esp_hal::timer::systimer::Alarm;
#[cfg(timergroup_driver_supported)]
use esp_hal::timer::timg::Timer;
use esp_hal::{
    Blocking,
    peripherals::FROM_CPU_INTR0,
    system::Cpu,
    time::Instant,
    timer::{AnyTimer, OneShotTimer, any::Degrade},
};
#[cfg(multi_core)]
use esp_hal::{
    peripherals::{CPU_CTRL, FROM_CPU_INTR1},
    system::{CpuControl, Stack},
    time::Duration,
};
#[cfg(feature = "embassy")]
#[cfg_attr(docsrs, doc(cfg(feature = "embassy")))]
pub use macros::main;
#[cfg(multi_core)]
use scheduler::ActiveCores;
pub(crate) use scheduler::SCHEDULER;
pub use task::CurrentThreadHandle;

use crate::{task::IdleFn, timer::TimeDriver};

type TimeBase = OneShotTimer<'static, Blocking>;

/// Trace events, emitted via `marker_begin` and `marker_end`
#[cfg(feature = "rtos-trace")]
pub enum TraceEvents {
    /// The scheduler function is running.
    RunSchedule,

    /// A task has yielded.
    YieldTask,

    /// The timer tick handler is running.
    TimerTickHandler,

    /// Process timer queue.
    ProcessTimerQueue,

    /// Process embassy timer queue.
    #[cfg(feature = "embassy")]
    ProcessEmbassyTimerQueue,
}

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
            // We assume malloc returns a 4-byte aligned pointer. We can skip aligning types
            // that are already aligned to 4 bytes or less.
            let ptr = if layout.align() <= 4 {
                unsafe { malloc_internal(layout.size()) }
            } else {
                // We allocate extra memory so that we can store the number of prefix bytes in the
                // bytes before the actual allocation. We will then use this to
                // restore the pointer to the original allocation.

                // If we can get away with 0 padding bytes, let's do that. In this case, we need
                // space for the prefix length only.
                // We assume malloc returns a 4-byte aligned pointer. This means any higher
                // alignment requirements can be satisfied by at most align - 4
                // bytes of shift, and we can use the remaining 4 bytes for the prefix length.
                let extra = layout.align().max(4);

                let allocation = unsafe { malloc_internal(layout.size() + extra) };

                if allocation.is_null() {
                    return Err(AllocError);
                }

                // reserve at least 4 bytes for the prefix
                let ptr = allocation.wrapping_add(4);

                let align_offset = ptr.align_offset(layout.align());

                let data_ptr = ptr.wrapping_add(align_offset);
                let prefix_ptr = data_ptr.wrapping_sub(4);

                // Store the amount of padding bytes used for alignment.
                unsafe { prefix_ptr.cast::<usize>().write(align_offset) };

                data_ptr
            };

            let ptr = NonNull::new(ptr).ok_or(AllocError)?;
            Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
        }

        unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
            // We assume malloc returns a 4-byte aligned pointer. In that case we don't have to
            // align, so we don't have a prefix.
            if layout.align() <= 4 {
                unsafe { free_internal(ptr.as_ptr()) };
            } else {
                // Retrieve the amount of padding bytes used for alignment.
                let prefix_ptr = ptr.as_ptr().wrapping_sub(4);
                let prefix_bytes = unsafe { prefix_ptr.cast::<usize>().read() };

                unsafe { free_internal(prefix_ptr.wrapping_sub(prefix_bytes)) };
            }
        }
    }
}

/// Timers that can be used as time drivers.
///
/// This trait is meant to be used only for the [`start`] function.
pub trait TimerSource: private::Sealed + 'static {
    /// Returns the timer source.
    fn timer(self) -> TimeBase;
}

mod private {
    pub trait Sealed {}
}

impl private::Sealed for TimeBase {}
impl private::Sealed for AnyTimer<'static> {}
#[cfg(timergroup_driver_supported)]
impl private::Sealed for Timer<'static> {}
#[cfg(systimer_driver_supported)]
impl private::Sealed for Alarm<'static> {}

impl TimerSource for TimeBase {
    fn timer(self) -> TimeBase {
        self
    }
}

impl TimerSource for AnyTimer<'static> {
    fn timer(self) -> TimeBase {
        TimeBase::new(self)
    }
}

#[cfg(timergroup_driver_supported)]
impl TimerSource for Timer<'static> {
    fn timer(self) -> TimeBase {
        TimeBase::new(self.degrade())
    }
}

#[cfg(systimer_driver_supported)]
impl TimerSource for Alarm<'static> {
    fn timer(self) -> TimeBase {
        TimeBase::new(self.degrade())
    }
}

fn init_tracing() {
    #[cfg(feature = "rtos-trace")]
    {
        rtos_trace::trace::name_marker(TraceEvents::YieldTask as u32, "yield task");
        rtos_trace::trace::name_marker(TraceEvents::RunSchedule as u32, "run scheduler");
        rtos_trace::trace::name_marker(TraceEvents::TimerTickHandler as u32, "timer tick handler");
        rtos_trace::trace::name_marker(
            TraceEvents::ProcessTimerQueue as u32,
            "process timer queue",
        );
        rtos_trace::trace::name_marker(
            TraceEvents::ProcessEmbassyTimerQueue as u32,
            "process embassy timer queue",
        );
        rtos_trace::trace::start();
    }
}

fn assert_thread_mode(function: &str) {
    assert!(
        esp_hal::interrupt::RunLevel::current().is_thread(),
        "{} must not be called from an interrupt handler",
        function
    );
}

/// Starts the scheduler.
///
/// The current context will be converted into the main task, and will be pinned to the first core.
///
/// This function is equivalent to [`start_with_idle_hook`], with the default idle hook. The default
/// idle hook will wait for an interrupt.
///
/// For information about the arguments, see [`start_with_idle_hook`].
pub fn start(timer: impl TimerSource, int0: FROM_CPU_INTR0<'static>) {
    start_with_idle_hook(timer, int0, crate::task::idle_hook)
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
/// - A `OneShotTimer` instance
///
/// For an example, see the [crate-level documentation][self].
pub fn start_with_idle_hook(
    timer: impl TimerSource,
    int0: FROM_CPU_INTR0<'static>,
    idle_hook: IdleFn,
) {
    init_tracing();

    trace!("Starting scheduler for the first core");
    assert_eq!(Cpu::current(), Cpu::ProCpu);
    assert_thread_mode("esp_rtos::start");

    SCHEDULER.with(move |scheduler| {
        scheduler.setup(TimeDriver::new(timer.timer()), idle_hook);
        syscall::setup_syscalls();

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
            (stack_top as usize - stack_bottom as usize) / 4,
        );

        task::allocate_main_task(
            scheduler,
            stack_slice,
            esp_config::esp_config_int!(usize, "ESP_HAL_CONFIG_STACK_GUARD_OFFSET"),
            // For compatibility with -Zstack-protector, we read and use the value of
            // `__stack_chk_guard`.
            unsafe { (&raw const __stack_chk_guard).read_volatile() },
        );

        task::setup_multitasking(int0);

        // Set up the main task's context.
        task::yield_task();
    })
}

/// Starts the scheduler on the second CPU core.
///
/// Note that the scheduler must be started first, before starting the second core.
///
/// The supplied stack and function will be used as the main thread of the second core. The thread
/// will be pinned to the second core.
///
/// You can return from the second core's main thread function. This will cause the scheduler to
/// enter the idle state, but the second core will continue to run interrupt handlers and other
/// tasks.
#[cfg(multi_core)]
pub fn start_second_core<const STACK_SIZE: usize>(
    cpu_control: CPU_CTRL,
    int1: FROM_CPU_INTR1<'static>,
    stack: &'static mut Stack<STACK_SIZE>,
    func: impl FnOnce() + Send + 'static,
) {
    start_second_core_with_stack_guard_offset::<STACK_SIZE>(cpu_control, int1, stack, None, func);
}

/// The stack of the second core, in a form that can be moved into the second core's main function.
#[cfg(multi_core)]
struct SecondCoreStack {
    stack: *mut [MaybeUninit<u32>],
}

#[cfg(multi_core)]
unsafe impl Send for SecondCoreStack {}

#[cfg(multi_core)]
impl SecondCoreStack {
    fn new<const STACK_SIZE: usize>(stack: &mut Stack<STACK_SIZE>) -> Self {
        Self {
            stack: core::ptr::slice_from_raw_parts_mut(
                stack.bottom().cast::<MaybeUninit<u32>>(),
                STACK_SIZE / 4,
            ),
        }
    }

    /// Turns the current context of the second core into its main task.
    ///
    /// This takes `self` by value, so that a `move` closure captures the whole struct instead of
    /// its `!Send` field only.
    fn allocate_main_task(self, scheduler: &mut scheduler::SchedulerState, guard_offset: usize) {
        // esp-hal may be configured to use a watchpoint. To work around that, we read the memory at
        // the stack guard, and we'll use whatever we find as the main task's stack guard value,
        // instead of writing our own stack guard value.
        let stack_bottom = self.stack.cast::<u32>();
        let stack_guard = unsafe { stack_bottom.byte_add(guard_offset) };

        task::allocate_main_task(scheduler, self.stack, guard_offset, unsafe {
            stack_guard.read()
        });
    }
}

#[cfg(multi_core)]
fn default_stack_guard_offset() -> usize {
    esp_config::esp_config_int!(usize, "ESP_HAL_CONFIG_STACK_GUARD_OFFSET")
}

/// Waits for the scheduler of the second core to start.
#[cfg(multi_core)]
fn wait_for_second_core_scheduler() {
    let start = Instant::now();

    while start.elapsed() < Duration::from_secs(1) {
        if SCHEDULER.with(|s| s.active_cores.contains(Cpu::AppCpu)) {
            return;
        }
        esp_hal::rom::ets_delay_us(1);
    }

    panic!(
        "Second core scheduler failed to initialize. \
        This can happen if its main function overflowed the stack."
    );
}

#[cfg(multi_core)]
fn suspend_main_task() {
    loop {
        SCHEDULER.sleep_until(Instant::EPOCH + Duration::MAX);
    }
}

/// Starts the scheduler on the second CPU core.
///
/// Note that the scheduler must be started first, before starting the second core.
///
/// The supplied stack and function will be used as the main thread of the second core. The thread
/// will be pinned to the second core.
///
/// The stack guard offset is used to reserve a portion of the stack for the stack guard, for safety
/// purposes. Passing `None` will result in the default value configured by the
/// `ESP_HAL_CONFIG_STACK_GUARD_OFFSET` esp-hal configuration.
///
/// You can return from the second core's main thread function. This will cause the scheduler to
/// enter the idle state, but the second core will continue to run interrupt handlers and other
/// tasks.
#[cfg(multi_core)]
pub fn start_second_core_with_stack_guard_offset<const STACK_SIZE: usize>(
    cpu_control: CPU_CTRL,
    int1: FROM_CPU_INTR1<'static>,
    stack: &'static mut Stack<STACK_SIZE>,
    stack_guard_offset: Option<usize>,
    func: impl FnOnce() + Send + 'static,
) {
    trace!("Starting scheduler for the second core");

    match SCHEDULER.with(|scheduler| scheduler.active_cores) {
        ActiveCores::Single(Cpu::ProCpu) => {}
        _ => unreachable!(),
    }

    let stack_ptrs = SecondCoreStack::new(stack);
    let stack_guard_offset = stack_guard_offset.unwrap_or_else(default_stack_guard_offset);

    let mut cpu_control = CpuControl::new(cpu_control);
    let guard = cpu_control
        .start_app_core_with_stack_guard_offset(stack, Some(stack_guard_offset), move || {
            trace!("Second core running");
            SCHEDULER.with(move |scheduler| {
                task::setup_smp(int1);
                assert!(
                    scheduler.time_driver.is_some(),
                    "The scheduler must be started on the first core first."
                );

                scheduler.active_cores = ActiveCores::All;

                stack_ptrs.allocate_main_task(scheduler, stack_guard_offset);
                task::yield_task();
                trace!("Second core scheduler initialized");
            });

            func();
            suspend_main_task();
        })
        .unwrap();

    wait_for_second_core_scheduler();

    core::mem::forget(guard);
}

/// Starts the scheduler on the second CPU core only.
///
/// Use this function if you want to keep the first core free for bare-metal code, and run all
/// RTOS-managed work on the second core. Call it from the first core. It returns on the first core,
/// which then continues to run bare-metal code.
///
/// The scheduler does not run on the first core in this configuration. This function must not be
/// combined with [`start`] or [`start_second_core`]; either combination panics.
///
/// The supplied stack and function become the main thread of the second core. The thread is pinned
/// to the second core. For the list of accepted timer sources, see [`start_with_idle_hook`].
///
/// You can return from the second core's main thread function. This will cause the scheduler to
/// enter the idle state, but the second core will continue to run interrupt handlers and other
/// tasks.
///
/// ## Restrictions
///
/// - The first core must not call any `esp-rtos` API, not even to wake a task that runs on the
///   second core. The scheduler protects its state with a lock that disables interrupts and spins,
///   which would break the timing of the bare-metal code on the first core.
/// - Automatic light sleep is not available, because this function takes no idle hook.
/// - The Wi-Fi and Bluetooth LE drivers of `esp-radio` are not supported.
/// - `#[esp_rtos::main]` on an `async fn` cannot be used, because it creates a thread-mode executor
///   on the first core. Create the thread-mode executor (`embassy::Executor`) inside `func`
///   instead.
///
/// ## Example
///
/// ```rust, no_run
#[doc = esp_hal::before_snippet!()]
/// # struct FakeHeap;
/// # unsafe impl core::alloc::GlobalAlloc for FakeHeap {
/// #     unsafe fn alloc(&self, _: core::alloc::Layout) -> *mut u8 {
/// #         unimplemented!()
/// #     }
/// #     unsafe fn dealloc(&self, _: *mut u8, _: core::alloc::Layout) {
/// #         unimplemented!()
/// #     }
/// # }
/// # #[global_allocator]
/// # static ALLOCATOR: FakeHeap = FakeHeap;
/// use esp_hal::{
///     system::Stack,
///     timer::timg::TimerGroup,
/// };
/// use static_cell::ConstStaticCell;
///
/// static STACK: ConstStaticCell<Stack<8192>> = ConstStaticCell::new(Stack::new());
///
/// let timg0 = TimerGroup::new(peripherals.TIMG0);
///
/// esp_rtos::start_on_second_core_only(
///     peripherals.CPU_CTRL,
///     peripherals.FROM_CPU_INTR1,
///     timg0.timer0,
///     STACK.take(),
///     || {
///         // The main thread of the second core.
///     },
/// );
///
/// // The first core continues here, and must not call any esp-rtos API.
#[doc = esp_hal::after_snippet!()]
/// ```
#[cfg(multi_core)]
pub fn start_on_second_core_only<const STACK_SIZE: usize>(
    cpu_control: CPU_CTRL<'static>,
    int1: FROM_CPU_INTR1<'static>,
    timer: impl TimerSource + Send,
    stack: &'static mut Stack<STACK_SIZE>,
    func: impl FnOnce() + Send + 'static,
) {
    init_tracing();

    trace!("Starting scheduler for the second core only");
    assert_eq!(Cpu::current(), Cpu::ProCpu);
    assert_thread_mode("esp_rtos::start_on_second_core_only");
    assert!(
        SCHEDULER.with(|scheduler| scheduler.time_driver.is_none()),
        "The scheduler has already been started. \
        esp_rtos::start_on_second_core_only must not be combined with esp_rtos::start."
    );

    let stack_ptrs = SecondCoreStack::new(stack);
    let stack_guard_offset = default_stack_guard_offset();

    let mut cpu_control = CpuControl::new(cpu_control);
    let guard = cpu_control
        .start_app_core_with_stack_guard_offset(stack, Some(stack_guard_offset), move || {
            trace!("Second core running");
            SCHEDULER.with(move |scheduler| {
                task::setup_multitasking(int1);

                // The time driver must be created here, and not on the first core:
                // `Timer::set_interrupt_handler` binds the timer interrupt to the core that calls
                // it.
                scheduler.setup(TimeDriver::new(timer.timer()), crate::task::idle_hook);
                syscall::setup_syscalls();

                stack_ptrs.allocate_main_task(scheduler, stack_guard_offset);
                task::yield_task();
                trace!("Second core scheduler initialized");
            });

            func();
            suspend_main_task();
        })
        .unwrap();

    wait_for_second_core_scheduler();

    core::mem::forget(guard);
}

const TICK_RATE: u32 = esp_config::esp_config_int!(u32, "ESP_RTOS_CONFIG_TICK_RATE_HZ");

pub(crate) fn now() -> u64 {
    Instant::now().duration_since_epoch().as_micros()
}

/// Rearms the alarm timer.
///
/// This function may be necessary to call after waking up from light sleep.
pub fn rearm_alarm() {
    SCHEDULER.with(|s| {
        if let Some(time_driver) = s.time_driver.as_mut() {
            time_driver.rearm(now());
        }
    });
}

#[cfg(feature = "embassy")]
embassy_time_driver::time_driver_impl!(static TIMER_QUEUE: crate::timer::embassy::EmbassyTimeDriver = crate::timer::embassy::EmbassyTimeDriver);
