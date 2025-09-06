//! This crate allows using esp-radio on top of esp-hal, without any other OS.
//!
//! This crate needs to be initialized with an esp-hal timer before the scheduler can be started.
//!
//! ```rust, no_run
#![doc = esp_hal::before_snippet!()]
//! use esp_hal::timer::timg::TimerGroup;
//!
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! esp_preempt::init(timg0.timer0);
//!
//! // You can now start esp-radio:
//! // let esp_radio_controller = esp_radio::init().unwrap();
//! # }
//! ```
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![no_std]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]

extern crate alloc;

// MUST be the first module
mod fmt;

mod mutex;
mod queue;
mod semaphore;
mod task;
mod timer;
mod timer_queue;

use core::{ffi::c_void, ptr::NonNull};

use allocator_api2::boxed::Box;
use esp_hal::{
    Blocking,
    time::{Duration, Instant, Rate},
    timer::{AnyTimer, PeriodicTimer},
};
use esp_radio_preempt_driver::semaphore::{SemaphoreImplementation, SemaphorePtr};
use esp_sync::NonReentrantMutex;

use crate::{
    semaphore::Semaphore,
    task::{
        Context,
        TaskAllocListElement,
        TaskDeleteListElement,
        TaskList,
        TaskPtr,
        TaskQueue,
        TaskReadyListElement,
    },
    timer::TIMER,
};

type TimeBase = PeriodicTimer<'static, Blocking>;

// Polyfill the InternalMemory allocator
#[cfg(not(feature = "esp-alloc"))]
mod esp_alloc {
    use core::{alloc::Layout, ptr::NonNull};

    use allocator_api2::alloc::{AllocError, Allocator};

    /// An allocator that uses internal memory only.
    pub struct InternalMemory;

    unsafe impl Allocator for InternalMemory {
        fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
            unsafe extern "C" {
                fn esp_radio_allocate_from_internal_ram(size: usize) -> *mut u8;
            }
            let raw_ptr = unsafe { esp_radio_allocate_from_internal_ram(layout.size()) };
            let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
            Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
        }

        unsafe fn deallocate(&self, ptr: NonNull<u8>, _layout: Layout) {
            unsafe extern "C" {
                fn esp_radio_deallocate_internal_ram(ptr: *mut u8);
            }
            unsafe {
                esp_radio_deallocate_internal_ram(ptr.as_ptr());
            }
        }
    }
}

pub(crate) use esp_alloc::InternalMemory;

/// A trait to allow better UX for initializing esp-preempt.
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

struct SchedulerState {
    /// Pointer to the current task.
    current_task: Option<TaskPtr>,

    /// A list of all allocated tasks
    all_tasks: TaskList<TaskAllocListElement>,

    /// A list of tasks ready to run
    ready_tasks: TaskQueue<TaskReadyListElement>,

    /// Pointer to the task that is scheduled for deletion.
    to_delete: TaskList<TaskDeleteListElement>,
}

unsafe impl Send for SchedulerState {}

impl SchedulerState {
    const fn new() -> Self {
        Self {
            current_task: None,
            all_tasks: TaskList::new(),
            ready_tasks: TaskQueue::new(),
            to_delete: TaskList::new(),
        }
    }

    fn delete_marked_tasks(&mut self) {
        while let Some(to_delete) = self.to_delete.pop() {
            self.all_tasks.remove(to_delete);
            self.ready_tasks.remove(to_delete);

            unsafe {
                let task = Box::from_raw_in(to_delete.as_ptr(), InternalMemory);
                core::mem::drop(task);
            }
        }
    }

    fn select_next_task(&mut self, currently_active_task: TaskPtr) -> Option<TaskPtr> {
        // At least one task must be ready to run. If there are none, we don't switch tasks.
        let next = self.ready_tasks.pop()?;

        // TODO: do not mark current task immediately ready to run
        self.ready_tasks.push(currently_active_task);

        if next == currently_active_task {
            return None;
        }

        Some(next)
    }

    #[cfg(xtensa)]
    fn switch_task(&mut self, trap_frame: &mut esp_hal::trapframe::TrapFrame) {
        self.delete_marked_tasks();
        let mut current_task = unwrap!(self.current_task);

        if let Some(mut next_task) = self.select_next_task(current_task) {
            task::save_task_context(unsafe { current_task.as_mut() }, trap_frame);
            task::restore_task_context(unsafe { next_task.as_mut() }, trap_frame);

            self.current_task = Some(next_task);
        }
    }

    #[cfg(riscv)]
    fn switch_task(&mut self) {
        self.delete_marked_tasks();
        let mut current = unwrap!(self.current_task);

        if let Some(mut next) = self.select_next_task(current) {
            let old_ctx = unsafe { &raw mut current.as_mut().trap_frame };
            let new_ctx = unsafe { &raw mut next.as_mut().trap_frame };

            if crate::task::arch_specific::task_switch(old_ctx, new_ctx) {
                self.current_task = Some(next);
            }
        }
    }

    fn schedule_task_deletion(&mut self, task_to_delete: *mut Context) -> bool {
        let current_task = unwrap!(self.current_task);
        let task_to_delete = NonNull::new(task_to_delete).unwrap_or(current_task);
        let is_current = task_to_delete == current_task;

        self.to_delete.push(task_to_delete);

        is_current
    }
}

fn usleep(us: u32) {
    trace!("usleep");
    unsafe extern "C" {
        fn esp_rom_delay_us(us: u32);
    }

    const MIN_YIELD_TIME: u32 = 1_000_000 / TICK_RATE;
    if us < MIN_YIELD_TIME {
        // Short wait, just sleep
        unsafe { esp_rom_delay_us(us) };
    } else {
        const MIN_YIELD_DURATION: Duration = Duration::from_micros(MIN_YIELD_TIME as u64);
        let sleep_for = Duration::from_micros(us as u64);
        let start = Instant::now();
        loop {
            // Yield to other tasks
            timer::yield_task();

            let elapsed = start.elapsed();
            if elapsed.as_micros() > us as u64 {
                break;
            }

            let remaining = sleep_for - elapsed;

            if remaining < MIN_YIELD_DURATION {
                // If the remaining time is less than the minimum yield time, we can just sleep
                // for the remaining time.
                unsafe { esp_rom_delay_us(remaining.as_micros() as u32) };
                break;
            }
        }
    }
}

struct Scheduler {
    inner: NonReentrantMutex<SchedulerState>,
}

impl Scheduler {
    pub(crate) fn with<R>(&self, cb: impl FnMut(&mut SchedulerState) -> R) -> R {
        self.inner.with(cb)
    }
}

esp_radio_preempt_driver::scheduler_impl!(static SCHEDULER: Scheduler = Scheduler {
    inner: NonReentrantMutex::new(SchedulerState::new())
});

/// Initializes the scheduler.
///
/// # The `timer` argument
///
/// The `timer` argument is a timer source that is used by the scheduler to
/// schedule internal tasks. The timer source can be any of the following:
///
/// - A timg `Timer` instance
/// - A systimer `Alarm` instance
/// - An `AnyTimer` instance
///
/// For an example, see the [crate-level documentation][self].
pub fn init(timer: impl TimerSource) {
    timer::setup_timebase(timer.timer());
}

const TICK_RATE: u32 = esp_config::esp_config_int!(u32, "ESP_PREEMPT_CONFIG_TICK_RATE_HZ");
const TIMESLICE_FREQUENCY: Rate = Rate::from_hz(TICK_RATE);

impl esp_radio_preempt_driver::Scheduler for Scheduler {
    fn initialized(&self) -> bool {
        timer::initialized()
    }

    fn enable(&self) {
        // allocate the main task
        task::allocate_main_task();
        timer::setup_multitasking();
        timer_queue::create_timer_task();

        TIMER.with(|t| {
            let t = unwrap!(t.as_mut());
            unwrap!(t.start(TIMESLICE_FREQUENCY.as_duration()));
        });
    }

    fn disable(&self) {
        timer::disable_timebase();
        timer::disable_multitasking();
        task::delete_all_tasks();
    }

    fn yield_task(&self) {
        timer::yield_task()
    }

    fn yield_task_from_isr(&self) {
        self.yield_task();
    }

    fn max_task_priority(&self) -> u32 {
        255
    }

    fn task_create(
        &self,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        _priority: u32,
        _pin_to_core: Option<u32>,
        task_stack_size: usize,
    ) -> *mut c_void {
        let task = Box::new_in(Context::new(task, param, task_stack_size), InternalMemory);
        let task_ptr = NonNull::from(Box::leak(task));

        SCHEDULER.with(|state| {
            state.all_tasks.push(task_ptr);
            state.ready_tasks.push(task_ptr);
        });

        task_ptr.as_ptr().cast()
    }

    fn current_task(&self) -> *mut c_void {
        task::current_task() as *mut c_void
    }

    fn schedule_task_deletion(&self, task_handle: *mut c_void) {
        task::schedule_task_deletion(task_handle as *mut Context)
    }

    fn current_task_thread_semaphore(&self) -> SemaphorePtr {
        task::with_current_task(|task| {
            if task.thread_semaphore.is_none() {
                task.thread_semaphore = Some(Semaphore::create(1, 0));
            }

            unwrap!(task.thread_semaphore)
        })
    }

    fn usleep(&self, us: u32) {
        usleep(us)
    }

    fn now(&self) -> u64 {
        // FIXME: this function needs to return the timestamp of the scheduler's timer
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros()
    }
}
