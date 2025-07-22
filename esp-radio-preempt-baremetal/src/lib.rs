//! This crate allows using esp-wifi on top of esp-hal, without any other OS.
//!
//! This crate needs to be initialized with an esp-hal timer before the scheduler can be started.
//!
//! ```rust, no_run
#![doc = esp_hal::before_snippet!()]
//! use esp_hal::timer::timg::TimerGroup;
//!
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! esp_radio_preempt_baremetal::init(timg0.timer0);
//!
//! // You can now start esp-wifi:
//! // let esp_wifi_controller = esp_wifi::init().unwrap();
//! # }
//! ```

#![no_std]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]

extern crate alloc;

// MUST be the first module
mod fmt;

mod task;
mod timer;

use core::ffi::c_void;

use allocator_api2::boxed::Box;
use esp_hal::{
    Blocking,
    sync::Locked,
    time::{Duration, Instant, Rate},
    timer::{AnyTimer, PeriodicTimer},
    trapframe::TrapFrame,
};

use crate::{task::Context, timer::TIMER};

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
                fn esp_wifi_allocate_from_internal_ram(size: usize) -> *mut u8;
            }
            let raw_ptr = unsafe { esp_wifi_allocate_from_internal_ram(layout.size()) };
            let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
            Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
        }

        unsafe fn deallocate(&self, ptr: NonNull<u8>, _layout: Layout) {
            unsafe extern "C" {
                fn esp_wifi_deallocate_internal_ram(ptr: *mut u8);
            }
            unsafe {
                esp_wifi_deallocate_internal_ram(ptr.as_ptr());
            }
        }
    }
}

pub(crate) use esp_alloc::InternalMemory;

/// A trait to allow better UX for initializing esp-radio-preempt-baremetal.
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
    ///
    /// Tasks are stored in a circular linked list. CTX_NOW points to the
    /// current task.
    current_task: *mut Context,

    /// Pointer to the task that is scheduled for deletion.
    to_delete: *mut Context,
}

impl SchedulerState {
    const fn new() -> Self {
        Self {
            current_task: core::ptr::null_mut(),
            to_delete: core::ptr::null_mut(),
        }
    }

    fn delete_task(&mut self, task: *mut Context) {
        let mut current_task = self.current_task;
        // Save the first pointer so we can prevent an accidental infinite loop.
        let initial = current_task;
        loop {
            // We don't have the previous pointer, so we need to walk forward in the circle
            // even if we need to delete the first task.

            // If the next task is the one we want to delete, we need to remove it from the
            // list, then drop it.
            let next_task = unsafe { (*current_task).next };
            if core::ptr::eq(next_task, task) {
                unsafe {
                    (*current_task).next = (*next_task).next;

                    core::ptr::drop_in_place(task);
                    break;
                }
            }

            // If the next task is the first task, we can stop. If we needed to delete the
            // first task, we have already handled it in the above case. If we needed to
            // delete another task, it has already been deleted in a previous iteration.
            if core::ptr::eq(next_task, initial) {
                break;
            }

            // Move to the next task.
            current_task = next_task;
        }
    }

    fn switch_task(&mut self, trap_frame: &mut TrapFrame) {
        task::save_task_context(unsafe { &mut *self.current_task }, trap_frame);

        if !self.to_delete.is_null() {
            let task_to_delete = core::mem::take(&mut self.to_delete);
            self.delete_task(task_to_delete);
        }

        unsafe { self.current_task = (*self.current_task).next };

        task::restore_task_context(unsafe { &mut *self.current_task }, trap_frame);
    }

    fn schedule_task_deletion(&mut self, task: *mut Context) -> bool {
        if task.is_null() {
            self.to_delete = self.current_task;
            true
        } else {
            self.to_delete = task;
            core::ptr::eq(task, self.current_task)
        }
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

static SCHEDULER_STATE: Locked<SchedulerState> = Locked::new(SchedulerState::new());

struct Scheduler {}

esp_radio_preempt_driver::scheduler_impl!(static SCHEDULER: Scheduler = Scheduler {});

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

const TICK_RATE: u32 =
    esp_config::esp_config_int!(u32, "ESP_RADIO_PREEMPT_BAREMETAL_CONFIG_TICK_RATE_HZ");
const TIMESLICE_FREQUENCY: Rate = Rate::from_hz(TICK_RATE);

impl esp_radio_preempt_driver::Scheduler for Scheduler {
    fn initialized(&self) -> bool {
        timer::initialized()
    }

    fn usleep(&self, us: u32) {
        usleep(us)
    }

    fn enable(&self) {
        // allocate the main task
        task::allocate_main_task();
        timer::setup_multitasking();

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

    fn task_create(
        &self,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
    ) -> *mut c_void {
        let task = Box::new_in(Context::new(task, param, task_stack_size), InternalMemory);
        let task_ptr = Box::into_raw(task);

        SCHEDULER_STATE.with(|state| unsafe {
            let current_task = state.current_task;
            debug_assert!(
                !current_task.is_null(),
                "Tried to allocate a task before allocating the main task"
            );
            // Insert the new task at the next position.
            let next = (*current_task).next;
            (*task_ptr).next = next;
            (*current_task).next = task_ptr;
        });

        task_ptr as *mut c_void
    }

    fn current_task(&self) -> *mut c_void {
        task::current_task() as *mut c_void
    }

    fn schedule_task_deletion(&self, task_handle: *mut c_void) {
        task::schedule_task_deletion(task_handle as *mut Context)
    }

    fn current_task_thread_semaphore(&self) -> *mut c_void {
        unsafe { &mut ((*task::current_task()).thread_semaphore) as *mut _ as *mut c_void }
    }
}
