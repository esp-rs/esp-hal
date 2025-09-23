//! This crate allows using esp-radio on top of esp-hal, without any other OS.
//!
//! This crate requires an esp-hal timer to operate.
//!
//! ```rust, no_run
#![doc = esp_hal::before_snippet!()]
//! use esp_hal::timer::timg::TimerGroup;
//!
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! esp_preempt::start(timg0.timer0);
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

mod queue;
mod run_queue;
mod scheduler;
mod semaphore;
mod task;
mod timer;
mod timer_queue;
mod wait_queue;

use core::mem::MaybeUninit;

pub(crate) use esp_alloc::InternalMemory;
use esp_hal::{
    Blocking,
    timer::{AnyTimer, OneShotTimer},
};
pub(crate) use scheduler::SCHEDULER;

use crate::timer::TimeDriver;

type TimeBase = OneShotTimer<'static, Blocking>;

// Polyfill the InternalMemory allocator
#[cfg(not(feature = "esp-alloc"))]
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

/// Starts the scheduler.
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
#[cfg_attr(
    multi_core,
    doc = " \nNote that `esp_radio::init()` must be called on the same core as `esp_preempt::start()`."
)]
pub fn start(timer: impl TimerSource) {
    SCHEDULER.with(move |scheduler| {
        scheduler.setup(TimeDriver::new(timer.timer()));

        // Allocate the default task. The stack bottom is set to the stack guard's address as the
        // rest of the stack is unusable anyway.

        unsafe extern "C" {
            static _stack_start_cpu0: u32;
            static __stack_chk_guard: u32;
        }
        let stack_top = &raw const _stack_start_cpu0;
        let stack_bottom = (&raw const __stack_chk_guard).cast::<MaybeUninit<u32>>();
        let stack_slice = core::ptr::slice_from_raw_parts_mut(
            stack_bottom.cast_mut(),
            stack_top as usize - stack_bottom as usize,
        );
        task::allocate_main_task(scheduler, stack_slice);

        task::setup_multitasking();

        unwrap!(scheduler.time_driver.as_mut()).start();
        task::yield_task();
    })
}

const TICK_RATE: u32 = esp_config::esp_config_int!(u32, "ESP_PREEMPT_CONFIG_TICK_RATE_HZ");
