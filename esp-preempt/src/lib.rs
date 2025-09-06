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
mod scheduler;
mod semaphore;
mod task;
mod timer;
mod timer_queue;

use esp_hal::{
    Blocking,
    timer::{AnyTimer, PeriodicTimer},
};
pub(crate) use scheduler::SCHEDULER;

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
