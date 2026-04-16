use critical_section::RawRestoreState;

use crate::interrupt;

struct CriticalSection;
critical_section::set_impl!(CriticalSection);

unsafe impl critical_section::Impl for CriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        interrupt::disable_cpu_interrupts()
    }

    unsafe fn release(previous_state: RawRestoreState) {
        // Only re-enable interrupts if they were enabled before the critical section.
        interrupt::mask_cpu_interrupts(previous_state);
    }
}
