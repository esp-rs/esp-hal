use critical_section::RawRestoreState;

use crate::interrupt;

struct UlpCriticalSection;
critical_section::set_impl!(UlpCriticalSection);

unsafe impl critical_section::Impl for UlpCriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        interrupt::disable_cpu_interrupts()
    }

    unsafe fn release(previous_state: RawRestoreState) {
        // Only re-enable interrupts if they were enabled before the critical section.
        interrupt::mask_cpu_interrupts(previous_state);
    }
}
