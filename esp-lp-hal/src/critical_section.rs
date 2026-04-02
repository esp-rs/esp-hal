use critical_section::RawRestoreState;

use crate::interrupt;

struct UlpCriticalSection;
critical_section::set_impl!(UlpCriticalSection);

unsafe impl critical_section::Impl for UlpCriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        interrupt::disable()
    }

    unsafe fn release(previous_state: RawRestoreState) {
        // Only re-enable interrupts if that were enabled before the critical section.
        // interrupt::enable();
        interrupt::maskirq(previous_state);
    }
}
