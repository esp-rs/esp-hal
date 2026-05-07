use critical_section::RawRestoreState;

use crate::interrupt;

struct CriticalSection;
critical_section::set_impl!(CriticalSection);

unsafe impl critical_section::Impl for CriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        interrupt::machine_interrupt_enable(false)
    }

    unsafe fn release(previous_state: RawRestoreState) {
        interrupt::machine_interrupt_enable(previous_state);
    }
}
