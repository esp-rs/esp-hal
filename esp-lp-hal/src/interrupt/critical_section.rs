use critical_section::RawRestoreState;

#[cfg(feature = "interrupts")]
use crate::interrupt;

struct CriticalSection;
critical_section::set_impl!(CriticalSection);

/// Machine-mode critical section implementation for
/// LP and ULP cores.
unsafe impl critical_section::Impl for CriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        #[cfg(feature = "interrupts")]
        {
            interrupt::machine_interrupt_enable(false)
        }
        #[cfg(not(feature = "interrupts"))]
        {
            true
        }
    }

    unsafe fn release(_previous_state: RawRestoreState) {
        #[cfg(feature = "interrupts")]
        interrupt::machine_interrupt_enable(_previous_state);
    }
}
