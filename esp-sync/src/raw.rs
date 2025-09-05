//! TODO

use core::sync::atomic::{Ordering, compiler_fence};

use crate::RestoreState;

/// Trait for single-core locks.
pub trait RawLock {
    /// Acquires the raw lock
    ///
    /// # Safety
    ///
    /// The returned tokens must be released in reverse order, on the same thread that they were
    /// created on.
    unsafe fn enter(&self) -> RestoreState;

    /// Releases the raw lock
    ///
    /// # Safety
    ///
    /// - The `token` must be created by `self.enter()`
    /// - Tokens must be released in reverse order to their creation, on the same thread that they
    ///   were created on.
    unsafe fn exit(&self, token: RestoreState);
}

/// A lock that disables interrupts.
pub struct SingleCoreInterruptLock;

impl RawLock for SingleCoreInterruptLock {
    unsafe fn enter(&self) -> RestoreState {
        cfg_if::cfg_if! {
            if #[cfg(riscv)] {
                let mut mstatus = 0u32;
                unsafe { core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus); }
                let token = mstatus & 0b1000;
            } else if #[cfg(xtensa)] {
                let token: u32;
                unsafe { core::arch::asm!("rsil {0}, 5", out(reg) token); }
            } else {
                compile_error!("Unsupported architecture")
            }
        };

        // Ensure no subsequent memory accesses are reordered to before interrupts are
        // disabled.
        compiler_fence(Ordering::SeqCst);

        unsafe { RestoreState::new(token) }
    }

    unsafe fn exit(&self, token: RestoreState) {
        // Ensure no preceeding memory accesses are reordered to after interrupts are
        // enabled.
        compiler_fence(Ordering::SeqCst);

        let token = token.inner();

        cfg_if::cfg_if! {
            if #[cfg(riscv)] {
                if token != 0 {
                    unsafe {
                        riscv::interrupt::enable();
                    }
                }
            } else if #[cfg(xtensa)] {
                // Reserved bits in the PS register, these must be written as 0.
                const RESERVED_MASK: u32 = 0b1111_1111_1111_1000_1111_0000_0000_0000;
                debug_assert!(token & RESERVED_MASK == 0);
                unsafe {
                    core::arch::asm!(
                        "wsr.ps {0}",
                        "rsync", in(reg) token)
                }
            } else {
                compile_error!("Unsupported architecture")
            }
        }
    }
}
