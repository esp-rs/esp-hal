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

// Reserved bits in the PS register, these must be written as 0.
#[cfg(all(xtensa, debug_assertions))]
const RESERVED_MASK: u32 = 0b1111_1111_1111_1000_1111_0000_0000_0000;

impl RawLock for SingleCoreInterruptLock {
    #[inline]
    unsafe fn enter(&self) -> RestoreState {
        cfg_if::cfg_if! {
            if #[cfg(esp32p4)] { // TODO: any with zcmp
                // ESP32-P4 (v3.2/ECO7 etc.) Zcmp hardware bug workaround (IDF-14279 / DIG-661):
                // Clearing mstatus.mie alone does not fully mask CLIC interrupts -- an
                // interrupt can still fire mid-instruction on cm.push (and possibly on
                // other multi-cycle sequences). Fix: raise mintthresh (CSR 0x347) to 0xFF
                // while mie is cleared, then restore the previous mintthresh on exit.
                // Ref: esp-idf commit c27c33a83 "fix(riscv): implement a workaround for
                // Zcmp hardware bug".
                let old_mintthresh: u32;
                unsafe {
                    core::arch::asm!(
                        "li   t0, 0xff",
                        "csrrw {0}, 0x347, t0",
                        out(reg) old_mintthresh,
                        out("t0") _,
                    );
                }
                let mut mstatus = 0u32;
                unsafe { core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus); }
                let mie_bit = mstatus & 0b1000;
                let token = mie_bit | ((old_mintthresh & 0xff) << 8);
            } else if #[cfg(riscv)] {
                let mut mstatus = 0u32;
                unsafe { core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus); }
                let token = mstatus & 0b1000;
            } else if #[cfg(xtensa)] {
                let token: u32;
                unsafe { core::arch::asm!("rsil {0}, 5", out(reg) token); }
                #[cfg(debug_assertions)]
                let token = token & !RESERVED_MASK;
            } else {
                compile_error!("Unsupported architecture")
            }
        };

        // Ensure no subsequent memory accesses are reordered to before interrupts are
        // disabled.
        compiler_fence(Ordering::SeqCst);

        unsafe { RestoreState::new(token) }
    }

    #[inline]
    unsafe fn exit(&self, token: RestoreState) {
        // Ensure no preceeding memory accesses are reordered to after interrupts are
        // enabled.
        compiler_fence(Ordering::SeqCst);

        let token = token.inner();

        cfg_if::cfg_if! {
            if #[cfg(esp32p4)] {
                if (token & 0b1000) != 0 {
                    unsafe {
                        riscv::interrupt::enable();
                    }
                }
                // Restore mintthresh AFTER re-enabling mie (P4 Zcmp workaround, see enter()).
                let old_mintthresh = (token >> 8) & 0xff;
                unsafe {
                    core::arch::asm!(
                        "csrw 0x347, {0}",
                        in(reg) old_mintthresh,
                    );
                }

                // The delay between the moment we unmask the interrupt threshold register
                // and the moment the potential requested interrupt is triggered is not
                // null: up to three machine cycles/instructions can be executed.
                riscv::asm::nop();
                riscv::asm::nop();
                riscv::asm::nop();
            } else if #[cfg(riscv)] {
                if token != 0 {
                    unsafe {
                        riscv::interrupt::enable();
                    }
                }
            } else if #[cfg(xtensa)] {
                #[cfg(debug_assertions)]
                if token & RESERVED_MASK != 0 {
                    // We could do this transformation in fmt.rs automatically, but experiments
                    // show this is only worth it in terms of binary size for code inlined into many places.
                    #[cold]
                    #[inline(never)]
                    fn __assert_failed() {
                        panic!("Reserved bits in PS register must be written as 0");
                    }

                    __assert_failed();
                }

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
