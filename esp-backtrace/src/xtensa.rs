use core::arch::asm;

#[cfg(feature = "panic-handler")]
pub(crate) use xtensa_lx::interrupt::free as interrupt_free;

use crate::{Backtrace, BacktraceFrame};

// subtract 3 from the return address
// the return address is the address following the callxN
// we get better results (especially if the caller was the last function in the
// calling function) if we report the address of callxN itself
pub(super) const RA_OFFSET: usize = 3;

/// This function returns the caller's frame pointer.
#[inline(never)]
#[cold]
fn sp() -> u32 {
    let mut sp: u32;
    unsafe {
        asm!(
            "mov {0}, a1", // current stack pointer
            // Spill registers, otherwise `sp - 12` will not contain the previous stack pointer
            "add a12,a12,a12",
            "rotw 3",
            "add a12,a12,a12",
            "rotw 3",
            "add a12,a12,a12",
            "rotw 3",
            "add a12,a12,a12",
            "rotw 3",
            "add a12,a12,a12",
            "rotw 4",
            out(reg) sp
        );
    }

    // current frame pointer, caller's stack pointer
    unsafe { ((sp - 12) as *const u32).read_volatile() }
}

/// Get an array of backtrace addresses.
#[inline(never)]
#[cold]
pub fn backtrace() -> Backtrace {
    let sp = sp();

    backtrace_internal(sp, 0)
}

pub(crate) fn remove_window_increment(address: u32) -> u32 {
    (address & 0x3fff_ffff) | 0x4000_0000
}

pub(crate) fn backtrace_internal(sp: u32, suppress: u32) -> Backtrace {
    let mut result = Backtrace(heapless::Vec::new());

    let mut fp = sp;
    let mut suppress = suppress;

    if !crate::is_valid_ram_address(fp) {
        return result;
    }

    while !result.0.is_full() {
        // RA/PC
        let address = unsafe { (fp as *const u32).offset(-4).read_volatile() };
        let address = remove_window_increment(address);
        // next FP
        fp = unsafe { (fp as *const u32).offset(-3).read_volatile() };

        // the return address is 0 but we sanitized the address - then 0 becomes
        // 0x40000000
        if address == 0x40000000 {
            break;
        }

        if !crate::is_valid_ram_address(fp) {
            break;
        }

        if suppress == 0 {
            _ = result.0.push(BacktraceFrame {
                pc: address as usize,
            });
        } else {
            suppress -= 1;
        }
    }

    result
}
