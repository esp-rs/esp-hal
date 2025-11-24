use core::arch::asm;

#[cfg(feature = "panic-handler")]
pub(crate) use riscv::interrupt::free as interrupt_free;

use crate::{Backtrace, BacktraceFrame};

// subtract 4 from the return address
// the return address is the address following the JALR
// we get better results (especially if the caller was the last instruction in
// the calling function) if we report the address of the JALR itself
// even if it was a C.JALR we should get good results using RA - 4
pub(super) const RA_OFFSET: usize = 4;

/// Get an array of backtrace addresses.
///
/// This needs `force-frame-pointers` enabled.
#[inline(never)]
#[cold]
pub fn backtrace() -> Backtrace {
    let fp = unsafe {
        let mut _tmp: u32;
        asm!("mv {0}, x8", out(reg) _tmp);
        _tmp
    };

    backtrace_internal(fp, 2)
}

pub(crate) fn backtrace_internal(fp: u32, suppress: u32) -> Backtrace {
    let mut result = Backtrace(heapless::Vec::new());

    let mut fp = fp;
    let mut suppress = suppress;

    if !crate::is_valid_ram_address(fp) {
        return result;
    }

    while !result.0.is_full() {
        // RA/PC
        let address = unsafe { (fp as *const u32).offset(-1).read_volatile() };
        // next FP
        fp = unsafe { (fp as *const u32).offset(-2).read_volatile() };

        if address == 0 {
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

#[cfg(all(stack_dump, feature = "panic-handler"))]
pub(super) fn dump_stack() {
    cfg_if::cfg_if! {
        if #[cfg(feature = "println")] {
            const MAX_STACK_DUMP_SIZE: u32 = match () {
                _ if cfg!(stack_dump_max_size_4k) => 4 * 1024,
                _ if cfg!(stack_dump_max_size_8k) => 8 * 1024,
                _ if cfg!(stack_dump_max_size_16k) => 16 * 1024,
                _ if cfg!(stack_dump_max_size_32k) => 32 * 1024,
                _ => core::unreachable!(),
            };

            let (pc, sp) = unsafe {
                unsafe extern "C" {
                    #[allow(improper_ctypes)]
                    fn __get_pc_sp() -> (u32, u32);
                }

                __get_pc_sp()
            };

            unsafe extern "C" {
                static _stack_start: u32;
            }

            let end = u32::min(
                core::ptr::addr_of!(_stack_start) as u32,
                sp + MAX_STACK_DUMP_SIZE,
            );

            esp_println::print!("STACKDUMP: {:08x} ", pc);
            for address in sp..end {
                let byte = unsafe { (address as *const u8).read_volatile() };
                esp_println::print!("{:02x}", byte);
            }
            esp_println::println!();
        } else {
            super::println!("Stack dumps are not supported with DEFMT.");
        }
    }
}

#[cfg(all(stack_dump, feature = "panic-handler"))]
core::arch::global_asm!(
    "
.globl __get_pc_sp
__get_pc_sp:
    mv a0, ra
    mv a1, sp
    ret
"
);
