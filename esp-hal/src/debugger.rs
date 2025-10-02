//! Debugger utilities

/// Checks if a debugger is connected.
pub fn debugger_connected() -> bool {
    #[cfg(xtensa)]
    {
        xtensa_lx::is_debugger_attached()
    }

    #[cfg(riscv)]
    {
        crate::peripherals::ASSIST_DEBUG::regs()
            .cpu(0)
            .debug_mode()
            .read()
            .debug_module_active()
            .bit_is_set()
    }

    #[cfg(not(any(xtensa, riscv)))]
    {
        false
    }
}

/// Set a word-sized data breakpoint at the given address.
/// No breakpoint will be set if a debugger is currently attached.
///
/// Breakpoint 0 is used.
///
/// # Safety
/// The address must be word aligned.
pub unsafe fn set_stack_watchpoint(addr: usize) {
    assert!(addr.is_multiple_of(4));

    if !crate::debugger::debugger_connected() {
        cfg_if::cfg_if! {
            if #[cfg(xtensa)] {
                let addr = addr & !0b11;
                let dbreakc = 0b1111100 | (1 << 31); // bit 31 = STORE

                unsafe {
                    core::arch::asm!(
                        "
                        wsr {addr}, 144 // 144 = dbreaka0
                        wsr {dbreakc}, 160 // 160 = dbreakc0
                        ",
                        addr = in(reg) addr,
                        dbreakc = in(reg) dbreakc,
                    );
                }
            } else {
                let addr = (addr & !0b11) | 0b01;

                let id = 0; // breakpoint 0
                let tdata = (1 << 3) | (1 << 6) | (1 << 1); // bits: 0 = load, 1 = store, 6 = m-mode, 3 = u-mode
                let tcontrol = 1 << 3; // M-mode trigger

                unsafe {
                    core::arch::asm!(
                        "
                        csrw 0x7a0, {id} // tselect
                        csrrs {tcontrol}, 0x7a5, {tcontrol} // tcontrol
                        csrrs {tdata}, 0x7a1, {tdata} // tdata1
                        csrw 0x7a2, {addr} // tdata2
                        ", id = in(reg) id,
                        addr = in(reg) addr,
                        tdata = in(reg) tdata,
                        tcontrol = in(reg) tcontrol,
                    );
                }
            }
        }
    }
}
