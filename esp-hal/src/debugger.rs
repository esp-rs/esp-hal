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
/// No breakpoint will be set when a debugger is currently attached if
/// the `stack_guard_monitoring_with_debugger_connected` option is false.
///
/// Breakpoint 0 is used.
///
/// # Safety
/// The address must be word aligned.
pub unsafe fn set_stack_watchpoint(addr: usize) {
    assert!(addr.is_multiple_of(4));

    if cfg!(stack_guard_monitoring_with_debugger_connected)
        || !crate::debugger::debugger_connected()
    {
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
                unsafe { set_watchpoint(0, addr, 4); }
            }
        }
    }
}

#[cfg(riscv)]
const NAPOT_MATCH: u8 = 1;

#[cfg(riscv)]
bitfield::bitfield! {
    /// Only match type (0x2) triggers are supported.
    #[derive(Clone, Copy, Default)]
    pub(crate) struct Tdata1(u32);

    /// Set this for configuring the selected trigger to fire right before a load operation with matching
    /// data address is executed by the CPU.
    pub bool, load, set_load: 0;

    /// Set this for configuring the selected trigger to fire right before a store operation with matching
    /// data address is executed by the CPU.
    pub bool, store, set_store: 1;

    /// Set this for configuring the selected trigger to fire right before an instruction with matching
    /// virtual address is executed by the CPU.
    pub bool, execute, set_execute: 2;

    /// Set this for enabling selected trigger to operate in user mode.
    pub bool, u, set_u: 3;

    /// Set this for enabling selected trigger to operate in machine mode.
    pub bool, m, set_m: 6;

    /// Configures the selected trigger to perform one of the available matching operations on a
    /// data/instruction address. Valid options are:
    /// 0x0: exact byte match, i.e. address corresponding to one of the bytes in an access must match
    /// the value of maddress exactly.
    /// 0x1: NAPOT match, i.e. at least one of the bytes of an access must lie in the NAPOT region
    /// specified in maddress.
    /// Note: Writing a larger value will clip it to the largest possible value 0x1.
    pub u8, _match, set_match: 10, 7;

    /// Configures the selected trigger to perform one of the available actions when firing. Valid
    /// options are:
    /// 0x0: cause breakpoint exception.
    /// 0x1: enter debug mode (only valid when dmode = 1)
    /// Note: Writing an invalid value will set this to the default value 0x0.
    pub u8, action, set_action: 15, 12;

    /// This is found to be 1 if the selected trigger had fired previously. This bit is to be cleared manually.
    pub bool, hit, set_hit: 20;

    /// 0: Both Debug and M mode can write the tdata1 and tdata2 registers at the selected tselect.
    /// 1: Only Debug Mode can write the tdata1 and tdata2 registers at the selected tselect. Writes from
    /// other modes are ignored.
    /// Note: Only writable from debug mode.
    pub bool, dmode, set_dmode: 27;
}

#[cfg(riscv)]
bitfield::bitfield! {
    /// Only match type (0x2) triggers are supported.
    #[derive(Clone, Copy, Default)]
    pub(crate) struct Tcontrol(u32);

    /// Current M mode trigger enable bit
    pub bool, mte, set_mte: 3;

    /// Previous M mode trigger enable bit
    pub bool, mpte, set_mpte: 7;

}

/// Clear the watchpoint
#[cfg(riscv)]
pub(crate) unsafe fn clear_watchpoint(id: u8) {
    assert!(id < 4);

    // tdata1 is a WARL(write any read legal) register. We can just write 0 to it.
    let tdata = 0;

    unsafe {
        core::arch::asm!(
            "
            csrw 0x7a0, {id} // tselect
            csrw 0x7a1, {tdata} // tdata1
            ", id = in(reg) id,
            tdata = in(reg) tdata,
        );
    }
}

/// Clear the watchpoint
#[cfg(all(riscv, feature = "exception-handler"))]
pub(crate) unsafe fn watchpoint_hit(id: u8) -> bool {
    assert!(id < 4);
    let mut tdata = Tdata1::default();

    unsafe {
        core::arch::asm!(
            "
            csrw 0x7a0, {id} // tselect
            csrr {tdata}, 0x7a1 // tdata1
            ", id = in(reg) id,
            tdata = out(reg) tdata.0,
        );
    }

    tdata.hit()
}

/// Set watchpoint and enable triggers.
#[cfg(riscv)]
pub(crate) unsafe fn set_watchpoint(id: u8, addr: usize, len: usize) {
    assert!(id < 4);
    assert!(len.is_power_of_two());
    assert!(addr.is_multiple_of(len));

    let z = len.trailing_zeros();
    let mask = {
        let mut mask: usize = 0;
        for i in 0..z {
            mask |= 1 << i;
        }
        mask
    };

    let napot_encoding = { mask & !(1 << (z - 1)) };
    let addr = (addr & !mask) | napot_encoding;

    let mut tdata = Tdata1::default();
    tdata.set_m(true);
    tdata.set_store(true);
    tdata.set_match(NAPOT_MATCH);
    let tdata: u32 = tdata.0;

    let mut tcontrol = Tcontrol::default();
    tcontrol.set_mte(true);
    let tcontrol: u32 = tcontrol.0;

    unsafe {
        core::arch::asm!(
            "
            csrw 0x7a0, {id} // tselect
            csrw 0x7a5, {tcontrol} // tcontrol
            csrw 0x7a1, {tdata} // tdata1
            csrw 0x7a2, {addr} // tdata2
            ", id = in(reg) id,
            addr = in(reg) addr,
            tdata = in(reg) tdata,
            tcontrol = in(reg) tcontrol,
        );
    }
}
