use crate::trapframe::TrapFrame;

#[cfg(xtensa)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".rwtext")]
unsafe extern "C" fn __user_exception(
    cause: xtensa_lx_rt::exception::ExceptionCause,
    context: &TrapFrame,
) {
    panic!("\n\nException occurred '{:?}'\n{:?}", cause, context);
}

#[cfg(riscv)]
#[unsafe(no_mangle)]
unsafe extern "C" fn ExceptionHandler(context: &TrapFrame) -> ! {
    let mepc = riscv::register::mepc::read();
    let code = riscv::register::mcause::read().code();
    let mtval = riscv::register::mtval::read();

    if code == 14 {
        panic!(
            "Stack overflow detected at 0x{:x} called by 0x{:x}",
            mepc, context.ra
        );
    } else {
        let code = match code {
            0 => "Instruction address misaligned",
            1 => "Instruction access fault",
            2 => "Illegal instruction",
            3 => "Breakpoint",
            4 => "Load address misaligned",
            5 => "Load access fault",
            6 => "Store/AMO address misaligned",
            7 => "Store/AMO access fault",
            8 => "Environment call from U-mode",
            9 => "Environment call from S-mode",
            10 => "Reserved",
            11 => "Environment call from M-mode",
            12 => "Instruction page fault",
            13 => "Load page fault",
            14 => "Reserved",
            15 => "Store/AMO page fault",
            _ => "UNKNOWN",
        };

        panic!(
            "Exception '{}' mepc={:08x}, mtval={:08x}\n{:?}",
            code, mepc, mtval, context
        );
    }
}
