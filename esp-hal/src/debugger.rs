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
