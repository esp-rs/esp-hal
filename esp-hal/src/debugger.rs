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
            .core_0_debug_mode()
            .read()
            .core_0_debug_module_active()
            .bit_is_set()
    }

    #[cfg(not(any(xtensa, riscv)))]
    {
        false
    }
}
