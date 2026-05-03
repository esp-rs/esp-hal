//! Interrupt handling for ESP32-C6 LP core
use riscv::{CoreInterruptNumber, InterruptNumber};

use super::Interrupt;

/// Setup interrupt handlers, including any default ones
#[inline(always)]
pub fn setup_interrupts() {
    machine_interrupt_enable(true);
}

/// Enables or disables a peripheral interrupt.
///
/// Note that interrupts still need to be enabled globally for interrupts
/// to be serviced.
///
/// Internally, this function maps the interrupt to the appropriate CPU interrupt.
#[inline]
pub fn set_enabled(_interrupt: Interrupt, _enable: bool) {
    // todo!()
}

/// Clears a peripheral interrupt.
pub fn clear(_interrupt: Interrupt) {
    todo!()
}

/// Returns a bitmask of active peripheral interrupts
pub fn status() -> u32 {
    todo!()
}

/// Returns the cause of a machine interrupt,
/// which contains the exception code, and if a peripheral interrupt was flagged.
#[unsafe(link_section = ".trap.rust")]
#[inline(always)]
pub fn trap_cause() -> riscv::interrupt::Trap<usize, usize> {
    let mcause_reg = riscv::register::mcause::read();
    mcause_reg.cause()
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(usize)]
enum LpMachineInterrupt {
    MachineSoft = 3,  // Global enable/disable bit in mstatus
    Peripheral  = 30, // Custom interrupt for peripherals
}

unsafe impl InterruptNumber for LpMachineInterrupt {
    const MAX_INTERRUPT_NUMBER: usize = 31_usize;

    #[inline]
    fn number(self) -> usize {
        self as usize
    }

    #[inline]
    fn from_number(value: usize) -> riscv::result::Result<Self> {
        match value {
            3 => Ok(Self::MachineSoft),
            30 => Ok(Self::Peripheral),
            _ => Err(riscv::result::Error::InvalidVariant(value)),
        }
    }
}

unsafe impl CoreInterruptNumber for LpMachineInterrupt {}

/// Set the mie register (or equivalent),
/// returning the previous value.
#[inline(always)]
pub fn machine_interrupt_enable(enable: bool) -> bool {
    // ESP32C6-LP must enable both the
    // global machine soft interrupt, (mie bit of the mstatus register),
    // and also the external peripheral interrupt (bit 30 in mie register).

    let mut mstatus_reg = riscv::register::mstatus::read();
    let mut mie_reg = riscv::register::mie::read();

    // The original value of the global interrupt enable bit, will be returned.
    let old_mie_enable = mstatus_reg.mie();

    // Update register values
    if enable {
        mstatus_reg.set_mie(true);
        mie_reg.enable(LpMachineInterrupt::Peripheral);
    } else {
        mstatus_reg.set_mie(false);
        mie_reg.disable(LpMachineInterrupt::Peripheral);
    }

    // Write them back to memory
    unsafe {
        riscv::register::mstatus::write(mstatus_reg);
        riscv::register::mie::write(mie_reg);
    }

    old_mie_enable
}
