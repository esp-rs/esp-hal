use crate::pac::rsa::RegisterBlock;

pub(super) fn enable_disable_interrupt(regs: &RegisterBlock, enable: bool) {
    // Can't seem to actually disable the interrupt, but esp-idf still writes the register.
    regs.interrupt().write(|w| w.interrupt().bit(enable));
}

pub(super) fn ready(regs: &RegisterBlock) -> bool {
    regs.clean().read().clean().bit_is_set()
}

pub(super) fn start_modexp(regs: &RegisterBlock) {
    regs.modexp_start().write(|w| w.modexp_start().set_bit());
}

pub(super) fn start_multi(regs: &RegisterBlock) {
    regs.mult_start().write(|w| w.mult_start().set_bit());
}

pub(super) fn start_modmulti(regs: &RegisterBlock) {
    // Modular-ness is encoded in the multi_mode register value.
    start_multi(regs);
}

pub(super) fn clear_interrupt(regs: &RegisterBlock) {
    regs.interrupt().write(|w| w.interrupt().set_bit());
}

pub(super) fn is_idle(regs: &RegisterBlock) -> bool {
    regs.interrupt().read().interrupt().bit_is_set()
}

pub(super) fn write_multi_mode(regs: &RegisterBlock, mode: u32, modular: bool) {
    let mode = if !modular {
        const NON_MODULAR: u32 = 8;
        mode | NON_MODULAR
    } else {
        mode
    };

    regs.mult_mode().write(|w| unsafe { w.bits(mode) });
}

pub(super) fn write_modexp_mode(regs: &RegisterBlock, mode: u32) {
    regs.modexp_mode().write(|w| unsafe { w.bits(mode) });
}
