use crate::pac::rsa::RegisterBlock;

pub(super) fn enable_disable_interrupt(regs: &RegisterBlock, enable: bool) {
    regs.int_ena().write(|w| w.int_ena().bit(enable));
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
    regs.modmult_start().write(|w| w.modmult_start().set_bit());
}

pub(super) fn clear_interrupt(regs: &RegisterBlock) {
    regs.int_clr().write(|w| w.int_clr().set_bit());
}

pub(super) fn is_idle(regs: &RegisterBlock) -> bool {
    regs.idle().read().idle().bit_is_set()
}

pub(super) fn write_multi_mode(regs: &RegisterBlock, mode: u32, _modular: bool) {
    regs.mode().write(|w| unsafe { w.bits(mode) });
}

pub(super) fn write_modexp_mode(regs: &RegisterBlock, mode: u32) {
    regs.mode().write(|w| unsafe { w.bits(mode) });
}
