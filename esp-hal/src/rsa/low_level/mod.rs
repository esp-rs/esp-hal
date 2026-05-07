use crate::pac::rsa::RegisterBlock;

#[cfg_attr(rsa_version = "1", path = "v1.rs")]
#[cfg_attr(rsa_version = "2", path = "v2.rs")]
#[cfg_attr(rsa_version = "3", path = "v3.rs")]
mod version;

pub(super) fn enable_disable_interrupt(regs: &RegisterBlock, enable: bool) {
    version::enable_disable_interrupt(regs, enable);
}

pub(super) fn ready(regs: &RegisterBlock) -> bool {
    version::ready(regs)
}

pub(super) fn start_modexp(regs: &RegisterBlock) {
    version::start_modexp(regs);
}

pub(super) fn start_multi(regs: &RegisterBlock) {
    version::start_multi(regs);
}

pub(super) fn start_modmulti(regs: &RegisterBlock) {
    version::start_modmulti(regs);
}

pub(super) fn clear_interrupt(regs: &RegisterBlock) {
    version::clear_interrupt(regs);
}

pub(super) fn is_idle(regs: &RegisterBlock) -> bool {
    version::is_idle(regs)
}

pub(super) fn write_multi_mode(regs: &RegisterBlock, mode: u32, modular: bool) {
    version::write_multi_mode(regs, mode, modular);
}

pub(super) fn write_modexp_mode(regs: &RegisterBlock, mode: u32) {
    version::write_modexp_mode(regs, mode);
}
