#[inline(always)]
pub(super) fn initialize() {}

#[inline(always)]
pub(super) fn set_output_enabled(_mask: u32, _en: bool) {
    // Nothing to do.
}

#[inline(always)]
pub(super) fn read_in() -> u32 {
    let val;
    unsafe { core::arch::asm!("ee.get_gpio_in {0}", out(reg) val) };
    val
}

#[inline(always)]
pub(super) fn read_out() -> u32 {
    let val;
    unsafe { core::arch::asm!("rur.gpio_out {0}", out(reg) val) };
    val
}

#[inline(always)]
pub(super) fn write(mask: u32, value: u32) {
    unsafe { core::arch::asm!("ee.wr_mask_gpio_out {0}, {1}", in(reg) mask, in(reg) value) }
}
