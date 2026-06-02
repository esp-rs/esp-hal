#[inline(always)]
pub(super) fn initialize() {
    // Allow instruction access, which has better performance than register access.
    let regs = unsafe { esp32s2::DEDICATED_GPIO::steal() };
    regs.out_cpu()
        .write(|w| unsafe { w.bits((1 << property!("dedicated_gpio.channel_count")) - 1) });
}

#[inline(always)]
pub(super) fn set_output_enabled(_mask: u32, _en: bool) {
    // Nothing to do.
}

#[inline(always)]
pub(super) fn read_in() -> u32 {
    let val;
    unsafe { core::arch::asm!("get_gpio_in {0}", out(reg) val) };
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
    unsafe { core::arch::asm!("wr_mask_gpio_out {0}, {1}", in(reg) mask, in(reg) value) }
}
