#[cfg_attr(ledc_version = "1", path = "v1.rs")]
#[cfg_attr(ledc_version = "2", path = "v2.rs")]
#[cfg_attr(ledc_version = "3", path = "v3.rs")]
mod version;

#[cfg(ledc_version = "1")]
use super::timer::HSClockSource;
use super::{
    LSGlobalClkSource,
    channel::Number as ChannelNumber,
    timer::{LSClockSource, Number as TimerNumber},
};
use crate::{gpio::OutputSignal, pac::ledc::RegisterBlock, time::Rate};

#[inline(always)]
pub(super) fn set_global_slow_clock(ledc: &RegisterBlock, clock_source: LSGlobalClkSource) {
    version::set_global_slow_clock(ledc, clock_source)
}

#[inline(always)]
pub(super) fn ls_freq_hw(clock_source: LSClockSource) -> Rate {
    version::ls_freq_hw(clock_source)
}

#[inline(always)]
pub(super) fn ls_configure_hw(
    ledc: &RegisterBlock,
    number: TimerNumber,
    divisor: u32,
    duty: u8,
    use_ref_tick: bool,
) {
    version::ls_configure_hw(ledc, number, divisor, duty, use_ref_tick)
}

#[inline(always)]
pub(super) fn ls_update_hw(ledc: &RegisterBlock, number: TimerNumber) {
    version::ls_update_hw(ledc, number)
}

#[cfg(ledc_version = "1")]
#[inline(always)]
pub(super) fn hs_freq_hw(clock_source: HSClockSource) -> Rate {
    version::hs_freq_hw(clock_source)
}

#[cfg(ledc_version = "1")]
#[inline(always)]
pub(super) fn hs_configure_hw(
    ledc: &RegisterBlock,
    number: TimerNumber,
    divisor: u32,
    duty: u8,
    clock_source: HSClockSource,
) {
    version::hs_configure_hw(ledc, number, divisor, duty, clock_source)
}

#[cfg(ledc_version = "1")]
#[inline(always)]
pub(super) fn hs_update_hw() {
    version::hs_update_hw()
}

#[inline(always)]
pub(super) fn output_signal(ch_num: ChannelNumber, is_hs: bool) -> OutputSignal {
    version::output_signal(ch_num, is_hs)
}

#[inline(always)]
pub(super) fn set_channel(
    ledc: &RegisterBlock,
    ch_num: ChannelNumber,
    timer_number: u8,
    is_hs: bool,
) {
    version::set_channel(ledc, ch_num, timer_number, is_hs)
}

#[inline(always)]
pub(super) fn start_duty_without_fading(ledc: &RegisterBlock, ch_num: ChannelNumber, is_hs: bool) {
    version::start_duty_without_fading(ledc, ch_num, is_hs)
}

#[inline(always)]
pub(super) fn update_channel(ledc: &RegisterBlock, ch_num: ChannelNumber, is_hs: bool) {
    version::update_channel(ledc, ch_num, is_hs)
}

#[inline(always)]
pub(super) fn set_duty_hw(ledc: &RegisterBlock, ch_num: ChannelNumber, is_hs: bool, duty: u32) {
    version::set_duty_hw(ledc, ch_num, is_hs, duty)
}

#[allow(clippy::too_many_arguments)]
#[inline(always)]
pub(super) fn start_duty_fade_hw(
    ledc: &RegisterBlock,
    ch_num: ChannelNumber,
    is_hs: bool,
    start_duty: u32,
    duty_inc: bool,
    duty_steps: u16,
    cycles_per_step: u16,
    duty_per_cycle: u16,
) {
    version::start_duty_fade_hw(
        ledc,
        ch_num,
        is_hs,
        start_duty,
        duty_inc,
        duty_steps,
        cycles_per_step,
        duty_per_cycle,
    )
}

#[inline(always)]
pub(super) fn is_duty_fade_running_hw(
    ledc: &RegisterBlock,
    ch_num: ChannelNumber,
    is_hs: bool,
) -> bool {
    version::is_duty_fade_running_hw(ledc, ch_num, is_hs)
}
