use super::super::{
    LSGlobalClkSource,
    channel::Number as ChannelNumber,
    timer::{LSClockSource, Number as TimerNumber},
};
use crate::{gpio::OutputSignal, pac::ledc::RegisterBlock, soc::clocks, time::Rate};

pub(super) fn set_global_slow_clock(ledc: &RegisterBlock, clock_source: LSGlobalClkSource) {
    match clock_source {
        LSGlobalClkSource::APBClk => {
            ledc.conf().write(|w| unsafe { w.apb_clk_sel().bits(1) });
        }
    }
    ledc.timer(0).conf().modify(|_, w| w.para_up().set_bit());
}

pub(super) fn ls_freq_hw(_clock_source: LSClockSource) -> Rate {
    Rate::from_hz(clocks::apb_clk_frequency())
}

pub(super) fn ls_configure_hw(
    ledc: &RegisterBlock,
    number: TimerNumber,
    divisor: u32,
    duty: u8,
    use_ref_tick: bool,
) {
    ledc.timer(number as usize).conf().modify(|_, w| unsafe {
        #[cfg(soc_has_clock_node_ref_tick)]
        w.tick_sel().bit(use_ref_tick);
        #[cfg(not(soc_has_clock_node_ref_tick))]
        let _ = use_ref_tick;
        w.rst().clear_bit();
        w.pause().clear_bit();
        w.clk_div().bits(divisor);
        w.duty_res().bits(duty)
    });
}

pub(super) fn ls_update_hw(ledc: &RegisterBlock, number: TimerNumber) {
    ledc.timer(number as usize)
        .conf()
        .modify(|_, w| w.para_up().set_bit());
}

pub(super) fn output_signal(ch_num: ChannelNumber, _is_hs: bool) -> OutputSignal {
    match ch_num {
        ChannelNumber::Channel0 => OutputSignal::LEDC_LS_SIG0,
        ChannelNumber::Channel1 => OutputSignal::LEDC_LS_SIG1,
        ChannelNumber::Channel2 => OutputSignal::LEDC_LS_SIG2,
        ChannelNumber::Channel3 => OutputSignal::LEDC_LS_SIG3,
        ChannelNumber::Channel4 => OutputSignal::LEDC_LS_SIG4,
        ChannelNumber::Channel5 => OutputSignal::LEDC_LS_SIG5,
        #[cfg(ledc_channel_count = "8")]
        ChannelNumber::Channel6 => OutputSignal::LEDC_LS_SIG6,
        #[cfg(ledc_channel_count = "8")]
        ChannelNumber::Channel7 => OutputSignal::LEDC_LS_SIG7,
    }
}

pub(super) fn set_channel(
    ledc: &RegisterBlock,
    ch_num: ChannelNumber,
    timer_number: u8,
    _is_hs: bool,
) {
    let ch = ledc.ch(ch_num as usize);
    ch.hpoint().write(|w| unsafe { w.hpoint().bits(0x0) });
    ch.conf0().modify(|_, w| {
        w.sig_out_en().set_bit();
        unsafe { w.timer_sel().bits(timer_number) }
    });
}

pub(super) fn start_duty_without_fading(ledc: &RegisterBlock, ch_num: ChannelNumber, _is_hs: bool) {
    ledc.ch(ch_num as usize).conf1().write(|w| {
        w.duty_start().set_bit();
        w.duty_inc().set_bit();
        unsafe {
            w.duty_num().bits(0x1);
            w.duty_cycle().bits(0x1);
            w.duty_scale().bits(0x0)
        }
    });
}

pub(super) fn start_duty_fade_inner(
    ledc: &RegisterBlock,
    ch_num: ChannelNumber,
    _is_hs: bool,
    duty_inc: bool,
    duty_steps: u16,
    cycles_per_step: u16,
    duty_per_cycle: u16,
) {
    ledc.ch(ch_num as usize).conf1().write(|w| unsafe {
        w.duty_start().set_bit();
        w.duty_inc().variant(duty_inc);
        w.duty_num().bits(duty_steps); // count of incs before stopping
        w.duty_cycle().bits(cycles_per_step); // overflows between incs
        w.duty_scale().bits(duty_per_cycle)
    });
}

pub(super) fn update_channel(ledc: &RegisterBlock, ch_num: ChannelNumber, _is_hs: bool) {
    ledc.ch(ch_num as usize)
        .conf0()
        .modify(|_, w| w.para_up().set_bit());
}

pub(super) fn set_duty_hw(ledc: &RegisterBlock, ch_num: ChannelNumber, _is_hs: bool, duty: u32) {
    ledc.ch(ch_num as usize)
        .duty()
        .write(|w| unsafe { w.duty().bits(duty << 4) });
}

#[allow(clippy::too_many_arguments)]
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
    ledc.ch(ch_num as usize)
        .duty()
        .write(|w| unsafe { w.duty().bits(start_duty << 4) });
    ledc.int_clr()
        .write(|w| w.duty_chng_end_ch(ch_num as u8).clear_bit_by_one());
    start_duty_fade_inner(
        ledc,
        ch_num,
        is_hs,
        duty_inc,
        duty_steps,
        cycles_per_step,
        duty_per_cycle,
    );
}

pub(super) fn is_duty_fade_running_hw(
    ledc: &RegisterBlock,
    ch_num: ChannelNumber,
    _is_hs: bool,
) -> bool {
    ledc.int_raw()
        .read()
        .duty_chng_end_ch(ch_num as u8)
        .bit_is_clear()
}
