use super::super::{
    LSGlobalClkSource,
    channel::Number as ChannelNumber,
    timer::{LSClockSource, Number as TimerNumber},
};
use crate::{gpio::OutputSignal, pac::ledc::RegisterBlock, soc::clocks, time::Rate};

pub(super) fn set_global_slow_clock(ledc: &RegisterBlock, clock_source: LSGlobalClkSource) {
    let pcr = unsafe { &*crate::peripherals::PCR::ptr() };
    pcr.ledc_sclk_conf().write(|w| w.ledc_sclk_en().set_bit());
    match clock_source {
        LSGlobalClkSource::APBClk => {
            #[cfg(esp32c6)]
            pcr.ledc_sclk_conf()
                .write(|w| unsafe { w.ledc_sclk_sel().bits(1) });
            #[cfg(esp32h2)]
            pcr.ledc_sclk_conf()
                .write(|w| unsafe { w.ledc_sclk_sel().bits(0) });
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
    _use_ref_tick: bool,
) {
    ledc.timer(number as usize).conf().modify(|_, w| unsafe {
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
    }
}

pub(super) fn set_channel(
    ledc: &RegisterBlock,
    ch_num: ChannelNumber,
    timer_number: u8,
    _is_hs: bool,
) {
    let cnum = ch_num as usize;
    let ch = ledc.ch(cnum);
    ch.hpoint().write(|w| unsafe { w.hpoint().bits(0x0) });
    ch.conf0().modify(|_, w| {
        w.sig_out_en().set_bit();
        unsafe { w.timer_sel().bits(timer_number) }
    });
    // this is needed to make low duty-resolutions / high frequencies work
    ledc.ch_gamma_wr_addr(cnum).write(|w| unsafe { w.bits(0) });
}

pub(super) fn start_duty_without_fading(ledc: &RegisterBlock, ch_num: ChannelNumber, _is_hs: bool) {
    let cnum = ch_num as usize;
    ledc.ch(cnum).conf1().write(|w| w.duty_start().set_bit());
    ledc.ch_gamma_wr(cnum).write(|w| {
        w.ch_gamma_duty_inc().set_bit();
        unsafe {
            w.ch_gamma_duty_num().bits(0x1);
            w.ch_gamma_duty_cycle().bits(0x1);
            w.ch_gamma_scale().bits(0x0)
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
    let cnum = ch_num as usize;
    ledc.ch(cnum).conf1().write(|w| w.duty_start().set_bit());
    ledc.ch_gamma_wr(cnum).write(|w| unsafe {
        w.ch_gamma_duty_inc()
            .variant(duty_inc)
            .ch_gamma_duty_num() // count of incs before stopping
            .bits(duty_steps)
            .ch_gamma_duty_cycle() // overflows between incs
            .bits(cycles_per_step)
            .ch_gamma_scale()
            .bits(duty_per_cycle)
    });
    ledc.ch_gamma_wr_addr(cnum)
        .write(|w| unsafe { w.ch_gamma_wr_addr().bits(0) });
    ledc.ch_gamma_conf(cnum)
        .write(|w| unsafe { w.ch_gamma_entry_num().bits(0x1) });
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
