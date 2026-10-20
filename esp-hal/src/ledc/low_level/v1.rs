use super::super::{
    LSGlobalClkSource,
    channel::Number as ChannelNumber,
    timer::{HSClockSource, LSClockSource, Number as TimerNumber},
};
use crate::{gpio::OutputSignal, pac::ledc::RegisterBlock, soc::clocks, time::Rate};

pub(super) fn set_global_slow_clock(ledc: &RegisterBlock, clock_source: LSGlobalClkSource) {
    match clock_source {
        LSGlobalClkSource::APBClk => {
            ledc.conf().write(|w| w.apb_clk_sel().set_bit());
        }
    }
    ledc.lstimer(0).conf().modify(|_, w| w.para_up().set_bit());
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
    let use_apb = !use_ref_tick;
    ledc.lstimer(number as usize).conf().modify(|_, w| unsafe {
        w.tick_sel().bit(use_apb);
        w.rst().clear_bit();
        w.pause().clear_bit();
        w.div_num().bits(divisor);
        w.duty_res().bits(duty)
    });
}

pub(super) fn ls_update_hw(ledc: &RegisterBlock, number: TimerNumber) {
    ledc.lstimer(number as usize)
        .conf()
        .modify(|_, w| w.para_up().set_bit());
}

pub(super) fn hs_freq_hw(_clock_source: HSClockSource) -> Rate {
    Rate::from_hz(clocks::apb_clk_frequency())
}

pub(super) fn hs_configure_hw(
    ledc: &RegisterBlock,
    number: TimerNumber,
    divisor: u32,
    duty: u8,
    clock_source: HSClockSource,
) {
    let sel_hstimer = clock_source == HSClockSource::APBClk;
    ledc.hstimer(number as usize).conf().modify(|_, w| unsafe {
        w.tick_sel().bit(sel_hstimer);
        w.rst().clear_bit();
        w.pause().clear_bit();
        w.div_num().bits(divisor);
        w.duty_res().bits(duty)
    });
}

pub(super) fn hs_update_hw() {
    // Nothing to do for HS timers
}

pub(super) fn output_signal(ch_num: ChannelNumber, is_hs: bool) -> OutputSignal {
    if is_hs {
        match ch_num {
            ChannelNumber::Channel0 => OutputSignal::LEDC_HS_SIG0,
            ChannelNumber::Channel1 => OutputSignal::LEDC_HS_SIG1,
            ChannelNumber::Channel2 => OutputSignal::LEDC_HS_SIG2,
            ChannelNumber::Channel3 => OutputSignal::LEDC_HS_SIG3,
            ChannelNumber::Channel4 => OutputSignal::LEDC_HS_SIG4,
            ChannelNumber::Channel5 => OutputSignal::LEDC_HS_SIG5,
            ChannelNumber::Channel6 => OutputSignal::LEDC_HS_SIG6,
            ChannelNumber::Channel7 => OutputSignal::LEDC_HS_SIG7,
        }
    } else {
        match ch_num {
            ChannelNumber::Channel0 => OutputSignal::LEDC_LS_SIG0,
            ChannelNumber::Channel1 => OutputSignal::LEDC_LS_SIG1,
            ChannelNumber::Channel2 => OutputSignal::LEDC_LS_SIG2,
            ChannelNumber::Channel3 => OutputSignal::LEDC_LS_SIG3,
            ChannelNumber::Channel4 => OutputSignal::LEDC_LS_SIG4,
            ChannelNumber::Channel5 => OutputSignal::LEDC_LS_SIG5,
            ChannelNumber::Channel6 => OutputSignal::LEDC_LS_SIG6,
            ChannelNumber::Channel7 => OutputSignal::LEDC_LS_SIG7,
        }
    }
}

pub(super) fn set_channel(
    ledc: &RegisterBlock,
    ch_num: ChannelNumber,
    timer_number: u8,
    is_hs: bool,
) {
    if is_hs {
        let ch = ledc.hsch(ch_num as usize);
        ch.hpoint().write(|w| unsafe { w.hpoint().bits(0x0) });
        ch.conf0()
            .modify(|_, w| unsafe { w.sig_out_en().set_bit().timer_sel().bits(timer_number) });
    } else {
        let ch = ledc.lsch(ch_num as usize);
        ch.hpoint().write(|w| unsafe { w.hpoint().bits(0x0) });
        ch.conf0()
            .modify(|_, w| unsafe { w.sig_out_en().set_bit().timer_sel().bits(timer_number) });
    }
}

pub(super) fn start_duty_without_fading(ledc: &RegisterBlock, ch_num: ChannelNumber, is_hs: bool) {
    if is_hs {
        ledc.hsch(ch_num as usize).conf1().write(|w| unsafe {
            w.duty_start().set_bit();
            w.duty_inc().set_bit();
            w.duty_num().bits(0x1);
            w.duty_cycle().bits(0x1);
            w.duty_scale().bits(0x0)
        });
    } else {
        ledc.lsch(ch_num as usize).conf1().write(|w| unsafe {
            w.duty_start().set_bit();
            w.duty_inc().set_bit();
            w.duty_num().bits(0x1);
            w.duty_cycle().bits(0x1);
            w.duty_scale().bits(0x0)
        });
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn start_duty_fade_inner(
    ledc: &RegisterBlock,
    ch_num: ChannelNumber,
    is_hs: bool,
    duty_inc: bool,
    duty_steps: u16,
    cycles_per_step: u16,
    duty_per_cycle: u16,
) {
    if is_hs {
        ledc.hsch(ch_num as usize).conf1().write(|w| unsafe {
            w.duty_start()
                .set_bit()
                .duty_inc()
                .variant(duty_inc)
                .duty_num() // count of incs before stopping
                .bits(duty_steps)
                .duty_cycle() // overflows between incs
                .bits(cycles_per_step)
                .duty_scale()
                .bits(duty_per_cycle)
        });
    } else {
        ledc.lsch(ch_num as usize).conf1().write(|w| unsafe {
            w.duty_start()
                .set_bit()
                .duty_inc()
                .variant(duty_inc)
                .duty_num() // count of incs before stopping
                .bits(duty_steps)
                .duty_cycle() // overflows between incs
                .bits(cycles_per_step)
                .duty_scale()
                .bits(duty_per_cycle)
        });
    }
}

pub(super) fn update_channel(ledc: &RegisterBlock, ch_num: ChannelNumber, is_hs: bool) {
    if !is_hs {
        ledc.lsch(ch_num as usize)
            .conf0()
            .modify(|_, w| w.para_up().set_bit());
    }
}

pub(super) fn set_duty_hw(ledc: &RegisterBlock, ch_num: ChannelNumber, is_hs: bool, duty: u32) {
    if is_hs {
        ledc.hsch(ch_num as usize)
            .duty()
            .write(|w| unsafe { w.duty().bits(duty << 4) });
    } else {
        ledc.lsch(ch_num as usize)
            .duty()
            .write(|w| unsafe { w.duty().bits(duty << 4) });
    }
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
    if is_hs {
        ledc.hsch(ch_num as usize)
            .duty()
            .write(|w| unsafe { w.duty().bits(start_duty << 4) });
        ledc.int_clr()
            .write(|w| w.duty_chng_end_hsch(ch_num as u8).clear_bit_by_one());
    } else {
        ledc.lsch(ch_num as usize)
            .duty()
            .write(|w| unsafe { w.duty().bits(start_duty << 4) });
        ledc.int_clr()
            .write(|w| w.duty_chng_end_lsch(ch_num as u8).clear_bit_by_one());
    }
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
    is_hs: bool,
) -> bool {
    let reg = ledc.int_raw().read();
    if is_hs {
        reg.duty_chng_end_hsch(ch_num as u8).bit_is_clear()
    } else {
        reg.duty_chng_end_lsch(ch_num as u8).bit_is_clear()
    }
}
