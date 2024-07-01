use core::ops::Div;

use crate::{
    clock::{Clock, CpuClock},
    rom,
};

pub(crate) fn set_cpu_clock(cpu_clock_speed: CpuClock) {
    let system_control = unsafe { &*crate::peripherals::SYSTEM::PTR };

    unsafe {
        system_control
            .sysclk_conf()
            .modify(|_, w| w.soc_clk_sel().bits(1));
        system_control.cpu_per_conf().modify(|_, w| {
            w.pll_freq_sel()
                .set_bit()
                .cpuperiod_sel()
                .bits(match cpu_clock_speed {
                    CpuClock::Clock80MHz => 0,
                    CpuClock::Clock160MHz => 1,
                    CpuClock::Clock240MHz => 2,
                })
        });
    }

    let ticks_per_us = cpu_clock_speed.frequency().div(1_000_000);
    rom::ets_update_cpu_frequency_rom(ticks_per_us.raw());
}
