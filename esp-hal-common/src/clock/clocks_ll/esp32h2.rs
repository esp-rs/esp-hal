use crate::clock::{ApbClock, CpuClock, PllClock, XtalClock};

extern "C" {
    fn ets_update_cpu_frequency(ticks_per_us: u32);
}

pub(crate) fn esp32h2_rtc_bbpll_configure(_xtal_freq: XtalClock, _pll_freq: PllClock) {
    todo!()
}

pub(crate) fn esp32h2_rtc_bbpll_enable() {
    todo!()
}

pub(crate) fn esp32h2_rtc_update_to_xtal(freq: XtalClock, _div: u8) {
    todo!()
}

pub(crate) fn esp32h2_rtc_freq_to_pll_mhz(cpu_clock_speed: CpuClock) {
    todo!()
}

pub(crate) fn esp32h2_rtc_apb_freq_update(apb_freq: ApbClock) {
    todo!()
}
