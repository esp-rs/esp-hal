use crate::{
    clock::XtalClock,
    pac::RTC_CNTL,
    rtc_cntl::{RtcCalSel, RtcClock, RtcFastClock, RtcSlowClock},
};

pub(crate) fn init() {}

pub(crate) fn configure_clock() {
    assert!(matches!(
        RtcClock::get_xtal_freq(),
        XtalClock::RtcXtalFreq40M
    ));

    RtcClock::set_fast_freq(RtcFastClock::RtcFastClock8m);

    let cal_val = loop {
        RtcClock::set_slow_freq(RtcSlowClock::RtcSlowClockRtc);

        let res = RtcClock::calibrate(RtcCalSel::RtcCalRtcMux, 1024);
        if res != 0 {
            break res;
        }
    };

    unsafe {
        let rtc_cntl = &*RTC_CNTL::ptr();
        rtc_cntl.store1.write(|w| w.bits(cal_val));
    }
}
