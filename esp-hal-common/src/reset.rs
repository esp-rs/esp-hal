use crate::rtc_cntl::SocResetReason;

pub fn software_reset() {
    unsafe { crate::rtc_cntl::software_reset() }

}
pub fn software_reset_cpu() {
    unsafe { crate::rtc_cntl::software_reset_cpu() }
}

pub fn get_reset_reason() -> Option<SocResetReason> {
    crate::rtc_cntl::get_reset_reason(crate::get_core())
}

pub fn get_wakeup_cause() -> u32 {
    crate::rtc_cntl::get_wakeup_cause()
}
