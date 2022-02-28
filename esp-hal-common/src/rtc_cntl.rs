use crate::pac::RTC_CNTL;

pub struct RtcCntl {
    rtc_cntl: RTC_CNTL,
}

impl RtcCntl {
    pub fn new(rtc_cntl: RTC_CNTL) -> Self {
        Self { rtc_cntl }
    }

    /// Enable/disable write protection for WDT registers
    fn set_wdt_write_protection(&mut self, enable: bool) {
        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };

        // FIXME: To be removed once the ESP32-S3 SVD
        //        register naming is aligned!
        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32s3"))] {
                let register = &self.rtc_cntl.rtc_wdtwprotect;
            } else {
                let register = &self.rtc_cntl.wdtwprotect;
            }
        }

        register.write(|w| unsafe { w.bits(wkey) });
    }

    /// Global switch for RTC_CNTL watchdog functionality
    pub fn set_wdt_global_enable(&mut self, enable: bool) {
        self.set_wdt_write_protection(false);

        // FIXME: To be removed once the ESP32-S3 SVD
        //        register naming is aligned!
        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32s3"))] {
                let register = &self.rtc_cntl.rtc_wdtconfig0;
            } else {
                let register = &self.rtc_cntl.wdtconfig0;
            }
        }

        register.modify(|_, w| w.wdt_en().bit(enable).wdt_flashboot_mod_en().clear_bit());

        self.set_wdt_write_protection(true);
    }
}
