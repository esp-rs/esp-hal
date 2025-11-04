use core::{marker::PhantomData, mem::MaybeUninit};

use crate::{
    esp_wifi_result,
    sys::include,
    wifi::{
        AccessPointInfo,
        AuthMethod,
        AuthMethodExt as _,
        Country,
        SecondaryChannel,
        WifiController,
        WifiError,
    },
};

pub struct ScanResults<'d> {
    /// Number of APs to return
    remaining: usize,

    /// Ensures the result list is free'd when this struct is dropped.
    _drop_guard: FreeApListOnDrop,

    /// Hold a lifetime to ensure the scan list is freed before a new scan is started.
    _marker: PhantomData<&'d mut ()>,
}

impl<'d> ScanResults<'d> {
    pub fn new(_controller: &'d mut WifiController<'_>) -> Result<Self, WifiError> {
        // Construct Self first. This ensures we'll free the result list even if `get_ap_num`
        // returns an error.
        let mut this = Self {
            remaining: 0,
            _drop_guard: FreeApListOnDrop,
            _marker: PhantomData,
        };

        let mut bss_total = 0;
        unsafe { esp_wifi_result!(include::esp_wifi_scan_get_ap_num(&mut bss_total))? };

        this.remaining = bss_total as usize;

        Ok(this)
    }
}

impl Iterator for ScanResults<'_> {
    type Item = AccessPointInfo;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }

        self.remaining -= 1;

        let mut record: MaybeUninit<include::wifi_ap_record_t> = MaybeUninit::uninit();

        // We could detect ESP_FAIL to see if we've exhausted the list, but we know the number of
        // results. Reading the number of results also ensures we're in the correct state, so
        // unwrapping here should never fail.
        unwrap!(unsafe {
            esp_wifi_result!(include::esp_wifi_scan_get_ap_record(record.as_mut_ptr()))
        });

        Some(convert_ap_info(unsafe { record.assume_init_ref() }))
    }
}

fn convert_ap_info(record: &include::wifi_ap_record_t) -> AccessPointInfo {
    let str_len = record
        .ssid
        .iter()
        .position(|&c| c == 0)
        .unwrap_or(record.ssid.len());
    let ssid = alloc::string::String::from_utf8_lossy(&record.ssid[..str_len]).into_owned();

    AccessPointInfo {
        ssid,
        bssid: record.bssid,
        channel: record.primary,
        secondary_channel: match record.second {
            include::wifi_second_chan_t_WIFI_SECOND_CHAN_NONE => SecondaryChannel::None,
            include::wifi_second_chan_t_WIFI_SECOND_CHAN_ABOVE => SecondaryChannel::Above,
            include::wifi_second_chan_t_WIFI_SECOND_CHAN_BELOW => SecondaryChannel::Below,
            _ => panic!(),
        },
        signal_strength: record.rssi,
        auth_method: Some(AuthMethod::from_raw(record.authmode)),
        country: Country::try_from_c(&record.country),
    }
}

pub struct FreeApListOnDrop;
impl FreeApListOnDrop {
    pub fn defuse(self) {
        core::mem::forget(self);
    }
}

impl Drop for FreeApListOnDrop {
    fn drop(&mut self) {
        unsafe {
            include::esp_wifi_clear_ap_list();
        }
    }
}
