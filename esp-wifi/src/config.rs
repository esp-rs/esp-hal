#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Tunable parameters for the WiFi driver
#[allow(unused)] // currently there are no ble tunables
pub(crate) struct EspWifiConfig {
    pub(crate) rx_queue_size: usize,
    pub(crate) tx_queue_size: usize,
    pub(crate) static_rx_buf_num: usize,
    pub(crate) dynamic_rx_buf_num: usize,
    pub(crate) static_tx_buf_num: usize,
    pub(crate) dynamic_tx_buf_num: usize,
    pub(crate) ampdu_rx_enable: bool,
    pub(crate) ampdu_tx_enable: bool,
    pub(crate) amsdu_tx_enable: bool,
    pub(crate) rx_ba_win: usize,
    pub(crate) max_burst_size: usize,
    pub(crate) country_code: &'static str,
    pub(crate) country_code_operating_class: u8,
    pub(crate) mtu: usize,
    pub(crate) tick_rate_hz: u32,
    pub(crate) listen_interval: u16,
    pub(crate) beacon_timeout: u16,
    pub(crate) ap_beacon_timeout: u16,
    pub(crate) failure_retry_cnt: u8,
    pub(crate) scan_method: u32,
}

#[non_exhaustive]
#[derive(Default)]
pub enum PowerSaveMode {
    None,
    #[default]
    Minimum,
    Maximum,
}

impl From<PowerSaveMode> for esp_wifi_sys::include::wifi_ps_type_t {
    fn from(s: PowerSaveMode) -> Self {
        match s {
            PowerSaveMode::None => esp_wifi_sys::include::wifi_ps_type_t_WIFI_PS_NONE,
            PowerSaveMode::Minimum => esp_wifi_sys::include::wifi_ps_type_t_WIFI_PS_MIN_MODEM,
            PowerSaveMode::Maximum => esp_wifi_sys::include::wifi_ps_type_t_WIFI_PS_MAX_MODEM,
        }
    }
}
