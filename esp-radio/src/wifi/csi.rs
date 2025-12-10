//! Wi-Fi Channel State Information (CSI).

use alloc::boxed::Box;

use super::{WifiError, c_types::c_void, esp_wifi_result};
#[cfg(esp32c6)]
use crate::sys::include::wifi_csi_acquire_config_t;
use crate::{
    sys::include::{
        esp_wifi_set_csi,
        esp_wifi_set_csi_config,
        esp_wifi_set_csi_rx_cb,
        wifi_csi_config_t,
        wifi_csi_info_t,
    },
    wifi::wifi_pkt_rx_ctrl_t,
};

/// Wrapper around the C `wifi_csi_info_t` struct.
#[repr(transparent)]
pub struct CWifiCsiInfo(wifi_csi_info_t);

impl CWifiCsiInfo {
    /// Get metadata header for the CSI data of a received packet.
    pub fn rx_ctrl(&self) -> &wifi_pkt_rx_ctrl_t {
        &self.0.rx_ctrl
    }

    /// Source MAC address of the CSI data.
    pub fn mac(&self) -> &[u8; 6] {
        &self.0.mac
    }

    /// Destination MAC address of the CSI data.
    pub fn dmac(&self) -> &[u8; 6] {
        &self.0.dmac
    }

    /// First four bytes of the CSI data is invalid or not, true indicates the first four bytes is
    /// invalid due to hardware limitation.
    pub fn first_word_invalid(&self) -> bool {
        self.0.first_word_invalid
    }

    /// Valid buffer of CSI data.
    pub fn buf(&self) -> &[i8] {
        unsafe {
            if self.0.buf.is_null() || self.0.len == 0 {
                &[]
            } else {
                core::slice::from_raw_parts(self.0.buf, self.0.len as usize)
            }
        }
    }

    /// Header of the wifi packet.
    pub fn hdr(&self) -> &[u8] {
        unsafe {
            if self.0.hdr.is_null() {
                &[]
            } else {
                // what is the length of hdr?
                core::slice::from_raw_parts(self.0.hdr, 30 as usize)
            }
        }
    }

    /// Payload of the wifi packet.
    pub fn payload(&self) -> &[u8] {
        unsafe {
            if self.0.payload.is_null() || self.0.payload_len == 0 {
                &[]
            } else {
                core::slice::from_raw_parts(self.0.payload, self.0.payload_len as usize)
            }
        }
    }

    /// Rx sequence number of the wifi packet.
    pub fn rx_seq(&self) -> u16 {
        self.0.rx_seq
    }
}

pub(crate) trait CsiCallback: FnMut(CWifiCsiInfo) {}

impl<T> CsiCallback for T where T: FnMut(CWifiCsiInfo) {}

unsafe extern "C" fn csi_rx_cb<C: CsiCallback>(ctx: *mut c_void, data: *mut wifi_csi_info_t) {
    unsafe {
        let csi_callback = &mut *(ctx as *mut C);

        let data = CWifiCsiInfo(*data);
        csi_callback(data);
    }
}

/// Channel state information (CSI) configuration.
#[derive(Clone, PartialEq, Eq)]
#[cfg(not(esp32c6))]
pub struct CsiConfig {
    /// Enable to receive legacy long training field(lltf) data.
    pub lltf_en: bool,
    /// Enable to receive HT long training field(htltf) data.
    pub htltf_en: bool,
    /// Enable to receive space time block code HT long training
    /// field(stbc-htltf2) data.
    pub stbc_htltf2_en: bool,
    /// Enable to generate htlft data by averaging lltf and ht_ltf data when
    /// receiving HT packet. Otherwise, use ht_ltf data directly.
    pub ltf_merge_en: bool,
    /// Enable to turn on channel filter to smooth adjacent sub-carrier. Disable
    /// it to keep independence of adjacent sub-carrier.
    pub channel_filter_en: bool,
    /// Manually scale the CSI data by left shifting or automatically scale the
    /// CSI data. If set true, please set the shift bits. false: automatically.
    /// true: manually.
    pub manu_scale: bool,
    /// Manually left shift bits of the scale of the CSI data. The range of the
    /// left shift bits is 0~15.
    pub shift: u8,
    /// Enable to dump 802.11 ACK frame.
    pub dump_ack_en: bool,
}

/// Channel state information (CSI) configuration.
#[derive(Clone, PartialEq, Eq)]
#[cfg(esp32c6)]
pub struct CsiConfig {
    /// Enable to acquire CSI.
    pub enable: u32,
    /// Enable to acquire L-LTF when receiving a 11g PPDU.
    pub acquire_csi_legacy: u32,
    /// Enable to acquire HT-LTF when receiving an HT20 PPDU.
    pub acquire_csi_ht20: u32,
    /// Enable to acquire HT-LTF when receiving an HT40 PPDU.
    pub acquire_csi_ht40: u32,
    /// Enable to acquire HE-LTF when receiving an HE20 SU PPDU.
    pub acquire_csi_su: u32,
    /// Enable to acquire HE-LTF when receiving an HE20 MU PPDU.
    pub acquire_csi_mu: u32,
    /// Enable to acquire HE-LTF when receiving an HE20 DCM applied PPDU.
    pub acquire_csi_dcm: u32,
    /// Enable to acquire HE-LTF when receiving an HE20 Beamformed applied PPDU.
    pub acquire_csi_beamformed: u32,
    /// When receiving an STBC applied HE PPDU, 0- acquire the complete
    /// HE-LTF1,  1- acquire the complete HE-LTF2, 2- sample evenly among the
    /// HE-LTF1 and HE-LTF2.
    pub acquire_csi_he_stbc: u32,
    /// Value 0-3.
    pub val_scale_cfg: u32,
    /// Enable to dump 802.11 ACK frame, default disabled.
    pub dump_ack_en: u32,
    /// Reserved.
    pub reserved: u32,
}

impl CsiConfig {
    /// Set CSI data configuration
    pub(crate) fn apply_config(&self) -> Result<(), WifiError> {
        let conf: wifi_csi_config_t = self.clone().into();
        unsafe { esp_wifi_result!(esp_wifi_set_csi_config(&conf))? };
        Ok(())
    }

    /// Register the RX callback function of CSI data. Each time a CSI data is
    /// received, the callback function will be called.
    pub(crate) fn set_receive_cb<C>(&mut self, cb: C) -> Result<(), WifiError>
    where
        C: CsiCallback,
    {
        let cb = Box::new(cb);
        let cb_ptr = Box::into_raw(cb) as *mut c_void;

        unsafe { esp_wifi_result!(esp_wifi_set_csi_rx_cb(Some(csi_rx_cb::<C>), cb_ptr))? };
        Ok(())
    }

    /// Enable or disable CSI
    pub(crate) fn set_csi(&self, enable: bool) -> Result<(), WifiError> {
        // https://github.com/esp-rs/esp-wifi-sys/blob/2a466d96fe8119d49852fc794aea0216b106ba7b/esp-wifi-sys/headers/esp_wifi.h#L1241
        unsafe { esp_wifi_result!(esp_wifi_set_csi(enable))? };
        Ok(())
    }
}

impl Default for CsiConfig {
    #[cfg(not(esp32c6))]
    fn default() -> Self {
        Self {
            lltf_en: true,
            htltf_en: true,
            stbc_htltf2_en: true,
            ltf_merge_en: true,
            channel_filter_en: true,
            manu_scale: false,
            shift: 0,
            dump_ack_en: false,
        }
    }

    #[cfg(esp32c6)]
    fn default() -> Self {
        // https://github.com/esp-rs/esp-wifi-sys/blob/2a466d96fe8119d49852fc794aea0216b106ba7b/esp-wifi-sys/headers/esp_wifi_he_types.h#L67-L82
        Self {
            enable: 1,
            acquire_csi_legacy: 1,
            acquire_csi_ht20: 1,
            acquire_csi_ht40: 1,
            acquire_csi_su: 1,
            acquire_csi_mu: 1,
            acquire_csi_dcm: 1,
            acquire_csi_beamformed: 1,
            acquire_csi_he_stbc: 2,
            val_scale_cfg: 2,
            dump_ack_en: 1,
            reserved: 19,
        }
    }
}

#[doc(hidden)]
impl From<CsiConfig> for wifi_csi_config_t {
    fn from(config: CsiConfig) -> Self {
        #[cfg(not(esp32c6))]
        {
            wifi_csi_config_t {
                lltf_en: config.lltf_en,
                htltf_en: config.htltf_en,
                stbc_htltf2_en: config.stbc_htltf2_en,
                ltf_merge_en: config.ltf_merge_en,
                channel_filter_en: config.channel_filter_en,
                manu_scale: config.manu_scale,
                shift: config.shift,
                dump_ack_en: config.dump_ack_en,
            }
        }
        #[cfg(esp32c6)]
        {
            wifi_csi_acquire_config_t {
                _bitfield_align_1: [0; 0],
                _bitfield_1: wifi_csi_acquire_config_t::new_bitfield_1(
                    config.enable,
                    config.acquire_csi_legacy,
                    config.acquire_csi_ht20,
                    config.acquire_csi_ht40,
                    config.acquire_csi_su,
                    config.acquire_csi_mu,
                    config.acquire_csi_dcm,
                    config.acquire_csi_beamformed,
                    config.acquire_csi_he_stbc,
                    config.val_scale_cfg,
                    config.dump_ack_en,
                    config.reserved,
                ),
            }
        }
    }
}
