//! Wi-Fi Channel State Information (CSI).

use alloc::boxed::Box;
use core::marker::PhantomData;

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

/// CSI (Channel State Information) packet metadata and associated packet details.
///
/// This structure contains the raw CSI data, along with necessary metadata
/// from the received WiFi packet (MAC addresses, sequence number, packet headers).
#[repr(transparent)]
pub struct WifiCsiInfo<'a> {
    inner: *const wifi_csi_info_t,
    _lt: PhantomData<&'a ()>,
}

impl<'a> WifiCsiInfo<'_> {
    /// Metadata header for the CSI data of a received packet.
    pub fn rx_ctrl(&self) -> WifiPktRxCtrl<'a> {
        WifiPktRxCtrl {
            inner: unsafe { &(*self.inner).rx_ctrl },
            _lt: PhantomData,
        }
    }

    /// Source MAC address of the CSI data.
    pub fn mac(&self) -> &[u8; 6] {
        unsafe { &(*self.inner).mac }
    }

    /// Destination MAC address of the CSI data.
    pub fn destination_mac(&self) -> &[u8; 6] {
        unsafe { &(*self.inner).dmac }
    }

    /// First four bytes of the CSI data is invalid or not, true indicates the first four bytes is
    /// invalid due to hardware limitation.
    pub fn first_word_invalid(&self) -> bool {
        unsafe { (*self.inner).first_word_invalid }
    }

    /// Valid buffer of CSI data.
    pub fn buf(&self) -> &[i8] {
        unsafe {
            if (*self.inner).buf.is_null() || (*self.inner).len == 0 {
                &[]
            } else {
                core::slice::from_raw_parts((*self.inner).buf, (*self.inner).len as usize)
            }
        }
    }

    /// Header of the wifi packet.
    pub fn header(&self) -> &[u8] {
        unsafe {
            if (*self.inner).hdr.is_null() {
                &[]
            } else {
                const HDR_LEN: usize = 24;
                core::slice::from_raw_parts((*self.inner).hdr, HDR_LEN)
            }
        }
    }

    /// Payload of the wifi packet.
    pub fn payload(&self) -> &[u8] {
        unsafe {
            if (*self.inner).payload.is_null() || (*self.inner).payload_len == 0 {
                &[]
            } else {
                core::slice::from_raw_parts(
                    (*self.inner).payload,
                    (*self.inner).payload_len as usize,
                )
            }
        }
    }

    /// Rx sequence number of the wifi packet.
    pub fn rx_sequence(&self) -> u16 {
        unsafe { (*self.inner).rx_seq }
    }
}

/// Received packet radio metadata header.
#[repr(transparent)]
pub struct WifiPktRxCtrl<'a> {
    inner: *const wifi_pkt_rx_ctrl_t,
    _lt: PhantomData<&'a ()>,
}
#[cfg(not(esp32c6))]
impl WifiPktRxCtrl<'_> {
    /// Received Signal Strength Indicator (RSSI) of the packet.
    pub fn rssi(&self) -> i8 {
        // Signed bitfields are broken in rust-bingen, see https://github.com/esp-rs/esp-wifi-sys/issues/482
        // Hard-casting it from C-signed to i8 gives correct values, so no need for workaround
        // here.
        unsafe { (*self.inner).rssi() as i8 }
    }

    /// Data rate of the packet.
    pub fn rate(&self) -> u8 {
        unsafe { (*self.inner).rate() as u8 }
    }

    /// Protocol of the received packet, 0: non HT(11bg) packet; 1: HT(11n) packet; 3: VHT(11ac)
    /// packet.
    pub fn packet_mode(&self) -> u8 {
        unsafe { (*self.inner).sig_mode() as u8 }
    }

    /// Modulation Coding Scheme. If is HT(11n) packet, shows the modulation, range from 0 to
    /// 76(MSC0 ~ MCS76).
    pub fn modulation_coding_scheme(&self) -> u8 {
        unsafe { (*self.inner).mcs() as u8 }
    }

    /// Channel Bandwidth of the packet. 0: 20MHz; 1: 40MHz.
    pub fn cwb(&self) -> bool {
        unsafe { (*self.inner).cwb() != 0 }
    }

    /// Set to 1 indicates that channel estimate smoothing is recommended.
    /// Set to 0 indicates that only per-carrier independent (unsmoothed) channel estimate is
    /// recommended.
    pub fn smoothing(&self) -> bool {
        unsafe { (*self.inner).smoothing() != 0 }
    }

    /// Set to 1 indicates that the PPDU is not a sounding PPDU.
    /// Set to 0 indicates that PPDU is a sounding PPDU.
    pub fn not_sounding(&self) -> bool {
        unsafe { (*self.inner).not_sounding() != 0 }
    }

    /// Aggregation. 0: MPDU packet; 1: AMPDU packet
    pub fn aggregation(&self) -> bool {
        unsafe { (*self.inner).aggregation() != 0 }
    }

    /// Space Time Block Code(STBC). 0: non STBC packet; 1: STBC packet
    pub fn space_time_block_code(&self) -> u8 {
        unsafe { (*self.inner).stbc() as u8 }
    }

    /// Forward Error Correction(FEC). Flag is set for 11n packets which are LDPC
    pub fn forward_error_correction_coding(&self) -> bool {
        unsafe { (*self.inner).fec_coding() != 0 }
    }

    /// Short Guide Interval(SGI). 0: Long GI; 1: Short GI.
    pub fn short_guide_interval(&self) -> bool {
        unsafe { (*self.inner).sgi() != 0 }
    }

    /// Noise floor in dBm of Radio Frequency Module(RF).
    pub fn noise_floor(&self) -> i8 {
        unsafe { (*self.inner).noise_floor() as i8 }
    }

    /// The number of subframes aggregated in AMPDU.
    pub fn ampdu_count(&self) -> u8 {
        unsafe { (*self.inner).ampdu_cnt() as u8 }
    }

    /// Primary channel on which this packet is received.
    pub fn channel(&self) -> u8 {
        unsafe { (*self.inner).channel() as u8 }
    }

    /// Secondary channel on which this packet is received.
    /// 0: none; 1: above; 2: below.
    pub fn secondary_channel(&self) -> u8 {
        unsafe { (*self.inner).secondary_channel() as u8 }
    }

    /// The local time in microseconds when this packet is received. It is precise only if modem
    /// sleep or light sleep is not enabled.
    pub fn timestamp(&self) -> u32 {
        unsafe { (*self.inner).timestamp() as u32 }
    }

    /// antenna number from which this packet is received.
    /// 0: WiFi antenna 0; 1: WiFi antenna 1
    pub fn antenna(&self) -> u8 {
        unsafe { (*self.inner).ant() as u8 }
    }

    /// Length of packet including Frame Check Sequence(FCS).
    pub fn signal_length(&self) -> u16 {
        unsafe { (*self.inner).sig_len() as u16 }
    }

    /// State of the packet.
    /// 0: no error; others: failure.
    pub fn rx_state(&self) -> u8 {
        unsafe { (*self.inner).rx_state() as u8 }
    }
}

#[cfg(esp32c6)]
impl WifiPktRxCtrl<'_> {
    /// Received Signal Strength Indicator (RSSI) of the packet.
    pub fn rssi(&self) -> i8 {
        unsafe { (*self.inner).rssi() as i8 }
    }

    /// Data rate of the packet.
    pub fn rate(&self) -> u8 {
        unsafe { (*self.inner).rate() as u8 }
    }

    /// Indicate whether the reception frame is from interface 0.
    pub fn rxmatch0(&self) -> bool {
        unsafe { (*self.inner).rxmatch0() != 0 }
    }

    /// Indicate whether the reception frame is from interface 1.
    pub fn rxmatch1(&self) -> bool {
        unsafe { (*self.inner).rxmatch1() != 0 }
    }

    /// Indicate whether the reception frame is from interface 2.
    pub fn rxmatch2(&self) -> bool {
        unsafe { (*self.inner).rxmatch2() != 0 }
    }

    /// Indicate whether the reception frame is from interface 3.
    pub fn rxmatch3(&self) -> bool {
        unsafe { (*self.inner).rxmatch3() != 0 }
    }

    /// HE-SIGA1 or HT-SIG.
    pub fn he_siga1(&self) -> u32 {
        unsafe { (*self.inner).he_siga1 }
    }

    /// Reception state, 0: successful, others: failure.
    pub fn rxend_state(&self) -> u8 {
        unsafe { (*self.inner).rxend_state() as u8 }
    }

    /// HE-SIGA2.
    pub fn he_siga2(&self) -> u16 {
        unsafe { (*self.inner).he_siga2 }
    }

    /// Indicate whether the reception is a group addressed frame.
    pub fn is_group(&self) -> bool {
        unsafe { (*self.inner).is_group() != 0 }
    }

    /// The local time in microseconds when this packet is received. It is precise only if modem
    /// sleep or light sleep is not enabled.
    pub fn timestamp(&self) -> u32 {
        unsafe { (*self.inner).timestamp() }
    }

    /// Noise floor in dBm of Radio Frequency Module(RF).
    pub fn noise_floor(&self) -> i8 {
        unsafe { (*self.inner).noise_floor() as i8 }
    }

    /// Primary channel on which this packet is received.
    pub fn channel(&self) -> u8 {
        unsafe { (*self.inner).channel() as u8 }
    }

    /// Secondary channel if in HT40 on which this packet is received.
    pub fn secondary_channel(&self) -> u8 {
        unsafe { (*self.inner).second() as u8 }
    }

    /// The length of the channel information.
    pub fn rx_channel_estimate_length(&self) -> u32 {
        unsafe { (*self.inner).rx_channel_estimate_len() }
    }

    /// Indicate the channel information is valid.
    pub fn rx_channel_estimate_info_valid(&self) -> bool {
        unsafe { (*self.inner).rx_channel_estimate_info_vld() != 0 }
    }

    /// The format of the reception frame.
    pub fn cur_bb_format(&self) -> u8 {
        unsafe { (*self.inner).cur_bb_format() as u8 }
    }

    /// Indicate whether the reception MPDU is a S-MPDU.
    pub fn cur_single_mpdu(&self) -> bool {
        unsafe { (*self.inner).cur_single_mpdu() != 0 }
    }

    /// The length of HE-SIGB.
    pub fn he_sigb_length(&self) -> u8 {
        unsafe { (*self.inner).he_sigb_len() as u8 }
    }

    /// The length of the reception MPDU.
    pub fn signal_length(&self) -> u32 {
        unsafe { (*self.inner).sig_len() }
    }

    /// The length of the reception MPDU excluding the FCS.
    pub fn dump_length(&self) -> u32 {
        unsafe { (*self.inner).dump_len() }
    }

    /// State of the packet.
    /// 0: no error; others: failure.
    pub fn rx_state(&self) -> u8 {
        unsafe { (*self.inner).rx_state() as u8 }
    }
}

pub(crate) trait CsiCallback: FnMut(WifiCsiInfo<'_>) {}

impl<T> CsiCallback for T where T: FnMut(WifiCsiInfo<'_>) {}

unsafe extern "C" fn csi_rx_cb<C: CsiCallback>(ctx: *mut c_void, data: *mut wifi_csi_info_t) {
    unsafe {
        let csi_callback = &mut *(ctx as *mut C);

        let data = WifiCsiInfo {
            inner: data,
            _lt: PhantomData,
        };
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
