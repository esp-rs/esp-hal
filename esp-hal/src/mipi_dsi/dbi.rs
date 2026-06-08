//! MIPI DSI command-mode (DBI) interface.

use crate::{mipi_dsi::MipiDsi, peripherals::MIPI_DSI_HOST};

// DCS data type identifiers (MIPI DSI spec Table 7-1).
const DT_DCS_SHORT_WRITE_0: u8 = 0x05;
const DT_DCS_SHORT_WRITE_1: u8 = 0x15;
const DT_DCS_LONG_WRITE: u8 = 0x39;
const DT_DCS_READ_0: u8 = 0x06;
const DT_SET_MAX_RETURN_PKT: u8 = 0x37;

/// Error returned by DBI operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The parameter buffer is larger than the FIFO can handle.
    PayloadTooLarge,
}

/// Command-mode (DBI) handle.
///
/// Holds a mutable borrow of the [`MipiDsi`] bus so that command and video
/// modes cannot coexist without explicit sequencing.
pub struct DsiDbi<'bus, 'd> {
    _bus: &'bus mut MipiDsi<'d>,
    virtual_channel: u8,
}

impl<'bus, 'd> DsiDbi<'bus, 'd> {
    pub(crate) fn new(bus: &'bus mut MipiDsi<'d>, virtual_channel: u8) -> Self {
        let h = MIPI_DSI_HOST::regs();

        // All TX paths use LP mode; disable TE ack; enable cmd ack.
        h.cmd_mode_cfg().modify(|_, w| {
            w.tear_fx_en().clear_bit();
            w.ack_rqst_en().set_bit();
            w.gen_sw_0p_tx().set_bit();
            w.gen_sw_1p_tx().set_bit();
            w.gen_sw_2p_tx().set_bit();
            w.gen_sr_0p_tx().set_bit();
            w.gen_sr_1p_tx().set_bit();
            w.gen_sr_2p_tx().set_bit();
            w.gen_lw_tx().set_bit();
            w.dcs_sw_0p_tx().set_bit();
            w.dcs_sw_1p_tx().set_bit();
            w.dcs_sr_0p_tx().set_bit();
            w.dcs_lw_tx().set_bit();
            w.max_rd_pkt_size().set_bit()
        });

        Self {
            _bus: bus,
            virtual_channel,
        }
    }

    /// Returns the virtual channel ID this interface was configured with.
    pub fn virtual_channel(&self) -> u8 {
        self.virtual_channel
    }

    /// Send a DCS write command with zero or more parameters.
    ///
    /// Uses a short-write packet for 0 or 1 parameters, a long-write packet
    /// otherwise.
    pub fn write_cmd(&mut self, cmd: u8, params: &[u8]) -> Result<(), Error> {
        let h = MIPI_DSI_HOST::regs();
        let vc = self.virtual_channel;
        let payload_size = 1 + params.len(); // cmd byte + params

        if payload_size > u16::MAX as usize {
            return Err(Error::PayloadTooLarge);
        }

        if payload_size > 2 {
            // Long write: push payload into FIFO 4 bytes at a time.
            // First word: [cmd, params[0..3]].
            let merged = params.len().min(3);
            let mut word: u32 = cmd as u32;
            for (i, &b) in params[..merged].iter().enumerate() {
                word |= (b as u32) << (8 * (i + 1));
            }
            while h.cmd_pkt_status().read().gen_pld_w_full().bit_is_set() {}
            h.gen_pld_data().write(|w| unsafe { w.bits(word) });

            let rest = &params[merged..];
            let mut chunks = rest.chunks_exact(4);
            for chunk in chunks.by_ref() {
                let w32 = u32::from_le_bytes(chunk.try_into().unwrap());
                while h.cmd_pkt_status().read().gen_pld_w_full().bit_is_set() {}
                h.gen_pld_data().write(|w| unsafe { w.bits(w32) });
            }
            let tail = chunks.remainder();
            if !tail.is_empty() {
                let mut w32: u32 = 0;
                for (i, &b) in tail.iter().enumerate() {
                    w32 |= (b as u32) << (8 * i);
                }
                while h.cmd_pkt_status().read().gen_pld_w_full().bit_is_set() {}
                h.gen_pld_data().write(|w| unsafe { w.bits(w32) });
            }

            let wc = payload_size as u16;
            while h.cmd_pkt_status().read().gen_cmd_full().bit_is_set() {}
            h.gen_hdr().write(|w| unsafe {
                w.gen_vc().bits(vc);
                w.gen_dt().bits(DT_DCS_LONG_WRITE);
                w.gen_wc_lsbyte().bits((wc & 0xFF) as u8);
                w.gen_wc_msbyte().bits((wc >> 8) as u8)
            });
        } else if payload_size == 2 {
            // Short write with 1 param.
            while h.cmd_pkt_status().read().gen_cmd_full().bit_is_set() {}
            h.gen_hdr().write(|w| unsafe {
                w.gen_vc().bits(vc);
                w.gen_dt().bits(DT_DCS_SHORT_WRITE_1);
                w.gen_wc_lsbyte().bits(cmd);
                w.gen_wc_msbyte().bits(params[0])
            });
        } else {
            // Short write with 0 params.
            while h.cmd_pkt_status().read().gen_cmd_full().bit_is_set() {}
            h.gen_hdr().write(|w| unsafe {
                w.gen_vc().bits(vc);
                w.gen_dt().bits(DT_DCS_SHORT_WRITE_0);
                w.gen_wc_lsbyte().bits(cmd);
                w.gen_wc_msbyte().bits(0)
            });
        }

        Ok(())
    }

    /// Issue a DCS read command (BTA) and drain the response into `out`.
    ///
    /// Returns the number of bytes placed into `out`.
    pub fn read_cmd(&mut self, cmd: u8, out: &mut [u8]) -> Result<usize, Error> {
        let h = MIPI_DSI_HOST::regs();
        let vc = self.virtual_channel;

        if out.len() > u16::MAX as usize {
            return Err(Error::PayloadTooLarge);
        }

        // SET_MAXIMUM_RETURN_PKT_SIZE.
        let max_ret = out.len() as u16;
        while h.cmd_pkt_status().read().gen_cmd_full().bit_is_set() {}
        h.gen_hdr().write(|w| unsafe {
            w.gen_vc().bits(vc);
            w.gen_dt().bits(DT_SET_MAX_RETURN_PKT);
            w.gen_wc_lsbyte().bits((max_ret & 0xFF) as u8);
            w.gen_wc_msbyte().bits((max_ret >> 8) as u8)
        });

        // Ensure command mode is active.
        h.mode_cfg().modify(|_, w| w.cmd_video_mode().set_bit());

        // Enable BTA and set RX virtual channel.
        h.pckhdl_cfg().modify(|_, w| w.bta_en().set_bit());
        h.gen_vcid().modify(|_, w| unsafe { w.rx().bits(vc) });

        // Send DCS READ_0.
        while h.cmd_pkt_status().read().gen_cmd_full().bit_is_set() {}
        h.gen_hdr().write(|w| unsafe {
            w.gen_vc().bits(vc);
            w.gen_dt().bits(DT_DCS_READ_0);
            w.gen_wc_lsbyte().bits(cmd);
            w.gen_wc_msbyte().bits(0)
        });

        // Wait for BTA read to complete.
        while h.cmd_pkt_status().read().gen_rd_cmd_busy().bit_is_set() {}

        // Drain the read FIFO.
        while h.cmd_pkt_status().read().gen_pld_r_empty().bit_is_set() {}
        let mut count = 0usize;
        while !h.cmd_pkt_status().read().gen_pld_r_empty().bit_is_set() {
            let word = h.gen_pld_data().read().bits();
            for i in 0..4 {
                if count < out.len() {
                    out[count] = ((word >> (8 * i)) & 0xFF) as u8;
                    count += 1;
                }
            }
        }

        Ok(count)
    }
}
