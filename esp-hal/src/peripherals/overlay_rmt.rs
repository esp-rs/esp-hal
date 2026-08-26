//! Register-block overlay for ESP32-P4 and ESP32-S31 RMT.
//!
//! The peripheral matches ESP32-S3. The PAC uses different SVD names, so this
//! overlay presents the ESP32-S3 register API. RMT must be `virtual` for this
//! to work.

use core::ops::Deref;

use super::*;
use crate::soc::pac::rmt;

impl RMT<'_> {
    /// Returns a reference to the register block.
    #[inline(always)]
    #[instability::unstable]
    pub const fn regs<'a>() -> &'a RmtRegisterBlock {
        &RmtRegisterBlock
    }

    /// Returns a reference to the register block.
    #[inline(always)]
    #[instability::unstable]
    pub fn register_block(&self) -> &RmtRegisterBlock {
        &RmtRegisterBlock
    }
}

/// Registers block overlay for the RMT peripheral.
#[instability::unstable]
pub struct RmtRegisterBlock;

impl Deref for RmtRegisterBlock {
    type Target = rmt::RegisterBlock;

    fn deref(&self) -> &Self::Target {
        unsafe { &*pac::RMT::ptr() }
    }
}

impl RmtRegisterBlock {
    /// Channel n TX configure register 0.
    #[inline]
    pub fn ch_tx_conf0(&self, n: usize) -> TxConf0<'_> {
        TxConf0(self.chconf0(n))
    }

    /// Channel n RX configure register 0.
    #[inline]
    pub fn ch_rx_conf0(&self, n: usize) -> ChRxConf0 {
        assert!(n < 4);
        ChRxConf0 {
            ptr: unsafe {
                core::ptr::from_ref(Deref::deref(self))
                    .cast::<u8>()
                    .add(0x30)
                    .add(8 * n)
                    .cast::<u32>()
                    .cast_mut()
            },
        }
    }

    /// Channel n RX configure register 1.
    #[inline]
    pub fn ch_rx_conf1(&self, n: usize) -> RxConf1<'_> {
        RxConf1(self.chconf1(n))
    }

    /// Channel n TX status register.
    #[inline]
    pub fn ch_tx_status(&self, n: usize) -> ChStatus<'_> {
        ChStatus(self.chstatus(n))
    }

    /// Channel n RX status register.
    #[inline]
    pub fn ch_rx_status(&self, n: usize) -> ChStatus<'_> {
        cfg_select! {
            esp32s31 => ChStatus(match n {
                0 => self.ch4status(),
                1 => self.ch5status(),
                2 => self.ch6status(),
                3 => self.ch7status(),
                _ => panic!("RMT RX status index must be 0..=3"),
            }),
            esp32p4 => {
                assert!(n < 4);
                ChStatus(unsafe {
                    &*core::ptr::from_ref(Deref::deref(self))
                        .cast::<u8>()
                        .add(0x60)
                        .add(4 * n)
                        .cast()
                })
            }
        }
    }

    /// Channel n TX event configuration register.
    #[inline]
    pub fn ch_tx_lim(&self, n: usize) -> TxLim<'_> {
        TxLim(Deref::deref(self).ch_tx_lim(n))
    }

    /// Channel n RX event configuration register.
    #[inline]
    pub fn ch_rx_lim(&self, n: usize) -> RxLim<'_> {
        RxLim(Deref::deref(self).ch_rx_lim(n))
    }

    /// Channel n carrier duty register.
    #[inline]
    pub fn chcarrier_duty(&self, n: usize) -> CarrierDuty<'_> {
        CarrierDuty(Deref::deref(self).chcarrier_duty(n))
    }

    /// Channel n RX carrier remove register.
    #[inline]
    pub fn ch_rx_carrier_rm(&self, n: usize) -> RxCarrierRm<'_> {
        RxCarrierRm(Deref::deref(self).ch_rx_carrier_rm(n))
    }

    /// Masked interrupt status.
    #[inline]
    pub fn int_st(&self) -> IntSt<'_> {
        IntSt(Deref::deref(self).int_st())
    }

    /// Raw interrupt status.
    #[inline]
    pub fn int_raw(&self) -> IntRaw<'_> {
        IntRaw(Deref::deref(self).int_raw())
    }

    /// Interrupt enable bits.
    #[inline]
    pub fn int_ena(&self) -> IntEna<'_> {
        IntEna(Deref::deref(self).int_ena())
    }

    /// Interrupt clear bits.
    #[inline]
    pub fn int_clr(&self) -> IntClr<'_> {
        IntClr(Deref::deref(self).int_clr())
    }
}

/// Bit flag reader with a PAC-like [`bit`](BitFlag::bit) method
pub struct BitFlag(bool);

impl BitFlag {
    #[inline]
    pub fn bit(self) -> bool {
        self.0
    }
}

/// Field reader with a PAC-like [`bits`](FieldVal::bits) method
pub struct FieldVal<T>(T);

impl<T> FieldVal<T> {
    #[inline]
    pub fn bits(self) -> T {
        self.0
    }
}

/// Bit field writer with PAC-like `bit` / `set_bit` / `clear_bit` methods
pub struct BitProxy<'a, W>(&'a mut W, fn(&mut W, bool));

impl<'a, W> BitProxy<'a, W> {
    #[inline]
    pub fn bit(self, val: bool) -> &'a mut W {
        (self.1)(self.0, val);
        self.0
    }

    #[inline]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }

    #[inline]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
}

/// Multi-bit field writer with a PAC-like `bits` method
pub struct FieldProxy<'a, W, T>(&'a mut W, fn(&mut W, T));

impl<'a, W, T> FieldProxy<'a, W, T> {
    #[inline]
    pub unsafe fn bits(self, val: T) -> &'a mut W {
        (self.1)(self.0, val);
        self.0
    }
}

fn overlay_w<P, O>(w: &mut P) -> &mut O {
    unsafe { &mut *core::ptr::from_mut(w).cast() }
}

macro_rules! bit_proxy {
    ($self:ident, $s31:ident, $p4:ident) => {
        BitProxy($self, |w, val| {
            #[cfg(esp32s31)]
            {
                w.0.$s31().bit(val);
            }
            #[cfg(esp32p4)]
            {
                w.0.$p4().bit(val);
            }
        })
    };
}

macro_rules! field_proxy {
    ($self:ident, $s31:ident, $p4:ident) => {
        FieldProxy($self, |w, val| {
            #[cfg(esp32s31)]
            {
                let _ = unsafe { w.0.$s31().bits(val) };
            }
            #[cfg(esp32p4)]
            {
                let _ = unsafe { w.0.$p4().bits(val) };
            }
        })
    };
}

/// Channel TX configure register 0.
pub struct TxConf0<'a>(&'a rmt::CHCONF0);

/// Reader for [`TxConf0`]
pub struct TxConf0R(u32);

/// Writer for [`TxConf0`]
#[repr(transparent)]
pub struct TxConf0W(rmt::chconf0::W);

impl TxConf0<'_> {
    /// Reads the register.
    #[inline]
    pub fn read(&self) -> TxConf0R {
        TxConf0R(self.0.read().bits())
    }

    /// Modifies the register.
    #[inline]
    pub fn modify<F>(&self, f: F)
    where
        F: for<'a> FnOnce(&TxConf0R, &'a mut TxConf0W) -> &'a mut TxConf0W,
    {
        self.0.modify(|r, w| {
            let overlay_r = TxConf0R(r.bits());
            let _ = f(&overlay_r, overlay_w(w));
            w
        });
    }
}

impl TxConf0R {
    #[inline]
    pub fn mem_size(&self) -> FieldVal<u8> {
        FieldVal(((self.0 >> 16) & 0x0f) as u8)
    }

    #[inline]
    pub fn idle_out_en(&self) -> BitFlag {
        BitFlag((self.0 >> 6) & 1 != 0)
    }

    #[inline]
    pub fn idle_out_lv(&self) -> BitFlag {
        BitFlag((self.0 >> 5) & 1 != 0)
    }
}

impl TxConf0W {
    #[inline]
    pub fn conf_update(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, conf_update_ch, conf_update_ch0)
    }

    #[inline]
    pub fn div_cnt(&mut self) -> FieldProxy<'_, Self, u8> {
        field_proxy!(self, div_cnt_ch, div_cnt_ch0)
    }

    #[inline]
    pub fn mem_size(&mut self) -> FieldProxy<'_, Self, u8> {
        field_proxy!(self, mem_size_ch, mem_size_ch0)
    }

    #[inline]
    pub fn tx_conti_mode(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, tx_conti_mode_ch, tx_conti_mode_ch0)
    }

    #[inline]
    pub fn mem_tx_wrap_en(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, mem_tx_wrap_en_ch, mem_tx_wrap_en_ch0)
    }

    #[inline]
    pub fn carrier_en(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, carrier_en_ch, carrier_en_ch0)
    }

    #[inline]
    pub fn carrier_eff_en(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, carrier_eff_en_ch, carrier_eff_en_ch0)
    }

    #[inline]
    pub fn carrier_out_lv(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, carrier_out_lv_ch, carrier_out_lv_ch0)
    }

    #[inline]
    pub fn idle_out_en(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, idle_out_en_ch, idle_out_en_ch0)
    }

    #[inline]
    pub fn idle_out_lv(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, idle_out_lv_ch, idle_out_lv_ch0)
    }

    #[inline]
    pub fn mem_rd_rst(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, mem_rd_rst_ch, mem_rd_rst_ch0)
    }

    #[inline]
    pub fn apb_mem_rst(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, apb_mem_rst_ch, apb_mem_rst_ch0)
    }

    #[inline]
    pub fn tx_start(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, tx_start_ch, tx_start_ch0)
    }

    #[inline]
    pub fn tx_stop(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, tx_stop_ch, tx_stop_ch0)
    }
}

/// RX CONF0 layout (ESP32-S3 `CH_RX_CONF0`). The PAC aliases this register to TX CONF0.
pub struct ChRxConf0 {
    ptr: *mut u32,
}

/// Reader for [`ChRxConf0`]
pub struct ChRxConf0R(u32);

/// Writer for [`ChRxConf0`]
pub struct ChRxConf0W(u32);

/// Bit writer for [`ChRxConf0`]
pub struct RxBit<'a> {
    w: &'a mut ChRxConf0W,
    bit: u8,
}

/// Field writer for [`ChRxConf0`]
pub struct RxField<'a> {
    w: &'a mut ChRxConf0W,
    offset: u8,
    width: u8,
}

impl ChRxConf0R {
    #[inline]
    pub fn mem_size(&self) -> FieldVal<u8> {
        FieldVal(((self.0 >> 24) & 0x0f) as u8)
    }
}

impl ChRxConf0W {
    #[inline]
    pub fn div_cnt(&mut self) -> RxField<'_> {
        RxField {
            w: self,
            offset: 0,
            width: 8,
        }
    }

    #[inline]
    pub fn idle_thres(&mut self) -> RxField<'_> {
        RxField {
            w: self,
            offset: 8,
            width: 15,
        }
    }

    #[inline]
    pub fn mem_size(&mut self) -> RxField<'_> {
        RxField {
            w: self,
            offset: 24,
            width: 4,
        }
    }

    #[inline]
    pub fn carrier_en(&mut self) -> RxBit<'_> {
        RxBit { w: self, bit: 28 }
    }

    #[inline]
    pub fn carrier_out_lv(&mut self) -> RxBit<'_> {
        RxBit { w: self, bit: 29 }
    }
}

impl<'a> RxBit<'a> {
    #[inline]
    pub fn bit(self, val: bool) -> &'a mut ChRxConf0W {
        if val {
            self.w.0 |= 1 << self.bit;
        } else {
            self.w.0 &= !(1 << self.bit);
        }
        self.w
    }
}

impl<'a> RxField<'a> {
    #[inline]
    pub unsafe fn bits(self, val: impl Into<u16>) -> &'a mut ChRxConf0W {
        let mask = (1u32 << self.width) - 1;
        self.w.0 &= !(mask << self.offset);
        self.w.0 |= (u32::from(val.into()) & mask) << self.offset;
        self.w
    }
}

impl ChRxConf0 {
    #[inline]
    pub fn read(&self) -> ChRxConf0R {
        ChRxConf0R(unsafe { self.ptr.read_volatile() })
    }

    #[inline]
    pub fn modify<F>(&self, f: F)
    where
        F: for<'a> FnOnce(&ChRxConf0R, &'a mut ChRxConf0W) -> &'a mut ChRxConf0W,
    {
        let bits = unsafe { self.ptr.read_volatile() };
        let r = ChRxConf0R(bits);
        let mut w = ChRxConf0W(bits);
        let _ = f(&r, &mut w);
        unsafe { self.ptr.write_volatile(w.0) };
    }
}

/// Channel RX configure register 1.
pub struct RxConf1<'a>(&'a rmt::CHCONF1);

/// Writer for [`RxConf1`]
#[repr(transparent)]
pub struct RxConf1W(rmt::chconf1::W);

impl RxConf1<'_> {
    #[inline]
    pub fn modify<F>(&self, f: F)
    where
        F: for<'a> FnOnce(&(), &'a mut RxConf1W) -> &'a mut RxConf1W,
    {
        self.0.modify(|_, w| {
            let _ = f(&(), overlay_w(w));
            w
        });
    }
}

impl RxConf1W {
    #[inline]
    pub fn conf_update(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, conf_update_ch, conf_update_ch4)
    }

    #[inline]
    pub fn mem_rx_wrap_en(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, mem_rx_wrap_en_ch, mem_rx_wrap_en_ch4)
    }

    #[inline]
    pub fn mem_owner(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, mem_owner_ch, mem_owner_ch4)
    }

    #[inline]
    pub fn mem_wr_rst(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, mem_wr_rst_ch, mem_wr_rst_ch4)
    }

    #[inline]
    pub fn apb_mem_rst(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, apb_mem_rst_ch, apb_mem_rst_ch4)
    }

    #[inline]
    pub fn rx_en(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, rx_en_ch, rx_en_ch4)
    }

    #[inline]
    pub fn rx_filter_en(&mut self) -> BitProxy<'_, Self> {
        bit_proxy!(self, rx_filter_en_ch, rx_filter_en_ch4)
    }

    #[inline]
    pub fn rx_filter_thres(&mut self) -> FieldProxy<'_, Self, u8> {
        field_proxy!(self, rx_filter_thres_ch, rx_filter_thres_ch4)
    }
}

/// Channel TX/RX status register.
pub struct ChStatus<'a>(&'a rmt::CHSTATUS);

/// Reader for [`ChStatus`]
pub struct StatusR(u32);

impl ChStatus<'_> {
    #[inline]
    pub fn read(&self) -> StatusR {
        StatusR(self.0.read().bits())
    }
}

impl StatusR {
    #[inline]
    pub fn mem_raddr_ex(&self) -> FieldVal<u16> {
        FieldVal((self.0 & 0x03ff) as u16)
    }

    #[inline]
    pub fn mem_waddr_ex(&self) -> FieldVal<u16> {
        self.mem_raddr_ex()
    }
}

/// Channel TX limit register.
pub struct TxLim<'a>(&'a rmt::CH_TX_LIM);

/// Writer for [`TxLim`]
#[repr(transparent)]
pub struct TxLimW(rmt::ch_tx_lim::W);

impl TxLim<'_> {
    #[inline]
    pub fn modify<F>(&self, f: F)
    where
        F: for<'a> FnOnce(&(), &'a mut TxLimW) -> &'a mut TxLimW,
    {
        self.0.modify(|_, w| {
            let _ = f(&(), overlay_w(w));
            w
        });
    }
}

impl TxLimW {
    #[inline]
    pub fn loop_count_reset(&mut self) -> BitProxy<'_, Self> {
        BitProxy(self, |w, val| {
            w.0.loop_count_reset_ch().bit(val);
        })
    }

    #[inline]
    pub fn tx_loop_cnt_en(&mut self) -> BitProxy<'_, Self> {
        BitProxy(self, |w, val| {
            w.0.tx_loop_cnt_en_ch().bit(val);
        })
    }

    #[inline]
    pub fn tx_loop_num(&mut self) -> FieldProxy<'_, Self, u16> {
        FieldProxy(self, |w, val| {
            let _ = unsafe { w.0.tx_loop_num_ch().bits(val) };
        })
    }

    #[inline]
    pub fn loop_stop_en(&mut self) -> BitProxy<'_, Self> {
        BitProxy(self, |w, val| {
            w.0.loop_stop_en_ch().bit(val);
        })
    }

    #[inline]
    pub fn tx_lim(&mut self) -> FieldProxy<'_, Self, u16> {
        FieldProxy(self, |w, val| {
            let _ = unsafe { w.0.tx_lim_ch().bits(val) };
        })
    }
}

/// Channel RX limit register.
pub struct RxLim<'a>(&'a rmt::CH_RX_LIM);

/// Writer for [`RxLim`]
#[repr(transparent)]
pub struct RxLimW(rmt::ch_rx_lim::W);

impl RxLim<'_> {
    #[inline]
    pub fn modify<F>(&self, f: F)
    where
        F: for<'a> FnOnce(&(), &'a mut RxLimW) -> &'a mut RxLimW,
    {
        self.0.modify(|_, w| {
            let _ = f(&(), overlay_w(w));
            w
        });
    }
}

impl RxLimW {
    #[inline]
    pub fn rx_lim(&mut self) -> FieldProxy<'_, Self, u16> {
        field_proxy!(self, rx_lim_ch, rx_lim_ch4)
    }
}

/// Channel carrier duty register.
pub struct CarrierDuty<'a>(&'a rmt::CHCARRIER_DUTY);

/// Writer for [`CarrierDuty`]
#[repr(transparent)]
pub struct CarrierDutyW(rmt::chcarrier_duty::W);

impl CarrierDuty<'_> {
    #[inline]
    pub fn write<F>(&self, f: F)
    where
        F: for<'a> FnOnce(&'a mut CarrierDutyW) -> &'a mut CarrierDutyW,
    {
        self.0.write(|w| {
            let _ = f(overlay_w(w));
            w
        });
    }
}

impl CarrierDutyW {
    #[inline]
    pub fn carrier_high(&mut self) -> FieldProxy<'_, Self, u16> {
        FieldProxy(self, |w, val| {
            let _ = unsafe { w.0.carrier_high_ch().bits(val) };
        })
    }

    #[inline]
    pub fn carrier_low(&mut self) -> FieldProxy<'_, Self, u16> {
        FieldProxy(self, |w, val| {
            let _ = unsafe { w.0.carrier_low_ch().bits(val) };
        })
    }
}

/// Channel RX carrier remove register.
pub struct RxCarrierRm<'a>(&'a rmt::CH_RX_CARRIER_RM);

/// Writer for [`RxCarrierRm`]
#[repr(transparent)]
pub struct RxCarrierRmW(rmt::ch_rx_carrier_rm::W);

impl RxCarrierRm<'_> {
    #[inline]
    pub fn write<F>(&self, f: F)
    where
        F: for<'a> FnOnce(&'a mut RxCarrierRmW) -> &'a mut RxCarrierRmW,
    {
        self.0.write(|w| {
            let _ = f(overlay_w(w));
            w
        });
    }
}

impl RxCarrierRmW {
    #[inline]
    pub fn carrier_high_thres(&mut self) -> FieldProxy<'_, Self, u16> {
        FieldProxy(self, |w, val| {
            let _ = unsafe { w.0.carrier_high_thres_ch().bits(val) };
        })
    }

    #[inline]
    pub fn carrier_low_thres(&mut self) -> FieldProxy<'_, Self, u16> {
        FieldProxy(self, |w, val| {
            let _ = unsafe { w.0.carrier_low_thres_ch().bits(val) };
        })
    }
}

/// Masked interrupt status.
pub struct IntSt<'a>(&'a rmt::INT_ST);

/// Raw interrupt status.
pub struct IntRaw<'a>(&'a rmt::INT_RAW);

/// Interrupt status reader.
pub struct IntR(u32);

impl IntSt<'_> {
    #[inline]
    pub fn read(&self) -> IntR {
        IntR(self.0.read().bits())
    }
}

impl IntRaw<'_> {
    #[inline]
    pub fn read(&self) -> IntR {
        IntR(self.0.read().bits())
    }
}

impl IntR {
    #[inline]
    pub fn ch_tx_end(&self, n: u8) -> BitFlag {
        BitFlag((self.0 >> n) & 1 != 0)
    }

    #[inline]
    pub fn ch_tx_err(&self, n: u8) -> BitFlag {
        BitFlag((self.0 >> (n + 4)) & 1 != 0)
    }

    #[inline]
    pub fn ch_tx_thr_event(&self, n: u8) -> BitFlag {
        BitFlag((self.0 >> (n + 8)) & 1 != 0)
    }

    #[inline]
    pub fn ch_tx_loop(&self, n: u8) -> BitFlag {
        BitFlag((self.0 >> (n + 12)) & 1 != 0)
    }

    #[inline]
    pub fn ch_rx_end(&self, n: u8) -> BitFlag {
        BitFlag((self.0 >> (n + 16)) & 1 != 0)
    }

    #[inline]
    pub fn ch_rx_err(&self, n: u8) -> BitFlag {
        BitFlag((self.0 >> (n + 20)) & 1 != 0)
    }

    #[inline]
    pub fn ch_rx_thr_event(&self, n: u8) -> BitFlag {
        BitFlag((self.0 >> (n + 24)) & 1 != 0)
    }
}

/// Interrupt enable register.
pub struct IntEna<'a>(&'a rmt::INT_ENA);

/// Writer for [`IntEna`]
#[repr(transparent)]
pub struct IntEnaW(rmt::int_ena::W);

/// Interrupt clear register.
pub struct IntClr<'a>(&'a rmt::INT_CLR);

/// Writer for [`IntClr`]
#[repr(transparent)]
pub struct IntClrW(rmt::int_clr::W);

/// Clustered interrupt bit writer.
pub struct IntBitW<'a, W> {
    w: &'a mut W,
    n: u8,
    set: fn(&mut W, u8, bool),
}

impl<'a, W> IntBitW<'a, W> {
    #[inline]
    pub fn bit(self, val: bool) -> &'a mut W {
        (self.set)(self.w, self.n, val);
        self.w
    }
}

impl IntEna<'_> {
    #[inline]
    pub fn modify<F>(&self, f: F)
    where
        F: for<'a> FnOnce(&(), &'a mut IntEnaW) -> &'a mut IntEnaW,
    {
        self.0.modify(|_, w| {
            let _ = f(&(), overlay_w(w));
            w
        });
    }
}

impl IntClr<'_> {
    #[inline]
    pub fn write<F>(&self, f: F)
    where
        F: for<'a> FnOnce(&'a mut IntClrW) -> &'a mut IntClrW,
    {
        self.0.write(|w| {
            let _ = f(overlay_w(w));
            w
        });
    }
}

macro_rules! set_tx_int {
    ($w:expr, $n:expr, $val:expr, $p4:ident, $s31:ident) => {
        paste::paste! {
            match $n {
                0 => {
                    #[cfg(esp32p4)]
                    {
                        $w.[<ch0_ $p4>]().bit($val);
                    }
                    #[cfg(esp32s31)]
                    {
                        $w.[<ch0_ $s31>]().bit($val);
                    }
                }
                1 => {
                    #[cfg(esp32p4)]
                    {
                        $w.[<ch1_ $p4>]().bit($val);
                    }
                    #[cfg(esp32s31)]
                    {
                        $w.[<ch1_ $s31>]().bit($val);
                    }
                }
                2 => {
                    #[cfg(esp32p4)]
                    {
                        $w.[<ch2_ $p4>]().bit($val);
                    }
                    #[cfg(esp32s31)]
                    {
                        $w.[<ch2_ $s31>]().bit($val);
                    }
                }
                3 => {
                    #[cfg(esp32p4)]
                    {
                        $w.[<ch3_ $p4>]().bit($val);
                    }
                    #[cfg(esp32s31)]
                    {
                        $w.[<ch3_ $s31>]().bit($val);
                    }
                }
                _ => unreachable!(),
            }
        }
    };
}

macro_rules! set_rx_int {
    ($w:expr, $n:expr, $val:expr, $p4:ident, $s31:ident) => {
        paste::paste! {
            match $n {
                0 => {
                    #[cfg(esp32p4)]
                    {
                        $w.[<ch4_ $p4>]().bit($val);
                    }
                    #[cfg(esp32s31)]
                    {
                        $w.[<ch4_ $s31>]().bit($val);
                    }
                }
                1 => {
                    #[cfg(esp32p4)]
                    {
                        $w.[<ch5_ $p4>]().bit($val);
                    }
                    #[cfg(esp32s31)]
                    {
                        $w.[<ch5_ $s31>]().bit($val);
                    }
                }
                2 => {
                    #[cfg(esp32p4)]
                    {
                        $w.[<ch6_ $p4>]().bit($val);
                    }
                    #[cfg(esp32s31)]
                    {
                        $w.[<ch6_ $s31>]().bit($val);
                    }
                }
                3 => {
                    #[cfg(esp32p4)]
                    {
                        $w.[<ch7_ $p4>]().bit($val);
                    }
                    #[cfg(esp32s31)]
                    {
                        $w.[<ch7_ $s31>]().bit($val);
                    }
                }
                _ => unreachable!(),
            }
        }
    };
}

macro_rules! impl_int_w {
    ($ty:ty, $tx_end_p4:ident, $tx_end_s31:ident, $tx_err_p4:ident, $tx_err_s31:ident, $tx_thr_p4:ident, $tx_thr_s31:ident, $tx_loop_p4:ident, $tx_loop_s31:ident, $rx_end_p4:ident, $rx_end_s31:ident, $rx_err_p4:ident, $rx_err_s31:ident, $rx_thr_p4:ident, $rx_thr_s31:ident) => {
        impl $ty {
            #[inline]
            pub fn ch_tx_end(&mut self, n: u8) -> IntBitW<'_, Self> {
                IntBitW {
                    w: self,
                    n,
                    set: |w, n, val| set_tx_int!(w.0, n, val, $tx_end_p4, $tx_end_s31),
                }
            }

            #[inline]
            pub fn ch_tx_err(&mut self, n: u8) -> IntBitW<'_, Self> {
                IntBitW {
                    w: self,
                    n,
                    set: |w, n, val| set_tx_int!(w.0, n, val, $tx_err_p4, $tx_err_s31),
                }
            }

            #[inline]
            pub fn ch_tx_thr_event(&mut self, n: u8) -> IntBitW<'_, Self> {
                IntBitW {
                    w: self,
                    n,
                    set: |w, n, val| set_tx_int!(w.0, n, val, $tx_thr_p4, $tx_thr_s31),
                }
            }

            #[inline]
            pub fn ch_tx_loop(&mut self, n: u8) -> IntBitW<'_, Self> {
                IntBitW {
                    w: self,
                    n,
                    set: |w, n, val| set_tx_int!(w.0, n, val, $tx_loop_p4, $tx_loop_s31),
                }
            }

            #[inline]
            pub fn ch_rx_end(&mut self, n: u8) -> IntBitW<'_, Self> {
                IntBitW {
                    w: self,
                    n,
                    set: |w, n, val| set_rx_int!(w.0, n, val, $rx_end_p4, $rx_end_s31),
                }
            }

            #[inline]
            pub fn ch_rx_err(&mut self, n: u8) -> IntBitW<'_, Self> {
                IntBitW {
                    w: self,
                    n,
                    set: |w, n, val| set_rx_int!(w.0, n, val, $rx_err_p4, $rx_err_s31),
                }
            }

            #[inline]
            pub fn ch_rx_thr_event(&mut self, n: u8) -> IntBitW<'_, Self> {
                IntBitW {
                    w: self,
                    n,
                    set: |w, n, val| set_rx_int!(w.0, n, val, $rx_thr_p4, $rx_thr_s31),
                }
            }
        }
    };
}

impl_int_w!(
    IntEnaW,
    tx_end,
    tx_end_int_ena,
    err,
    err_int_ena,
    tx_thr_event,
    tx_thr_event_int_ena,
    tx_loop,
    tx_loop_int_ena,
    rx_end,
    rx_end_int_ena,
    err,
    err_int_ena,
    rx_thr_event,
    rx_thr_event_int_ena
);

impl_int_w!(
    IntClrW,
    tx_end,
    tx_end_int_clr,
    err,
    err_int_clr,
    tx_thr_event,
    tx_thr_event_int_clr,
    tx_loop,
    tx_loop_int_clr,
    rx_end,
    rx_end_int_clr,
    err,
    err_int_clr,
    rx_thr_event,
    rx_thr_event_int_clr
);
