use core::marker::PhantomData;

use crate::{
    Async,
    Blocking,
    DriverMode,
    handler,
    interrupt::{self, InterruptConfigurable, InterruptHandler},
    peripherals::{EMAC_DMA, EMAC_EXT, EMAC_MAC, Interrupt},
    system::{self, Cpu, GenericPeripheralGuard},
};

pub enum Error {
    Timeout,
    InvalidArgument,
    NotFound,
    Full,
    InvalidState,
}

pub struct Rmii<'d, Dm: DriverMode> {
    emac_dma: EMAC_DMA<'d>,
    emac_ext: EMAC_EXT<'d>,
    emac_mac: EMAC_MAC<'d>,
    _mode: PhantomData<Dm>,
    //_guard: GenericPeripheralGuard<{ system::Peripheral::Emac as u8 }>,
}

impl crate::private::Sealed for Rmii<'_, Async> {}
impl crate::private::Sealed for Rmii<'_, Blocking> {}

impl<'d> Rmii<'d, Blocking> {
    pub fn new(emac_dma: EMAC_DMA<'d>, emac_ext: EMAC_EXT<'d>, emac_mac: EMAC_MAC<'d>) -> Self {
        // let _guard = GenericPeripheralGuard::new();

        // mask all interrupts
        EMAC_MAC::regs()
            .emacintmask()
            .write(|w| unsafe { w.bits(u32::MAX) });

        EMAC_MAC::regs().emacconfig().write(|w| {
            // Enable the watchdog on the receiver, frame longer than 2048 Bytes is not allowed
            w.watchdog().clear_bit();
            // Enable the jabber timer on the transmitter, frame longer than 2048 Bytes is not
            // allowed
            w.jabber().clear_bit();
            // minimum IFG between frames during transmission is 96 bit times
            unsafe { w.interframegap().bits(InterFrameGapBits::_96 as _) };
            // Enable Carrier Sense During Transmission
            w.disablecrs().clear_bit();
            // Select speed: port: 10/100 Mbps, here set default 100M, afterwards, will reset by
            // auto-negotiation
            w.mii().set_bit();
            w.fespeed().set_bit();
            // Allow the reception of frames when the TX_EN signal is asserted in Half-Duplex mode
            w.rxown().clear_bit();
            // Disable internal loopback mode
            w.loopback().clear_bit();
            // Select duplex mode: here set default full duplex, afterwards, will reset by
            // auto-negotiation
            w.duplex().set_bit();
            // Select the checksum mode for received frame payload's TCP/UDP/ICMP headers
            w.rxipcoffload().set_bit();
            // Enable MAC retry transmission when a collision occurs in half duplex mode
            w.retry().clear_bit();
            // MAC passes all incoming frames to host, without modifying them
            w.padcrcstrip().clear_bit();
            // Set Back-Off limit time before retry a transmission after a collision
            unsafe { w.backofflimit().bits(BackoffLimit::_10 as _) };
            // Disable deferral check, MAC defers until the CRS signal goes inactive
            w.deferralcheck().clear_bit();
            // Set preamble length 7 Bytes
            unsafe { w.pltf().bits(PreambleLengthBytes::_7 as _) }
        });

        EMAC_MAC::regs().emacff().write(|w| {
            // Receiver module passes only those frames to the Application that pass the SA or DA
            // address filter
            w.receive_all().clear_bit();
            // Disable source address filter
            w.safe().clear_bit();
            w.saif().clear_bit();
            // MAC blocks all control frames
            unsafe { w.pcf().bits(ControlFrameMode::PassFilter as _) };
            // AFM module passes all received broadcast frames
            w.dbf().clear_bit();
            // Address Check block operates in normal filtering mode for the DA address
            w.daif().clear_bit();
            // Disable Promiscuous Mode
            w.pmode().clear_bit()
        });

        EMAC_DMA::regs().dmaoperation_mode().write(|w| {
            // Enable Dropping of TCP/IP Checksum Error Frames
            w.dis_drop_tcpip_err_fram().clear_bit();
            // Enable Receive Store Forward
            w.rx_store_forward().set_bit();
            w.dis_flush_recv_frames().clear_bit();
            w.tx_str_fwd().clear_bit()
        });
        Self::flush_tx_fifo();
        EMAC_DMA::regs().dmaoperation_mode().write(|w| {
            unsafe { w.tx_thresh_ctrl().bits(TxThresholdControl::_64 as _) };
            w.fwd_err_frame().clear_bit();
            w.fwd_under_gf().clear_bit();
            unsafe { w.rx_thresh_ctrl().bits(RxThresholdControl::_64 as _) };
            w.opt_second_frame().set_bit()
        });

        EMAC_DMA::regs().dmabusmode().write(|w| {
            w.dmamixedburst().set_bit();
            w.dmaaddralibea().set_bit();
            w.use_sep_pbl().clear_bit();
            unsafe { w.prog_burst_len().bits(32) };
            w.alt_desc_size().set_bit();
            unsafe { w.desc_skip_len().bits(0) };
            w.dma_arb_sch().clear_bit();
            unsafe { w.pri_ratio().bits(ArbitrationRoundRobin::RxTx1_1 as _) }
        });

        Self {
            emac_dma,
            emac_ext,
            emac_mac,
            _mode: PhantomData,
            //_guard,
        }
    }

    pub fn start(&mut self) {
        let mut temp_mask = EMAC_DMA::regs().dmain_en().read().bits();
        temp_mask |= EMAC_LL_CONFIG_ENABLE_INTR_MASK;
        EMAC_DMA::regs()
            .dmain_en()
            .write(|w| unsafe { w.bits(temp_mask) });
        EMAC_DMA::regs()
            .dmastatus()
            .write(|w| unsafe { w.bits(0xffffffff) });

        EMAC_MAC::regs().emacconfig().write(|w| w.tx().set_bit());

        EMAC_DMA::regs().dmaoperation_mode().write(|w| {
            w.start_stop_transmission_command().set_bit();
            w.start_stop_rx().set_bit()
        });

        EMAC_MAC::regs().emacconfig().write(|w| w.rx().set_bit());
    }

    pub fn stop(&mut self) -> Result<(), Error> {
        EMAC_DMA::regs()
            .dmaoperation_mode()
            .write(|w| w.start_stop_transmission_command().clear_bit());

        if EMAC_MAC::regs().emacdebug().read().mactfcs().bits() != 0 {
            return Err(Error::InvalidState);
        }

        EMAC_MAC::regs().emacconfig().write(|w| {
            w.rx().clear_bit();
            w.tx().clear_bit()
        });

        if EMAC_MAC::regs().emacdebug().read().mtlrfrcs().bits() != 0 {
            return Err(Error::InvalidState);
        }

        EMAC_DMA::regs()
            .dmaoperation_mode()
            .write(|w| w.start_stop_rx().clear_bit());

        Self::flush_tx_fifo();

        EMAC_DMA::regs().dmain_en().write(|w| unsafe { w.bits(0) });

        Ok(())
    }

    pub fn set_mac(&mut self, mac_addr: [u8; 6]) -> Result<(), Error> {
        // Make sure mac address is unicast type
        if mac_addr[0] & 1 == 1 {
            return Err(Error::InvalidArgument);
        }

        EMAC_MAC::regs()
            .emacaddr0high()
            .write(|w| unsafe { w.address0_hi().bits(mac_to_bits_high(mac_addr)) });

        EMAC_MAC::regs()
            .emacaddr0low()
            .write(|w| unsafe { w.bits(mac_to_bits_low(mac_addr)) });

        Ok(())
    }

    unsafe fn phy_cmd(&mut self, phy_addr: u8, phy_reg: u8, write: bool) {
        EMAC_MAC::regs().emacgmiiaddr().write(|w| {
            unsafe { w.miidev().bits(phy_addr) };
            unsafe { w.miireg().bits(phy_reg) };
            w.miiwrite().bit(write);
            w.miibusy().set_bit()
        });
    }

    fn add_addr_da_filter(addr_num: MacAddr, mac_addr: [u8; 6]) {
        match addr_num {
            MacAddr::MacAddr1 => {
                EMAC_MAC::regs().emacaddr1high().write(|w| {
                    unsafe {
                        w.mac_address1_hi().bits(mac_to_bits_high(mac_addr));
                        w.mask_byte_control().bits(0);
                    }
                    w.source_address().clear_bit();
                    w.address_enable1().set_bit()
                });

                EMAC_MAC::regs()
                    .emacaddr1low()
                    .write(|w| unsafe { w.bits(mac_to_bits_low(mac_addr)) });
            }

            MacAddr::MacAddr2 => {
                EMAC_MAC::regs().emacaddr2high().write(|w| {
                    unsafe {
                        w.mac_address2_hi().bits(mac_to_bits_high(mac_addr));
                        w.mask_byte_control2().bits(0);
                    }
                    w.source_address2().clear_bit();
                    w.address_enable2().set_bit()
                });

                EMAC_MAC::regs()
                    .emacaddr2low()
                    .write(|w| unsafe { w.bits(mac_to_bits_low(mac_addr)) });
            }

            MacAddr::MacAddr3 => {
                EMAC_MAC::regs().emacaddr3high().write(|w| {
                    unsafe {
                        w.mac_address3_hi().bits(mac_to_bits_high(mac_addr));
                        w.mask_byte_control3().bits(0);
                    }
                    w.source_address3().clear_bit();
                    w.address_enable3().set_bit()
                });

                EMAC_MAC::regs()
                    .emacaddr3low()
                    .write(|w| unsafe { w.bits(mac_to_bits_low(mac_addr)) });
            }

            MacAddr::MacAddr4 => {
                EMAC_MAC::regs().emacaddr4high().write(|w| {
                    unsafe {
                        w.mac_address4_hi().bits(mac_to_bits_high(mac_addr));
                        w.mask_byte_control4().bits(0);
                    }
                    w.source_address4().clear_bit();
                    w.address_enable4().set_bit()
                });

                EMAC_MAC::regs()
                    .emacaddr4low()
                    .write(|w| unsafe { w.bits(mac_to_bits_low(mac_addr)) });
            }

            MacAddr::MacAddr5 => {
                EMAC_MAC::regs().emacaddr5high().write(|w| {
                    unsafe {
                        w.mac_address5_hi().bits(mac_to_bits_high(mac_addr));
                        w.mask_byte_control5().bits(0);
                    }
                    w.source_address5().clear_bit();
                    w.address_enable5().set_bit()
                });

                EMAC_MAC::regs()
                    .emacaddr5low()
                    .write(|w| unsafe { w.bits(mac_to_bits_low(mac_addr)) });
            }

            MacAddr::MacAddr6 => {
                EMAC_MAC::regs().emacaddr6high().write(|w| {
                    unsafe {
                        w.mac_address6_hi().bits(mac_to_bits_high(mac_addr));
                        w.mask_byte_control6().bits(0);
                    }
                    w.source_address6().clear_bit();
                    w.address_enable6().set_bit()
                });

                EMAC_MAC::regs()
                    .emacaddr6low()
                    .write(|w| unsafe { w.bits(mac_to_bits_low(mac_addr)) });
            }

            MacAddr::MacAddr7 => {
                EMAC_MAC::regs().emacaddr7high().write(|w| {
                    unsafe {
                        w.mac_address7_hi().bits(mac_to_bits_high(mac_addr));
                        w.mask_byte_control7().bits(0);
                    }
                    w.source_address7().clear_bit();
                    w.address_enable7().set_bit()
                });

                EMAC_MAC::regs()
                    .emacaddr7low()
                    .write(|w| unsafe { w.bits(mac_to_bits_low(mac_addr)) });
            }
        }
    }

    // TODO are `filter_for_source` & `mask` actually used anywhere?
    fn get_addr_da_filter(
        addr_num: MacAddr,
        mask: Option<&mut u8>,
        filter_for_source: Option<&mut bool>,
    ) -> Result<[u8; 6], Error> {
        let mut ret = [0; 6];
        match addr_num {
            MacAddr::MacAddr1 => {
                if EMAC_MAC::regs()
                    .emacaddr1high()
                    .read()
                    .address_enable1()
                    .bit_is_set()
                {
                    bits_to_mac_high(
                        EMAC_MAC::regs()
                            .emacaddr1high()
                            .read()
                            .mac_address1_hi()
                            .bits(),
                        &mut ret,
                    );

                    bits_to_mac_low(EMAC_MAC::regs().emacaddr1low().read().bits(), &mut ret);

                    if let Some(mask) = mask {
                        *mask = EMAC_MAC::regs()
                            .emacaddr1high()
                            .read()
                            .mask_byte_control()
                            .bits();
                    }

                    if let Some(filter_for_source) = filter_for_source {
                        *filter_for_source = EMAC_MAC::regs()
                            .emacaddr1high()
                            .read()
                            .source_address()
                            .bit_is_set();
                    }

                    return Ok(ret);
                }
            }

            MacAddr::MacAddr2 => {
                if EMAC_MAC::regs()
                    .emacaddr2high()
                    .read()
                    .address_enable2()
                    .bit_is_set()
                {
                    bits_to_mac_high(
                        EMAC_MAC::regs()
                            .emacaddr2high()
                            .read()
                            .mac_address2_hi()
                            .bits(),
                        &mut ret,
                    );

                    bits_to_mac_low(EMAC_MAC::regs().emacaddr2low().read().bits(), &mut ret);

                    if let Some(mask) = mask {
                        *mask = EMAC_MAC::regs()
                            .emacaddr2high()
                            .read()
                            .mask_byte_control2()
                            .bits();
                    }

                    if let Some(filter_for_source) = filter_for_source {
                        *filter_for_source = EMAC_MAC::regs()
                            .emacaddr2high()
                            .read()
                            .source_address2()
                            .bit_is_set();
                    }

                    return Ok(ret);
                }
            }

            MacAddr::MacAddr3 => {
                if EMAC_MAC::regs()
                    .emacaddr3high()
                    .read()
                    .address_enable3()
                    .bit_is_set()
                {
                    bits_to_mac_high(
                        EMAC_MAC::regs()
                            .emacaddr3high()
                            .read()
                            .mac_address3_hi()
                            .bits(),
                        &mut ret,
                    );

                    bits_to_mac_low(EMAC_MAC::regs().emacaddr3low().read().bits(), &mut ret);

                    if let Some(mask) = mask {
                        *mask = EMAC_MAC::regs()
                            .emacaddr3high()
                            .read()
                            .mask_byte_control3()
                            .bits();
                    }

                    if let Some(filter_for_source) = filter_for_source {
                        *filter_for_source = EMAC_MAC::regs()
                            .emacaddr3high()
                            .read()
                            .source_address3()
                            .bit_is_set();
                    }

                    return Ok(ret);
                }
            }

            MacAddr::MacAddr4 => {
                if EMAC_MAC::regs()
                    .emacaddr4high()
                    .read()
                    .address_enable4()
                    .bit_is_set()
                {
                    bits_to_mac_high(
                        EMAC_MAC::regs()
                            .emacaddr4high()
                            .read()
                            .mac_address4_hi()
                            .bits(),
                        &mut ret,
                    );

                    bits_to_mac_low(EMAC_MAC::regs().emacaddr4low().read().bits(), &mut ret);

                    if let Some(mask) = mask {
                        *mask = EMAC_MAC::regs()
                            .emacaddr4high()
                            .read()
                            .mask_byte_control4()
                            .bits();
                    }

                    if let Some(filter_for_source) = filter_for_source {
                        *filter_for_source = EMAC_MAC::regs()
                            .emacaddr4high()
                            .read()
                            .source_address4()
                            .bit_is_set();
                    }

                    return Ok(ret);
                }
            }

            MacAddr::MacAddr5 => {
                if EMAC_MAC::regs()
                    .emacaddr5high()
                    .read()
                    .address_enable5()
                    .bit_is_set()
                {
                    bits_to_mac_high(
                        EMAC_MAC::regs()
                            .emacaddr5high()
                            .read()
                            .mac_address5_hi()
                            .bits(),
                        &mut ret,
                    );

                    bits_to_mac_low(EMAC_MAC::regs().emacaddr5low().read().bits(), &mut ret);

                    if let Some(mask) = mask {
                        *mask = EMAC_MAC::regs()
                            .emacaddr5high()
                            .read()
                            .mask_byte_control5()
                            .bits();
                    }

                    if let Some(filter_for_source) = filter_for_source {
                        *filter_for_source = EMAC_MAC::regs()
                            .emacaddr5high()
                            .read()
                            .source_address5()
                            .bit_is_set();
                    }

                    return Ok(ret);
                }
            }

            MacAddr::MacAddr6 => {
                if EMAC_MAC::regs()
                    .emacaddr6high()
                    .read()
                    .address_enable6()
                    .bit_is_set()
                {
                    bits_to_mac_high(
                        EMAC_MAC::regs()
                            .emacaddr6high()
                            .read()
                            .mac_address6_hi()
                            .bits(),
                        &mut ret,
                    );

                    bits_to_mac_low(EMAC_MAC::regs().emacaddr6low().read().bits(), &mut ret);

                    if let Some(mask) = mask {
                        *mask = EMAC_MAC::regs()
                            .emacaddr6high()
                            .read()
                            .mask_byte_control6()
                            .bits();
                    }

                    if let Some(filter_for_source) = filter_for_source {
                        *filter_for_source = EMAC_MAC::regs()
                            .emacaddr6high()
                            .read()
                            .source_address6()
                            .bit_is_set();
                    }

                    return Ok(ret);
                }
            }

            MacAddr::MacAddr7 => {
                if EMAC_MAC::regs()
                    .emacaddr7high()
                    .read()
                    .address_enable7()
                    .bit_is_set()
                {
                    bits_to_mac_high(
                        EMAC_MAC::regs()
                            .emacaddr7high()
                            .read()
                            .mac_address7_hi()
                            .bits(),
                        &mut ret,
                    );

                    bits_to_mac_low(EMAC_MAC::regs().emacaddr7low().read().bits(), &mut ret);

                    if let Some(mask) = mask {
                        *mask = EMAC_MAC::regs()
                            .emacaddr7high()
                            .read()
                            .mask_byte_control7()
                            .bits();
                    }

                    if let Some(filter_for_source) = filter_for_source {
                        *filter_for_source = EMAC_MAC::regs()
                            .emacaddr7high()
                            .read()
                            .source_address7()
                            .bit_is_set();
                    }

                    return Ok(ret);
                }
            }
        }

        Err(Error::NotFound)
    }

    fn add_addr_da_filter_auto(mac_addr: [u8; 6]) -> Result<(), Error> {
        if Self::get_addr_da_filter(MacAddr::MacAddr1, None, None).is_err() {
            return Ok(Self::add_addr_da_filter(MacAddr::MacAddr1, mac_addr));
        }

        if Self::get_addr_da_filter(MacAddr::MacAddr2, None, None).is_err() {
            return Ok(Self::add_addr_da_filter(MacAddr::MacAddr2, mac_addr));
        }

        if Self::get_addr_da_filter(MacAddr::MacAddr3, None, None).is_err() {
            return Ok(Self::add_addr_da_filter(MacAddr::MacAddr3, mac_addr));
        }

        if Self::get_addr_da_filter(MacAddr::MacAddr4, None, None).is_err() {
            return Ok(Self::add_addr_da_filter(MacAddr::MacAddr4, mac_addr));
        }

        if Self::get_addr_da_filter(MacAddr::MacAddr5, None, None).is_err() {
            return Ok(Self::add_addr_da_filter(MacAddr::MacAddr5, mac_addr));
        }

        if Self::get_addr_da_filter(MacAddr::MacAddr6, None, None).is_err() {
            return Ok(Self::add_addr_da_filter(MacAddr::MacAddr6, mac_addr));
        }

        if Self::get_addr_da_filter(MacAddr::MacAddr7, None, None).is_err() {
            return Ok(Self::add_addr_da_filter(MacAddr::MacAddr7, mac_addr));
        }

        Err(Error::Full)
    }

    fn rm_addr_da_filter(mac_addr: [u8; 6], addr_num: MacAddr) -> Result<(), Error> {
        if let Ok(addr) = Self::get_addr_da_filter(addr_num, None, None) {
            if addr == mac_addr {
                EMAC_MAC::regs().emacaddr1high().write(|w| {
                    w.address_enable1().clear_bit();
                    unsafe { w.mac_address1_hi().bits(0) }
                });
                EMAC_MAC::regs()
                    .emacaddr1low()
                    .write(|w| unsafe { w.bits(0) });

                return Ok(());
            }
        }

        Err(Error::NotFound)
    }

    fn rm_addr_da_filter_auto(mac_addr: [u8; 6]) -> Result<(), Error> {
        Self::rm_addr_da_filter(mac_addr, MacAddr::MacAddr1)
            .or_else(|_| Self::rm_addr_da_filter(mac_addr, MacAddr::MacAddr2))
            .or_else(|_| Self::rm_addr_da_filter(mac_addr, MacAddr::MacAddr3))
            .or_else(|_| Self::rm_addr_da_filter(mac_addr, MacAddr::MacAddr4))
            .or_else(|_| Self::rm_addr_da_filter(mac_addr, MacAddr::MacAddr5))
            .or_else(|_| Self::rm_addr_da_filter(mac_addr, MacAddr::MacAddr6))
            .or_else(|_| Self::rm_addr_da_filter(mac_addr, MacAddr::MacAddr7))
    }

    fn clear_addr_da_filters() {
        if let Ok(addr) = Self::get_addr_da_filter(MacAddr::MacAddr1, None, None) {
            Self::rm_addr_da_filter(addr, MacAddr::MacAddr1);
        }

        if let Ok(addr) = Self::get_addr_da_filter(MacAddr::MacAddr2, None, None) {
            Self::rm_addr_da_filter(addr, MacAddr::MacAddr2);
        }

        if let Ok(addr) = Self::get_addr_da_filter(MacAddr::MacAddr3, None, None) {
            Self::rm_addr_da_filter(addr, MacAddr::MacAddr3);
        }

        if let Ok(addr) = Self::get_addr_da_filter(MacAddr::MacAddr4, None, None) {
            Self::rm_addr_da_filter(addr, MacAddr::MacAddr4);
        }

        if let Ok(addr) = Self::get_addr_da_filter(MacAddr::MacAddr5, None, None) {
            Self::rm_addr_da_filter(addr, MacAddr::MacAddr5);
        }

        if let Ok(addr) = Self::get_addr_da_filter(MacAddr::MacAddr6, None, None) {
            Self::rm_addr_da_filter(addr, MacAddr::MacAddr6);
        }

        if let Ok(addr) = Self::get_addr_da_filter(MacAddr::MacAddr7, None, None) {
            Self::rm_addr_da_filter(addr, MacAddr::MacAddr7);
        }
    }

    fn flush_tx_fifo() -> Result<(), Error> {
        EMAC_DMA::regs()
            .dmaoperation_mode()
            .write(|w| w.flush_tx_fifo().set_bit());
        for _ in 0..1_000 {
            if EMAC_DMA::regs()
                .dmaoperation_mode()
                .read()
                .flush_tx_fifo()
                .bit_is_clear()
            {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    pub fn flow_control(&mut self, enabled: bool) {
        if enabled {
            EMAC_MAC::regs().emacfc().write(|w| {
                // Pause time
                unsafe { w.pause_time().bits(EMAC_LL_PAUSE_TIME) };
                // Enable generation of Zero-Quanta Pause Control frames
                w.dzpq().clear_bit();
                // Threshold of the PAUSE to be checked for automatic retransmission of PAUSE Frame
                unsafe { w.plt().bits(PauseLowThreshold::_28 as _) };
                // Don't allow MAC detect Pause frames with MAC address0 unicast address and unique
                // multicast address
                w.upfd().clear_bit();
                // Enable MAC to decode the received Pause frame and disable its transmitter for a
                // specific time
                w.rfce().set_bit();
                // Enable MAC to transmit Pause frames in full duplex mode or the MAC back-pressure
                // operation in half duplex mode
                w.tfce().set_bit()
            });
        } else {
            EMAC_MAC::regs().emacfc().reset()
        }
    }

    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in system::Cpu::other() {
            interrupt::disable(core, Interrupt::ETH_MAC);
        }
        unsafe {
            interrupt::bind_interrupt(Interrupt::ETH_MAC, handler.handler());
        }
        unwrap!(interrupt::enable(Interrupt::ETH_MAC, handler.priority()));
    }

    pub fn into_async(mut self) -> Rmii<'d, Async> {
        self.set_interrupt_handler(interrupt_handler);
        Rmii {
            emac_dma: self.emac_dma,
            emac_ext: self.emac_ext,
            emac_mac: self.emac_mac,
            _mode: PhantomData,
            //_guard: self._guard,
        }
    }
}

impl InterruptConfigurable for Rmii<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

impl<'d> Rmii<'d, Async> {
    pub fn into_blocking(self) -> Rmii<'d, Blocking> {
        interrupt::disable(Cpu::current(), Interrupt::ETH_MAC);
        Rmii {
            emac_dma: self.emac_dma,
            emac_ext: self.emac_ext,
            emac_mac: self.emac_mac,
            _mode: PhantomData,
            //_guard: self._guard,
        }
    }
}

#[handler]
fn interrupt_handler() {}

const EMAC_LL_PAUSE_TIME: u16 = 0x1648;
const EMAC_LL_MAX_MAC_ADDR_NUM: u8 = 8;

enum InterFrameGapBits {
    _96 = 0,
    _88 = 1,
    _80 = 2,
    _72 = 3,
    _64 = 4,
    _56 = 5,
    _48 = 6,
    _40 = 7,
}

enum BackoffLimit {
    _10 = 0,
    _8  = 1,
    _4  = 2,
    _1  = 3,
}

enum PreambleLengthBytes {
    _7 = 0,
    _5 = 1,
    _3 = 2,
}

enum ControlFrameMode {
    BlockAll           = 0,
    PassAllExceptPause = 1,
    PassAll            = 2,
    PassFilter         = 3,
}

enum PauseLowThreshold {
    _4   = 0,
    _28  = 1,
    _144 = 2,
    _256 = 3,
}

enum TxThresholdControl {
    _64  = 0,
    _128 = 1,
    _192 = 2,
    _256 = 3,
    _40  = 4,
    _32  = 5,
    _24  = 6,
    _16  = 7,
}

enum RxThresholdControl {
    _64  = 0,
    _32  = 1,
    _96  = 2,
    _128 = 3,
}

enum ArbitrationRoundRobin {
    RxTx1_1 = 0,
    RxTx2_1 = 1,
    RxTx3_2 = 2,
    RxTx4_2 = 3,
}

enum MacAddr {
    MacAddr1 = 1,
    MacAddr2 = 2,
    MacAddr3 = 3,
    MacAddr4 = 4,
    MacAddr5 = 5,
    MacAddr6 = 6,
    MacAddr7 = 7,
}

fn mac_to_bits_low(mac_addr: [u8; 6]) -> u32 {
    ((mac_addr[3] as u32) << 24)
        | ((mac_addr[2] as u32) << 16)
        | ((mac_addr[1] as u32) << 8)
        | (mac_addr[0] as u32)
}

fn mac_to_bits_high(mac_addr: [u8; 6]) -> u16 {
    ((mac_addr[5] as u16) << 8) | (mac_addr[4] as u16)
}

fn bits_to_mac_low(bits: u32, mac: &mut [u8; 6]) {
    mac[0] = (bits & 0xff) as u8;
    mac[1] = ((bits >> 8) & 0xff) as u8;
    mac[2] = ((bits >> 16) & 0xff) as u8;
    mac[3] = ((bits >> 24) & 0xff) as u8;
}

fn bits_to_mac_high(bits: u16, mac: &mut [u8; 6]) {
    mac[4] = (bits & 0xff) as u8;
    mac[5] = ((bits >> 8) & 0xff) as u8;
}

const EMAC_LL_INTR_RECEIVE_ENABLE: u32 = 0x00000040;
const EMAC_LL_INTR_NORMAL_SUMMARY_ENABLE: u32 = 0x00010000;
const EMAC_LL_CONFIG_ENABLE_INTR_MASK: u32 =
    EMAC_LL_INTR_RECEIVE_ENABLE | EMAC_LL_INTR_NORMAL_SUMMARY_ENABLE;
