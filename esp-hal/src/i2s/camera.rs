//! # I2S in Camera Slave receiving mode.
//!
//! ## Overview
//! The I2S peripheral supports a camera slave mode for high-speed data
//! transfer from external camera modules.
//! The driver mandates DMA for efficient data transfer.
//!
//! ## Examples
//! ```rust, no_run
//! ```

use embedded_dma::WriteBuffer;

use crate::{
    dma::{
        dma_private::{DmaSupport, DmaSupportRx},
        ChannelRx,
        ChannelTypes,
        DmaTransferRx,
        DmaTransferRxCircular,
        I2s0Peripheral,
        Rx,
        RxChannel,
        RxPrivate,
    },
    gpio::{connect_high_to_peripheral, InputPin},
    i2s::{private::ExtendedSignals, Error, RegisterAccess},
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralClockControl,
};

/// Supported data formats
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataFormat {
    DualChannel16   = 0,
    SingleChannel16 = 1,
    DualChannel32   = 2,
    SingleChannel32 = 3,
}

pub struct Camera<'d, I2S, RX> {
    _i2s: PeripheralRef<'d, I2S>,
    rx_channel: RX,
}

impl<'d, I2S, T, R> Camera<'d, I2S, ChannelRx<'d, T, R>>
where
    I2S: RegisterAccess + ExtendedSignals,
    T: RxChannel<R>,
    R: ChannelTypes + crate::dma::RegisterAccess,
    R::P: I2s0Peripheral,
{
    pub fn new(
        i2s: impl Peripheral<P = I2S> + 'd,
        data_format: DataFormat,
        mut channel: ChannelRx<'d, T, R>,
    ) -> Self {
        crate::into_ref!(i2s);

        PeripheralClockControl::enable(I2S::get_peripheral());

        let reg_block = I2S::register_block();

        // Configuration and start/stop bits
        reg_block.conf().modify(|_, w| {
            // Enable slave receiver mode
            w.rx_slave_mod()
                .set_bit()
                // Receive left-channel data first.
                .rx_right_first()
                .clear_bit()
                // Place left-channel data at the MSB in the FIFO
                .rx_msb_right()
                .clear_bit()
                // Do not enable receiver in Philips standard mode
                .rx_msb_shift()
                .clear_bit()
                // Do not enable receiver’s mono mode in PCM standard mode
                .rx_mono()
                .clear_bit()
                // Do not enable receiver in PCM standard mode
                .rx_short_sync()
                .clear_bit()
        });

        // ADC/LCD/camera configuration register
        reg_block.conf2().modify(|_, w| {
            // Enable LCD mode.
            w.lcd_en()
                .set_bit()
                // Enable camera mode.
                .camera_en()
                .set_bit()
        });

        // Configure clock divider
        reg_block.clkm_conf().modify(|_, w| unsafe {
            w.clkm_div_a()
                .bits(0) // Fractional clock divider’s denominator value (6 bits)
                .clkm_div_b()
                .bits(0) // Fractional clock divider’s numerator value. (6 bits)
                .clkm_div_num()
                .bits(2) // I2S clock divider’s integral value. (8 bits)
        });

        reg_block.fifo_conf().modify(|_, w| unsafe {
            // Enable I2S DMA mode.
            w.dscr_en()
                .set_bit()
                // Receive FIFO mode configuration bit. (3 bits)
                .rx_fifo_mod()
                .bits(data_format as _)
                // The bit should always be set to 1.
                .rx_fifo_mod_force_en()
                .set_bit()
        });

        reg_block.conf_chan().modify(|_, w| unsafe {
            // I2S receiver channel mode configuration bit. (2 bits)
            w.rx_chan_mod().bits(1)
        });

        // Configure the bit length of I2S receiver channel. (6 bits)
        reg_block
            .sample_rate_conf()
            .modify(|_, w| unsafe { w.rx_bits_mod().bits(16) });

        // Synchronize signals into the receiver in double sync method.
        reg_block.timing().write(|w| w.rx_dsync_sw().set_bit());

        // Default these signals to high in case user doesn't provide them.
        connect_high_to_peripheral(I2S::v_sync_signal());
        connect_high_to_peripheral(I2S::h_sync_signal());
        connect_high_to_peripheral(I2S::h_enable_signal());

        channel.init_channel();

        Self {
            _i2s: i2s,
            rx_channel: channel,
        }
    }
}

impl<'d, I2S, RX> Camera<'d, I2S, RX>
where
    I2S: RegisterAccess + ExtendedSignals,
{
    #[allow(clippy::too_many_arguments)]
    pub fn with_data_pins<
        D0: InputPin,
        D1: InputPin,
        D2: InputPin,
        D3: InputPin,
        D4: InputPin,
        D5: InputPin,
        D6: InputPin,
        D7: InputPin,
    >(
        self,
        d0: impl Peripheral<P = D0>,
        d1: impl Peripheral<P = D1>,
        d2: impl Peripheral<P = D2>,
        d3: impl Peripheral<P = D3>,
        d4: impl Peripheral<P = D4>,
        d5: impl Peripheral<P = D5>,
        d6: impl Peripheral<P = D6>,
        d7: impl Peripheral<P = D7>,
    ) -> Self {
        crate::into_ref!(d0);
        crate::into_ref!(d1);
        crate::into_ref!(d2);
        crate::into_ref!(d3);
        crate::into_ref!(d4);
        crate::into_ref!(d5);
        crate::into_ref!(d6);
        crate::into_ref!(d7);

        d0.set_to_input(crate::private::Internal);
        d1.set_to_input(crate::private::Internal);
        d2.set_to_input(crate::private::Internal);
        d3.set_to_input(crate::private::Internal);
        d4.set_to_input(crate::private::Internal);
        d5.set_to_input(crate::private::Internal);
        d6.set_to_input(crate::private::Internal);
        d7.set_to_input(crate::private::Internal);

        d0.connect_input_to_peripheral(I2S::din0_signal(), crate::private::Internal);
        d1.connect_input_to_peripheral(I2S::din1_signal(), crate::private::Internal);
        d2.connect_input_to_peripheral(I2S::din2_signal(), crate::private::Internal);
        d3.connect_input_to_peripheral(I2S::din3_signal(), crate::private::Internal);
        d4.connect_input_to_peripheral(I2S::din4_signal(), crate::private::Internal);
        d5.connect_input_to_peripheral(I2S::din5_signal(), crate::private::Internal);
        d6.connect_input_to_peripheral(I2S::din6_signal(), crate::private::Internal);
        d7.connect_input_to_peripheral(I2S::din7_signal(), crate::private::Internal);

        self
    }

    pub fn with_ws<PIN: InputPin>(self, pin: impl Peripheral<P = PIN>) -> Self {
        crate::into_ref!(pin);

        pin.set_to_input(crate::private::Internal);
        pin.connect_input_to_peripheral(I2S::ws_in_signal(), crate::private::Internal);

        self
    }

    pub fn with_vsync<PIN: InputPin>(self, pin: impl Peripheral<P = PIN>) -> Self {
        crate::into_ref!(pin);

        pin.set_to_input(crate::private::Internal);
        pin.connect_input_to_peripheral(I2S::v_sync_signal(), crate::private::Internal);

        self
    }

    pub fn with_hsync<PIN: InputPin>(self, pin: impl Peripheral<P = PIN>) -> Self {
        crate::into_ref!(pin);

        pin.set_to_input(crate::private::Internal);
        pin.connect_input_to_peripheral(I2S::h_sync_signal(), crate::private::Internal);

        self
    }

    pub fn with_henable<PIN: InputPin>(self, pin: impl Peripheral<P = PIN>) -> Self {
        crate::into_ref!(pin);

        pin.set_to_input(crate::private::Internal);
        pin.connect_input_to_peripheral(I2S::h_enable_signal(), crate::private::Internal);

        self
    }
}

impl<'d, I2S, T, R> Camera<'d, I2S, ChannelRx<'d, T, R>>
where
    I2S: RegisterAccess,
    T: RxChannel<R>,
    R: ChannelTypes + crate::dma::RegisterAccess,
{
    fn start_rx_transfer<'t, RXBUF>(
        &'t mut self,
        words: &'t mut RXBUF,
        circular: bool,
    ) -> Result<(), Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { words.write_buffer() };

        if len % 4 != 0 {
            return Err(Error::IllegalArgument);
        }

        // Reset RX unit and RX FIFO
        I2S::reset_rx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        unsafe {
            self.rx_channel
                .prepare_transfer_without_start(circular, I2S::get_dma_peripheral(), ptr, len)
                .and_then(|_| self.rx_channel.start_transfer())?;
        }

        // set I2S_RX_STOP_EN if needed

        // start: set I2S_RX_START
        I2S::rx_start(len);
        Ok(())
    }

    pub fn read_dma<'t, RXBUF>(
        &'t mut self,
        words: &'t mut RXBUF,
    ) -> Result<DmaTransferRx<Self>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        self.start_rx_transfer(words, false)?;
        Ok(DmaTransferRx::new(self))
    }

    pub fn read_dma_circular<'t, RXBUF>(
        &'t mut self,
        words: &'t mut RXBUF,
    ) -> Result<DmaTransferRxCircular<Self>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        self.start_rx_transfer(words, true)?;
        Ok(DmaTransferRxCircular::new(self))
    }
}

impl<'d, I2S: RegisterAccess, RX: Rx> DmaSupport for Camera<'d, I2S, RX> {
    fn peripheral_wait_dma(&mut self, _is_tx: bool, _is_rx: bool) {
        loop {
            // Wait for IN_SUC_EOF (i.e. I2S_RXEOF_NUM_REG / byte count threshold)
            let i2s = I2S::register_block();
            if i2s.int_raw().read().in_suc_eof().bit_is_set() {
                break;
            }

            // Or for IN_DSCR_EMPTY (i.e. No more buffer space)
            if self.rx_channel.has_dscr_empty_error() {
                break;
            }

            // Or for IN_DSCR_ERR (i.e. bad descriptor)
            if self.rx_channel.has_error() {
                break;
            }
        }

        // Terminate RX transfer.
        I2S::rx_stop();
    }

    fn peripheral_dma_stop(&mut self) {
        // Terminate RX transfer.
        I2S::rx_stop();
    }
}

impl<'d, I2S: RegisterAccess, RX: Rx> DmaSupportRx for Camera<'d, I2S, RX> {
    type RX = RX;

    fn rx(&mut self) -> &mut Self::RX {
        &mut self.rx_channel
    }
}
