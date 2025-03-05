//! # I2S in Camera Slave receiving mode.
//!
//! ## Overview
//! The I2S peripheral supports a camera slave mode for high-speed data
//! transfer from external camera modules.
//! The driver mandates DMA for efficient data transfer.
//!
//! ## Examples
//! ```rust, no_run
//! // FIXME: add example
//! ```

use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use crate::{
    dma::{ChannelRx, DmaEligible, DmaError, DmaRxBuffer, PeripheralRxChannel, Rx, RxChannelFor},
    gpio::{InputPin, Level, Pull},
    i2s::master::{Error, ExtendedSignals, RegisterAccess},
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralClockControl,
    Blocking,
};

/// Supported data formats
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataFormat {
    /// FIXME: docs
    DualChannel16   = 0,
    /// FIXME: docs
    SingleChannel16 = 1,
    /// FIXME: docs
    DualChannel32   = 2,
    /// FIXME: docs
    SingleChannel32 = 3,
}

/// Represents the camera interface.
pub struct Camera<'d, I2S: DmaEligible> {
    _i2s: PeripheralRef<'d, I2S>,
    rx_channel: ChannelRx<'d, Blocking, PeripheralRxChannel<I2S>>,
}

impl<'d, I2S> Camera<'d, I2S>
where
    I2S: RegisterAccess + ExtendedSignals,
{
    /// Creates a new `Camera` instance with DMA support.
    pub fn new<CH: RxChannelFor<I2S>>(
        i2s: impl Peripheral<P = I2S> + 'd,
        data_format: DataFormat,
        channel: impl Peripheral<P = CH> + 'd,
    ) -> Self {
        crate::into_ref!(i2s);

        PeripheralClockControl::enable(i2s.peripheral());

        let regs = i2s.regs();

        // Configuration and start/stop bits
        regs.conf().modify(|_, w| {
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
        regs.conf2().modify(|_, w| {
            // Enable LCD mode.
            w.lcd_en()
                .set_bit()
                // Enable camera mode.
                .camera_en()
                .set_bit()
        });

        // Configure clock divider
        regs.clkm_conf().modify(|_, w| unsafe {
            w.clkm_div_a()
                .bits(0) // Fractional clock divider’s denominator value (6 bits)
                .clkm_div_b()
                .bits(0) // Fractional clock divider’s numerator value. (6 bits)
                .clkm_div_num()
                .bits(2) // I2S clock divider’s integral value. (8 bits)
        });

        regs.fifo_conf().modify(|_, w| unsafe {
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

        regs.conf_chan().modify(|_, w| unsafe {
            // I2S receiver channel mode configuration bit. (2 bits)
            w.rx_chan_mod().bits(1)
        });

        // Configure the bit length of I2S receiver channel. (6 bits)
        regs.sample_rate_conf()
            .modify(|_, w| unsafe { w.rx_bits_mod().bits(16) });

        // Synchronize signals into the receiver in double sync method.
        regs.timing().write(|w| w.rx_dsync_sw().set_bit());

        // Default these signals to high in case user doesn't provide them.
        I2S::v_sync_signal().connect_to(Level::High);
        I2S::h_sync_signal().connect_to(Level::High);
        I2S::h_enable_signal().connect_to(Level::High);

        let rx_channel = ChannelRx::new(channel.map(|ch| ch.degrade()));

        Self {
            _i2s: i2s,
            rx_channel,
        }
    }
}

impl<I2S> Camera<'_, I2S>
where
    I2S: RegisterAccess + ExtendedSignals,
{
    /// Configures the data pins for the camera interface.
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
        crate::into_mapped_ref!(d0);
        crate::into_mapped_ref!(d1);
        crate::into_mapped_ref!(d2);
        crate::into_mapped_ref!(d3);
        crate::into_mapped_ref!(d4);
        crate::into_mapped_ref!(d5);
        crate::into_mapped_ref!(d6);
        crate::into_mapped_ref!(d7);

        let pairs = [
            (d0, I2S::din0_signal()),
            (d1, I2S::din1_signal()),
            (d2, I2S::din2_signal()),
            (d3, I2S::din3_signal()),
            (d4, I2S::din4_signal()),
            (d5, I2S::din5_signal()),
            (d6, I2S::din6_signal()),
            (d7, I2S::din7_signal()),
        ];

        for (pin, signal) in pairs {
            pin.init_input(Pull::None);
            signal.connect_to(pin);
        }

        self
    }

    /// Configures the pixel clock pin for the camera interface.
    pub fn with_ws<PIN: InputPin>(self, pin: impl Peripheral<P = PIN>) -> Self {
        crate::into_mapped_ref!(pin);
        pin.init_input(Pull::None);
        I2S::ws_in_signal().connect_to(pin);
        self
    }

    /// Configures the vertical sync (VSYNC) pin for the camera interface.
    pub fn with_vsync<PIN: InputPin>(self, pin: impl Peripheral<P = PIN>) -> Self {
        crate::into_mapped_ref!(pin);
        pin.init_input(Pull::None);
        I2S::v_sync_signal().connect_to(pin);
        self
    }

    /// Configures the horizontal sync (HSYNC) pin for the camera interface.
    pub fn with_hsync<PIN: InputPin>(self, pin: impl Peripheral<P = PIN>) -> Self {
        crate::into_mapped_ref!(pin);
        pin.init_input(Pull::None);
        I2S::h_sync_signal().connect_to(pin);
        self
    }

    /// Configures the vertical sync enable pin for the camera interface.
    pub fn with_henable<PIN: InputPin>(self, pin: impl Peripheral<P = PIN>) -> Self {
        crate::into_mapped_ref!(pin);
        pin.init_input(Pull::None);
        I2S::h_enable_signal().connect_to(pin);
        self
    }
}

impl<'d, I2S> Camera<'d, I2S>
where
    I2S: RegisterAccess,
{
    fn start_rx_transfer<'t, RXBUF>(
        &'t mut self,
        buf: &'t mut RXBUF,
        len: usize,
    ) -> Result<(), Error>
    where
        RXBUF: DmaRxBuffer,
    {
        if len % 4 != 0 {
            return Err(Error::IllegalArgument);
        }

        // Reset RX unit and RX FIFO
        self._i2s.reset_rx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        unsafe {
            self.rx_channel
                .prepare_transfer(self._i2s.dma_peripheral(), buf)
                .and_then(|_| self.rx_channel.start_transfer())?;
        }

        // set I2S_RX_STOP_EN if needed

        // start: set I2S_RX_START
        self._i2s.rx_start(len);
        Ok(())
    }

    /// Starts a DMA transfer to receive data from the camera peripheral.
    pub fn receive<BUF: DmaRxBuffer>(
        mut self,
        mut buf: BUF,
        len: usize,
    ) -> Result<CameraTransfer<'d, I2S, BUF>, Error> {
        self.start_rx_transfer(&mut buf, len)?;
        Ok(CameraTransfer {
            camera: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buf.into_view()),
        })
    }
}

/// Represents an ongoing (or potentially stopped) transfer from the Camera to a
/// DMA buffer.
pub struct CameraTransfer<'d, I2S: DmaEligible + RegisterAccess, BUF: DmaRxBuffer> {
    camera: ManuallyDrop<Camera<'d, I2S>>,
    buffer_view: ManuallyDrop<BUF::View>,
}

impl<'d, I2S: RegisterAccess, BUF: DmaRxBuffer> CameraTransfer<'d, I2S, BUF> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.camera.rx_channel.has_dscr_empty_error() // IN_DSCR_EMPTY (i.e. No more buffer space)
            || self.camera.rx_channel.has_error() // IN_DSCR_ERR (i.e. bad
                                                  // descriptor)
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn stop(mut self) -> (Camera<'d, I2S>, BUF) {
        self.stop_peripherals();
        let (camera, view) = self.release();
        (camera, BUF::from_view(view))
    }

    /// Waits for the transfer to stop and returns the peripheral and buffer.
    ///
    /// Note: The camera doesn't really "finish" its transfer, so what you're
    /// really waiting for here is a DMA Error. You typically just want to
    /// call [Self::stop] once you have the data you need.
    pub fn wait(mut self) -> (Result<(), DmaError>, Camera<'d, I2S>, BUF) {
        while !self.is_done() {}

        // Stop the DMA as it doesn't know that the camera has stopped.
        self.camera.rx_channel.stop_transfer();

        // Note: There is no "done" interrupt to clear.

        let (camera, view) = self.release();

        let result = if camera.rx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        };

        (result, camera, BUF::from_view(view))
    }

    fn release(mut self) -> (Camera<'d, I2S>, BUF::View) {
        // SAFETY: Since forget is called on self, we know that self.camera and
        // self.buffer_view won't be touched again.
        let result = unsafe {
            let camera = ManuallyDrop::take(&mut self.camera);
            let view = ManuallyDrop::take(&mut self.buffer_view);
            (camera, view)
        };
        core::mem::forget(self);
        result
    }

    fn stop_peripherals(&mut self) {
        self.camera._i2s.rx_stop();
    }
}

impl<I2S: DmaEligible + RegisterAccess, BUF: DmaRxBuffer> Deref for CameraTransfer<'_, I2S, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<I2S: DmaEligible + RegisterAccess, BUF: DmaRxBuffer> DerefMut
    for CameraTransfer<'_, I2S, BUF>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<I2S: DmaEligible + RegisterAccess, BUF: DmaRxBuffer> Drop for CameraTransfer<'_, I2S, BUF> {
    fn drop(&mut self) {
        self.stop_peripherals();

        // SAFETY: This is Drop, we know that self.camera and self.buffer_view
        // won't be touched again.
        unsafe {
            ManuallyDrop::drop(&mut self.camera);
            ManuallyDrop::drop(&mut self.buffer_view);
        }
    }
}
