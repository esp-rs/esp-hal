#![cfg_attr(docsrs, procmacros::doc_replace(
    "dma_channel" => {
        cfg(lcd_cam_dma_engine = "AHB_GDMA") => "DMA_CH0",
        cfg(lcd_cam_dma_engine = "AXI_GDMA") => "DMA_AXI_CH0",
    },
    "dc_pin" => gpio_for_signal!(LCD_DC, "GPIO0"),
    "wrx_pin" => gpio_for_signal!(LCD_PCLK, "GPIO47"),
    "data0_pin" => gpio_for_signal!(LCD_DATA_0, "GPIO9"),
    "data1_pin" => gpio_for_signal!(LCD_DATA_1, "GPIO46"),
    "data2_pin" => gpio_for_signal!(LCD_DATA_2, "GPIO3"),
    "data3_pin" => gpio_for_signal!(LCD_DATA_3, "GPIO8"),
    "data4_pin" => gpio_for_signal!(LCD_DATA_4, "GPIO18"),
    "data5_pin" => gpio_for_signal!(LCD_DATA_5, "GPIO17"),
    "data6_pin" => gpio_for_signal!(LCD_DATA_6, "GPIO16"),
    "data7_pin" => gpio_for_signal!(LCD_DATA_7, "GPIO15"),
))]
//! # LCD - I8080/MOTO6800 Mode.
//!
//! ## Overview
//!
//! The LCD_CAM peripheral I8080 driver provides support for the I8080
//! format/timing. The driver mandates DMA (Direct Memory Access) for
//! efficient data transfer.
//!
//! ## Examples
//!
//! ### MIPI-DSI Display
//!
//! The following example shows how to send a command to a MIPI-DSI display over
//! the I8080 protocol.
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::lcd_cam::{LcdCam, lcd::i8080::{Config, I8080}};
//! # use esp_hal::dma_tx_buffer;
//! # use esp_hal::dma::DmaTxBuf;
//!
//! # let mut dma_buf = dma_tx_buffer!(32678)?;
//!
//! let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//!
//! let config = Config::default().with_frequency(Rate::from_mhz(20));
//!
//! let mut i8080 = I8080::new(lcd_cam.lcd, peripherals.__dma_channel__, config)?
//!     .with_dc(peripherals.__dc_pin__)
//!     .with_wrx(peripherals.__wrx_pin__)
//!     .with_data0(peripherals.__data0_pin__)
//!     .with_data1(peripherals.__data1_pin__)
//!     .with_data2(peripherals.__data2_pin__)
//!     .with_data3(peripherals.__data3_pin__)
//!     .with_data4(peripherals.__data4_pin__)
//!     .with_data5(peripherals.__data5_pin__)
//!     .with_data6(peripherals.__data6_pin__)
//!     .with_data7(peripherals.__data7_pin__);
//!
//! dma_buf.fill(&[0x55]);
//! let transfer = i8080.send(0x3Au8, 0, dma_buf)?; // RGB565
//! transfer.wait();
//! # {after_snippet}
//! ```
//!
//! ## Interrupts
//!
//! The I8080 driver owns two interrupt domains: the LCD half of the LCD_CAM
//! peripheral, and the DMA TX channel it was created with. The LCD sources
//! fire on the `LCD_CAM` interrupt, while the DMA sources fire on the DMA
//! channel's own interrupt, so each domain binds its own handler and controls
//! its sources separately. Handlers are registered with
//! [`I8080::set_interrupt_handler`] and [`I8080::set_dma_interrupt_handler`],
//! and sources are enabled with [`I8080::listen`] and [`I8080::listen_dma`].
//!
//! ### Notifying on descriptor completion
//!
//! In continuous output mode the transfer does not finish, but every
//! descriptor with the `suc_eof` bit set raises the DMA channel's
//! [`DmaTxInterrupt::Eof`](crate::dma::DmaTxInterrupt::Eof) source once its
//! data has been sent:
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::dma::{DmaTxBuf, DmaTxInterrupt};
//! # use esp_hal::dma_tx_buffer;
//! # use esp_hal::handler;
//! # use esp_hal::lcd_cam::{LcdCam, lcd::i8080::{Config, I8080}};
//! # use esp_hal::time::Rate;
//! #
//! # static EOF_COUNT: core::sync::atomic::AtomicUsize =
//! #     core::sync::atomic::AtomicUsize::new(0);
//! # let mut dma_buf = dma_tx_buffer!(32678)?;
//! # let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//! # let config = Config::default().with_frequency(Rate::from_mhz(20));
//! # let mut i8080 = I8080::new(lcd_cam.lcd, peripherals.DMA_CH0, config)?
//! #     .with_dc(peripherals.GPIO0)
//! #     .with_wrx(peripherals.GPIO47);
//! # dma_buf.fill(&[0x55]);
//!
//! #[handler]
//! fn dma_handler() {
//!     EOF_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
//!     // The source is cleared through the transfer, see below.
//! }
//!
//! // Binding the handler must happen before listening for DMA sources, and
//! // before `send` consumes the driver.
//! i8080.set_dma_interrupt_handler(dma_handler);
//! i8080.listen_dma(DmaTxInterrupt::Eof);
//!
//! let transfer = i8080.send(0x3Au8, 0, dma_buf)?; // RGB565
//!
//! // In `dma_handler`, clear the source through the transfer:
//! // transfer.clear_interrupts_dma(DmaTxInterrupt::Eof);
//! transfer.wait();
//! # {after_snippet}
//! ```

use core::{
    fmt::Formatter,
    mem::{ManuallyDrop, size_of},
    ops::{Deref, DerefMut},
};

use enumset::EnumSetType;

use crate::{
    Blocking,
    DriverMode,
    dma::{ChannelTx, DmaError, DmaPeripheral, DmaTxBuffer},
    gpio::{OutputConfig, OutputSignal, interconnect::PeripheralOutput},
    lcd_cam::{
        BitOrder,
        ByteOrder,
        ClockError,
        ErasedTxChannel,
        Instance,
        LCD_DONE_WAKER,
        Lcd,
        LcdDmaTxChannel,
        lcd::{ClockConfig, ClockMode, DelayMode},
        ll,
    },
    pac,
    time::Rate,
};

/// A configuration error.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// Clock configuration error.
    Clock(ClockError),
}

/// Interrupt sources of the LCD half of the LCD_CAM peripheral, as exposed by
/// the [`I8080`] driver.
///
/// These sources fire on the `LCD_CAM` interrupt, which is bound via
/// [`I8080::set_interrupt_handler`]. The DMA TX channel's sources fire on the
/// channel's own interrupt instead; see [`I8080::set_dma_interrupt_handler`]
/// and [`DmaTxInterrupt`](crate::dma::DmaTxInterrupt).
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum I8080Interrupt {
    /// The LCD has started outputting a new frame.
    LcdVsync,

    /// A DMA transfer to the LCD has finished.
    LcdTransDone,
}

/// Represents the I8080 LCD interface.
pub struct I8080<'d, Dm: DriverMode> {
    lcd: Lcd<'d, Dm>,
    tx_channel: ChannelTx<Blocking, ErasedTxChannel<'d>>,
}

impl<'d, Dm> I8080<'d, Dm>
where
    Dm: DriverMode,
{
    /// Creates a new instance of the I8080 LCD interface.
    pub fn new(
        lcd: Lcd<'d, Dm>,
        channel: impl LcdDmaTxChannel<'d>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let tx_channel = ChannelTx::new(channel.into());
        tx_channel.runtime_ensure_compatible(DmaPeripheral::LCD_CAM);

        let mut this = Self { lcd, tx_channel };

        this.apply_config(&config)?;

        Ok(this)
    }

    fn regs(&self) -> &pac::lcd_cam::RegisterBlock {
        self.lcd.regs()
    }

    /// Applies configuration.
    ///
    /// # Errors
    ///
    /// [`ConfigError::Clock`] when the frequency passed in `Config` is too low.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.lcd
            .configure_clocks(&ClockConfig {
                clock_mode: config.clock_mode,
                // ESP32-S3 errata requires LCD_PCLK to divide LCD_CLK by at least 2.
                // Double the requested frequency so the extra divider still matches.
                frequency: if cfg!(esp32s3) {
                    config.frequency * 2
                } else {
                    config.frequency
                },
            })
            .map_err(ConfigError::Clock)?;

        ll::set_rgb_mode_en(self.regs(), false);
        ll::set_lcd_conv_bypass(self.regs());

        self.regs().lcd_user().modify(|_, w| {
            w.lcd_bit_order().bit(false);
            w.lcd_byte_order().bit(false)
        });
        ll::set_8bits_order(self.regs(), false);
        ll::set_2byte_mode(self.regs(), false);
        self.regs().lcd_misc().write(|w| unsafe {
            #[cfg(not(esp32s31))]
            w.lcd_afifo_threshold_num().bits(0);
            // Configure the setup cycles in LCD non-RGB mode. Setup cycles
            // expected = this value + 1. (6 bit)
            w.lcd_vfk_cyclelen()
                .bits(config.setup_cycles.saturating_sub(1) as _);
            // Configure the hold time cycles in LCD non-RGB mode. Hold
            // cycles expected = this value + 1.
            w.lcd_vbk_cyclelen()
                .bits(config.hold_cycles.saturating_sub(1) as _);
            // 1: Send the next frame data when the current frame is sent out.
            // 0: LCD stops when the current frame is sent out.
            w.lcd_next_frame_en().clear_bit();
            // Enable blank region when LCD sends data out.
            w.lcd_bk_en().set_bit();
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DOUT phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_data_set()
                .bit(config.cd_data_edge != config.cd_idle_edge);
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DUMMY phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_dummy_set()
                .bit(config.cd_dummy_edge != config.cd_idle_edge);
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in CMD phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_cmd_set()
                .bit(config.cd_cmd_edge != config.cd_idle_edge);
            // The default value of LCD_CD
            w.lcd_cd_idle_edge().bit(config.cd_idle_edge)
        });
        ll::set_cd_delay(self.regs(), config.cd_mode as u8);
        ll::set_data_bit_delay(self.regs(), config.output_bit_mode as u8);

        self.regs()
            .lcd_user()
            .modify(|_, w| w.lcd_update().set_bit());

        Ok(())
    }

    /// Configures the byte order for data transmission in 16-bit mode.
    /// This must be set to [ByteOrder::default()] when transmitting in 8-bit
    /// mode.
    pub fn set_byte_order(&mut self, byte_order: ByteOrder) -> &mut Self {
        let is_inverted = byte_order != ByteOrder::default();
        self.regs()
            .lcd_user()
            .modify(|_, w| w.lcd_byte_order().bit(is_inverted));
        self
    }

    /// Configures the byte order for data transmission in 8-bit mode.
    /// This must be set to [ByteOrder::default()] when transmitting in 16-bit
    /// mode.
    pub fn set_8bits_order(&mut self, byte_order: ByteOrder) -> &mut Self {
        let is_inverted = byte_order != ByteOrder::default();
        ll::set_8bits_order(self.regs(), is_inverted);
        self
    }

    /// Configures the bit order for data transmission.
    pub fn set_bit_order(&mut self, bit_order: BitOrder) -> &mut Self {
        self.regs()
            .lcd_user()
            .modify(|_, w| w.lcd_bit_order().bit(bit_order != BitOrder::default()));
        self
    }

    /// Associates a CS pin with the I8080 interface.
    pub fn with_cs(self, cs: impl PeripheralOutput<'d>) -> Self {
        let cs = cs.into();

        cs.apply_output_config(&OutputConfig::default());
        cs.set_output_enable(true);

        OutputSignal::LCD_CS.connect_to(&cs);

        self
    }

    /// Associates a DC pin with the I8080 interface.
    pub fn with_dc(self, dc: impl PeripheralOutput<'d>) -> Self {
        let dc = dc.into();

        dc.apply_output_config(&OutputConfig::default());
        dc.set_output_enable(true);
        OutputSignal::LCD_DC.connect_to(&dc);

        self
    }

    /// Associates a WRX pin with the I8080 interface.
    pub fn with_wrx(self, wrx: impl PeripheralOutput<'d>) -> Self {
        let wrx = wrx.into();

        wrx.apply_output_config(&OutputConfig::default());
        wrx.set_output_enable(true);
        OutputSignal::LCD_PCLK.connect_to(&wrx);

        self
    }

    fn with_data_pin(self, signal: OutputSignal, pin: impl PeripheralOutput<'d>) -> Self {
        let pin = pin.into();

        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true);
        signal.connect_to(&pin);

        self
    }

    /// Associates a DATA 0 pin with the I8080 interface.
    pub fn with_data0(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_0, pin)
    }

    /// Associates a DATA 1 pin with the I8080 interface.
    pub fn with_data1(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_1, pin)
    }

    /// Associates a DATA 2 pin with the I8080 interface.
    pub fn with_data2(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_2, pin)
    }

    /// Associates a DATA 3 pin with the I8080 interface.
    pub fn with_data3(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_3, pin)
    }

    /// Associates a DATA 4 pin with the I8080 interface.
    pub fn with_data4(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_4, pin)
    }

    /// Associates a DATA 5 pin with the I8080 interface.
    pub fn with_data5(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_5, pin)
    }

    /// Associates a DATA 6 pin with the I8080 interface.
    pub fn with_data6(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_6, pin)
    }

    /// Associates a DATA 7 pin with the I8080 interface.
    pub fn with_data7(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_7, pin)
    }

    /// Associates a DATA 8 pin with the I8080 interface.
    pub fn with_data8(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_8, pin)
    }

    /// Associates a DATA 9 pin with the I8080 interface.
    pub fn with_data9(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_9, pin)
    }

    /// Associates a DATA 10 pin with the I8080 interface.
    pub fn with_data10(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_10, pin)
    }

    /// Associates a DATA 11 pin with the I8080 interface.
    pub fn with_data11(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_11, pin)
    }

    /// Associates a DATA 12 pin with the I8080 interface.
    pub fn with_data12(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_12, pin)
    }

    /// Associates a DATA 13 pin with the I8080 interface.
    pub fn with_data13(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_13, pin)
    }

    /// Associates a DATA 14 pin with the I8080 interface.
    pub fn with_data14(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_14, pin)
    }

    /// Associates a DATA 15 pin with the I8080 interface.
    pub fn with_data15(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_15, pin)
    }

    /// Sends a command and data to the LCD using DMA.
    ///
    /// Passing a `Command<u8>` will make this an 8-bit transfer and a
    /// `Command<u16>` will make this a 16-bit transfer.
    ///
    /// A 16-bit transfer on an 8-bit bus silently truncates the 2nd
    /// byte and an 8-bit transfer on a 16-bit bus silently pads each
    /// byte to 2 bytes.
    pub fn send<W: Into<u16> + Copy, BUF: DmaTxBuffer>(
        mut self,
        cmd: impl Into<Command<W>>,
        dummy: u8,
        mut data: BUF,
    ) -> Result<I8080Transfer<'d, BUF, Dm>, (DmaError, Self, BUF)> {
        let cmd = cmd.into();

        // Reset LCD control unit and Async Tx FIFO
        self.regs()
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());
        self.regs()
            .lcd_misc()
            .modify(|_, w| w.lcd_afifo_reset().set_bit());

        // Set cmd value
        match cmd {
            Command::None => {
                self.regs()
                    .lcd_user()
                    .modify(|_, w| w.lcd_cmd().clear_bit());
            }
            Command::One(value) => {
                self.regs().lcd_user().modify(|_, w| {
                    w.lcd_cmd().set_bit();
                    w.lcd_cmd_2_cycle_en().clear_bit()
                });
                ll::write_command(self.regs(), value.into() as u32, None);
            }
            Command::Two(first, second) => {
                self.regs().lcd_user().modify(|_, w| {
                    w.lcd_cmd().set_bit();
                    w.lcd_cmd_2_cycle_en().set_bit()
                });
                ll::write_command(self.regs(), first.into() as u32, Some(second.into() as u32));
            }
        }

        let is_2byte_mode = size_of::<W>() == 2;
        self.regs().lcd_user().modify(|_, w| unsafe {
            // Set dummy length
            if dummy > 0 {
                // Enable DUMMY phase in LCD sequence when LCD starts.
                w.lcd_dummy()
                    .set_bit()
                    // Configure DUMMY cycles. DUMMY cycles = this value + 1. (2 bits)
                    .lcd_dummy_cyclelen()
                    .bits((dummy - 1) as _)
            } else {
                w.lcd_dummy().clear_bit()
            }
        });
        ll::set_2byte_mode(self.regs(), is_2byte_mode);

        // Use continous mode for DMA. FROM the S3 TRM:
        // > In a continuous output, LCD module keeps sending data till:
        // > i. LCD_CAM_LCD_START is cleared;
        // > ii. or LCD_CAM_LCD_RESET is set;
        // > iii. or all the data in GDMA is sent out.
        self.regs()
            .lcd_user()
            .modify(|_, w| w.lcd_always_out_en().set_bit().lcd_dout().set_bit());

        let result = unsafe {
            self.tx_channel
                .prepare_transfer(DmaPeripheral::LCD_CAM, &mut data)
        }
        .and_then(|_| self.tx_channel.start_transfer());
        if let Err(err) = result {
            return Err((err, self, data));
        }

        // Setup interrupts.
        self.regs()
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());

        self.regs().lcd_user().modify(|_, w| {
            w.lcd_update().set_bit();
            w.lcd_start().set_bit()
        });

        Ok(I8080Transfer {
            i8080: ManuallyDrop::new(self),
            buf_view: ManuallyDrop::new(data.into_view()),
        })
    }
}

#[instability::unstable]
impl I8080<'_, Blocking> {
    /// Maps the LCD sources of the given set onto the peripheral's interrupt
    /// sources.
    fn map_i8080_to_lcdcam(
        interrupts: enumset::EnumSet<I8080Interrupt>,
    ) -> enumset::EnumSet<crate::lcd_cam::LcdCamInterrupt> {
        use crate::lcd_cam::LcdCamInterrupt;

        let mut sources = enumset::EnumSet::new();
        for interrupt in interrupts {
            sources.insert(LcdCamInterrupt::from(interrupt));
        }
        sources
    }

    /// Maps the peripheral's asserted LCD interrupt sources back to
    /// [`I8080Interrupt`].
    ///
    /// The mapping is lossy by design: Camera-only [`LcdCamInterrupt`]
    /// variants have no `I8080Interrupt` counterpart and are dropped. In
    /// practice they cannot occur here, since the peripheral interrupt
    /// helpers only read the `lcd_*` raw bits.
    fn map_lcdcam_to_i8080(
        sources: enumset::EnumSet<crate::lcd_cam::LcdCamInterrupt>,
    ) -> enumset::EnumSet<I8080Interrupt> {
        let mut interrupts = enumset::EnumSet::new();
        for source in sources {
            if let Ok(interrupt) = I8080Interrupt::try_from(source) {
                interrupts.insert(interrupt);
            }
        }
        interrupts
    }

    /// Registers an interrupt handler for the `LCD_CAM` interrupt.
    ///
    /// The handler services the sources enabled via [`Self::listen`].
    ///
    /// Note that this replaces any previously registered handler for the
    /// `LCD_CAM` interrupt, including the one the async driver binds.
    pub fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, crate::peripherals::Interrupt::LCD_CAM);
        }
        crate::interrupt::bind_handler(crate::peripherals::Interrupt::LCD_CAM, handler);
    }

    /// Registers an interrupt handler for the DMA TX channel used by this
    /// driver.
    ///
    /// The handler services the sources enabled via [`Self::listen_dma`]. It
    /// fires on the channel's own interrupt, which is separate from the
    /// `LCD_CAM` interrupt used by [`Self::set_interrupt_handler`].
    ///
    /// Binding the handler unlistens from and clears all DMA TX interrupt
    /// sources, so this function must be called before
    /// [`Self::listen_dma`]. It must also be called before [`Self::send`],
    /// which consumes the driver.
    pub fn set_dma_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.tx_channel.set_interrupt_handler(handler);
    }

    /// Listens for the given LCD interrupt sources.
    pub fn listen(&mut self, interrupts: impl Into<enumset::EnumSet<I8080Interrupt>>) {
        Instance::listen(Self::map_i8080_to_lcdcam(interrupts.into()));
    }

    /// Stops listening for the given LCD interrupt sources.
    pub fn unlisten(&mut self, interrupts: impl Into<enumset::EnumSet<I8080Interrupt>>) {
        Instance::unlisten(Self::map_i8080_to_lcdcam(interrupts.into()));
    }

    /// Returns the asserted LCD interrupt sources.
    pub fn interrupts(&mut self) -> enumset::EnumSet<I8080Interrupt> {
        Self::map_lcdcam_to_i8080(Instance::interrupts())
    }

    /// Clears the given asserted LCD interrupt sources.
    pub fn clear_interrupts(&mut self, interrupts: impl Into<enumset::EnumSet<I8080Interrupt>>) {
        Instance::clear_interrupts(Self::map_i8080_to_lcdcam(interrupts.into()));
    }

    /// Listens for the given DMA TX interrupt sources.
    ///
    /// A handler must have been registered via
    /// [`Self::set_dma_interrupt_handler`] first.
    pub fn listen_dma(
        &mut self,
        interrupts: impl Into<enumset::EnumSet<crate::dma::DmaTxInterrupt>>,
    ) {
        self.tx_channel.listen_out(interrupts.into());
    }

    /// Stops listening for the given DMA TX interrupt sources.
    pub fn unlisten_dma(
        &mut self,
        interrupts: impl Into<enumset::EnumSet<crate::dma::DmaTxInterrupt>>,
    ) {
        self.tx_channel.unlisten_out(interrupts.into());
    }

    /// Returns the asserted DMA TX interrupt sources.
    pub fn interrupts_dma(&mut self) -> enumset::EnumSet<crate::dma::DmaTxInterrupt> {
        self.tx_channel.pending_out_interrupts()
    }

    /// Clears the given asserted DMA TX interrupt sources.
    pub fn clear_interrupts_dma(
        &mut self,
        interrupts: impl Into<enumset::EnumSet<crate::dma::DmaTxInterrupt>>,
    ) {
        self.tx_channel.clear_out(interrupts.into());
    }
}

/// Variant-for-variant mapping onto the peripheral's interrupt sources.
///
/// This is total: every [`I8080Interrupt`] source has an
/// [`LcdCamInterrupt`] counterpart. Future Camera-only `LcdCamInterrupt`
/// variants are simply not reachable from [`I8080Interrupt`].
impl From<I8080Interrupt> for crate::lcd_cam::LcdCamInterrupt {
    fn from(value: I8080Interrupt) -> Self {
        match value {
            I8080Interrupt::LcdVsync => crate::lcd_cam::LcdCamInterrupt::LcdVsync,
            I8080Interrupt::LcdTransDone => crate::lcd_cam::LcdCamInterrupt::LcdTransDone,
        }
    }
}

/// Lossy reverse mapping: Camera-only [`LcdCamInterrupt`] variants have no
/// [`I8080Interrupt`] counterpart and are dropped.
impl TryFrom<crate::lcd_cam::LcdCamInterrupt> for I8080Interrupt {
    type Error = ();

    fn try_from(value: crate::lcd_cam::LcdCamInterrupt) -> Result<Self, ()> {
        match value {
            crate::lcd_cam::LcdCamInterrupt::LcdVsync => Ok(I8080Interrupt::LcdVsync),
            crate::lcd_cam::LcdCamInterrupt::LcdTransDone => Ok(I8080Interrupt::LcdTransDone),
        }
    }
}

impl<Dm: DriverMode> core::fmt::Debug for I8080<'_, Dm> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I8080").finish()
    }
}

/// Represents an ongoing (or potentially finished) transfer using the I8080 LCD
/// interface
pub struct I8080Transfer<'d, BUF: DmaTxBuffer, Dm: DriverMode> {
    i8080: ManuallyDrop<I8080<'d, Dm>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF: DmaTxBuffer, Dm: DriverMode> I8080Transfer<'d, BUF, Dm> {
    /// Returns whether [`Self::wait`] will not block.
    pub fn is_done(&self) -> bool {
        self.i8080
            .regs()
            .lcd_user()
            .read()
            .lcd_start()
            .bit_is_clear()
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn cancel(mut self) -> (I8080<'d, Dm>, BUF::Final) {
        self.stop_peripherals();
        let (_, i8080, buf) = self.wait();
        (i8080, buf)
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    ///
    /// Also clears the transfer interrupt so it can be used in
    /// interrupt handlers to handle the interrupt.
    pub fn wait(mut self) -> (Result<(), DmaError>, I8080<'d, Dm>, BUF::Final) {
        while !self.is_done() {}

        // Clear "done" interrupt.
        self.i8080
            .regs()
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());

        // SAFETY: Since forget is called on self, we know that self.i8080 and
        // self.buf_view won't be touched again.
        let (i8080, view) = unsafe {
            let i8080 = ManuallyDrop::take(&mut self.i8080);
            let view = ManuallyDrop::take(&mut self.buf_view);
            core::mem::forget(self);
            (i8080, view)
        };

        let result = if i8080.tx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        };

        (result, i8080, BUF::from_view(view))
    }

    fn stop_peripherals(&mut self) {
        // Stop the LCD_CAM peripheral.
        self.i8080
            .regs()
            .lcd_user()
            .modify(|_, w| w.lcd_start().clear_bit());

        // Stop the DMA
        self.i8080.tx_channel.stop_transfer();
    }
}

impl<BUF: DmaTxBuffer, Dm: DriverMode> Deref for I8080Transfer<'_, BUF, Dm> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<BUF: DmaTxBuffer, Dm: DriverMode> DerefMut for I8080Transfer<'_, BUF, Dm> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<BUF: DmaTxBuffer> I8080Transfer<'_, BUF, crate::Async> {
    /// Waits for [`Self::is_done`] to return true.
    pub async fn wait_for_done(&mut self) {
        use core::{
            future::Future,
            pin::Pin,
            task::{Context, Poll},
        };

        #[must_use = "futures do nothing unless you `.await` or poll them"]
        struct LcdDoneFuture {}

        impl Future for LcdDoneFuture {
            type Output = ();

            fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
                if Instance::is_lcd_done_set() {
                    // Interrupt bit will be cleared in Self::wait.
                    // This allows `wait_for_done` to be called more than once.
                    //
                    // Instance::clear_lcd_done();
                    Poll::Ready(())
                } else {
                    LCD_DONE_WAKER.register(cx.waker());
                    Instance::listen_lcd_done();
                    Poll::Pending
                }
            }
        }

        impl Drop for LcdDoneFuture {
            fn drop(&mut self) {
                Instance::unlisten_lcd_done();
            }
        }

        LcdDoneFuture {}.await
    }
}

/// Interrupt management for an ongoing transfer.
///
/// The driver itself has been consumed by [`I8080::send`], so these methods
/// take `&self` and allow an interrupt handler to manage the interrupt
/// sources through the transfer object.
#[instability::unstable]
impl<BUF: DmaTxBuffer> I8080Transfer<'_, BUF, Blocking> {
    /// Listens for the given LCD interrupt sources.
    pub fn listen(&self, interrupts: impl Into<enumset::EnumSet<I8080Interrupt>>) {
        Instance::listen(I8080::map_i8080_to_lcdcam(interrupts.into()));
    }

    /// Stops listening for the given LCD interrupt sources.
    pub fn unlisten(&self, interrupts: impl Into<enumset::EnumSet<I8080Interrupt>>) {
        Instance::unlisten(I8080::map_i8080_to_lcdcam(interrupts.into()));
    }

    /// Returns the asserted LCD interrupt sources.
    pub fn interrupts(&self) -> enumset::EnumSet<I8080Interrupt> {
        I8080::map_lcdcam_to_i8080(Instance::interrupts())
    }

    /// Clears the given asserted LCD interrupt sources.
    pub fn clear_interrupts(&self, interrupts: impl Into<enumset::EnumSet<I8080Interrupt>>) {
        Instance::clear_interrupts(I8080::map_i8080_to_lcdcam(interrupts.into()));
    }

    /// Listens for the given DMA TX interrupt sources.
    pub fn listen_dma(&self, interrupts: impl Into<enumset::EnumSet<crate::dma::DmaTxInterrupt>>) {
        self.i8080.tx_channel.listen_out(interrupts.into());
    }

    /// Stops listening for the given DMA TX interrupt sources.
    pub fn unlisten_dma(
        &self,
        interrupts: impl Into<enumset::EnumSet<crate::dma::DmaTxInterrupt>>,
    ) {
        self.i8080.tx_channel.unlisten_out(interrupts.into());
    }

    /// Returns the asserted DMA TX interrupt sources.
    pub fn interrupts_dma(&self) -> enumset::EnumSet<crate::dma::DmaTxInterrupt> {
        self.i8080.tx_channel.pending_out_interrupts()
    }

    /// Clears the given asserted DMA TX interrupt sources.
    pub fn clear_interrupts_dma(
        &self,
        interrupts: impl Into<enumset::EnumSet<crate::dma::DmaTxInterrupt>>,
    ) {
        self.i8080.tx_channel.clear_out(interrupts.into());
    }
}

impl<BUF: DmaTxBuffer, Dm: DriverMode> Drop for I8080Transfer<'_, BUF, Dm> {
    fn drop(&mut self) {
        self.stop_peripherals();

        // SAFETY: This is Drop, we know that self.i8080 and self.buf_view
        // won't be touched again.
        let view = unsafe {
            ManuallyDrop::drop(&mut self.i8080);
            ManuallyDrop::take(&mut self.buf_view)
        };
        let _ = BUF::from_view(view);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Configuration settings for the I8080 interface.
pub struct Config {
    /// Specifies the clock mode, including polarity and phase settings.
    clock_mode: ClockMode,

    /// The frequency of the pixel clock.
    frequency: Rate,

    /// Setup cycles expected, must be at least 1. (6 bits)
    setup_cycles: usize,

    /// Holds cycles expected, must be at least 1. (13 bits)
    hold_cycles: usize,

    /// The default value of LCD_CD.
    cd_idle_edge: bool,
    /// The value of LCD_CD during CMD phase.
    cd_cmd_edge: bool,
    /// The value of LCD_CD during dummy phase.
    cd_dummy_edge: bool,
    /// The value of LCD_CD during data phase.
    cd_data_edge: bool,

    /// The output LCD_CD is delayed by module clock LCD_CLK.
    cd_mode: DelayMode,
    /// The output data bits are delayed by module clock LCD_CLK.
    output_bit_mode: DelayMode,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_mode: Default::default(),
            frequency: Rate::from_mhz(20),
            setup_cycles: 1,
            hold_cycles: 1,
            cd_idle_edge: false,
            cd_cmd_edge: false,
            cd_dummy_edge: false,
            cd_data_edge: true,
            cd_mode: Default::default(),
            output_bit_mode: Default::default(),
        }
    }
}

/// LCD_CAM I8080 command.
///
/// Can be [Command::None] if command phase should be suppressed.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command<T> {
    /// Suppresses the command phase. No command is sent.
    None,
    /// Sends a single-word command.
    One(T),
    /// Sends a two-word command.
    Two(T, T),
}

impl From<u8> for Command<u8> {
    fn from(value: u8) -> Self {
        Command::One(value)
    }
}

impl From<u16> for Command<u16> {
    fn from(value: u16) -> Self {
        Command::One(value)
    }
}
