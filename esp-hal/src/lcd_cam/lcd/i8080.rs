//! # LCD - I8080/MOTO6800 Mode.
//!
//! ## Overview
//! The LCD_CAM peripheral I8080 driver provides support for the I8080
//! format/timing. The driver mandates DMA for DMA (Direct Memory Access) for
//! efficient data transfer.
//!
//! ## Examples
//! Following code show how to send a command to a MIPI-DSI display over I8080
//! protocol.
//!
//! ```no_run
//! let tx_pins = TxEightBits::new(
//!     io.pins.gpio9,
//!     io.pins.gpio46,
//!     io.pins.gpio3,
//!     io.pins.gpio8,
//!     io.pins.gpio18,
//!     io.pins.gpio17,
//!     io.pins.gpio16,
//!     io.pins.gpio15,
//! );
//! let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//!
//! let mut i8080 = I8080::new(
//!     lcd_cam.lcd,
//!     channel.tx,
//!     tx_pins,
//!     20.MHz(),
//!     Config::default(),
//!     &clocks,
//! )
//! .with_ctrl_pins(io.pins.gpio0, io.pins.gpio47);
//!
//! i8080.send(0x3A, 0, &[0x55]).unwrap(); // RGB565
//! ```

use core::{fmt::Formatter, mem::size_of};

use embedded_dma::ReadBuffer;
use fugit::HertzU32;

use crate::{
    clock::Clocks,
    dma::{
        ChannelTx,
        ChannelTypes,
        DmaError,
        DmaPeripheral,
        LcdCamPeripheral,
        RegisterAccess,
        Tx,
        TxChannel,
        TxPrivate,
    },
    gpio::{OutputPin, OutputSignal},
    lcd_cam::{
        lcd::{i8080::private::TxPins, ClockMode, DelayMode, Phase, Polarity},
        private::calculate_clkm,
        BitOrder,
        ByteOrder,
        Lcd,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::LCD_CAM,
};

pub struct I8080<'d, TX, P> {
    lcd_cam: PeripheralRef<'d, LCD_CAM>,
    tx_channel: TX,
    _pins: P,
}

impl<'d, T, R, P: TxPins> I8080<'d, ChannelTx<'d, T, R>, P>
where
    T: TxChannel<R>,
    R: ChannelTypes + RegisterAccess,
    R::P: LcdCamPeripheral,
    P::Word: Into<u16>,
{
    pub fn new(
        lcd: Lcd<'d>,
        mut channel: ChannelTx<'d, T, R>,
        mut pins: P,
        frequency: HertzU32,
        config: Config,
        clocks: &Clocks,
    ) -> Self {
        let is_2byte_mode = size_of::<P::Word>() == 2;

        let lcd_cam = lcd.lcd_cam;

        // Due to https://www.espressif.com/sites/default/files/documentation/esp32-s3_errata_en.pdf
        // the LCD_PCLK divider must be at least 2. To make up for this the user
        // provided frequency is doubled to match.

        let (i, divider) = calculate_clkm(
            (frequency.to_Hz() * 2) as _,
            &[
                clocks.xtal_clock.to_Hz() as _,
                clocks.cpu_clock.to_Hz() as _,
                clocks.crypto_pwm_clock.to_Hz() as _,
            ],
        );

        lcd_cam.lcd_clock().write(|w| unsafe {
            // Force enable the clock for all configuration registers.
            w.clk_en()
                .set_bit()
                .lcd_clk_sel()
                .bits((i + 1) as _)
                .lcd_clkm_div_num()
                .bits(divider.div_num as _)
                .lcd_clkm_div_b()
                .bits(divider.div_b as _)
                .lcd_clkm_div_a()
                .bits(divider.div_a as _)
                // LCD_PCLK = LCD_CLK / 2
                .lcd_clk_equ_sysclk()
                .clear_bit()
                .lcd_clkcnt_n()
                .bits(2 - 1) // Must not be 0.
                .lcd_ck_idle_edge()
                .bit(config.clock_mode.polarity == Polarity::IdleHigh)
                .lcd_ck_out_edge()
                .bit(config.clock_mode.phase == Phase::ShiftHigh)
        });

        lcd_cam
            .lcd_ctrl()
            .write(|w| w.lcd_rgb_mode_en().clear_bit());
        lcd_cam
            .lcd_rgb_yuv()
            .write(|w| w.lcd_conv_bypass().clear_bit());

        lcd_cam.lcd_user().modify(|_, w| {
            w.lcd_8bits_order()
                .bit(false)
                .lcd_bit_order()
                .bit(false)
                .lcd_byte_order()
                .bit(false)
                .lcd_2byte_en()
                .bit(is_2byte_mode)
        });
        lcd_cam.lcd_misc().write(|w| unsafe {
            // Set the threshold for Async Tx FIFO full event. (5 bits)
            w.lcd_afifo_threshold_num()
                .bits(0)
                // Configure the setup cycles in LCD non-RGB mode. Setup cycles
                // expected = this value + 1. (6 bit)
                .lcd_vfk_cyclelen()
                .bits(config.setup_cycles.saturating_sub(1) as _)
                // Configure the hold time cycles in LCD non-RGB mode. Hold
                // cycles expected = this value + 1.
                .lcd_vbk_cyclelen()
                .bits(config.hold_cycles.saturating_sub(1) as _)
                // 1: Send the next frame data when the current frame is sent out.
                // 0: LCD stops when the current frame is sent out.
                .lcd_next_frame_en()
                .clear_bit()
                // Enable blank region when LCD sends data out.
                .lcd_bk_en()
                .set_bit()
                // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DOUT phase.
                // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
                .lcd_cd_data_set()
                .bit(config.cd_data_edge != config.cd_idle_edge)
                // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DUMMY phase.
                // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
                .lcd_cd_dummy_set()
                .bit(config.cd_dummy_edge != config.cd_idle_edge)
                // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in CMD phase.
                // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
                .lcd_cd_cmd_set()
                .bit(config.cd_cmd_edge != config.cd_idle_edge)
                // The default value of LCD_CD
                .lcd_cd_idle_edge()
                .bit(config.cd_idle_edge)
        });
        lcd_cam
            .lcd_dly_mode()
            .write(|w| unsafe { w.lcd_cd_mode().bits(config.cd_mode as u8) });
        lcd_cam.lcd_data_dout_mode().write(|w| unsafe {
            w.dout0_mode()
                .bits(config.output_bit_mode as u8)
                .dout1_mode()
                .bits(config.output_bit_mode as u8)
                .dout2_mode()
                .bits(config.output_bit_mode as u8)
                .dout3_mode()
                .bits(config.output_bit_mode as u8)
                .dout4_mode()
                .bits(config.output_bit_mode as u8)
                .dout5_mode()
                .bits(config.output_bit_mode as u8)
                .dout6_mode()
                .bits(config.output_bit_mode as u8)
                .dout7_mode()
                .bits(config.output_bit_mode as u8)
                .dout8_mode()
                .bits(config.output_bit_mode as u8)
                .dout9_mode()
                .bits(config.output_bit_mode as u8)
                .dout10_mode()
                .bits(config.output_bit_mode as u8)
                .dout11_mode()
                .bits(config.output_bit_mode as u8)
                .dout12_mode()
                .bits(config.output_bit_mode as u8)
                .dout13_mode()
                .bits(config.output_bit_mode as u8)
                .dout14_mode()
                .bits(config.output_bit_mode as u8)
                .dout15_mode()
                .bits(config.output_bit_mode as u8)
        });

        lcd_cam.lcd_user().modify(|_, w| w.lcd_update().set_bit());

        channel.init_channel();
        pins.configure();

        Self {
            lcd_cam,
            tx_channel: channel,
            _pins: pins,
        }
    }
}

impl<'d, TX: Tx, P: TxPins> I8080<'d, TX, P>
where
    P::Word: Into<u16>,
{
    pub fn set_byte_order(&mut self, byte_order: ByteOrder) -> &mut Self {
        let is_inverted = byte_order != ByteOrder::default();
        self.lcd_cam.lcd_user().modify(|_, w| {
            if size_of::<P::Word>() == 2 {
                w.lcd_byte_order().bit(is_inverted)
            } else {
                w.lcd_8bits_order().bit(is_inverted)
            }
        });
        self
    }

    pub fn set_bit_order(&mut self, bit_order: BitOrder) -> &mut Self {
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_bit_order().bit(bit_order != BitOrder::default()));
        self
    }

    pub fn with_cs<CS: OutputPin>(self, cs: impl Peripheral<P = CS> + 'd) -> Self {
        crate::into_ref!(cs);
        cs.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_CS);

        self
    }

    pub fn with_ctrl_pins<DC: OutputPin, WRX: OutputPin>(
        self,
        dc: impl Peripheral<P = DC> + 'd,
        wrx: impl Peripheral<P = WRX> + 'd,
    ) -> Self {
        crate::into_ref!(dc);
        crate::into_ref!(wrx);

        dc.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DC);

        wrx.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_PCLK);

        self
    }

    pub fn send(
        &mut self,
        cmd: impl Into<Command<P::Word>>,
        dummy: u8,
        data: &[P::Word],
    ) -> Result<(), DmaError> {
        self.setup_send(cmd.into(), dummy);
        self.start_write_bytes_dma(data.as_ptr() as _, core::mem::size_of_val(data))?;
        self.start_send();

        let dma_int_raw = self.lcd_cam.lc_dma_int_raw();
        // Wait until LCD_TRANS_DONE is set.
        while dma_int_raw.read().lcd_trans_done_int_raw().bit_is_clear() {}

        self.tear_down_send();

        Ok(())
    }

    pub fn send_dma<'t, TXBUF>(
        &'t mut self,
        cmd: impl Into<Command<P::Word>>,
        dummy: u8,
        data: &'t TXBUF,
    ) -> Result<Transfer<'t, 'd, TX, P>, DmaError>
    where
        TXBUF: ReadBuffer<Word = P::Word>,
    {
        let (ptr, len) = unsafe { data.read_buffer() };

        self.setup_send(cmd.into(), dummy);
        self.start_write_bytes_dma(ptr as _, len * size_of::<P::Word>())?;
        self.start_send();

        Ok(Transfer {
            instance: Some(self),
        })
    }
}

impl<'d, TX: Tx, P> I8080<'d, TX, P> {
    fn setup_send<T: Copy + Into<u16>>(&mut self, cmd: Command<T>, dummy: u8) {
        // Reset LCD control unit and Async Tx FIFO
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());
        self.lcd_cam
            .lcd_misc()
            .modify(|_, w| w.lcd_afifo_reset().set_bit());

        // Set cmd value
        match cmd {
            Command::None => {
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_cmd().clear_bit());
            }
            Command::One(value) => {
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_cmd().set_bit().lcd_cmd_2_cycle_en().clear_bit());
                self.lcd_cam
                    .lcd_cmd_val()
                    .write(|w| unsafe { w.lcd_cmd_value().bits(value.into() as _) });
            }
            Command::Two(first, second) => {
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_cmd().set_bit().lcd_cmd_2_cycle_en().set_bit());
                let cmd = first.into() as u32 | (second.into() as u32) << 16;
                self.lcd_cam
                    .lcd_cmd_val()
                    .write(|w| unsafe { w.lcd_cmd_value().bits(cmd) });
            }
        }

        // Set dummy length
        self.lcd_cam.lcd_user().modify(|_, w| unsafe {
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
    }

    fn start_send(&mut self) {
        // Setup interrupts.
        self.lcd_cam
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());

        // Before issuing lcd_start need to wait shortly for fifo to get data
        // Otherwise, some garbage data will be sent out
        crate::rom::ets_delay_us(1);

        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_update().set_bit().lcd_start().set_bit());
    }

    fn tear_down_send(&mut self) {
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_start().clear_bit());

        self.lcd_cam
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().clear_bit());
    }

    fn start_write_bytes_dma(&mut self, ptr: *const u8, len: usize) -> Result<(), DmaError> {
        if len == 0 {
            // Set transfer length.
            self.lcd_cam
                .lcd_user()
                .modify(|_, w| w.lcd_dout().clear_bit());
        } else {
            // Set transfer length.
            self.lcd_cam.lcd_user().modify(|_, w| unsafe {
                if len <= 8192 {
                    // Data length in fixed mode. (13 bits)
                    w.lcd_always_out_en()
                        .clear_bit()
                        .lcd_dout_cyclelen()
                        .bits((len - 1) as _)
                } else {
                    // Enable continuous output.
                    w.lcd_always_out_en().set_bit()
                }
                .lcd_dout()
                .set_bit()
            });

            self.tx_channel.prepare_transfer_without_start(
                DmaPeripheral::LcdCam,
                false,
                ptr,
                len,
            )?;
            self.tx_channel.start_transfer()?;
        }
        Ok(())
    }
}

impl<'d, TX, P> core::fmt::Debug for I8080<'d, TX, P> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I8080").finish()
    }
}

/// An in-progress transfer
#[must_use]
pub struct Transfer<'t, 'd, TX: Tx, P> {
    instance: Option<&'t mut I8080<'d, TX, P>>,
}

impl<'t, 'd, TX: Tx, P> Transfer<'t, 'd, TX, P> {
    #[allow(clippy::type_complexity)]
    pub fn wait(mut self) -> Result<(), DmaError> {
        let instance = self
            .instance
            .take()
            .expect("instance must be available throughout object lifetime");

        {
            let dma_int_raw = instance.lcd_cam.lc_dma_int_raw();
            // Wait until LCD_TRANS_DONE is set.
            while dma_int_raw.read().lcd_trans_done_int_raw().bit_is_clear() {}
            instance.tear_down_send();
        }

        if instance.tx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    pub fn is_done(&self) -> bool {
        let int_raw = self
            .instance
            .as_ref()
            .expect("instance must be available throughout object lifetime")
            .lcd_cam
            .lc_dma_int_raw();
        int_raw.read().lcd_trans_done_int_raw().bit_is_set()
    }
}

impl<'t, 'd, TX: Tx, P> Drop for Transfer<'t, 'd, TX, P> {
    fn drop(&mut self) {
        if let Some(instance) = self.instance.as_mut() {
            // This will cancel the transfer.
            instance.tear_down_send();
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub clock_mode: ClockMode,

    /// Setup cycles expected, must be at least 1. (6 bits)
    pub setup_cycles: usize,

    /// Hold cycles expected, must be at least 1. (13 bits)
    pub hold_cycles: usize,

    /// The default value of LCD_CD.
    pub cd_idle_edge: bool,
    /// The value of LCD_CD during CMD phase.
    pub cd_cmd_edge: bool,
    /// The value of LCD_CD during dummy phase.
    pub cd_dummy_edge: bool,
    /// The value of LCD_CD during data phase.
    pub cd_data_edge: bool,

    /// The output LCD_CD is delayed by module clock LCD_CLK.
    pub cd_mode: DelayMode,
    /// The output data bits are delayed by module clock LCD_CLK.
    pub output_bit_mode: DelayMode,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_mode: Default::default(),
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
    None,
    One(T),
    Two(T, T),
}

impl<T> From<T> for Command<T> {
    fn from(value: T) -> Self {
        Command::One(value)
    }
}

pub struct TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7> {
    pin_0: PeripheralRef<'d, P0>,
    pin_1: PeripheralRef<'d, P1>,
    pin_2: PeripheralRef<'d, P2>,
    pin_3: PeripheralRef<'d, P3>,
    pin_4: PeripheralRef<'d, P4>,
    pin_5: PeripheralRef<'d, P5>,
    pin_6: PeripheralRef<'d, P6>,
    pin_7: PeripheralRef<'d, P7>,
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
where
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
    P3: OutputPin,
    P4: OutputPin,
    P5: OutputPin,
    P6: OutputPin,
    P7: OutputPin,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pin_0: impl Peripheral<P = P0> + 'd,
        pin_1: impl Peripheral<P = P1> + 'd,
        pin_2: impl Peripheral<P = P2> + 'd,
        pin_3: impl Peripheral<P = P3> + 'd,
        pin_4: impl Peripheral<P = P4> + 'd,
        pin_5: impl Peripheral<P = P5> + 'd,
        pin_6: impl Peripheral<P = P6> + 'd,
        pin_7: impl Peripheral<P = P7> + 'd,
    ) -> Self {
        crate::into_ref!(pin_0);
        crate::into_ref!(pin_1);
        crate::into_ref!(pin_2);
        crate::into_ref!(pin_3);
        crate::into_ref!(pin_4);
        crate::into_ref!(pin_5);
        crate::into_ref!(pin_6);
        crate::into_ref!(pin_7);

        Self {
            pin_0,
            pin_1,
            pin_2,
            pin_3,
            pin_4,
            pin_5,
            pin_6,
            pin_7,
        }
    }
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> TxPins for TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
where
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
    P3: OutputPin,
    P4: OutputPin,
    P5: OutputPin,
    P6: OutputPin,
    P7: OutputPin,
{
    type Word = u8;

    fn configure(&mut self) {
        self.pin_0
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_0);
        self.pin_1
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_1);
        self.pin_2
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_2);
        self.pin_3
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_3);
        self.pin_4
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_4);
        self.pin_5
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_5);
        self.pin_6
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_6);
        self.pin_7
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_7);
    }
}

pub struct TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15> {
    pin_0: PeripheralRef<'d, P0>,
    pin_1: PeripheralRef<'d, P1>,
    pin_2: PeripheralRef<'d, P2>,
    pin_3: PeripheralRef<'d, P3>,
    pin_4: PeripheralRef<'d, P4>,
    pin_5: PeripheralRef<'d, P5>,
    pin_6: PeripheralRef<'d, P6>,
    pin_7: PeripheralRef<'d, P7>,
    pin_8: PeripheralRef<'d, P8>,
    pin_9: PeripheralRef<'d, P9>,
    pin_10: PeripheralRef<'d, P10>,
    pin_11: PeripheralRef<'d, P11>,
    pin_12: PeripheralRef<'d, P12>,
    pin_13: PeripheralRef<'d, P13>,
    pin_14: PeripheralRef<'d, P14>,
    pin_15: PeripheralRef<'d, P15>,
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
    TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
where
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
    P3: OutputPin,
    P4: OutputPin,
    P5: OutputPin,
    P6: OutputPin,
    P7: OutputPin,
    P8: OutputPin,
    P9: OutputPin,
    P10: OutputPin,
    P11: OutputPin,
    P12: OutputPin,
    P13: OutputPin,
    P14: OutputPin,
    P15: OutputPin,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pin_0: impl Peripheral<P = P0> + 'd,
        pin_1: impl Peripheral<P = P1> + 'd,
        pin_2: impl Peripheral<P = P2> + 'd,
        pin_3: impl Peripheral<P = P3> + 'd,
        pin_4: impl Peripheral<P = P4> + 'd,
        pin_5: impl Peripheral<P = P5> + 'd,
        pin_6: impl Peripheral<P = P6> + 'd,
        pin_7: impl Peripheral<P = P7> + 'd,
        pin_8: impl Peripheral<P = P8> + 'd,
        pin_9: impl Peripheral<P = P9> + 'd,
        pin_10: impl Peripheral<P = P10> + 'd,
        pin_11: impl Peripheral<P = P11> + 'd,
        pin_12: impl Peripheral<P = P12> + 'd,
        pin_13: impl Peripheral<P = P13> + 'd,
        pin_14: impl Peripheral<P = P14> + 'd,
        pin_15: impl Peripheral<P = P15> + 'd,
    ) -> Self {
        crate::into_ref!(pin_0);
        crate::into_ref!(pin_1);
        crate::into_ref!(pin_2);
        crate::into_ref!(pin_3);
        crate::into_ref!(pin_4);
        crate::into_ref!(pin_5);
        crate::into_ref!(pin_6);
        crate::into_ref!(pin_7);
        crate::into_ref!(pin_8);
        crate::into_ref!(pin_9);
        crate::into_ref!(pin_10);
        crate::into_ref!(pin_11);
        crate::into_ref!(pin_12);
        crate::into_ref!(pin_13);
        crate::into_ref!(pin_14);
        crate::into_ref!(pin_15);

        Self {
            pin_0,
            pin_1,
            pin_2,
            pin_3,
            pin_4,
            pin_5,
            pin_6,
            pin_7,
            pin_8,
            pin_9,
            pin_10,
            pin_11,
            pin_12,
            pin_13,
            pin_14,
            pin_15,
        }
    }
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15> TxPins
    for TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
where
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
    P3: OutputPin,
    P4: OutputPin,
    P5: OutputPin,
    P6: OutputPin,
    P7: OutputPin,
    P8: OutputPin,
    P9: OutputPin,
    P10: OutputPin,
    P11: OutputPin,
    P12: OutputPin,
    P13: OutputPin,
    P14: OutputPin,
    P15: OutputPin,
{
    type Word = u16;
    fn configure(&mut self) {
        self.pin_0
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_0);
        self.pin_1
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_1);
        self.pin_2
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_2);
        self.pin_3
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_3);
        self.pin_4
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_4);
        self.pin_5
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_5);
        self.pin_6
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_6);
        self.pin_7
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_7);
        self.pin_8
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_8);
        self.pin_9
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_9);
        self.pin_10
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_10);
        self.pin_11
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_11);
        self.pin_12
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_12);
        self.pin_13
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_13);
        self.pin_14
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_14);
        self.pin_15
            .set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_15);
    }
}

mod private {
    pub trait TxPins {
        type Word: Copy;
        fn configure(&mut self);
    }
}
