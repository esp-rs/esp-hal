use core::fmt::Formatter;

use embedded_dma::ReadBuffer;
use fugit::HertzU32;

use crate::{
    clock::Clocks,
    dma::{DmaError, DmaPeripheral, Tx},
    gpio::{OutputPin, OutputSignal},
    lcd_cam::{
        lcd::{ClockMode, Phase, Polarity},
        private::calculate_clkm,
        BitOrder,
        ByteOrder,
        Lcd,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::LCD_CAM,
};

pub struct I8080<'d, TX> {
    lcd_cam: PeripheralRef<'d, LCD_CAM>,
    tx_channel: TX,
}

impl<'d, TX: Tx> I8080<'d, TX> {
    pub fn new(
        lcd: Lcd<'d>,
        mut channel: TX,
        frequency: HertzU32,
        mode: ClockMode,
        clocks: &Clocks,
    ) -> Self {
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

        lcd_cam.lcd_clock().write(|w| {
            // Force enable the clock for all configuration registers.
            w.clk_en().set_bit();

            w.lcd_clk_sel().variant((i + 1) as _);
            w.lcd_clkm_div_num().variant(divider.div_num as _);
            w.lcd_clkm_div_b().variant(divider.div_b as _);
            w.lcd_clkm_div_a().variant(divider.div_a as _);

            // LCD_PCLK = LCD_CLK / 2
            w.lcd_clk_equ_sysclk().clear_bit();
            w.lcd_clkcnt_n().variant(2 - 1); // Must not be 0.

            w.lcd_ck_idle_edge()
                .bit(mode.polarity == Polarity::IdleHigh);
            w.lcd_ck_out_edge().bit(mode.phase == Phase::ShiftHigh);
            w
        });

        lcd_cam
            .lcd_ctrl()
            .modify(|_, w| w.lcd_rgb_mode_en().clear_bit());
        lcd_cam
            .lcd_rgb_yuv()
            .modify(|_, w| w.lcd_conv_bypass().clear_bit());

        lcd_cam.lcd_user().modify(|_, w| {
            w.lcd_8bits_order().bit(false);
            w.lcd_bit_order().bit(false);
            w.lcd_byte_order().bit(false);
            w.lcd_2byte_en().bit(false);

            // Be able to send data out in LCD sequence when LCD starts.
            w.lcd_dout().set_bit();
            // Data length in fixed mode. (13 bits)
            w.lcd_dout_cyclelen().variant(0);
            // Disable continuous output.
            w.lcd_always_out_en().clear_bit();

            // w.lcd_update().set_bit();
            // w.lcd_start().set_bit();
            // w.lcd_reset().set_bit();
            w
        });
        lcd_cam.lcd_misc().modify(|_, w| {
            // Set the threshold for Async Tx FIFO full event. (5 bits)
            w.lcd_afifo_threshold_num().variant(0);

            // Configure the setup cycles in LCD non-RGB mode. Setup cycles
            // expected = this value + 1. (6 bit)
            w.lcd_vfk_cyclelen().variant(0);

            // Configure the hold time cycles in LCD non-RGB mode. Hold
            // cycles expected = this value + 1.
            w.lcd_vbk_cyclelen().variant(0);

            // 1: Send the next frame data when the current frame is sent out.
            // 0: LCD stops when the current frame is sent out.
            w.lcd_next_frame_en().clear_bit();

            // Enable blank region when LCD sends data out.
            w.lcd_bk_en().set_bit();

            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DOUT phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_data_set().set_bit();
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DUMMY phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_dummy_set().clear_bit();
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in CMD phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_cmd_set().clear_bit();

            // The default value of LCD_CD
            w.lcd_cd_idle_edge().clear_bit();

            // w.lcd_afifo_reset().set_bit();
            w
        });

        lcd_cam.lcd_user().modify(|_, w| w.lcd_update().set_bit());

        channel.init_channel();

        Self {
            lcd_cam,
            tx_channel: channel,
        }
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

    pub fn with_data_pins<
        D0: OutputPin,
        D1: OutputPin,
        D2: OutputPin,
        D3: OutputPin,
        D4: OutputPin,
        D5: OutputPin,
        D6: OutputPin,
        D7: OutputPin,
    >(
        self,
        d0: impl Peripheral<P = D0> + 'd,
        d1: impl Peripheral<P = D1> + 'd,
        d2: impl Peripheral<P = D2> + 'd,
        d3: impl Peripheral<P = D3> + 'd,
        d4: impl Peripheral<P = D4> + 'd,
        d5: impl Peripheral<P = D5> + 'd,
        d6: impl Peripheral<P = D6> + 'd,
        d7: impl Peripheral<P = D7> + 'd,
    ) -> Self {
        crate::into_ref!(d0);
        crate::into_ref!(d1);
        crate::into_ref!(d2);
        crate::into_ref!(d3);
        crate::into_ref!(d4);
        crate::into_ref!(d5);
        crate::into_ref!(d6);
        crate::into_ref!(d7);

        d0.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_0);
        d1.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_1);
        d2.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_2);
        d3.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_3);
        d4.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_4);
        d5.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_5);
        d6.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_6);
        d7.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_7);

        self
    }
}

impl<'d, TX: Tx> I8080<'d, TX> {
    pub fn set_byte_order(&mut self, byte_order: ByteOrder) -> &mut Self {
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_8bits_order().bit(byte_order != ByteOrder::default()));
        self
    }

    fn setup_send(&mut self, cmd: impl Into<Command>, dummy: u8) {
        // Reset LCD control unit and Async Tx FIFO
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());
        self.lcd_cam
            .lcd_misc()
            .modify(|_, w| w.lcd_afifo_reset().set_bit());

        // Set cmd value
        let cmd = cmd.into();
        match cmd {
            Command::None => {
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_cmd().clear_bit());
            }
            Command::One(value) => {
                self.lcd_cam.lcd_user().modify(|_, w| {
                    w.lcd_cmd().set_bit();
                    w.lcd_cmd_2_cycle_en().clear_bit();
                    w
                });
                self.lcd_cam
                    .lcd_cmd_val()
                    .write(|w| w.lcd_cmd_value().variant(value as _));
            }
            Command::Two(first, second) => {
                self.lcd_cam.lcd_user().modify(|_, w| {
                    w.lcd_cmd().set_bit();
                    w.lcd_cmd_2_cycle_en().set_bit();
                    w
                });
                let cmd = first as u32 | (second as u32) << 16;
                self.lcd_cam
                    .lcd_cmd_val()
                    .write(|w| w.lcd_cmd_value().variant(cmd));
            }
        }

        // Set dummy length
        self.lcd_cam.lcd_user().modify(|_, w| {
            if dummy > 0 {
                // Enable DUMMY phase in LCD sequence when LCD starts.
                w.lcd_dummy().set_bit();
                // Configure DUMMY cycles. DUMMY cycles = this value + 1. (2 bits)
                w.lcd_dummy_cyclelen().variant((dummy - 1) as _);
            } else {
                w.lcd_dummy().clear_bit();
            }
            w
        });
    }

    fn start_send(&mut self) {
        // Setup interrupts.
        self.lcd_cam
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());
        self.lcd_cam
            .lc_dma_int_ena()
            .modify(|_, w| w.lcd_trans_done_int_ena().set_bit());

        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_update().set_bit().lcd_start().set_bit());
    }

    fn tear_down_send(&mut self) {
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_start().clear_bit());

        self.lcd_cam
            .lc_dma_int_ena()
            .modify(|_, w| w.lcd_trans_done_int_ena().clear_bit());
        self.lcd_cam
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());
    }

    fn start_write_bytes_dma<'w>(&mut self, ptr: *const u8, len: usize) -> Result<(), DmaError> {
        if len == 0 {
            // Set transfer length.
            self.lcd_cam
                .lcd_user()
                .modify(|_, w| w.lcd_dout().clear_bit());
        } else {
            // TODO: Return an error instead.
            assert!(len <= 8192);

            // Set transfer length.
            self.lcd_cam.lcd_user().modify(|_, w| {
                w.lcd_dout()
                    .set_bit()
                    .lcd_dout_cyclelen()
                    .variant((len - 1) as _)
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

    pub fn send(
        &mut self,
        cmd: impl Into<Command>,
        dummy: u8,
        data: &[u8],
    ) -> Result<(), DmaError> {
        self.setup_send(cmd, dummy);

        self.start_write_bytes_dma(data.as_ptr(), data.len())?;

        self.start_send();

        while self
            .lcd_cam
            .lc_dma_int_st()
            .read()
            .lcd_trans_done_int_st()
            .bit_is_clear()
        {}

        self.tear_down_send();

        Ok(())
    }

    pub fn send_dma<TXBUF>(
        mut self,
        cmd: impl Into<Command>,
        dummy: u8,
        data: TXBUF,
    ) -> Result<Transfer<'d, TX, TXBUF>, DmaError>
    where
        TXBUF: ReadBuffer<Word = u8>,
    {
        self.setup_send(cmd, dummy);

        let (ptr, len) = unsafe { data.read_buffer() };

        self.start_write_bytes_dma(ptr, len)?;

        self.start_send();

        Ok(Transfer {
            instance: Some(self),
            buffer: Some(data),
        })
    }
}

impl<'d, TX> core::fmt::Debug for I8080<'d, TX> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I8080").finish()
    }
}

/// An in-progress transfer
pub struct Transfer<'d, TX: Tx, BUFFER> {
    instance: Option<I8080<'d, TX>>,
    buffer: Option<BUFFER>,
}

impl<'d, TX: Tx, BUFFER> Transfer<'d, TX, BUFFER> {
    pub fn wait(mut self) -> Result<(BUFFER, I8080<'d, TX>), (DmaError, BUFFER, I8080<'d, TX>)> {
        let mut instance = self
            .instance
            .take()
            .expect("instance must be available throughout object lifetime");

        {
            let int_st = instance.lcd_cam.lc_dma_int_st();
            while int_st.read().lcd_trans_done_int_st().bit_is_clear() {
                // Wait until LCD_TRANS_DONE is set.
            }
            instance.tear_down_send();
        }

        let buffer = self
            .buffer
            .take()
            .expect("buffer must be available throughout object lifetime");

        if instance.tx_channel.has_error() {
            Err((DmaError::DescriptorError, buffer, instance))
        } else {
            Ok((buffer, instance))
        }
    }

    pub fn is_done(&self) -> bool {
        // let ch = &self.instance.tx_channel;
        // ch.is_done()

        let int_st = self
            .instance
            .as_ref()
            .expect("instance must be available throughout object lifetime")
            .lcd_cam
            .lc_dma_int_st();
        int_st.read().lcd_trans_done_int_st().bit_is_set()
    }
}

impl<'d, TX: Tx, BUFFER> Drop for Transfer<'d, TX, BUFFER> {
    fn drop(&mut self) {
        if let Some(instance) = self.instance.as_mut() {
            // This will cancel the transfer.
            instance.tear_down_send();
        }
    }
}

/// LCD_CAM I8080 command.
///
/// Can be [Command::None] if command phase should be suppressed.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    None,
    One(u8),
    Two(u8, u8),
}

impl From<u8> for Command {
    fn from(value: u8) -> Self {
        Command::One(value)
    }
}
