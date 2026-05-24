use super::*;

#[cfg_attr(i2c_master_version = "1", path = "v1.rs")]
#[cfg_attr(i2c_master_version = "2", path = "v2.rs")]
#[cfg_attr(i2c_master_version = "3", path = "v3.rs")]
mod version;

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(super) struct I2cFuture<'a> {
    events: EnumSet<Event>,
    driver: Driver<'a>,
    deadline: Option<Instant>,
    /// True if the Future has been polled to completion.
    finished: bool,
}

impl<'a> I2cFuture<'a> {
    pub fn new(events: EnumSet<Event>, driver: Driver<'a>, deadline: Option<Instant>) -> Self {
        driver.regs().int_ena().modify(|_, w| {
            for event in events {
                match event {
                    Event::EndDetect => w.end_detect().set_bit(),
                    Event::TxComplete => w.trans_complete().set_bit(),
                    #[cfg(i2c_master_has_tx_fifo_watermark)]
                    Event::TxFifoWatermark => w.txfifo_wm().set_bit(),
                };
            }

            w.arbitration_lost().set_bit();
            w.time_out().set_bit();
            w.nack().set_bit();
            #[cfg(i2c_master_has_fsm_timeouts)]
            {
                w.scl_main_st_to().set_bit();
                w.scl_st_to().set_bit();
            }

            w
        });

        Self::new_blocking(events, driver, deadline)
    }

    pub fn new_blocking(
        events: EnumSet<Event>,
        driver: Driver<'a>,
        deadline: Option<Instant>,
    ) -> Self {
        Self {
            events,
            driver,
            deadline,
            finished: false,
        }
    }

    fn is_done(&self) -> bool {
        !self.driver.info.interrupts().is_disjoint(self.events)
    }

    fn poll_completion(&mut self) -> Poll<Result<(), Error>> {
        // Grab the current time before doing anything. This will ensure that a long
        // interruption still allows the peripheral sufficient time to complete the
        // operation (i.e. it ensures that the deadline is "at least", not "at most").
        let now = if self.deadline.is_some() {
            Instant::now()
        } else {
            Instant::EPOCH
        };
        let error = self.driver.check_errors();

        let result = if self.is_done() {
            // Even though we are done, we have to check for NACK and arbitration loss.
            let result = if error == Err(Error::Timeout) {
                // We are both done, and timed out. Likely the transaction has completed, but we
                // checked too late?
                Ok(())
            } else {
                error
            };
            Poll::Ready(result)
        } else if error.is_err() {
            Poll::Ready(error)
        } else if let Some(deadline) = self.deadline
            && now > deadline
        {
            // If the deadline is reached, we return an error.
            Poll::Ready(Err(Error::Timeout))
        } else {
            Poll::Pending
        };

        if result.is_ready() {
            self.finished = true;
        }

        result
    }
}

impl core::future::Future for I2cFuture<'_> {
    type Output = Result<(), Error>;

    fn poll(mut self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        self.driver.state.waker.register(ctx.waker());

        let result = self.poll_completion();

        if result.is_pending() && self.deadline.is_some() {
            ctx.waker().wake_by_ref();
        }

        result
    }
}

impl Drop for I2cFuture<'_> {
    fn drop(&mut self) {
        if !self.finished {
            let result = self.poll_completion();
            if result.is_pending() || result == Poll::Ready(Err(Error::Timeout)) {
                self.driver.reset_fsm(true);
            }
        }
    }
}

#[ram]
pub(super) fn async_handler(info: &Info, state: &State) {
    // Disable all interrupts. The I2C Future will check events based on the
    // interrupt status bits.
    info.regs().int_ena().write(|w| unsafe { w.bits(0) });

    state.waker.wake();
}

/// Sets the filter with a supplied threshold in clock cycles for which a
/// pulse must be present to pass the filter
fn set_filter(
    register_block: &RegisterBlock,
    sda_threshold: Option<u8>,
    scl_threshold: Option<u8>,
) {
    cfg_if::cfg_if! {
        if #[cfg(i2c_master_separate_filter_config_registers)] {
            register_block.sda_filter_cfg().modify(|_, w| {
                if let Some(threshold) = sda_threshold {
                    unsafe { w.sda_filter_thres().bits(threshold) };
                }
                w.sda_filter_en().bit(sda_threshold.is_some())
            });
            register_block.scl_filter_cfg().modify(|_, w| {
                if let Some(threshold) = scl_threshold {
                    unsafe { w.scl_filter_thres().bits(threshold) };
                }
                w.scl_filter_en().bit(scl_threshold.is_some())
            });
        } else {
            register_block.filter_cfg().modify(|_, w| {
                if let Some(threshold) = sda_threshold {
                    unsafe { w.sda_filter_thres().bits(threshold) };
                }
                if let Some(threshold) = scl_threshold {
                    unsafe { w.scl_filter_thres().bits(threshold) };
                }
                w.sda_filter_en().bit(sda_threshold.is_some());
                w.scl_filter_en().bit(scl_threshold.is_some())
            });
        }
    }
}

#[expect(clippy::too_many_arguments)]
#[allow(unused)]
/// Configures the clock and timing parameters for the I2C peripheral.
fn configure_clock(
    info: &Info,
    sclk_div: u32,
    scl_low_period: u32,
    scl_high_period: u32,
    scl_wait_high_period: u32,
    sda_hold_time: u32,
    sda_sample_time: u32,
    scl_rstart_setup_time: u32,
    scl_stop_setup_time: u32,
    scl_start_hold_time: u32,
    scl_stop_hold_time: u32,
    timeout: Option<u32>,
) -> Result<(), ConfigError> {
    unsafe {
        cfg_if::cfg_if! {
            if #[cfg(all(soc_has_pcr, soc_has_i2c1))] {
                crate::peripherals::PCR::regs().i2c_sclk_conf(info.id as usize).modify(|_, w| {
                    w.i2c_sclk_sel().clear_bit();
                    w.i2c_sclk_div_num().bits((sclk_div - 1) as u8);
                    w.i2c_sclk_en().set_bit()
                });
            } else if #[cfg(soc_has_pcr)] {
                crate::peripherals::PCR::regs().i2c_sclk_conf().modify(|_, w| {
                    w.i2c_sclk_sel().clear_bit();
                    w.i2c_sclk_div_num().bits((sclk_div - 1) as u8);
                    w.i2c_sclk_en().set_bit()
                });
            } else if #[cfg(i2c_master_version = "3")] {
                info.regs().clk_conf().modify(|_, w| {
                    w.sclk_sel().clear_bit();
                    w.sclk_div_num().bits((sclk_div - 1) as u8)
                });
            }
        }

        // scl period
        info.regs()
            .scl_low_period()
            .write(|w| w.scl_low_period().bits(scl_low_period as u16));

        #[cfg(not(i2c_master_version = "1"))]
        let scl_wait_high_period = scl_wait_high_period
            .try_into()
            .map_err(|_| ConfigError::FrequencyOutOfRange)?;

        info.regs().scl_high_period().write(|w| {
            #[cfg(not(i2c_master_version = "1"))] // ESP32 does not have a wait_high field
            w.scl_wait_high_period().bits(scl_wait_high_period);
            w.scl_high_period().bits(scl_high_period as u16)
        });

        // sda sample
        info.regs()
            .sda_hold()
            .write(|w| w.time().bits(sda_hold_time as u16));
        info.regs()
            .sda_sample()
            .write(|w| w.time().bits(sda_sample_time as u16));

        // setup
        info.regs()
            .scl_rstart_setup()
            .write(|w| w.time().bits(scl_rstart_setup_time as u16));
        info.regs()
            .scl_stop_setup()
            .write(|w| w.time().bits(scl_stop_setup_time as u16));

        // hold
        info.regs()
            .scl_start_hold()
            .write(|w| w.time().bits(scl_start_hold_time as u16));
        info.regs()
            .scl_stop_hold()
            .write(|w| w.time().bits(scl_stop_hold_time as u16));

        cfg_if::cfg_if! {
            if #[cfg(i2c_master_has_bus_timeout_enable)] {
                info.regs().to().write(|w| {
                    w.time_out_en().bit(timeout.is_some());
                    w.time_out_value().bits(timeout.unwrap_or(1) as _)
                });
            } else {
                info.regs()
                    .to()
                    .write(|w| w.time_out().bits(timeout.unwrap_or(1)));
            }
        }
    }
    Ok(())
}

/// Peripheral data describing a particular I2C instance.
#[doc(hidden)]
#[derive(Debug)]
#[non_exhaustive]
#[allow(private_interfaces, reason = "Unstable details")]
pub struct Info {
    /// Numeric instance id (0 = I2C0, 1 = I2C1, ...)
    #[cfg(soc_has_i2c1)]
    pub id: u8,

    /// Pointer to the register block for this I2C instance.
    ///
    /// Use [Self::register_block] to access the register block.
    pub register_block: *const RegisterBlock,

    /// System peripheral marker.
    pub peripheral: crate::system::Peripheral,

    /// Interrupt handler for the asynchronous operations of this I2C instance.
    pub async_handler: InterruptHandler,

    /// SCL output signal.
    pub scl_output: OutputSignal,

    /// SCL input signal.
    pub scl_input: InputSignal,

    /// SDA output signal.
    pub sda_output: OutputSignal,

    /// SDA input signal.
    pub sda_input: InputSignal,
}

impl Info {
    /// Returns the register block for this I2C instance.
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Listen for the given interrupts
    pub(super) fn enable_listen(&self, interrupts: EnumSet<Event>, enable: bool) {
        let reg_block = self.regs();

        reg_block.int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    Event::EndDetect => w.end_detect().bit(enable),
                    Event::TxComplete => w.trans_complete().bit(enable),
                    #[cfg(i2c_master_has_tx_fifo_watermark)]
                    Event::TxFifoWatermark => w.txfifo_wm().bit(enable),
                };
            }
            w
        });
    }

    pub(super) fn interrupts(&self) -> EnumSet<Event> {
        let mut res = EnumSet::new();
        let reg_block = self.regs();

        let ints = reg_block.int_raw().read();

        if ints.end_detect().bit_is_set() {
            res.insert(Event::EndDetect);
        }
        if ints.trans_complete().bit_is_set() {
            res.insert(Event::TxComplete);
        }
        #[cfg(i2c_master_has_tx_fifo_watermark)]
        if ints.txfifo_wm().bit_is_set() {
            res.insert(Event::TxFifoWatermark);
        }

        res
    }

    pub(super) fn clear_interrupts(&self, interrupts: EnumSet<Event>) {
        let reg_block = self.regs();

        reg_block.int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    Event::EndDetect => w.end_detect().clear_bit_by_one(),
                    Event::TxComplete => w.trans_complete().clear_bit_by_one(),
                    #[cfg(i2c_master_has_tx_fifo_watermark)]
                    Event::TxFifoWatermark => w.txfifo_wm().clear_bit_by_one(),
                };
            }
            w
        });
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_block, other.register_block)
    }
}

unsafe impl Sync for Info {}

#[derive(Clone, Copy)]
enum Deadline {
    None,
    Fixed(Instant),
    PerByte(Duration),
}

impl Deadline {
    fn start(self, data_len: usize) -> Option<Instant> {
        match self {
            Deadline::None => None,
            Deadline::Fixed(deadline) => Some(deadline),
            Deadline::PerByte(duration) => Some(Instant::now() + duration * data_len as u32),
        }
    }
}

#[allow(dead_code)] // Some versions don't need `state`
#[derive(Clone, Copy)]
pub(super) struct Driver<'a> {
    pub(super) info: &'a Info,
    pub(super) state: &'a State,
    pub(super) config: &'a DriverConfig,
}

impl Driver<'_> {
    fn regs(&self) -> &RegisterBlock {
        self.info.regs()
    }

    pub(super) fn connect_pin(
        pin: crate::gpio::interconnect::OutputSignal<'_>,
        input: InputSignal,
        output: OutputSignal,
        guard: &mut PinGuard,
    ) {
        // avoid the pin going low during configuration
        pin.set_output_high(true);

        pin.apply_output_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up),
        );
        pin.set_output_enable(true);
        pin.set_input_enable(true);

        input.connect_to(&pin);

        *guard = interconnect::OutputSignal::connect_with_guard(pin, output);
    }

    fn init_master(&self) {
        self.regs().ctr().write(|w| {
            // Set I2C controller to master mode
            w.ms_mode().set_bit();
            // Use open drain output for SDA and SCL
            w.sda_force_out().set_bit();
            w.scl_force_out().set_bit();
            // Use Most Significant Bit first for sending and receiving data
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit();

            #[cfg(i2c_master_has_arbitration_en)]
            w.arbitration_en().clear_bit();

            #[cfg(i2c_master_version = "2")]
            w.ref_always_on().set_bit();

            // Ensure that clock is enabled
            w.clk_en().set_bit()
        });
    }

    /// Configures the I2C peripheral with the specified frequency, clocks, and
    /// optional timeout.
    pub(super) fn setup(&self, config: &Config) -> Result<(), ConfigError> {
        self.init_master();

        // Configure filter
        // FIXME if we ever change this we need to adapt `set_frequency` for ESP32
        set_filter(self.regs(), Some(7), Some(7));

        // Configure frequency
        self.set_frequency(config)?;

        // Configure additional timeouts
        #[cfg(i2c_master_has_fsm_timeouts)]
        {
            self.regs()
                .scl_st_time_out()
                .write(|w| unsafe { w.scl_st_to().bits(config.scl_st_timeout.value()) });
            self.regs()
                .scl_main_st_time_out()
                .write(|w| unsafe { w.scl_main_st_to().bits(config.scl_main_st_timeout.value()) });
        }

        self.update_registers();

        Ok(())
    }

    fn do_fsm_reset(&self) {
        cfg_if::cfg_if! {
            if #[cfg(i2c_master_has_reliable_fsm_reset)] {
                // Device has a working FSM reset mechanism
                self.regs().ctr().modify(|_, w| w.fsm_rst().set_bit());
            } else {
                // Even though C2 and C3 have a FSM reset bit, esp-idf does not
                // define I2C_LL_SUPPORT_HW_FSM_RST for them, so include them in the fallback impl.

                crate::system::PeripheralClockControl::reset(self.info.peripheral);

                // Restore configuration. This operation has succeeded once, so we can
                // assume that the config is valid and we can ignore the result.
                self.setup(&self.config.config).ok();
            }
        }
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    // This function implements esp-idf's `s_i2c_hw_fsm_reset`
    // https://github.com/espressif/esp-idf/blob/27d68f57e6bdd3842cd263585c2c352698a9eda2/components/esp_driver_i2c/i2c_master.c#L115
    //
    // Make sure you don't call this function in the middle of a transaction. If the
    // first command in the command list is not a START, the hardware will hang
    // with no timeouts.
    pub(super) fn reset_fsm(&self, clear_bus: bool) {
        if clear_bus {
            self.clear_bus_blocking(true);
        } else {
            self.do_fsm_reset();
        }
    }

    fn bus_busy(&self) -> bool {
        self.regs().sr().read().bus_busy().bit_is_set()
    }

    fn ensure_idle_blocking(&self) {
        if self.bus_busy() {
            // If the bus is busy, we need to clear it.
            self.clear_bus_blocking(false);
        }
    }

    async fn ensure_idle(&self) {
        if self.bus_busy() {
            // If the bus is busy, we need to clear it.
            self.clear_bus().await;
        }
    }

    fn reset_before_transmission(&self) {
        // Clear all I2C interrupts
        self.clear_all_interrupts();

        // Reset fifo
        self.reset_fifo();

        // Reset the command list
        self.reset_command_list();
    }

    /// Implements s_i2c_master_clear_bus
    ///
    /// If a transaction ended incorrectly for some reason, the slave may drive
    /// SDA indefinitely. This function forces the slave to release the
    /// bus by sending 9 clock pulses.
    fn clear_bus_blocking(&self, reset_fsm: bool) {
        let mut future = ClearBusFuture::new(*self, reset_fsm);
        let start = Instant::now();
        while future.poll_completion().is_pending() {
            if start.elapsed() > CLEAR_BUS_TIMEOUT_MS {
                break;
            }
        }
    }

    async fn clear_bus(&self) {
        let clear_bus = ClearBusFuture::new(*self, true);
        let start = Instant::now();

        embassy_futures::select::select(clear_bus, async {
            while start.elapsed() < CLEAR_BUS_TIMEOUT_MS {
                embassy_futures::yield_now().await;
            }
        })
        .await;
    }

    /// Resets the I2C peripheral's command registers.
    fn reset_command_list(&self) {
        for cmd in self.regs().comd_iter() {
            cmd.reset();
        }
    }

    /// Configures the I2C peripheral for a write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation will end with a STOP condition.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn setup_write<'a, I>(
        &self,
        addr: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        // If start is true we need to send the address, too, which takes up a data
        // byte.
        let max_len = if start {
            I2C_CHUNK_SIZE
        } else {
            I2C_CHUNK_SIZE + 1
        };
        if bytes.len() > max_len {
            return Err(Error::FifoExceeded);
        }

        if start {
            add_cmd(cmd_iterator, Command::Start)?;
        }

        let write_len = if start { bytes.len() + 1 } else { bytes.len() };
        // don't issue write if there is no data to write
        if write_len > 0 {
            // ESP32 can't alter the position of END, so we need to split the chunk always into
            // 3-command sequences. Chunking makes sure not to place a 1-byte
            // command at the end, which would cause an arithmetic underflow.
            // The sequences we can generate are:
            // - START-WRITE-STOP
            // - START-WRITE-END-WRITE-STOP
            // - START-WRITE-END-(WRITE-WRITE-END)*-WRITE-STOP sequence.
            if cfg!(i2c_master_version = "1") && !(start || stop) {
                // Chunks that do not have a START or STOP command need to be split into multiple
                // commands.
                add_cmd(
                    cmd_iterator,
                    Command::Write {
                        ack_exp: Ack::Ack,
                        ack_check_en: true,
                        length: (write_len as u8) - 1,
                    },
                )?;
                add_cmd(
                    cmd_iterator,
                    Command::Write {
                        ack_exp: Ack::Ack,
                        ack_check_en: true,
                        length: 1,
                    },
                )?;
            } else {
                add_cmd(
                    cmd_iterator,
                    Command::Write {
                        ack_exp: Ack::Ack,
                        ack_check_en: true,
                        length: write_len as u8,
                    },
                )?;
            }
        }

        if start {
            // Load address and R/W bit into FIFO
            match addr {
                I2cAddress::SevenBit(addr) => {
                    self.write_fifo((addr << 1) | OperationType::Write as u8);
                }
            }
        }
        for b in bytes {
            self.write_fifo(*b);
        }

        Ok(())
    }

    /// Configures the I2C peripheral for a read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation will end with a STOP condition.
    /// - `will_continue` indicates whether there is another read operation following this one and
    ///   we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn setup_read<'a, I>(
        &self,
        addr: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        if buffer.is_empty() {
            return Err(Error::ZeroLengthInvalid);
        }
        let (max_len, initial_len) = if will_continue {
            (I2C_CHUNK_SIZE + 1, buffer.len())
        } else {
            (I2C_CHUNK_SIZE, buffer.len() - 1)
        };
        if buffer.len() > max_len {
            return Err(Error::FifoExceeded);
        }

        if start {
            add_cmd(cmd_iterator, Command::Start)?;
            // WRITE 7-bit address
            add_cmd(
                cmd_iterator,
                Command::Write {
                    ack_exp: Ack::Ack,
                    ack_check_en: true,
                    length: 1,
                },
            )?;
        }

        if initial_len > 0 {
            let extra_commands = if cfg!(i2c_master_version = "1") {
                match (start, will_continue) {
                    // No chunking (START-WRITE-READ-STOP) or first chunk (START-WRITE-READ-END)
                    (true, _) => 0,
                    // Middle chunk - (READ-READ-READ-END)
                    (false, true) => 2,
                    // Last chunk - (READ-READ-STOP-END)
                    (false, false) => 1 - stop as u8,
                }
            } else {
                0
            };

            add_cmd(
                cmd_iterator,
                Command::Read {
                    ack_value: Ack::Ack,
                    length: initial_len as u8 - extra_commands,
                },
            )?;
            for _ in 0..extra_commands {
                add_cmd(
                    cmd_iterator,
                    Command::Read {
                        ack_value: Ack::Ack,
                        length: 1,
                    },
                )?;
            }
        }

        if !will_continue {
            // this is the last read so we need to nack the last byte
            // READ w/o ACK
            add_cmd(
                cmd_iterator,
                Command::Read {
                    ack_value: Ack::Nack,
                    length: 1,
                },
            )?;
        }

        self.update_registers();

        if start {
            // Load address and R/W bit into FIFO
            match addr {
                I2cAddress::SevenBit(addr) => {
                    self.write_fifo((addr << 1) | OperationType::Read as u8);
                }
            }
        }
        Ok(())
    }

    /// Reads from RX FIFO into the given buffer.
    fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        if self.regs().sr().read().rxfifo_cnt().bits() < buffer.len() as u8 {
            return Err(Error::ExecutionIncomplete);
        }

        // Read bytes from FIFO
        for byte in buffer.iter_mut() {
            *byte = self.read_fifo();
        }

        // The RX FIFO should be empty now. If it is not, it means we queued up reading
        // more data than we read, which is an error.
        debug_assert!(self.regs().sr().read().rxfifo_cnt().bits() == 0);

        Ok(())
    }

    /// Clears all pending interrupts for the I2C peripheral.
    fn clear_all_interrupts(&self) {
        self.regs()
            .int_clr()
            .write(|w| unsafe { w.bits(property!("i2c_master.ll_intr_mask")) });
    }

    async fn wait_for_completion(&self, deadline: Option<Instant>) -> Result<(), Error> {
        I2cFuture::new(Event::TxComplete | Event::EndDetect, *self, deadline).await?;
        self.check_all_commands_done(deadline).await
    }

    /// Waits for the completion of an I2C transaction.
    fn wait_for_completion_blocking(&self, deadline: Option<Instant>) -> Result<(), Error> {
        let mut future =
            I2cFuture::new_blocking(Event::TxComplete | Event::EndDetect, *self, deadline);
        loop {
            if let Poll::Ready(result) = future.poll_completion() {
                result?;
                return self.check_all_commands_done_blocking(deadline);
            }
        }
    }

    fn all_commands_done(&self, deadline: Option<Instant>) -> Result<bool, Error> {
        // NOTE: on esp32 executing the end command generates the end_detect interrupt
        //       but does not seem to clear the done bit! So we don't check the done
        //       status of an end command
        let now = if deadline.is_some() {
            Instant::now()
        } else {
            Instant::EPOCH
        };

        self.check_errors()?;

        for cmd_reg in self.regs().comd_iter() {
            let cmd = cmd_reg.read();

            // if there is a valid command which is not END, check if it's marked as done
            if cmd.bits() != 0x0 && !cmd.opcode().is_end() && !cmd.command_done().bit_is_set() {
                // Let's retry
                if let Some(deadline) = deadline
                    && now > deadline
                {
                    return Err(Error::ExecutionIncomplete);
                }

                return Ok(false);
            }

            // once we hit END or STOP we can break the loop
            if cmd.opcode().is_end() {
                break;
            }
            if cmd.opcode().is_stop() {
                #[cfg(i2c_master_version = "1")]
                // wait for STOP - apparently on ESP32 we otherwise miss the ACK error for an
                // empty write
                if self.regs().sr().read().scl_state_last() == 6 {
                    self.check_errors()?;
                } else {
                    continue;
                }
                break;
            }
        }
        Ok(true)
    }

    /// Checks whether all I2C commands have completed execution.
    fn check_all_commands_done_blocking(&self, deadline: Option<Instant>) -> Result<(), Error> {
        // loop until commands are actually done
        while !self.all_commands_done(deadline)? {}
        self.check_errors()?;

        Ok(())
    }

    /// Checks whether all I2C commands have completed execution.
    async fn check_all_commands_done(&self, deadline: Option<Instant>) -> Result<(), Error> {
        // loop until commands are actually done
        while !self.all_commands_done(deadline)? {
            embassy_futures::yield_now().await;
        }
        self.check_errors()?;

        Ok(())
    }

    /// Checks for I2C transmission errors and handles them.
    ///
    /// This function inspects specific I2C-related interrupts to detect errors
    /// during communication, such as timeouts, failed acknowledgments, or
    /// arbitration loss. If an error is detected, the function handles it
    /// by resetting the I2C peripheral to clear the error condition and then
    /// returns an appropriate error.
    fn check_errors(&self) -> Result<(), Error> {
        let r = self.regs().int_raw().read();

        // Handle error cases
        if r.nack().bit_is_set() {
            return Err(Error::AcknowledgeCheckFailed(estimate_ack_failed_reason(
                self.regs(),
            )));
        }
        if r.arbitration_lost().bit_is_set() {
            return Err(Error::ArbitrationLost);
        }

        #[cfg(not(i2c_master_version = "1"))]
        if r.trans_complete().bit_is_set() && self.regs().sr().read().resp_rec().bit_is_clear() {
            return Err(Error::AcknowledgeCheckFailed(
                AcknowledgeCheckFailedReason::Data,
            ));
        }

        #[cfg(i2c_master_has_fsm_timeouts)]
        {
            if r.scl_st_to().bit_is_set() {
                return Err(Error::Timeout);
            }
            if r.scl_main_st_to().bit_is_set() {
                return Err(Error::Timeout);
            }
        }
        if r.time_out().bit_is_set() {
            return Err(Error::Timeout);
        }

        Ok(())
    }

    /// Updates the configuration of the I2C peripheral.
    ///
    /// This function ensures that the configuration values, such as clock
    /// settings, SDA/SCL filtering, timeouts, and other operational
    /// parameters, which are configured in other functions, are properly
    /// propagated to the I2C hardware. This step is necessary to synchronize
    /// the software-configured settings with the peripheral's internal
    /// registers, ensuring that the hardware behaves according to the
    /// current configuration.
    fn update_registers(&self) {
        // Ensure that the configuration of the peripheral is correctly propagated
        // (only necessary for C2, C3, C6, H2 and S3 variant)
        #[cfg(i2c_master_has_conf_update)]
        self.regs().ctr().modify(|_, w| w.conf_upgate().set_bit());
    }

    fn set_frequency(&self, config: &Config) -> Result<(), ConfigError> {
        version::set_frequency(self, config)
    }

    fn reset_fifo(&self) {
        version::reset_fifo(self);
    }

    fn read_fifo(&self) -> u8 {
        version::read_fifo(self.regs())
    }

    fn write_fifo(&self, data: u8) {
        version::write_fifo(self.regs(), data);
    }

    /// Starts an I2C transmission.
    fn start_transmission(&self) {
        // Start transmission
        self.regs().ctr().modify(|_, w| w.trans_start().set_bit());
    }

    fn start_write_operation(
        &self,
        address: I2cAddress,
        buffer: &[u8],
        start: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<Option<Instant>, Error> {
        let cmd_iterator = &mut self.regs().comd_iter();

        self.setup_write(address, buffer, start, stop, cmd_iterator)?;

        if stop {
            add_cmd(cmd_iterator, Command::Stop)?;
        }
        if !(start && stop) {
            // Multi-chunk write, terminate with END. ESP32 TRM suggests a write should work with
            // only a STOP at the end, but STOP does not seem to generate a TX_COMPLETE interrupt
            // without END.
            add_cmd(cmd_iterator, Command::End)?;
        }

        self.start_transmission();

        Ok(deadline.start(buffer.len() + address.bytes()))
    }

    /// Executes an I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `will_continue` indicates whether there is another read operation following this one and
    ///   we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn start_read_operation(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        will_continue: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<Option<Instant>, Error> {
        // We don't support single I2C reads larger than the FIFO. This should be
        // enforced by `VariableChunkIterMut` used in `Driver::read` and
        // `Driver::read_async`.
        debug_assert!(buffer.len() <= I2C_FIFO_SIZE);

        let cmd_iterator = &mut self.regs().comd_iter();

        self.setup_read(address, buffer, start, stop, will_continue, cmd_iterator)?;

        if stop {
            add_cmd(cmd_iterator, Command::Stop)?;
        }
        if !(start && stop) {
            // Multi-chunk read, terminate with END. On ESP32, assume same limitation as writes.
            add_cmd(cmd_iterator, Command::End)?;
        }

        self.start_transmission();

        Ok(deadline.start(buffer.len() + address.bytes()))
    }

    /// Executes an I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn write_operation_blocking(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        address.validate()?;

        self.reset_before_transmission();

        // Short circuit for zero length writes without start or end as that would be an
        // invalid operation write lengths in the TRM (at least for ESP32-S3) are 1-255
        if bytes.is_empty() && !start && !stop {
            return Ok(());
        }

        let deadline = self.start_write_operation(address, bytes, start, stop, deadline)?;
        self.wait_for_completion_blocking(deadline)?;

        Ok(())
    }

    /// Executes an I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `will_continue` indicates whether there is another read operation following this one and
    ///   we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn read_operation_blocking(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        address.validate()?;
        self.reset_before_transmission();

        // Short circuit for zero length reads as that would be an invalid operation
        // read lengths in the TRM (at least for ESP32-S3) are 1-255
        if buffer.is_empty() {
            return Ok(());
        }

        let deadline =
            self.start_read_operation(address, buffer, start, will_continue, stop, deadline)?;
        self.wait_for_completion_blocking(deadline)?;
        self.read_all_from_fifo(buffer)?;

        Ok(())
    }

    /// Executes an async I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `cmd_iterator` is an iterator over the command registers.
    async fn write_operation(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        address.validate()?;
        self.reset_before_transmission();

        // Short circuit for zero length writes without start or end as that would be an
        // invalid operation write lengths in the TRM (at least for ESP32-S3) are 1-255
        if bytes.is_empty() && !start && !stop {
            return Ok(());
        }

        let deadline = self.start_write_operation(address, bytes, start, stop, deadline)?;
        self.wait_for_completion(deadline).await?;

        Ok(())
    }

    /// Executes an async I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `will_continue` indicates whether there is another read operation following this one and
    ///   we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    async fn read_operation(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        address.validate()?;
        self.reset_before_transmission();

        // Short circuit for zero length reads as that would be an invalid operation
        // read lengths in the TRM (at least for ESP32-S3) are 1-255
        if buffer.is_empty() {
            return Ok(());
        }

        let deadline =
            self.start_read_operation(address, buffer, start, will_continue, stop, deadline)?;
        self.wait_for_completion(deadline).await?;
        self.read_all_from_fifo(buffer)?;

        Ok(())
    }

    fn read_blocking(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        let chunk_count = VariableChunkIterMut::new(buffer).count();
        for (idx, chunk) in VariableChunkIterMut::new(buffer).enumerate() {
            self.read_operation_blocking(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                will_continue || idx < chunk_count - 1,
                deadline,
            )?;
        }

        Ok(())
    }

    fn write_blocking(
        &self,
        address: I2cAddress,
        buffer: &[u8],
        start: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        if buffer.is_empty() {
            return self.write_operation_blocking(address, &[], start, stop, deadline);
        }

        let chunk_count = VariableChunkIter::new(buffer).count();
        for (idx, chunk) in VariableChunkIter::new(buffer).enumerate() {
            self.write_operation_blocking(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                deadline,
            )?;
        }

        Ok(())
    }

    async fn read(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        let chunk_count = VariableChunkIterMut::new(buffer).count();
        for (idx, chunk) in VariableChunkIterMut::new(buffer).enumerate() {
            self.read_operation(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                will_continue || idx < chunk_count - 1,
                deadline,
            )
            .await?;
        }

        Ok(())
    }

    async fn write(
        &self,
        address: I2cAddress,
        buffer: &[u8],
        start: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        if buffer.is_empty() {
            return self
                .write_operation(address, &[], start, stop, deadline)
                .await;
        }

        let chunk_count = VariableChunkIter::new(buffer).count();
        for (idx, chunk) in VariableChunkIter::new(buffer).enumerate() {
            self.write_operation(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                deadline,
            )
            .await?;
        }

        Ok(())
    }

    pub(super) fn transaction_impl<'a>(
        &self,
        address: I2cAddress,
        operations: impl Iterator<Item = Operation<'a>>,
    ) -> Result<(), Error> {
        address.validate()?;
        self.ensure_idle_blocking();

        let mut deadline = Deadline::None;

        if let SoftwareTimeout::Transaction(timeout) = self.config.config.software_timeout {
            deadline = Deadline::Fixed(Instant::now() + timeout);
        }

        let mut last_op: Option<OpKind> = None;
        // filter out 0 length read operations
        let mut op_iter = operations
            .filter(|op| op.is_write() || !op.is_empty())
            .peekable();

        while let Some(op) = op_iter.next() {
            let next_op = op_iter.peek().map(|v| v.kind());
            let kind = op.kind();
            match op {
                Operation::Write(buffer) => {
                    // execute a write operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    if let SoftwareTimeout::PerByte(timeout) = self.config.config.software_timeout {
                        deadline = Deadline::PerByte(timeout);
                    }
                    self.write_blocking(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Write)),
                        next_op.is_none(),
                        deadline,
                    )?;
                }
                Operation::Read(buffer) => {
                    if let SoftwareTimeout::PerByte(timeout) = self.config.config.software_timeout {
                        deadline = Deadline::PerByte(timeout);
                    }
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    self.read_blocking(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Read)),
                        next_op.is_none(),
                        matches!(next_op, Some(OpKind::Read)),
                        deadline,
                    )?;
                }
            }

            last_op = Some(kind);
        }

        Ok(())
    }

    pub(super) async fn transaction_impl_async<'a>(
        &self,
        address: I2cAddress,
        operations: impl Iterator<Item = Operation<'a>>,
    ) -> Result<(), Error> {
        address.validate()?;
        self.ensure_idle().await;

        let mut deadline = Deadline::None;

        if let SoftwareTimeout::Transaction(timeout) = self.config.config.software_timeout {
            deadline = Deadline::Fixed(Instant::now() + timeout);
        }

        let mut last_op: Option<OpKind> = None;
        // filter out 0 length read operations
        let mut op_iter = operations
            .filter(|op| op.is_write() || !op.is_empty())
            .peekable();

        while let Some(op) = op_iter.next() {
            let next_op = op_iter.peek().map(|v| v.kind());
            let kind = op.kind();
            match op {
                Operation::Write(buffer) => {
                    if let SoftwareTimeout::PerByte(timeout) = self.config.config.software_timeout {
                        deadline = Deadline::PerByte(timeout);
                    }
                    // execute a write operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    self.write(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Write)),
                        next_op.is_none(),
                        deadline,
                    )
                    .await?;
                }
                Operation::Read(buffer) => {
                    if let SoftwareTimeout::PerByte(timeout) = self.config.config.software_timeout {
                        deadline = Deadline::PerByte(timeout);
                    }
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    self.read(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Read)),
                        next_op.is_none(),
                        matches!(next_op, Some(OpKind::Read)),
                        deadline,
                    )
                    .await?;
                }
            }

            last_op = Some(kind);
        }

        Ok(())
    }
}

/// Chunks a slice by I2C_CHUNK_SIZE in a way to avoid the last chunk being
/// sized smaller than 2
struct VariableChunkIterMut<'a, T> {
    buffer: &'a mut [T],
}

impl<'a, T> VariableChunkIterMut<'a, T> {
    fn new(buffer: &'a mut [T]) -> Self {
        Self { buffer }
    }
}

impl<'a, T> Iterator for VariableChunkIterMut<'a, T> {
    type Item = &'a mut [T];

    fn next(&mut self) -> Option<Self::Item> {
        if self.buffer.is_empty() {
            return None;
        }

        let s = calculate_chunk_size(self.buffer.len());
        let (chunk, remaining) = core::mem::take(&mut self.buffer).split_at_mut(s);
        self.buffer = remaining;
        Some(chunk)
    }
}

/// Chunks a slice by I2C_CHUNK_SIZE in a way to avoid the last chunk being
/// sized smaller than 2
struct VariableChunkIter<'a, T> {
    buffer: &'a [T],
}

impl<'a, T> VariableChunkIter<'a, T> {
    fn new(buffer: &'a [T]) -> Self {
        Self { buffer }
    }
}

impl<'a, T> Iterator for VariableChunkIter<'a, T> {
    type Item = &'a [T];

    fn next(&mut self) -> Option<Self::Item> {
        if self.buffer.is_empty() {
            return None;
        }

        let s = calculate_chunk_size(self.buffer.len());
        let (chunk, remaining) = core::mem::take(&mut self.buffer).split_at(s);
        self.buffer = remaining;
        Some(chunk)
    }
}

fn calculate_chunk_size(remaining: usize) -> usize {
    if remaining <= I2C_CHUNK_SIZE {
        remaining
    } else if remaining > I2C_CHUNK_SIZE + 2 {
        I2C_CHUNK_SIZE
    } else {
        I2C_CHUNK_SIZE - 2
    }
}

#[cfg(i2c_master_has_hw_bus_clear)]
mod bus_clear {
    use esp_rom_sys::rom::ets_delay_us;

    use super::*;

    pub struct ClearBusFuture<'a> {
        driver: Driver<'a>,
    }

    impl<'a> ClearBusFuture<'a> {
        // Number of SCL pulses to clear the bus
        const BUS_CLEAR_BITS: u8 = 9;
        const DELAY_US: u32 = 5; // 5us -> 100kHz

        pub fn new(driver: Driver<'a>, reset_fsm: bool) -> Self {
            // If we have a HW implementation, reset FSM to make sure it's not trying to transmit
            // while we clear the bus.
            if reset_fsm {
                // Resetting the FSM may still generate a short SCL pulse, but I don't know how to
                // work around it - just waiting doesn't solve anything if the hardware is running.
                driver.do_fsm_reset();
            }

            let mut this = Self { driver };

            // Prevent SCL from going low immediately after FSM reset/previous operation has set
            // it high
            ets_delay_us(Self::DELAY_US);

            this.configure(Self::BUS_CLEAR_BITS);
            this
        }

        fn configure(&mut self, bits: u8) {
            self.driver.regs().scl_sp_conf().modify(|_, w| {
                unsafe { w.scl_rst_slv_num().bits(bits) };
                w.scl_rst_slv_en().bit(bits > 0)
            });
            self.driver.update_registers();
        }

        fn is_done(&self) -> bool {
            self.driver
                .regs()
                .scl_sp_conf()
                .read()
                .scl_rst_slv_en()
                .bit_is_clear()
        }

        pub fn poll_completion(&mut self) -> Poll<()> {
            if self.is_done() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl Drop for ClearBusFuture<'_> {
        fn drop(&mut self) {
            use crate::gpio::AnyPin;
            if !self.is_done() {
                self.configure(0);
            }

            // Generate a stop condition
            let sda = self
                .driver
                .config
                .sda_pin
                .pin_number()
                .map(|n| unsafe { AnyPin::steal(n) });
            let scl = self
                .driver
                .config
                .scl_pin
                .pin_number()
                .map(|n| unsafe { AnyPin::steal(n) });

            if let (Some(sda), Some(scl)) = (sda, scl) {
                // Prevent short SCL pulse right after HW clearing completes
                ets_delay_us(Self::DELAY_US);

                sda.set_output_high(true);
                scl.set_output_high(false);

                self.driver.info.scl_output.disconnect_from(&scl);
                self.driver.info.sda_output.disconnect_from(&sda);

                // Set SDA low - whatever state it was in, we need a low -> high transition.
                sda.set_output_high(false);
                ets_delay_us(Self::DELAY_US);

                // Set SCL high to prepare for STOP condition
                scl.set_output_high(true);
                ets_delay_us(Self::DELAY_US);

                // STOP
                sda.set_output_high(true);
                ets_delay_us(Self::DELAY_US);

                self.driver.info.sda_output.connect_to(&sda);
                self.driver.info.scl_output.connect_to(&scl);
            }

            // We don't care about errors during bus clearing
            self.driver.clear_all_interrupts();
        }
    }
}

#[cfg(not(i2c_master_has_hw_bus_clear))]
mod bus_clear {
    use super::*;
    use crate::gpio::AnyPin;

    /// State of the bus clearing operation.
    ///
    /// Pins are changed on the start of the state, and a wait is scheduled
    /// for the end of the state. At the end of the wait, the state is
    /// updated to the next state.
    enum State {
        Idle,
        SendStop,

        // Number of SCL pulses left to send, and the last SCL level.
        //
        // Our job is to send 9 high->low SCL transitions, followed by a STOP condition.
        SendClock(u8, bool),
    }

    pub struct ClearBusFuture<'a> {
        driver: Driver<'a>,
        wait: Instant,
        state: State,
        reset_fsm: bool,
        pins: Option<(AnyPin<'static>, AnyPin<'static>)>,
    }

    impl<'a> ClearBusFuture<'a> {
        // Number of SCL pulses to clear the bus (max 8 data bits sent by the device, + NACK)
        const BUS_CLEAR_BITS: u8 = 9;
        // use standard 100kHz data rate
        const SCL_DELAY: Duration = Duration::from_micros(5);

        pub fn new(driver: Driver<'a>, reset_fsm: bool) -> Self {
            let sda = driver
                .config
                .sda_pin
                .pin_number()
                .map(|n| unsafe { AnyPin::steal(n) });
            let scl = driver
                .config
                .scl_pin
                .pin_number()
                .map(|n| unsafe { AnyPin::steal(n) });

            let (Some(sda), Some(scl)) = (sda, scl) else {
                // If we don't have the pins, we can't clear the bus.
                if reset_fsm {
                    driver.do_fsm_reset();
                }
                return Self {
                    driver,
                    wait: Instant::now(),
                    state: State::Idle,
                    reset_fsm: false,
                    pins: None,
                };
            };

            sda.set_output_high(true);
            scl.set_output_high(false);

            driver.info.scl_output.disconnect_from(&scl);
            driver.info.sda_output.disconnect_from(&sda);

            // Starting from (9, false), becase:
            // - we start with SCL low
            // - a complete SCL cycle consists of a high period and a low period
            // - we decrement the remaining counter at the beginning of a cycle, which gives us 9
            //   complete SCL cycles.
            let state = State::SendClock(Self::BUS_CLEAR_BITS, false);

            Self {
                driver,
                wait: Instant::now() + Self::SCL_DELAY,
                state,
                reset_fsm,
                pins: Some((sda, scl)),
            }
        }
    }

    impl ClearBusFuture<'_> {
        pub fn poll_completion(&mut self) -> Poll<()> {
            let now = Instant::now();

            match self.state {
                State::Idle => {
                    if let Some((sda, _scl)) = self.pins.as_ref() {
                        // Pins are disconnected from the peripheral, we can't use `bus_busy`.
                        if !sda.is_input_high() {
                            return Poll::Pending;
                        }
                    }
                    return Poll::Ready(());
                }
                _ if now < self.wait => {
                    // Still waiting for the end of the SCL pulse
                    return Poll::Pending;
                }
                State::SendStop => {
                    if let Some((sda, _scl)) = self.pins.as_ref() {
                        sda.set_output_high(true); // STOP, SDA low -> high while SCL is HIGH
                    }
                    self.state = State::Idle;
                    return Poll::Pending;
                }
                State::SendClock(0, false) => {
                    if let Some((sda, scl)) = self.pins.as_ref() {
                        // Set up for STOP condition
                        sda.set_output_high(false);
                        scl.set_output_high(true);
                    }
                    self.state = State::SendStop;
                }
                State::SendClock(n, false) => {
                    if let Some((sda, scl)) = self.pins.as_ref() {
                        scl.set_output_high(true);
                        if sda.is_input_high() {
                            sda.set_output_high(false);
                            // The device has released SDA, we can move on to generating a STOP
                            // condition
                            self.wait = Instant::now() + Self::SCL_DELAY;
                            self.state = State::SendStop;
                            return Poll::Pending;
                        }
                    }
                    self.state = State::SendClock(n - 1, true);
                }
                State::SendClock(n, true) => {
                    if let Some((_sda, scl)) = self.pins.as_ref() {
                        scl.set_output_high(false);
                    }
                    self.state = State::SendClock(n, false);
                }
            }
            self.wait = Instant::now() + Self::SCL_DELAY;

            Poll::Pending
        }
    }

    impl Drop for ClearBusFuture<'_> {
        fn drop(&mut self) {
            if let Some((sda, scl)) = self.pins.take() {
                // Make sure _we_ release the bus.
                scl.set_output_high(true);
                sda.set_output_high(true);

                // If we don't have a HW implementation, reset the peripheral after clearing the
                // bus, but before we reconnect the pins in Drop. This should prevent glitches.
                if self.reset_fsm {
                    self.driver.do_fsm_reset();
                }

                self.driver.info.sda_output.connect_to(&sda);
                self.driver.info.scl_output.connect_to(&scl);

                // We don't care about errors during bus clearing. There shouldn't be any,
                // anyway.
                self.driver.clear_all_interrupts();
            }
        }
    }
}

use bus_clear::ClearBusFuture;

impl Future for ClearBusFuture<'_> {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        let pending = self.poll_completion();
        if pending.is_pending() {
            ctx.waker().wake_by_ref();
        }
        pending
    }
}

/// Peripheral state for an I2C instance.
#[doc(hidden)]
#[non_exhaustive]
pub struct State {
    /// Waker for the asynchronous operations.
    pub waker: AtomicWaker,
}

/// A peripheral singleton compatible with the I2C master driver.
pub trait Instance: crate::private::Sealed + any::Degrade {
    #[doc(hidden)]
    /// Returns the peripheral data and state describing this instance.
    fn parts(&self) -> (&Info, &State);

    /// Returns the peripheral data describing this instance.
    #[doc(hidden)]
    #[inline(always)]
    fn info(&self) -> &Info {
        self.parts().0
    }

    /// Returns the peripheral state for this instance.
    #[doc(hidden)]
    #[inline(always)]
    fn state(&self) -> &State {
        self.parts().1
    }
}

/// Adds a command to the I2C command sequence.
///
/// Make sure the first command after a FSM reset is a START, otherwise
/// the hardware will hang with no timeouts.
fn add_cmd<'a, I>(cmd_iterator: &mut I, command: Command) -> Result<(), Error>
where
    I: Iterator<Item = &'a COMD>,
{
    let cmd = cmd_iterator.next().ok_or(Error::CommandNumberExceeded)?;

    cmd.write(|w| match command {
        Command::Start => w.opcode().rstart(),
        Command::Stop => w.opcode().stop(),
        Command::End => w.opcode().end(),
        Command::Write {
            ack_exp,
            ack_check_en,
            length,
        } => unsafe {
            w.opcode().write();
            w.ack_exp().bit(ack_exp == Ack::Nack);
            w.ack_check_en().bit(ack_check_en);
            w.byte_num().bits(length);
            w
        },
        Command::Read { ack_value, length } => unsafe {
            w.opcode().read();
            w.ack_value().bit(ack_value == Ack::Nack);
            w.byte_num().bits(length);
            w
        },
    });

    Ok(())
}

// Estimate the reason for an acknowledge check failure on a best effort basis.
// When in doubt it's better to return `Unknown` than to return a wrong reason.
fn estimate_ack_failed_reason(_register_block: &RegisterBlock) -> AcknowledgeCheckFailedReason {
    cfg_if::cfg_if! {
        if #[cfg(i2c_master_can_estimate_nack_reason)] {
            // this is based on observations rather than documented behavior
            if _register_block.fifo_st().read().txfifo_raddr().bits() <= 1 {
                AcknowledgeCheckFailedReason::Address
            } else {
                AcknowledgeCheckFailedReason::Data
            }
        } else {
            AcknowledgeCheckFailedReason::Unknown
        }
    }
}

for_each_i2c_master!(
    ($id:literal, $inst:ident, $peri:ident, $scl:ident, $sda:ident) => {
        impl Instance for crate::peripherals::$inst<'_> {
            fn parts(&self) -> (&Info, &State) {
                #[handler]
                #[ram]
                pub(super) fn irq_handler() {
                    async_handler(&PERIPHERAL, &STATE);
                }

                static STATE: State = State {
                    waker: AtomicWaker::new(),
                };

                static PERIPHERAL: Info = Info {
                    #[cfg(soc_has_i2c1)]
                    id: $id,
                    register_block: crate::peripherals::$inst::ptr(),
                    peripheral: crate::system::Peripheral::$peri,
                    async_handler: irq_handler,
                    scl_output: OutputSignal::$scl,
                    scl_input: InputSignal::$scl,
                    sda_output: OutputSignal::$sda,
                    sda_input: InputSignal::$sda,
                };
                (&PERIPHERAL, &STATE)
            }
        }
    };
);

crate::any_peripheral! {
    /// Any I2C peripheral.
    pub peripheral AnyI2c<'d> {
        #[cfg(i2c_master_i2c0)]
        I2c0(crate::peripherals::I2C0<'d>),
        #[cfg(i2c_master_i2c1)]
        I2c1(crate::peripherals::I2C1<'d>),
    }
}

impl Instance for AnyI2c<'_> {
    fn parts(&self) -> (&Info, &State) {
        any::delegate!(self, i2c => { i2c.parts() })
    }
}

impl AnyI2c<'_> {
    fn bind_peri_interrupt(&self, handler: InterruptHandler) {
        any::delegate!(self, i2c => { i2c.bind_peri_interrupt(handler) })
    }

    pub(super) fn disable_peri_interrupt_on_all_cores(&self) {
        any::delegate!(self, i2c => { i2c.disable_peri_interrupt_on_all_cores() })
    }

    pub(super) fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.disable_peri_interrupt_on_all_cores();

        self.info().enable_listen(EnumSet::all(), false);
        self.info().clear_interrupts(EnumSet::all());

        self.bind_peri_interrupt(handler);
    }
}
