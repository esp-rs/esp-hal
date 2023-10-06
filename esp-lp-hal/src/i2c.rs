//! Low-power I2C driver

use esp32c6_lp::I2C;

use self::config::Config;


const LPPERI_CLK_EN_REG: u32 = 0x600B2800 + 0x0;
const LPPERI_RESET_EN_REG: u32 = 0x600B2800 + 0x4;
const LPPERI_LP_EXT_I2C_CK_EN: u32 = 1 << 28;
const LPPERI_LP_EXT_I2C_RESET_EN: u32 = 1 << 28;

const LP_I2C_FILTER_CYC_NUM_DEF: u32 = 7;

const LP_I2C_TRANS_COMPLETE_INT_ST_S: u32 = 7;
const LP_I2C_END_DETECT_INT_ST_S: u32 = 3;
const LP_I2C_NACK_INT_ST_S: u32 = 10;

const I2C_LL_INTR_MASK: u32 = (1 << LP_I2C_TRANS_COMPLETE_INT_ST_S) | (1 << LP_I2C_END_DETECT_INT_ST_S) | (1 << LP_I2C_NACK_INT_ST_S);

const SOC_LP_I2C_FIFO_LEN: u32 = 16; // TX RX FIFO depth on esp32c6 

pub struct I2C {
  i2c: LP_I2C
}

enum OperationType {
    Write = 0,
    Read  = 1,
}

#[derive(Eq, PartialEq, Copy, Clone)]
enum Ack {
    Ack,
    Nack,
}

enum Opcode {
    RStart = 6,
    Write  = 1,
    Read   = 3,
    Stop   = 2,
    End    = 4,
}

enum Command {
    Start,
    Stop,
    End,
    Write {
        /// This bit is to set an expected ACK value for the transmitter.
        ack_exp: Ack,
        /// Enables checking the ACK value received against the ack_exp value.
        ack_check_en: bool,
        /// Length of data (in bytes) to be written. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
    Read {
        /// Indicates whether the receiver will send an ACK after this byte has
        /// been received.
        ack_value: Ack,
        /// Length of data (in bytes) to be read. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
}

impl From<Command> for u16 {
    fn from(c: Command) -> u16 {
        let opcode = match c {
            Command::Start => Opcode::RStart,
            Command::Stop => Opcode::Stop,
            Command::End => Opcode::End,
            Command::Write { .. } => Opcode::Write,
            Command::Read { .. } => Opcode::Read,
        };

        let length = match c {
            Command::Start | Command::Stop => 0,
            Command::Write { length: l, .. } | Command::Read { length: l, .. } => l,
        };

        let ack_exp = match c {
            Command::Start | Command::Stop | Command::Read { .. } => Ack::Nack,
            Command::Write { ack_exp: exp, .. } => exp,
        };

        let ack_check_en = match c {
            Command::Start | Command::Stop | Command::Read { .. } => false,
            Command::Write {
                ack_check_en: en, ..
            } => en,
        };

        let ack_value = match c {
            Command::Start | Command::Stop | Command::Write { .. } => Ack::Nack,
            Command::Read { ack_value: ack, .. } => ack,
        };

        let mut cmd: u16 = length.into();

        if ack_check_en {
            cmd |= 1 << 8;
        } else {
            cmd &= !(1 << 8);
        }

        if ack_exp == Ack::Nack {
            cmd |= 1 << 9;
        } else {
            cmd &= !(1 << 9);
        }

        if ack_value == Ack::Nack {
            cmd |= 1 << 10;
        } else {
            cmd &= !(1 << 10);
        }

        cmd |= (opcode as u16) << 11;

        cmd
    }
}

// https://github.com/espressif/esp-idf/blob/master/components/ulp/lp_core/lp_core_i2c.c#L122
// TX/RX RAM size is 16*8 bit
// TX RX FIFO has 16 bit depth 
/*
The clock source of APB_CLK in LP_I2C is CLK_AON_FAST.
Configure LP_I2C_SCLK_SEL to select the clock source for I2C_SCLK.
When LP_I2C_SCLK_SEL is 0, select CLK_ROOT_FAST as clock source,
and when LP_I2C_SCLK_SEL is 1, select CLK _XTALD2 as the clock source.
Configure LP_EXT_I2C_CK_EN high to enable the clock source of I2C_SCLK.
Adjust the timing registers accordingly when the clock frequency changes.
 */

impl I2C {
    pub fn new<SDA: OutputPin + InputPin, SCL: OutputPin + InputPin> (
        lp_i2c: LP_I2C, 
        sda: impl Peripheral<P = SDA> + 'd,
        scl: impl Peripheral<P = SCL> + 'd,
        frequency: HertzU32,
        clocks: &Clocks,
    ) -> Self {
        let mut me = Self { lp_i2c };

        // peripheral_clock_control.enable(crate::system::Peripheral::I2cExt0);  ???

        // Initialize LP I2C HAL */
        me.i2c.
            clk_conf
            .modify(|_, w| w.sclk_active().set_bit());

        /* Enable LP I2C controller clock */
        set_peri_reg_mask(LPPERI_CLK_EN_REG, LPPERI_LP_EXT_I2C_CK_EN);
        clear_peri_reg_mask(LPPERI_RESET_EN_REG, LPPERI_LP_EXT_I2C_RESET_EN)

        /* Initialize LP I2C Master mode */
        me.i2c.ctr.modify(|_, w| unsafe {
            // Clear register
            w.bits(0)
                // Set I2C controller to master mode
                .ms_mode()
                .set_bit()
                // Use open drain output for SDA and SCL
                .sda_force_out()
                .set_bit()
                .scl_force_out()
                .set_bit()
                // Use Most Significant Bit first for sending and receiving data
                .tx_lsb_first()
                .clear_bit()
                .rx_lsb_first()
                .clear_bit()
                // Ensure that clock is enabled
                .clk_en()
                .set_bit()
        });

        /* Set LP I2C source clock */
        unsafe { &*crate::peripherals::LP_CLKRST::PTR }.lpperi.modify(|_, w| w.lp_i2c_clk_sel().set_bit());

        /* Configure LP I2C timing paramters. source_clk is ignored for LP_I2C in this call */
        
        let source_clk = clocks.i2c_clock.convert().raw();
        let bus_freq = frequency.raw();

        let clkm_div: u32 = source_clk / (bus_freq * 1024) + 1;
        let sclk_freq: u32 = source_clk / clkm_div;
        let half_cycle: u32 = sclk_freq / bus_freq / 2;
        // SCL
        let clkm_div = clkm_div;
        let scl_low = half_cycle;
        // default, scl_wait_high < scl_high
        // Make 80KHz as a boundary here, because when working at lower frequency, too
        // much scl_wait_high will faster the frequency according to some
        // hardware behaviors.
        let scl_wait_high = if bus_freq >= 80 * 1000 {
            half_cycle / 2 - 2
        } else {
            half_cycle / 4
        };
        let scl_high = half_cycle - scl_wait_high;
        let sda_hold = half_cycle / 4;
        let sda_sample = half_cycle / 2 + scl_wait_high;
        let setup = half_cycle;
        let hold = half_cycle;
        // default we set the timeout value to about 10 bus cycles
        // log(20*half_cycle)/log(2) = log(half_cycle)/log(2) +  log(20)/log(2)
        let tout = (4 * 8 - (5 * half_cycle).leading_zeros()) + 2;

        // According to the Technical Reference Manual, the following timings must be
        // subtracted by 1. However, according to the practical measurement and
        // some hardware behaviour, if wait_high_period and scl_high minus one.
        // The SCL frequency would be a little higher than expected. Therefore, the
        // solution here is not to minus scl_high as well as scl_wait high, and
        // the frequency will be absolutely accurate to all frequency
        // to some extent.
        let scl_low_period = scl_low - 1;
        let scl_high_period = scl_high;
        let scl_wait_high_period = scl_wait_high;
        // sda sample
        let sda_hold_time = sda_hold - 1;
        let sda_sample_time = sda_sample - 1;
        // setup
        let scl_rstart_setup_time = setup - 1;
        let scl_stop_setup_time = setup - 1;
        // hold
        let scl_start_hold_time = hold - 1;
        let scl_stop_hold_time = hold - 1;
        let time_out_value = tout;
        let time_out_en = true;

        unsafe {
            me.i2c.clk_conf.modify(|_, w| {
                w.sclk_sel()
                    .clear_bit()
                    .sclk_div_num()
                    .bits((sclk_div - 1) as u8)
            });

            // scl period
            me.i2c
                .scl_low_period
                .write(|w| w.scl_low_period().bits(scl_low_period as u16));

            me.i2c.scl_high_period.write(|w| {
                w.scl_high_period()
                    .bits(scl_high_period as u16)
                    .scl_wait_high_period()
                    .bits(scl_wait_high_period.try_into().unwrap())
            });

            // sda sample
            me.i2c
                .sda_hold
                .write(|w| w.time().bits(sda_hold_time as u16));
            me.i2c
                .sda_sample
                .write(|w| w.time().bits(sda_sample_time as u16));

            // setup
            me.i2c
                .scl_rstart_setup
                .write(|w| w.time().bits(scl_rstart_setup_time as u16));
            me.i2c
                .scl_stop_setup
                .write(|w| w.time().bits(scl_stop_setup_time as u16));

            // hold
            me.i2c
                .scl_start_hold
                .write(|w| w.time().bits(scl_start_hold_time as u16));
            me.i2c
                .scl_stop_hold
                .write(|w| w.time().bits(scl_stop_hold_time as u16));

            me.i2c
                .to
                .write(|w| w.time_out_en().bit(time_out_en)
                .time_out_value()
                .variant(time_out_value.try_into().unwrap())
            );
        }

        /* Enable SDA and SCL filtering. This configuration matches the HP I2C filter config */

        me.i2c.filter_cfg.modify(|_, w| unsafe {w.sda_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF)});
        me.i2c.filter_cfg.modify(|_, w| w.sda_filter_en().set_bit());

        me.i2c.filter_cfg.modify(|_, w| unsafe {w.scl_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF)});
        me.i2c.filter_cfg.modify(|_, w| w.scl_filter_en().set_bit());

        /* Configure the I2C master to send a NACK when the Rx FIFO count is full */
        me.i2c.ctr.modify(|_, w| w.rx_full_ack_level().set_bit());

        /* Synchronize the config register values to the LP I2C peripheral clock */
        me.i2c.ctr.modify(|_, w| w.conf_upgate().set_bit());

        me
    }
    
    /// Send data bytes from the `bytes` array to a target slave with the
    /// address `addr`
    fn master_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // If SCL is busy, reset the Master FSM
        if self.i2c.sr.read().bus_busy().bit_is_set() {
            self.i2c.ctr.modify(|_, w| unsafe {w.fsm_rst().set_bit()});
        }
        
        // Reset FIFO and command list
        self.reset_fifo();
        // self.reset_command_list();
        
        if bytes.len() > 255 {
            return Err(Error::ExceedingFifo);
        }
        
        self.enable_interrupts();

        add_cmd(&mut self.i2c.comd.iter(), Command::Start)?;

        // Load device address and R/W bit into FIFO
        write_fifo(
            self.i2c,
            addr << 1 | OperationType::Write as u8,
        );

        for chunk in bytes.chunks(LP_I2C_FIFO_LEN) {

            self.write_fifo(chunk);

            /* Update the HW command register. Expect an ACK from the device */
            add_cmd(
                &mut self.i2c.comd.iter(),
                Command::Write {
                    ack_exp: Ack::Ack,
                    ack_check_en: true,
                    length: chunk.len() as u8,
                },
            )?;

            let cmd: Command = if chunk.len() == bytes.len() { Command::Stop } else { Command::End };

            add_cmd(&mut self.i2c.comd.iter(), cmd);

            self.i2c.ctr.modify(|_, w| w.conf_upgate().set_bit());
            self.i2c.ctr.modify(|_, w| w.trans_start().set_bit());

            self.wait_for_completion()?;
        }

        Ok(())
    }

    fn master_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.reset_fifo();

    }
    
    fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.i2c.fifo_conf.modify(|_, w| {
            w.tx_fifo_rst()
            .set_bit()
            .rx_fifo_rst()
            .set_bit()
            .nonfifo_en()
            .clear_bit()
            .fifo_prt_en()
            .set_bit()
            .rxfifo_wm_thrhd()
            .variant(1)
            .txfifo_wm_thrhd()
            .variant(8)
        });
        
        self.i2c
        .fifo_conf
        .modify(|_, w| w.tx_fifo_rst().clear_bit().rx_fifo_rst().clear_bit());
    
        self.i2c.int_clr.write(|w| {
            w.rxfifo_wm_int_clr()
            .set_bit()
            .txfifo_wm_int_clr()
            .set_bit()
        });
        
        self.i2c.ctr.modify(|_, w| w.conf_upgate().set_bit());;
    }

    fn wait_for_completion(&self) -> Result<(), Error> {
        loop {
            let interrupts = self.i2c.int_raw.read();

            self.check_errors()?;

            // Handle completion cases
            // A full transmission was completed
            if interrupts.trans_complete_int_raw().bit_is_set()
                || interrupts.end_detect_int_raw().bit_is_set()
            {
                break;
            }
        }
        for cmd in self.i2c.comd.iter() {
            if cmd.read().command().bits() != 0x0 && cmd.read().command_done().bit_is_clear() {
                return Err(Error::ExecIncomplete);
            }
        }

        Ok(())
    }

    fn enable_interrupts(&self) {
        self.i2c
            .int_clr
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
    }

    fn reset_command_list(&self) {
        // Confirm that all commands that were configured were actually executed
        for cmd in self.i2c.comd.iter() {
            cmd.reset();
        }
    }

    fn fill_tx_fifo(&self, bytes: &[u8]) -> usize {
        let mut index = 0;
        while index < bytes.len()
            && !self
                .i2c
                .int_raw
                .read()
                .txfifo_ovf_int_raw()
                .bit_is_set()
        {
            write_fifo(self.i2c, bytes[index]);
            index += 1;
        }
        if self
            .i2c
            .int_raw
            .read()
            .txfifo_ovf_int_raw()
            .bit_is_set()
        {
            index -= 1;
            self.i2c
                .int_clr
                .write(|w| w.txfifo_ovf_int_clr().set_bit());
        }
        index
    }

    fn write_fifo(&self, data: u8) {
        self.me
            .data
            .write(|w| unsafe { w.fifo_rdata().bits(data) });
    }

}


fn set_peri_reg_mask(reg: u32, mask: u32) {
  unsafe {
      (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask);
  }
}

fn clear_peri_reg_mask(reg: u32, mask: u32) {
  unsafe {
      (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask);
  }
}


fn add_cmd<'a, I>(cmd_iterator: &mut I, command: Command) -> Result<(), Error>
where
    I: Iterator<Item = &'a COMD>,
{
    let cmd = cmd_iterator.next().ok_or(Error::CommandNrExceeded)?;
    cmd.write(|w| unsafe { w.command().bits(command.into()) });
    Ok(())
}