//! Low-power I2C driver

use fugit::HertzU32;

use crate::{
    gpio::lp_io::LowPowerOutputOpenDrain,
    peripherals::{LP_CLKRST, LP_I2C0},
};

const LP_I2C_FILTER_CYC_NUM_DEF: u8 = 7;

/// I2C-specific transmission errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The transmission exceeded the FIFO size.
    ExceedingFifo,
    /// The acknowledgment check failed.
    AckCheckFailed,
    /// A timeout occurred during transmission.
    TimeOut,
    /// The arbitration for the bus was lost.
    ArbitrationLost,
    /// The execution of the I2C command was incomplete.
    ExecIncomplete,
    /// The number of commands issued exceeded the limit.
    CommandNrExceeded,
    /// The response received from the I2C device was invalid.
    InvalidResponse,
}

#[allow(unused)]
enum OperationType {
    Write = 0,
    Read  = 1,
}

#[allow(unused)]
#[derive(Eq, PartialEq, Copy, Clone)]
enum Ack {
    Ack,
    Nack,
}

#[derive(PartialEq)]
#[allow(unused)]
enum Command {
    Start,
    Stop,
    End,
    Write {
        /// This bit is to set an expected ACK value for the transmitter.
        ack_exp: Ack,
        /// Enables checking the ACK value received against the ack_exp
        /// value.
        ack_check_en: bool,
        /// Length of data (in bytes) to be written. The maximum length is
        /// 255, while the minimum is 1.
        length: u8,
    },
    Read {
        /// Indicates whether the receiver will send an ACK after this byte
        /// has been received.
        ack_value: Ack,
        /// Length of data (in bytes) to be read. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
}

// https://github.com/espressif/esp-idf/blob/master/components/ulp/lp_core/lp_core_i2c.c#L122
// TX/RX RAM size is 16*8 bit
// TX RX FIFO has 16 bit depth
// The clock source of APB_CLK in LP_I2C is CLK_AON_FAST.
// Configure LP_I2C_SCLK_SEL to select the clock source for I2C_SCLK.
// When LP_I2C_SCLK_SEL is 0, select CLK_ROOT_FAST as clock source,
// and when LP_I2C_SCLK_SEL is 1, select CLK _XTALD2 as the clock source.
// Configure LP_EXT_I2C_CK_EN high to enable the clock source of I2C_SCLK.
// Adjust the timing registers accordingly when the clock frequency changes.

/// Represents a Low-Power I2C peripheral.
pub struct LpI2c {
    i2c: LP_I2C0,
}

impl LpI2c {
    /// Creates a new instance of the `LpI2c` peripheral.
    pub fn new(
        i2c: LP_I2C0,
        _sda: LowPowerOutputOpenDrain<'_, 6>,
        _scl: LowPowerOutputOpenDrain<'_, 7>,
        frequency: HertzU32,
    ) -> Self {
        let me = Self { i2c };

        // Configure LP I2C GPIOs
        // Initialize IO Pins
        let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
        let lp_aon = unsafe { &*crate::peripherals::LP_AON::PTR };
        let lp_peri = unsafe { &*crate::peripherals::LP_PERI::PTR };

        unsafe {
            // FIXME: use GPIO APIs to configure pins
            lp_aon
                .gpio_mux()
                .modify(|r, w| w.sel().bits(r.sel().bits() | (1 << 6)));
            lp_aon
                .gpio_mux()
                .modify(|r, w| w.sel().bits(r.sel().bits() | (1 << 7)));
            lp_io.gpio(6).modify(|_, w| w.mcu_sel().bits(1)); // TODO

            lp_io.gpio(7).modify(|_, w| w.mcu_sel().bits(1));

            // Set output mode to Normal
            lp_io.pin(6).modify(|_, w| w.pad_driver().set_bit());
            // Enable output (writing to write-1-to-set register, then internally the
            // `GPIO_OUT_REG` will be set)
            lp_io
                .out_enable_w1ts()
                .write(|w| w.enable_w1ts().bits(1 << 6));
            // Enable input
            lp_io.gpio(6).modify(|_, w| w.fun_ie().set_bit());

            // Disable pulldown (enable internal weak pull-down)
            lp_io.gpio(6).modify(|_, w| w.fun_wpd().clear_bit());
            // Enable pullup
            lp_io.gpio(6).modify(|_, w| w.fun_wpu().set_bit());

            // Same process for SCL pin
            lp_io.pin(7).modify(|_, w| w.pad_driver().set_bit());
            // Enable output (writing to write-1-to-set register, then internally the
            // `GPIO_OUT_REG` will be set)
            lp_io
                .out_enable_w1ts()
                .write(|w| w.enable_w1ts().bits(1 << 7));
            // Enable input
            lp_io.gpio(7).modify(|_, w| w.fun_ie().set_bit());
            // Disable pulldown (enable internal weak pull-down)
            lp_io.gpio(7).modify(|_, w| w.fun_wpd().clear_bit());
            // Enable pullup
            lp_io.gpio(7).modify(|_, w| w.fun_wpu().set_bit());

            // Select LP I2C function for the SDA and SCL pins
            lp_io.gpio(6).modify(|_, w| w.mcu_sel().bits(1));
            lp_io.gpio(7).modify(|_, w| w.mcu_sel().bits(1));
        }

        // Initialize LP I2C HAL */
        me.i2c.clk_conf().modify(|_, w| w.sclk_active().set_bit());

        // Enable LP I2C controller clock
        lp_peri
            .clk_en()
            .modify(|_, w| w.lp_ext_i2c_ck_en().set_bit());

        lp_peri
            .reset_en()
            .modify(|_, w| w.lp_ext_i2c_reset_en().set_bit());
        lp_peri
            .reset_en()
            .modify(|_, w| w.lp_ext_i2c_reset_en().clear_bit());

        // Set LP I2C source clock
        unsafe { &*LP_CLKRST::PTR }
            .lpperi()
            .modify(|_, w| w.lp_i2c_clk_sel().clear_bit());

        // Initialize LP I2C Master mode
        me.i2c.ctr().modify(|_, w| unsafe {
            // Clear register
            w.bits(0);
            // Use open drain output for SDA and SCL
            w.sda_force_out().set_bit();
            w.scl_force_out().set_bit();
            // Ensure that clock is enabled
            w.clk_en().set_bit()
        });

        // First, reset the fifo buffers
        me.i2c.fifo_conf().modify(|_, w| w.nonfifo_en().clear_bit());

        me.i2c.ctr().modify(|_, w| {
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit()
        });

        me.reset_fifo();

        // Set LP I2C source clock
        unsafe { &*LP_CLKRST::PTR }
            .lpperi()
            .modify(|_, w| w.lp_i2c_clk_sel().clear_bit());

        // Configure LP I2C timing paramters. source_clk is ignored for LP_I2C in this
        // call

        let source_clk = 16_000_000;
        let bus_freq = frequency.raw();

        let clkm_div: u32 = source_clk / (bus_freq * 1024) + 1;
        let sclk_freq: u32 = source_clk / clkm_div;
        let half_cycle: u32 = sclk_freq / bus_freq / 2;

        // SCL
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
        let sda_sample = half_cycle / 2; // TODO + scl_wait_high;
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

        // Write data to registers
        unsafe {
            me.i2c.clk_conf().modify(|_, w| {
                w.sclk_sel().clear_bit();
                w.sclk_div_num().bits((clkm_div - 1) as u8)
            });

            // scl period
            me.i2c
                .scl_low_period()
                .write(|w| w.scl_low_period().bits(scl_low_period as u16));

            me.i2c.scl_high_period().write(|w| {
                w.scl_high_period().bits(scl_high_period as u16);
                w.scl_wait_high_period().bits(scl_wait_high_period as u8)
            });
            // sda sample
            me.i2c
                .sda_hold()
                .write(|w| w.time().bits(sda_hold_time as u16));
            me.i2c
                .sda_sample()
                .write(|w| w.time().bits(sda_sample_time as u16));

            // setup
            me.i2c
                .scl_rstart_setup()
                .write(|w| w.time().bits(scl_rstart_setup_time as u16));
            me.i2c
                .scl_stop_setup()
                .write(|w| w.time().bits(scl_stop_setup_time as u16));

            // hold
            me.i2c
                .scl_start_hold()
                .write(|w| w.time().bits(scl_start_hold_time as u16));
            me.i2c
                .scl_stop_hold()
                .write(|w| w.time().bits(scl_stop_hold_time as u16));

            me.i2c.to().write(|w| {
                w.time_out_en().bit(time_out_en);
                w.time_out_value().bits(time_out_value.try_into().unwrap())
            });
        }

        // Enable SDA and SCL filtering. This configuration matches the HP I2C filter
        // config

        me.i2c
            .filter_cfg()
            .modify(|_, w| unsafe { w.sda_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF) });
        me.i2c
            .filter_cfg()
            .modify(|_, w| unsafe { w.scl_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF) });

        me.i2c
            .filter_cfg()
            .modify(|_, w| w.sda_filter_en().set_bit());
        me.i2c
            .filter_cfg()
            .modify(|_, w| w.scl_filter_en().set_bit());

        // Configure the I2C master to send a NACK when the Rx FIFO count is full
        me.i2c.ctr().modify(|_, w| w.rx_full_ack_level().set_bit());

        // Synchronize the config register values to the LP I2C peripheral clock
        me.lp_i2c_update();

        me
    }

    /// Update I2C configuration
    fn lp_i2c_update(&self) {
        self.i2c.ctr().modify(|_, w| w.conf_upgate().set_bit());
    }

    /// Resets the transmit and receive FIFO buffers.
    fn reset_fifo(&self) {
        self.i2c
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().set_bit());

        self.i2c
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().clear_bit());

        self.i2c
            .fifo_conf()
            .modify(|_, w| w.rx_fifo_rst().set_bit());

        self.i2c
            .fifo_conf()
            .modify(|_, w| w.rx_fifo_rst().clear_bit());
    }
}
