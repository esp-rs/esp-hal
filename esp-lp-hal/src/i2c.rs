//! Low-power I2C driver

use esp32c6_lp::I2C;

use self::config::Config;

pub struct I2C {
  i2c: LP_I2C0
}

const LPPERI_CLK_EN_REG: u32 = 0x600B2800 + 0x0
const LPPERI_RESET_EN_REG: u32 = 0x600B2800 + 0x4
const LPPERI_LP_EXT_I2C_CK_EN: u32 = 1 << 28;
const LPPERI_LP_EXT_I2C_RESET_EN: u32 = 1 << 28;

const LP_I2C_FILTER_CYC_NUM_DEF: u32 = 7;

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
        lp_i2c: impl Peripheral<P = T> + 'd, 
        sda: impl Peripheral<P = SDA> + 'd,
        scl: impl Peripheral<P = SCL> + 'd,
        frequency: HertzU32,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        let mut me = Self { uart };

        // Initialize LP I2C HAL */
        me.i2c.
            i2c_clk_conf
            .modify(|_, w| w.i2c_sclk_active().set_bit());

        /* Enable LP I2C controller clock */
        set_peri_reg_mask(LPPERI_CLK_EN_REG, LPPERI_LP_EXT_I2C_CK_EN);
        clear_peri_reg_mask(LPPERI_RESET_EN_REG, LPPERI_LP_EXT_I2C_RESET_EN)

        /* Initialize LP I2C Master mode */
        me.i2c.i2c_ctr.modify(|_, w| unsafe {
            // Clear register
            w.bits(0)
                // Set I2C controller to master mode
                .ms_mode()
                .set_bit()
                // Use open drain output for SDA and SCL
                .i2c_sda_force_out()
                .set_bit()
                .i2c_scl_force_out()
                .set_bit()
                // Use Most Significant Bit first for sending and receiving data
                .i2c_tx_lsb_first()
                .clear_bit()
                .i2c_rx_lsb_first()
                .clear_bit()
                // Ensure that clock is enabled
                .i2c_clk_en()
                .set_bit()
        });

        /* Enable SDA and SCL filtering. This configuration matches the HP I2C filter config */

        me.i2c.i2c_filter_cfg.modify(|_, w| unsafe {w.i2c_sda_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF)});
        me.i2c.i2c_filter_cfg.modify(|_, w| w.i2c_sda_filter_en().set_bit());

        me.i2c.i2c_filter_cfg.modify(|_, w| unsafe {w.i2c_scl_filter_thres().bits(LP_I2C_FILTER_CYC_NUM_DEF)});
        me.i2c.i2c_filter_cfg.modify(|_, w| w.i2c_scl_filter_en().set_bit());

        /* Set LP I2C source clock */
        unsafe { &*crate::peripherals::LP_CLKRST::PTR }.lpperi.modify(|_, w| w.lp_i2c_clk_sel().set_bit());
        /* 
        /* Configure LP I2C timing paramters. source_clk is ignored for LP_I2C in this call */
        i2c_hal_set_bus_timing(&i2c_hal, cfg->i2c_timing_cfg.clk_speed_hz, (i2c_clock_source_t)source_clk, source_freq);

        return ret;*/
    } 


    
    /*
    ESP_RETURN_ON_ERROR(lp_i2c_config_clk(cfg), LPI2C_TAG, "Failed to configure LP I2C source clock");

    /* Enable SDA and SCL filtering. This configuration matches the HP I2C filter config */
    i2c_ll_master_set_filter(i2c_hal.dev, LP_I2C_FILTER_CYC_NUM_DEF);

    /* Configure the I2C master to send a NACK when the Rx FIFO count is full */
    i2c_ll_master_rx_full_ack_level(i2c_hal.dev, 1);

    /* Synchronize the config register values to the LP I2C peripheral clock */
    i2c_ll_update(i2c_hal.dev); */

    // peripheral_clock_control.enable(crate::system::Peripheral::I2cExt0); 


    // FIXME : check if will be necessary at all
    fn set_filter(&mut self, sda_threshold: Option<u8>, scl_threshold: Option<u8>) {

        let sda_register = &self.register_block().filter_cfg;
        let scl_register = &self.register_block().filter_cfg;
        
        match sda_threshold {
            Some(threshold) => {
                sda_register.modify(|_, w| unsafe { w.sda_filter_thres().bits(threshold) });
                sda_register.modify(|_, w| w.sda_filter_en().set_bit());
            }
            None => sda_register.modify(|_, w| w.sda_filter_en().clear_bit()),
        }
    
        match scl_threshold {
            Some(threshold) => {
                scl_register.modify(|_, w| unsafe { w.scl_filter_thres().bits(threshold) });
                scl_register.modify(|_, w| w.scl_filter_en().set_bit());
            }
            None => scl_register.modify(|_, w| w.scl_filter_en().clear_bit()),
        }
    }

  }
}

fn set_peri_reg_mask(reg: u32, mask: u32) { // Check if same with XTENSA chips
  unsafe {
      (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask);
  }
}

fn clear_peri_reg_mask(reg: u32, mask: u32) { // Check if same with XTENSA chips
  unsafe {
      (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask);
  }
}
