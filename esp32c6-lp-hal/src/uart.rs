//! Low-power UART driver

use esp32c6_lp::LP_UART;

use self::config::Config;

const UART_FIFO_SIZE: u16 = 20;

#[derive(Debug)]
pub enum Error {}

/// UART configuration
pub mod config {
    /// Number of data bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum DataBits {
        DataBits5 = 0,
        DataBits6 = 1,
        DataBits7 = 2,
        DataBits8 = 3,
    }

    /// Parity check
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum Parity {
        ParityNone = 0,
        ParityEven = 1,
        ParityOdd  = 2,
    }

    /// Number of stop bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum StopBits {
        /// 1 stop bit
        STOP1   = 1,
        /// 1.5 stop bits
        STOP1P5 = 2,
        /// 2 stop bits
        STOP2   = 3,
    }

    /// UART configuration
    #[derive(Debug, Copy, Clone)]
    pub struct Config {
        pub baudrate: u32,
        pub data_bits: DataBits,
        pub parity: Parity,
        pub stop_bits: StopBits,
    }

    impl Config {
        pub fn baudrate(mut self, baudrate: u32) -> Self {
            self.baudrate = baudrate;
            self
        }

        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        pub fn data_bits(mut self, data_bits: DataBits) -> Self {
            self.data_bits = data_bits;
            self
        }

        pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
            self.stop_bits = stop_bits;
            self
        }
    }

    impl Default for Config {
        fn default() -> Config {
            Config {
                baudrate: 115_200,
                data_bits: DataBits::DataBits8,
                parity: Parity::ParityNone,
                stop_bits: StopBits::STOP1,
            }
        }
    }
}

/// UART driver
pub struct Uart {
    uart: LP_UART,
}

impl Uart {
    /// Initialize the UART driver using the default configuration
    pub fn new(uart: LP_UART) -> Self {
        Self::new_with_config(uart, Config::default())
    }

    /// Initialize the UART driver using the provided configuration
    pub fn new_with_config(uart: LP_UART, config: Config) -> Self {
        let mut me = Self { uart };

        // begin: lp_core_uart_param_config

        // Set UART mode.
        // uart_ll_set_mode(hal->dev, UART_MODE_UART);

        // Disable UART parity
        // 8-bit world
        // 1-bit stop bit
        me.uart.conf0_sync.modify(|_, w| unsafe {
            w.parity()
                .clear_bit()
                .parity_en()
                .clear_bit()
                .bit_num()
                .bits(0x3)
                .stop_bit_num()
                .bits(0x1)
        });
        // Set tx idle
        me.uart
            .idle_conf_sync
            .modify(|_, w| unsafe { w.tx_idle_num().bits(0) });
        // Disable hw-flow control
        me.uart
            .hwfc_conf_sync
            .modify(|_, w| unsafe { w.rx_flow_en().clear_bit() });

        //

        // Override protocol parameters from the configuration
        // uart_hal_set_baudrate(&hal, cfg->uart_proto_cfg.baud_rate, sclk_freq);
        // uart_hal_set_parity(&hal, cfg->uart_proto_cfg.parity);
        // uart_hal_set_data_bit_num(&hal, cfg->uart_proto_cfg.data_bits);
        // uart_hal_set_stop_bits(&hal, cfg->uart_proto_cfg.stop_bits);
        // uart_hal_set_tx_idle_num(&hal, LP_UART_TX_IDLE_NUM_DEFAULT);
        // uart_hal_set_hw_flow_ctrl(
        //     &hal,
        //     cfg->uart_proto_cfg.flow_ctrl,
        //     cfg->uart_proto_cfg.rx_flow_ctrl_thresh
        // );

        //

        // Reset Tx/Rx FIFOs
        me.rxfifo_reset();
        me.txfifo_reset();

        // end: lp_core_uart_param_config

        // begin: lp_core_uart_set_pin

        // static esp_err_t
        // lp_uart_config_io(gpio_num_t pin, rtc_gpio_mode_t direction)
        // {
        //     /* Initialize LP_IO */
        //     esp_err_t ret = rtc_gpio_init(pin);
        //     if (ret != ESP_OK) {
        //         return ESP_FAIL;
        //     }
        //
        //     /* Set LP_IO direction */
        //     ret = rtc_gpio_set_direction(pin, direction);
        //     if (ret != ESP_OK) {
        //         return ESP_FAIL;
        //     }
        //
        //     /* Set LP_IO function */
        //     ret = rtc_gpio_iomux_func_sel(pin, 1);
        //
        //     return ret;
        // }

        // Configure Tx Pin
        // ret = lp_uart_config_io(
        //     cfg->uart_pin_cfg.tx_io_num, RTC_GPIO_MODE_OUTPUT_ONLY
        // );
        // Configure Rx Pin
        // ret = lp_uart_config_io(
        //     cfg->uart_pin_cfg.rx_io_num, RTC_GPIO_MODE_INPUT_ONLY
        // );
        // Configure RTS Pin
        // ret = lp_uart_config_io(
        //     cfg->uart_pin_cfg.rts_io_num, RTC_GPIO_MODE_OUTPUT_ONLY
        // );
        // Configure CTS Pin
        // ret = lp_uart_config_io(
        //     cfg->uart_pin_cfg.cts_io_num, RTC_GPIO_MODE_INPUT_ONLY
        // );

        // end: lp_core_uart_set_pin

        me
    }

    // ---

    fn rxfifo_reset(&mut self) {
        self.uart.conf0_sync.modify(|_, w| w.rxfifo_rst().set_bit());
        self.update();

        self.uart
            .conf0_sync
            .modify(|_, w| w.rxfifo_rst().clear_bit());
        self.update();
    }

    fn txfifo_reset(&mut self) {
        self.uart.conf0_sync.modify(|_, w| w.txfifo_rst().set_bit());
        self.update();

        self.uart
            .conf0_sync
            .modify(|_, w| w.txfifo_rst().clear_bit());
        self.update();
    }

    fn update(&mut self) {
        self.uart.reg_update.modify(|_, w| w.reg_update().set_bit());
        while self.uart.reg_update.read().reg_update().bit_is_set() {
            // wait
        }
    }

    // ---

    fn read_byte(&mut self) -> nb::Result<u8, Error> {
        if self.get_rx_fifo_count() > 0 {
            let byte = self.uart.fifo.read().rxfifo_rd_byte().bits();
            Ok(byte)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write_byte(&mut self, byte: u8) -> nb::Result<(), Error> {
        if self.get_tx_fifo_count() < UART_FIFO_SIZE {
            self.uart
                .fifo
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(byte) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write_bytes(&mut self, data: &[u8]) -> nb::Result<(), Error> {
        data.iter().try_for_each(|c| self.write_byte(*c))
    }

    fn flush_tx(&mut self) -> nb::Result<(), Error> {
        if self.is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn get_rx_fifo_count(&mut self) -> u16 {
        self.uart.status.read().rxfifo_cnt().bits().into()
    }

    fn get_tx_fifo_count(&mut self) -> u16 {
        self.uart.status.read().txfifo_cnt().bits().into()
    }

    fn is_tx_idle(&self) -> bool {
        self.uart.fsm_status.read().st_utx_out().bits() == 0
    }
}

impl core::fmt::Write for Uart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

impl embedded_hal::serial::Read<u8> for Uart {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl embedded_hal::serial::Write<u8> for Uart {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}
