//! Low-power UART

use crate::{
    gpio::{InputPin, OutputPin, RtcPin},
    peripherals::{LP_CLKRST, LP_UART, LPWR},
    uart::{DataBits, Parity, StopBits},
};

/// Trait representing the LP_UART TX pin.
pub trait Tx: RtcPin + OutputPin {
    #[doc(hidden)]
    fn connect_tx(&self);
}

/// Trait representing the LP_UART RX pin.
pub trait Rx: RtcPin + InputPin {
    #[doc(hidden)]
    fn connect_rx(&self);
}

// Chips with an LP GPIO matrix can route the LP UART signals to any LP pin. When a pad's LP IO MUX
// has an LP UART function, use that; otherwise route through the matrix.
#[cfg(lp_io_has_gpio_matrix)]
for_each_lp_function! {
    (($_signal:ident, LP_GPIOn, $_pin:literal), $gpio:ident, $_af:literal, $_lp_in:tt $_lp_out:tt) => {
        impl Tx for crate::peripherals::$gpio<'_> {
            fn connect_tx(&self) {
                crate::gpio::lp_io::connect_output_signal(
                    self,
                    crate::gpio::lp_io::LpOutputSignal::LP_UART_TXD,
                );
            }
        }

        impl Rx for crate::peripherals::$gpio<'_> {
            fn connect_rx(&self) {
                crate::gpio::lp_io::connect_input_signal(
                    self,
                    crate::gpio::lp_io::LpInputSignal::LP_UART_RXD,
                );
            }
        }
    };
}

#[cfg(not(lp_io_has_gpio_matrix))]
for_each_lp_function! {
    (LP_UART_TXD, $gpio:ident, $af:literal) => {
        impl Tx for crate::peripherals::$gpio<'_> {
            fn connect_tx(&self) {
                configure_pad(self.rtc_number(), $af, false);
            }
        }
    };
    (LP_UART_RXD, $gpio:ident, $af:literal) => {
        impl Rx for crate::peripherals::$gpio<'_> {
            fn connect_rx(&self) {
                configure_pad(self.rtc_number(), $af, true);
            }
        }
    };
}

/// Hands an LP pad over to the LP domain and selects its LP UART function in the LP IO MUX.
///
/// The output enable is left to the peripheral: selecting a function other than LP GPIO takes the
/// pad's direction out of the LP GPIO peripheral's hands.
#[cfg(not(lp_io_has_gpio_matrix))]
fn configure_pad(lp_pin: u8, function: u8, input_enable: bool) {
    use crate::peripherals::{LP_AON, LP_IO};

    let ionum = lp_pin as usize;

    LP_AON::regs()
        .gpio_mux()
        .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() | (1 << ionum)) });

    LP_IO::regs().gpio(ionum).modify(|_, w| unsafe {
        w.fun_ie().bit(input_enable);
        w.mcu_sel().bits(function)
    });
}

/// LP-UART Configuration
#[derive(Debug, Clone, Copy, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The baud rate (speed) of the UART communication in bits per second
    /// (bps).
    baudrate: u32,
    /// Number of data bits in each frame (5, 6, 7, or 8 bits).
    data_bits: DataBits,
    /// Parity setting (None, Even, or Odd).
    parity: Parity,
    /// Number of stop bits in each frame (1, 1.5, or 2 bits).
    stop_bits: StopBits,
    /// Clock source used by the UART peripheral.
    #[builder_lite(unstable)]
    clock_source: ClockSource,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            baudrate: 115_200,
            data_bits: Default::default(),
            parity: Default::default(),
            stop_bits: Default::default(),
            clock_source: Default::default(),
        }
    }
}

/// LP-UART clock source
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum ClockSource {
    /// RC_FAST_CLK clock source
    RcFast,

    /// XTAL_D2 clock source
    #[default]
    Xtal,
}

/// LP-UART driver
pub struct LpUart {
    uart: LP_UART<'static>,
}

impl LpUart {
    /// Initialize the UART driver using the provided configuration
    // TODO: CTS and RTS pins
    pub fn new(
        uart: LP_UART<'static>,
        config: Config,
        tx: impl Tx + 'static,
        rx: impl Rx + 'static,
    ) -> Self {
        rx.connect_rx();
        tx.connect_tx();

        let mut me = Self { uart };
        let uart = me.uart.register_block();

        // Set UART mode - do nothing for LP

        // Disable UART parity
        // 8-bit world
        // 1-bit stop bit
        uart.conf0().modify(|_, w| unsafe {
            w.parity().clear_bit();
            w.parity_en().clear_bit();
            w.bit_num().bits(0x3);
            w.stop_bit_num().bits(0x1)
        });
        // Set tx idle
        uart.idle_conf()
            .modify(|_, w| unsafe { w.tx_idle_num().bits(0) });
        // Disable hw-flow control
        uart.hwfc_conf().modify(|_, w| w.rx_flow_en().clear_bit());

        // Get source clock frequency
        // default == SOC_MOD_CLK_RTC_FAST == 2

        // LPWR.lpperi.lp_uart_clk_sel = 0;
        LPWR::regs()
            .lpperi()
            .modify(|_, w| w.lp_uart_clk_sel().clear_bit());

        // Override protocol parameters from the configuration
        // uart_hal_set_baudrate(&hal, cfg->uart_proto_cfg.baud_rate, sclk_freq);
        me.change_baud_internal(&config);
        // uart_hal_set_parity(&hal, cfg->uart_proto_cfg.parity);
        me.change_parity(config.parity);
        // uart_hal_set_data_bit_num(&hal, cfg->uart_proto_cfg.data_bits);
        me.change_data_bits(config.data_bits);
        // uart_hal_set_stop_bits(&hal, cfg->uart_proto_cfg.stop_bits);
        me.change_stop_bits(config.stop_bits);
        // uart_hal_set_tx_idle_num(&hal, LP_UART_TX_IDLE_NUM_DEFAULT);
        me.change_tx_idle(0); // LP_UART_TX_IDLE_NUM_DEFAULT == 0

        // Reset Tx/Rx FIFOs
        me.rxfifo_reset();
        me.txfifo_reset();

        me
    }

    fn rxfifo_reset(&mut self) {
        self.uart
            .register_block()
            .conf0()
            .modify(|_, w| w.rxfifo_rst().set_bit());
        self.update();

        self.uart
            .register_block()
            .conf0()
            .modify(|_, w| w.rxfifo_rst().clear_bit());
        self.update();
    }

    fn txfifo_reset(&mut self) {
        self.uart
            .register_block()
            .conf0()
            .modify(|_, w| w.txfifo_rst().set_bit());
        self.update();

        self.uart
            .register_block()
            .conf0()
            .modify(|_, w| w.txfifo_rst().clear_bit());
        self.update();
    }

    fn update(&mut self) {
        let register_block = self.uart.register_block();
        register_block
            .reg_update()
            .modify(|_, w| w.reg_update().set_bit());
        while register_block.reg_update().read().reg_update().bit_is_set() {
            // wait
        }
    }

    fn change_baud_internal(&mut self, config: &Config) {
        let clk = match config.clock_source {
            ClockSource::RcFast => crate::soc::clocks::rc_fast_clk_frequency(),
            ClockSource::Xtal => crate::soc::clocks::xtal_d2_clk_frequency(),
        };

        LP_CLKRST::regs().lpperi().modify(|_, w| {
            w.lp_uart_clk_sel().bit(match config.clock_source {
                ClockSource::RcFast => false,
                ClockSource::Xtal => true,
            })
        });
        self.uart.register_block().clk_conf().modify(|_, w| {
            w.rx_sclk_en().set_bit();
            w.tx_sclk_en().set_bit()
        });

        let divider = clk / config.baudrate;
        let divider = divider as u16;

        self.uart
            .register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

        self.update();
    }

    /// Modify UART baud rate and reset TX/RX fifo.
    pub fn change_baud(&mut self, config: &Config) {
        self.change_baud_internal(config);
        self.txfifo_reset();
        self.rxfifo_reset();
    }

    fn change_parity(&mut self, parity: Parity) -> &mut Self {
        if parity != Parity::None {
            self.uart
                .register_block()
                .conf0()
                .modify(|_, w| w.parity().bit((parity as u8 & 0x1) != 0));
        }

        self.uart
            .register_block()
            .conf0()
            .modify(|_, w| match parity {
                Parity::None => w.parity_en().clear_bit(),
                Parity::Even => w.parity_en().set_bit().parity().clear_bit(),
                Parity::Odd => w.parity_en().set_bit().parity().set_bit(),
            });

        self
    }

    fn change_data_bits(&mut self, data_bits: DataBits) -> &mut Self {
        self.uart
            .register_block()
            .conf0()
            .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });

        self.update();
        self
    }

    fn change_stop_bits(&mut self, stop_bits: StopBits) -> &mut Self {
        self.uart
            .register_block()
            .conf0()
            .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8 + 1) });

        self.update();
        self
    }

    fn change_tx_idle(&mut self, idle_num: u16) -> &mut Self {
        self.uart
            .register_block()
            .idle_conf()
            .modify(|_, w| unsafe { w.tx_idle_num().bits(idle_num) });

        self.update();
        self
    }
}
