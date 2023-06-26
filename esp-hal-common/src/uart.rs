//! UART driver

use self::config::Config;
#[cfg(uart2)]
use crate::peripherals::UART2;
use crate::{
    clock::Clocks,
    gpio::{InputPin, InputSignal, OutputPin, OutputSignal},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{
        uart0::{fifo::FIFO_SPEC, RegisterBlock},
        UART0,
        UART1,
    },
    system::PeripheralClockControl,
};

const UART_FIFO_SIZE: u16 = 128;

/// Custom serial error type
#[derive(Debug)]
pub enum Error {
    #[cfg(feature = "async")]
    ReadBufferFull,
}

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
        ParityNone,
        ParityEven,
        ParityOdd,
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

    /// Configuration for the AT-CMD detection functionality
    pub struct AtCmdConfig {
        pub pre_idle_count: Option<u16>,
        pub post_idle_count: Option<u16>,
        pub gap_timeout: Option<u16>,
        pub cmd_char: u8,
        pub char_num: Option<u8>,
    }

    impl AtCmdConfig {
        pub fn new(
            pre_idle_count: Option<u16>,
            post_idle_count: Option<u16>,
            gap_timeout: Option<u16>,
            cmd_char: u8,
            char_num: Option<u8>,
        ) -> AtCmdConfig {
            Self {
                pre_idle_count,
                post_idle_count,
                gap_timeout,
                cmd_char,
                char_num,
            }
        }
    }
}

/// Pins used by the UART interface
pub trait UartPins {
    fn configure_pins(
        &mut self,
        tx_signal: OutputSignal,
        rx_signal: InputSignal,
        cts_signal: InputSignal,
        rts_signal: OutputSignal,
    );
}

/// All pins offered by UART
pub struct AllPins<'d, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin> {
    pub(crate) tx: Option<PeripheralRef<'d, TX>>,
    pub(crate) rx: Option<PeripheralRef<'d, RX>>,
    pub(crate) cts: Option<PeripheralRef<'d, CTS>>,
    pub(crate) rts: Option<PeripheralRef<'d, RTS>>,
}

/// Tx and Rx pins
impl<'d, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin> AllPins<'d, TX, RX, CTS, RTS> {
    pub fn new(
        tx: impl Peripheral<P = TX> + 'd,
        rx: impl Peripheral<P = RX> + 'd,
        cts: impl Peripheral<P = CTS> + 'd,
        rts: impl Peripheral<P = RTS> + 'd,
    ) -> AllPins<'d, TX, RX, CTS, RTS> {
        crate::into_ref!(tx, rx, cts, rts);
        AllPins {
            tx: Some(tx),
            rx: Some(rx),
            cts: Some(cts),
            rts: Some(rts),
        }
    }
}

impl<TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin> UartPins
    for AllPins<'_, TX, RX, CTS, RTS>
{
    fn configure_pins(
        &mut self,
        tx_signal: OutputSignal,
        rx_signal: InputSignal,
        cts_signal: InputSignal,
        rts_signal: OutputSignal,
    ) {
        if let Some(ref mut tx) = self.tx {
            tx.set_to_push_pull_output()
                .connect_peripheral_to_output(tx_signal);
        }

        if let Some(ref mut rx) = self.rx {
            rx.set_to_input().connect_input_to_peripheral(rx_signal);
        }

        if let Some(ref mut cts) = self.cts {
            cts.set_to_input().connect_input_to_peripheral(cts_signal);
        }

        if let Some(ref mut rts) = self.rts {
            rts.set_to_push_pull_output()
                .connect_peripheral_to_output(rts_signal);
        }
    }
}

pub struct TxRxPins<'d, TX: OutputPin, RX: InputPin> {
    pub tx: Option<PeripheralRef<'d, TX>>,
    pub rx: Option<PeripheralRef<'d, RX>>,
}

impl<'d, TX: OutputPin, RX: InputPin> TxRxPins<'d, TX, RX> {
    pub fn new_tx_rx(
        tx: impl Peripheral<P = TX> + 'd,
        rx: impl Peripheral<P = RX> + 'd,
    ) -> TxRxPins<'d, TX, RX> {
        crate::into_ref!(tx, rx);
        TxRxPins {
            tx: Some(tx),
            rx: Some(rx),
        }
    }
}

impl<TX: OutputPin, RX: InputPin> UartPins for TxRxPins<'_, TX, RX> {
    fn configure_pins(
        &mut self,
        tx_signal: OutputSignal,
        rx_signal: InputSignal,
        _cts_signal: InputSignal,
        _rts_signal: OutputSignal,
    ) {
        if let Some(ref mut tx) = self.tx {
            tx.set_to_push_pull_output()
                .connect_peripheral_to_output(tx_signal);
        }

        if let Some(ref mut rx) = self.rx {
            rx.set_to_input().connect_input_to_peripheral(rx_signal);
        }
    }
}

#[cfg(feature = "eh1")]
impl embedded_hal_1::serial::Error for Error {
    fn kind(&self) -> embedded_hal_1::serial::ErrorKind {
        embedded_hal_1::serial::ErrorKind::Other
    }
}

/// UART driver
pub struct Uart<'d, T> {
    uart: PeripheralRef<'d, T>,
}

impl<'d, T> Uart<'d, T>
where
    T: Instance,
{
    /// Create a new UART instance with defaults
    pub fn new_with_config<P>(
        uart: impl Peripheral<P = T> + 'd,
        config: Option<Config>,
        mut pins: Option<P>,
        clocks: &Clocks,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self
    where
        P: UartPins,
    {
        crate::into_ref!(uart);
        uart.enable_peripheral(peripheral_clock_control);
        let mut serial = Uart { uart };
        serial.uart.disable_rx_interrupts();
        serial.uart.disable_tx_interrupts();

        if let Some(ref mut pins) = pins {
            pins.configure_pins(
                serial.uart.tx_signal(),
                serial.uart.rx_signal(),
                serial.uart.cts_signal(),
                serial.uart.rts_signal(),
            );
        }

        config.map(|config| {
            serial.change_data_bits(config.data_bits);
            serial.change_parity(config.parity);
            serial.change_stop_bits(config.stop_bits);
            serial.change_baud(config.baudrate, clocks);
        });

        serial
    }

    /// Create a new UART instance with defaults
    pub fn new(
        uart: impl Peripheral<P = T> + 'd,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(uart);
        uart.enable_peripheral(peripheral_clock_control);
        let mut serial = Uart { uart };
        serial.uart.disable_rx_interrupts();
        serial.uart.disable_tx_interrupts();

        serial
    }

    /// Writes bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        data.iter()
            .try_for_each(|c| nb::block!(self.write_byte(*c)))
    }

    /// Configures the AT-CMD detection settings.
    pub fn set_at_cmd(&mut self, config: config::AtCmdConfig) {
        #[cfg(not(any(esp32, esp32s2)))]
        self.uart
            .register_block()
            .clk_conf
            .modify(|_, w| w.sclk_en().clear_bit());

        self.uart.register_block().at_cmd_char.write(|w| unsafe {
            w.at_cmd_char()
                .bits(config.cmd_char)
                .char_num()
                .bits(config.char_num.or(Some(1)).unwrap())
        });

        if let Some(pre_idle_count) = config.pre_idle_count {
            self.uart
                .register_block()
                .at_cmd_precnt
                .write(|w| unsafe { w.pre_idle_num().bits(pre_idle_count.into()) });
        }

        if let Some(post_idle_count) = config.post_idle_count {
            self.uart
                .register_block()
                .at_cmd_postcnt
                .write(|w| unsafe { w.post_idle_num().bits(post_idle_count.into()) });
        }

        if let Some(gap_timeout) = config.gap_timeout {
            self.uart
                .register_block()
                .at_cmd_gaptout
                .write(|w| unsafe { w.rx_gap_tout().bits(gap_timeout.into()) });
        }

        #[cfg(not(any(esp32, esp32s2)))]
        self.uart
            .register_block()
            .clk_conf
            .modify(|_, w| w.sclk_en().set_bit());

        self.sync_regs();
    }

    /// Configures the RX-FIFO threshold
    pub fn set_rx_fifo_full_threshold(&mut self, threshold: u16) {
        #[cfg(any(esp32, esp32c6, esp32h2))]
        let threshold: u8 = threshold as u8;

        self.uart
            .register_block()
            .conf1
            .modify(|_, w| unsafe { w.rxfifo_full_thrhd().bits(threshold) });
    }

    /// Listen for AT-CMD interrupts
    pub fn listen_at_cmd(&mut self) {
        self.uart
            .register_block()
            .int_ena
            .modify(|_, w| w.at_cmd_char_det_int_ena().set_bit());
    }

    /// Stop listening for AT-CMD interrupts
    pub fn unlisten_at_cmd(&mut self) {
        self.uart
            .register_block()
            .int_ena
            .modify(|_, w| w.at_cmd_char_det_int_ena().clear_bit());
    }

    /// Listen for TX-DONE interrupts
    pub fn listen_tx_done(&mut self) {
        self.uart
            .register_block()
            .int_ena
            .modify(|_, w| w.tx_done_int_ena().set_bit());
    }

    /// Stop listening for TX-DONE interrupts
    pub fn unlisten_tx_done(&mut self) {
        self.uart
            .register_block()
            .int_ena
            .modify(|_, w| w.tx_done_int_ena().clear_bit());
    }

    /// Listen for RX-FIFO-FULL interrupts
    pub fn listen_rx_fifo_full(&mut self) {
        self.uart
            .register_block()
            .int_ena
            .modify(|_, w| w.rxfifo_full_int_ena().set_bit());
    }

    /// Stop listening for RX-FIFO-FULL interrupts
    pub fn unlisten_rx_fifo_full(&mut self) {
        self.uart
            .register_block()
            .int_ena
            .modify(|_, w| w.rxfifo_full_int_ena().clear_bit());
    }

    /// Checks if AT-CMD interrupt is set
    pub fn at_cmd_interrupt_set(&self) -> bool {
        self.uart
            .register_block()
            .int_raw
            .read()
            .at_cmd_char_det_int_raw()
            .bit_is_set()
    }

    /// Checks if TX-DONE interrupt is set
    pub fn tx_done_interrupt_set(&self) -> bool {
        self.uart
            .register_block()
            .int_raw
            .read()
            .tx_done_int_raw()
            .bit_is_set()
    }

    /// Checks if RX-FIFO-FULL interrupt is set
    pub fn rx_fifo_full_interrupt_set(&self) -> bool {
        self.uart
            .register_block()
            .int_raw
            .read()
            .rxfifo_full_int_raw()
            .bit_is_set()
    }

    /// Reset AT-CMD interrupt
    pub fn reset_at_cmd_interrupt(&self) {
        self.uart
            .register_block()
            .int_clr
            .write(|w| w.at_cmd_char_det_int_clr().set_bit());
    }

    /// Reset TX-DONE interrupt
    pub fn reset_tx_done_interrupt(&self) {
        self.uart
            .register_block()
            .int_clr
            .write(|w| w.tx_done_int_clr().set_bit());
    }

    /// Reset RX-FIFO-FULL interrupt
    pub fn reset_rx_fifo_full_interrupt(&self) {
        self.uart
            .register_block()
            .int_clr
            .write(|w| w.rxfifo_full_int_clr().set_bit());
    }

    fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        if self.uart.get_tx_fifo_count() < UART_FIFO_SIZE {
            self.uart
                .register_block()
                .fifo
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush_tx(&self) -> nb::Result<(), Error> {
        if self.uart.is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn read_byte(&mut self) -> nb::Result<u8, Error> {
        #[allow(unused_variables)]
        let offset = 0;

        // on ESP32-S2 we need to use PeriBus2 to read the FIFO
        #[cfg(esp32s2)]
        let offset = 0x20c00000;

        if self.uart.get_rx_fifo_count() > 0 {
            let value = unsafe {
                let fifo = (self.uart.register_block().fifo.as_ptr() as *mut u8).offset(offset)
                    as *mut crate::peripherals::generic::Reg<FIFO_SPEC>;
                (*fifo).read().rxfifo_rd_byte().bits()
            };

            Ok(value)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Change the number of stop bits
    pub fn change_stop_bits(&mut self, stop_bits: config::StopBits) -> &mut Self {
        // workaround for hardware issue, when UART stop bit set as 2-bit mode.
        #[cfg(esp32)]
        if stop_bits == config::StopBits::STOP2 {
            self.uart
                .register_block()
                .rs485_conf
                .modify(|_, w| w.dl1_en().bit(true));

            self.uart
                .register_block()
                .conf0
                .modify(|_, w| unsafe { w.stop_bit_num().bits(1) });
        } else {
            self.uart
                .register_block()
                .rs485_conf
                .modify(|_, w| w.dl1_en().bit(false));

            self.uart
                .register_block()
                .conf0
                .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });
        }

        #[cfg(not(esp32))]
        self.uart
            .register_block()
            .conf0
            .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });

        self
    }

    /// Change the number of data bits
    fn change_data_bits(&mut self, data_bits: config::DataBits) -> &mut Self {
        self.uart
            .register_block()
            .conf0
            .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });

        self
    }

    /// Change the type of parity checking
    fn change_parity(&mut self, parity: config::Parity) -> &mut Self {
        self.uart
            .register_block()
            .conf0
            .modify(|_, w| match parity {
                config::Parity::ParityNone => w.parity_en().clear_bit(),
                config::Parity::ParityEven => w.parity_en().set_bit().parity().clear_bit(),
                config::Parity::ParityOdd => w.parity_en().set_bit().parity().set_bit(),
            });

        self
    }

    #[cfg(any(esp32c2, esp32c3, esp32s3))]
    fn change_baud(&self, baudrate: u32, clocks: &Clocks) {
        // we force the clock source to be APB and don't use the decimal part of the
        // divider
        let clk = clocks.apb_clock.to_Hz();
        let max_div = 0b1111_1111_1111 - 1;
        let clk_div = ((clk) + (max_div * baudrate) - 1) / (max_div * baudrate);

        self.uart.register_block().clk_conf.write(|w| unsafe {
            w.sclk_sel()
                .bits(1) // APB
                .sclk_div_a()
                .bits(0)
                .sclk_div_b()
                .bits(0)
                .sclk_div_num()
                .bits(clk_div as u8 - 1)
                .rx_sclk_en()
                .bit(true)
                .tx_sclk_en()
                .bit(true)
        });

        let clk = clk / clk_div;
        let divider = clk / baudrate;
        let divider = divider as u16;

        self.uart
            .register_block()
            .clkdiv
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });
    }

    #[cfg(any(esp32c6, esp32h2))]
    fn change_baud(&self, baudrate: u32, clocks: &Clocks) {
        // we force the clock source to be APB and don't use the decimal part of the
        // divider
        let clk = clocks.apb_clock.to_Hz();
        let max_div = 0b1111_1111_1111 - 1;
        let clk_div = ((clk) + (max_div * baudrate) - 1) / (max_div * baudrate);

        // UART clocks are configured via PCR
        let pcr = unsafe { &*crate::peripherals::PCR::PTR };

        match self.uart.uart_number() {
            0 => {
                pcr.uart0_conf
                    .modify(|_, w| w.uart0_rst_en().clear_bit().uart0_clk_en().set_bit());

                pcr.uart0_sclk_conf.modify(|_, w| unsafe {
                    w.uart0_sclk_div_a()
                        .bits(0)
                        .uart0_sclk_div_b()
                        .bits(0)
                        .uart0_sclk_div_num()
                        .bits(clk_div as u8 - 1)
                        .uart0_sclk_sel()
                        .bits(0x1) // TODO: this probably shouldn't be hard-coded
                        .uart0_sclk_en()
                        .set_bit()
                });
            }
            1 => {
                pcr.uart1_conf
                    .modify(|_, w| w.uart1_rst_en().clear_bit().uart1_clk_en().set_bit());

                pcr.uart1_sclk_conf.modify(|_, w| unsafe {
                    w.uart1_sclk_div_a()
                        .bits(0)
                        .uart1_sclk_div_b()
                        .bits(0)
                        .uart1_sclk_div_num()
                        .bits(clk_div as u8 - 1)
                        .uart1_sclk_sel()
                        .bits(0x1) // TODO: this probably shouldn't be hard-coded
                        .uart1_sclk_en()
                        .set_bit()
                });
            }
            _ => unreachable!(), // ESP32-C6 only has 2 UART instances
        }

        let clk = clk / clk_div;
        let divider = clk / baudrate;
        let divider = divider as u16;

        self.uart
            .register_block()
            .clkdiv
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });
    }

    #[cfg(any(esp32, esp32s2))]
    fn change_baud(&self, baudrate: u32, clocks: &Clocks) {
        // we force the clock source to be APB and don't use the decimal part of the
        // divider
        let clk = clocks.apb_clock.to_Hz();

        self.uart
            .register_block()
            .conf0
            .modify(|_, w| w.tick_ref_always_on().bit(true));
        let divider = clk / baudrate;

        self.uart
            .register_block()
            .clkdiv
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });
    }

    #[cfg(any(esp32c6, esp32h2))] // TODO introduce a cfg symbol for this
    #[inline(always)]
    fn sync_regs(&mut self) {
        self.uart
            .register_block()
            .reg_update
            .modify(|_, w| w.reg_update().set_bit());

        while self
            .uart
            .register_block()
            .reg_update
            .read()
            .reg_update()
            .bit_is_set()
        {
            // wait
        }
    }

    #[cfg(not(any(esp32c6, esp32h2)))]
    #[inline(always)]
    fn sync_regs(&mut self) {}

    #[cfg(feature = "async")]
    pub(crate) fn inner(&self) -> &T {
        &self.uart
    }

    #[cfg(feature = "async")]
    pub(crate) fn inner_mut(&mut self) -> &mut T {
        &mut self.uart
    }
}

/// UART peripheral instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn uart_number(&self) -> usize;

    fn disable_tx_interrupts(&mut self) {
        self.register_block().int_clr.write(|w| {
            w.txfifo_empty_int_clr()
                .set_bit()
                .tx_brk_done_int_clr()
                .set_bit()
                .tx_brk_idle_done_int_clr()
                .set_bit()
                .tx_done_int_clr()
                .set_bit()
        });

        self.register_block().int_ena.write(|w| {
            w.txfifo_empty_int_ena()
                .clear_bit()
                .tx_brk_done_int_ena()
                .clear_bit()
                .tx_brk_idle_done_int_ena()
                .clear_bit()
                .tx_done_int_ena()
                .clear_bit()
        });
    }

    fn disable_rx_interrupts(&mut self) {
        self.register_block().int_clr.write(|w| {
            w.rxfifo_full_int_clr()
                .set_bit()
                .rxfifo_ovf_int_clr()
                .set_bit()
                .rxfifo_tout_int_clr()
                .set_bit()
        });

        self.register_block().int_ena.write(|w| {
            w.rxfifo_full_int_ena()
                .clear_bit()
                .rxfifo_ovf_int_ena()
                .clear_bit()
                .rxfifo_tout_int_ena()
                .clear_bit()
        });
    }

    fn get_tx_fifo_count(&mut self) -> u16 {
        self.register_block()
            .status
            .read()
            .txfifo_cnt()
            .bits()
            .into()
    }

    fn get_rx_fifo_count(&mut self) -> u16 {
        self.register_block()
            .status
            .read()
            .rxfifo_cnt()
            .bits()
            .into()
    }

    fn is_tx_idle(&self) -> bool {
        #[cfg(esp32)]
        let idle = self.register_block().status.read().st_utx_out().bits() == 0x0u8;
        #[cfg(not(esp32))]
        let idle = self.register_block().fsm_status.read().st_utx_out().bits() == 0x0u8;

        idle
    }

    fn is_rx_idle(&self) -> bool {
        #[cfg(esp32)]
        let idle = self.register_block().status.read().st_urx_out().bits() == 0x0u8;
        #[cfg(not(esp32))]
        let idle = self.register_block().fsm_status.read().st_urx_out().bits() == 0x0u8;

        idle
    }

    fn tx_signal(&self) -> OutputSignal;

    fn rx_signal(&self) -> InputSignal;

    fn cts_signal(&self) -> InputSignal;

    fn rts_signal(&self) -> OutputSignal;

    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl);
}

impl Instance for UART0 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn uart_number(&self) -> usize {
        0
    }

    fn tx_signal(&self) -> OutputSignal {
        OutputSignal::U0TXD
    }

    fn rx_signal(&self) -> InputSignal {
        InputSignal::U0RXD
    }

    fn cts_signal(&self) -> InputSignal {
        InputSignal::U0CTS
    }

    fn rts_signal(&self) -> OutputSignal {
        OutputSignal::U0RTS
    }

    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Uart0);
    }
}

impl Instance for UART1 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn uart_number(&self) -> usize {
        1
    }

    fn tx_signal(&self) -> OutputSignal {
        OutputSignal::U1TXD
    }

    fn rx_signal(&self) -> InputSignal {
        InputSignal::U1RXD
    }

    fn cts_signal(&self) -> InputSignal {
        InputSignal::U1CTS
    }

    fn rts_signal(&self) -> OutputSignal {
        OutputSignal::U1RTS
    }

    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Uart1);
    }
}

#[cfg(uart2)]
impl Instance for UART2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn uart_number(&self) -> usize {
        2
    }

    fn tx_signal(&self) -> OutputSignal {
        OutputSignal::U2TXD
    }

    fn rx_signal(&self) -> InputSignal {
        InputSignal::U2RXD
    }

    fn cts_signal(&self) -> InputSignal {
        InputSignal::U2CTS
    }

    fn rts_signal(&self) -> OutputSignal {
        OutputSignal::U2RTS
    }

    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Uart2);
    }
}

#[cfg(feature = "ufmt")]
impl<T> ufmt_write::uWrite for Uart<'_, T>
where
    T: Instance,
{
    type Error = Error;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_bytes(s.as_bytes())
    }

    #[inline]
    fn write_char(&mut self, ch: char) -> Result<(), Self::Error> {
        let mut buffer = [0u8; 4];
        self.write_bytes(ch.encode_utf8(&mut buffer).as_bytes())
    }
}

impl<T> core::fmt::Write for Uart<'_, T>
where
    T: Instance,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

impl<T> embedded_hal::serial::Write<u8> for Uart<'_, T>
where
    T: Instance,
{
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

impl<T> embedded_hal::serial::Read<u8> for Uart<'_, T>
where
    T: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::serial::ErrorType for Uart<'_, T> {
    type Error = Error;
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_nb::serial::Read for Uart<'_, T>
where
    T: Instance,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_nb::serial::Write for Uart<'_, T>
where
    T: Instance,
{
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

#[cfg(feature = "async")]
mod asynch {
    use core::task::Poll;

    use cfg_if::cfg_if;
    use embassy_futures::select::select;
    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::interrupt;

    use super::{Error, Instance};
    use crate::{
        uart::{RegisterBlock, UART_FIFO_SIZE},
        Uart,
    };

    cfg_if! {
        if #[cfg(all(uart0, uart1, uart2))] {
            const NUM_UART: usize = 3;
        } else if #[cfg(all(uart0, uart1))] {
            const NUM_UART: usize = 2;
        } else if #[cfg(uart0)] {
            const NUM_UART: usize = 1;
        }
    }

    const INIT: AtomicWaker = AtomicWaker::new();
    static WAKERS: [AtomicWaker; NUM_UART] = [INIT; NUM_UART];

    pub(crate) enum Event {
        TxDone,
        TxFiFoEmpty,
        RxFifoFull,
        RxCmdCharDetected,
    }

    pub(crate) struct UartFuture<'a, T: Instance> {
        event: Event,
        instance: &'a T,
    }

    impl<'a, T: Instance> UartFuture<'a, T> {
        pub fn new(event: Event, instance: &'a T) -> Self {
            match event {
                Event::TxDone => instance
                    .register_block()
                    .int_ena
                    .modify(|_, w| w.tx_done_int_ena().set_bit()),
                Event::TxFiFoEmpty => instance
                    .register_block()
                    .int_ena
                    .modify(|_, w| w.txfifo_empty_int_ena().set_bit()),
                Event::RxFifoFull => instance
                    .register_block()
                    .int_ena
                    .modify(|_, w| w.rxfifo_full_int_ena().set_bit()),
                Event::RxCmdCharDetected => instance
                    .register_block()
                    .int_ena
                    .modify(|_, w| w.at_cmd_char_det_int_ena().set_bit()),
            }

            Self { event, instance }
        }

        fn event_bit_is_clear(&self) -> bool {
            match self.event {
                Event::TxDone => self
                    .instance
                    .register_block()
                    .int_ena
                    .read()
                    .tx_done_int_ena()
                    .bit_is_clear(),
                Event::TxFiFoEmpty => self
                    .instance
                    .register_block()
                    .int_ena
                    .read()
                    .txfifo_empty_int_ena()
                    .bit_is_clear(),
                Event::RxFifoFull => self
                    .instance
                    .register_block()
                    .int_ena
                    .read()
                    .rxfifo_full_int_ena()
                    .bit_is_clear(),
                Event::RxCmdCharDetected => self
                    .instance
                    .register_block()
                    .int_ena
                    .read()
                    .at_cmd_char_det_int_ena()
                    .bit_is_clear(),
            }
        }
    }

    impl<'a, T: Instance> core::future::Future for UartFuture<'a, T> {
        type Output = ();

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            WAKERS[self.instance.uart_number()].register(cx.waker());
            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<T> Uart<'_, T>
    where
        T: Instance,
    {
        pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
            let mut read_bytes = 0;

            select(
                UartFuture::new(Event::RxCmdCharDetected, self.inner()),
                UartFuture::new(Event::RxFifoFull, self.inner()),
            )
            .await;

            while let Ok(byte) = self.read_byte() {
                if read_bytes < buf.len() {
                    buf[read_bytes] = byte;
                    read_bytes += 1;
                } else {
                    return Err(Error::ReadBufferFull);
                }
            }

            Ok(read_bytes)
        }

        async fn write(&mut self, words: &[u8]) -> Result<(), Error> {
            let mut offset: usize = 0;
            loop {
                let mut next_offset =
                    offset + (UART_FIFO_SIZE - self.uart.get_tx_fifo_count()) as usize;
                if next_offset > words.len() {
                    next_offset = words.len();
                }
                for &byte in &words[offset..next_offset] {
                    self.write_byte(byte).unwrap(); // should never fail
                }
                if next_offset == words.len() {
                    break;
                }
                offset = next_offset;
                UartFuture::new(Event::TxFiFoEmpty, self.inner()).await;
            }
            Ok(())
        }

        async fn flush(&mut self) -> Result<(), Error> {
            let count = self.inner_mut().get_tx_fifo_count();
            if count > 0 {
                UartFuture::new(Event::TxDone, self.inner()).await;
            }
            Ok(())
        }
    }

    impl<T> embedded_hal_async::serial::Write for Uart<'_, T>
    where
        T: Instance,
    {
        async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.write(words).await
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            self.flush().await
        }
    }

    fn intr_handler(uart: &RegisterBlock) -> bool {
        let int_ena_val = uart.int_ena.read();
        let int_raw_val = uart.int_raw.read();
        if int_ena_val.txfifo_empty_int_ena().bit_is_set()
            && int_raw_val.txfifo_empty_int_raw().bit_is_set()
        {
            uart.int_ena.write(|w| w.txfifo_empty_int_ena().clear_bit());
            return true;
        }
        if int_ena_val.tx_done_int_ena().bit_is_set() && int_raw_val.tx_done_int_raw().bit_is_set()
        {
            uart.int_ena.write(|w| w.tx_done_int_ena().clear_bit());
            return true;
        }
        if int_ena_val.at_cmd_char_det_int_ena().bit_is_set()
            && int_raw_val.at_cmd_char_det_int_raw().bit_is_set()
        {
            uart.int_clr
                .write(|w| w.at_cmd_char_det_int_clr().set_bit());
            uart.int_ena
                .write(|w| w.at_cmd_char_det_int_ena().clear_bit());
            return true;
        }
        if int_ena_val.rxfifo_full_int_ena().bit_is_set()
            && int_raw_val.rxfifo_full_int_raw().bit_is_set()
        {
            uart.int_clr.write(|w| w.rxfifo_full_int_clr().set_bit());
            uart.int_ena.write(|w| w.rxfifo_full_int_ena().clear_bit());
            return true;
        }
        false
    }

    #[cfg(uart0)]
    #[interrupt]
    fn UART0() {
        let uart = unsafe { &*crate::peripherals::UART0::ptr() };
        if intr_handler(uart) {
            WAKERS[0].wake();
        }
    }

    #[cfg(uart1)]
    #[interrupt]
    fn UART1() {
        let uart = unsafe { &*crate::peripherals::UART1::ptr() };
        if intr_handler(uart) {
            WAKERS[1].wake();
        }
    }

    #[cfg(uart2)]
    #[interrupt]
    fn UART2() {
        let uart = unsafe { &*crate::peripherals::UART2::ptr() };
        if intr_handler(uart) {
            WAKERS[2].wake();
        }
    }
}
