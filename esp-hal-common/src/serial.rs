//! UART driver

use self::config::Config;
#[cfg(any(feature = "esp32", feature = "esp32s3"))]
use crate::pac::UART2;
use crate::{
    clock::Clocks,
    pac::{
        uart0::{fifo::FIFO_SPEC, RegisterBlock},
        UART0,
        UART1,
    },
    types::{InputSignal, OutputSignal},
    Floating,
    Input,
    InputPin,
    NoPin,
    Output,
    OutputPin,
};

const UART_FIFO_SIZE: u16 = 128;

/// Custom serial error type
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
}

/// Pins used by the UART interface
pub struct Pins<TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin> {
    pub tx: Option<TX>,
    pub rx: Option<RX>,
    pub cts: Option<CTS>,
    pub rts: Option<RTS>,
}

impl<TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin> Pins<TX, RX, CTS, RTS> {
    pub fn new(tx: TX, rx: RX, cts: CTS, rts: RTS) -> Pins<TX, RX, CTS, RTS> {
        Pins {
            tx: Some(tx),
            rx: Some(rx),
            cts: Some(cts),
            rts: Some(rts),
        }
    }
}

impl<TX: OutputPin, RX: InputPin> Pins<TX, RX, NoPin<Input<Floating>>, NoPin<Output<Floating>>> {
    pub fn new_tx_rx(
        tx: TX,
        rx: RX,
    ) -> Pins<TX, RX, NoPin<Input<Floating>>, NoPin<Output<Floating>>> {
        let pins: Pins<TX, RX, NoPin<Input<Floating>>, NoPin<Output<Floating>>> = Pins {
            tx: Some(tx),
            rx: Some(rx),
            cts: None,
            rts: None,
        };
        pins
    }
}

#[cfg(feature = "eh1")]
impl embedded_hal_1::serial::Error for Error {
    fn kind(&self) -> embedded_hal_1::serial::ErrorKind {
        embedded_hal_1::serial::ErrorKind::Other
    }
}

/// UART driver
pub struct Serial<T> {
    uart: T,
}

impl<T> Serial<T>
where
    T: Instance,
{
    /// Create a new UART instance with defaults
    pub fn new_with_config<
        TX: OutputPin<OutputSignal = OutputSignal>,
        RX: InputPin<InputSignal = InputSignal>,
        CTS: InputPin<InputSignal = InputSignal>,
        RTS: OutputPin<OutputSignal = OutputSignal>,
    >(
        uart: T,
        config: Option<Config>,
        pins: Option<Pins<TX, RX, CTS, RTS>>,
        clocks: &Clocks,
    ) -> Result<Self, Error> {
        let mut serial = Serial { uart };
        serial.uart.disable_rx_interrupts();
        serial.uart.disable_tx_interrupts();

        serial.uart.configure_pins(pins);

        config.map(|config| {
            serial.change_data_bits(config.data_bits);
            serial.change_parity(config.parity);
            serial.change_stop_bits(config.stop_bits);
            serial.change_baud(config.baudrate, clocks);
        });

        Ok(serial)
    }

    /// Create a new UART instance with defaults
    pub fn new(uart: T) -> Result<Self, Error> {
        let mut serial = Serial { uart };
        serial.uart.disable_rx_interrupts();
        serial.uart.disable_tx_interrupts();

        Ok(serial)
    }

    /// Return the raw interface to the underlying UART instance
    pub fn free(self) -> T {
        self.uart
    }

    /// Writes bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        data.iter()
            .try_for_each(|c| nb::block!(self.write_byte(*c)))
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
        #[cfg(feature = "esp32s2")]
        let offset = 0x20c00000;

        if self.uart.get_rx_fifo_count() > 0 {
            let value = unsafe {
                let fifo = (self.uart.register_block().fifo.as_ptr() as *mut u8).offset(offset)
                    as *mut crate::pac::generic::Reg<FIFO_SPEC>;
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
        #[cfg(feature = "esp32")]
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

        #[cfg(not(feature = "esp32"))]
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

    #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
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

    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
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
}

/// UART peripheral instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

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
        #[cfg(feature = "esp32")]
        let idle = self.register_block().status.read().st_utx_out().bits() == 0x0u8;
        #[cfg(not(feature = "esp32"))]
        let idle = self.register_block().fsm_status.read().st_utx_out().bits() == 0x0u8;

        idle
    }

    fn is_rx_idle(&self) -> bool {
        #[cfg(feature = "esp32")]
        let idle = self.register_block().status.read().st_urx_out().bits() == 0x0u8;
        #[cfg(not(feature = "esp32"))]
        let idle = self.register_block().fsm_status.read().st_urx_out().bits() == 0x0u8;

        idle
    }

    fn configure_pins<
        TX: OutputPin<OutputSignal = OutputSignal>,
        RX: InputPin<InputSignal = InputSignal>,
        CTS: InputPin<InputSignal = InputSignal>,
        RTS: OutputPin<OutputSignal = OutputSignal>,
    >(
        &self,
        pins: Option<Pins<TX, RX, CTS, RTS>>,
    ) {
        if pins.is_some() {
            let pins = pins.unwrap();

            if pins.tx.is_some() {
                pins.tx
                    .unwrap()
                    .set_to_push_pull_output()
                    .connect_peripheral_to_output(self.tx_signal());
            }

            if pins.rx.is_some() {
                pins.rx
                    .unwrap()
                    .set_to_input()
                    .connect_input_to_peripheral(self.rx_signal());
            }

            if pins.cts.is_some() {
                pins.cts
                    .unwrap()
                    .set_to_input()
                    .connect_input_to_peripheral(self.cts_signal());
            }

            if pins.rts.is_some() {
                pins.rts
                    .unwrap()
                    .set_to_push_pull_output()
                    .connect_peripheral_to_output(self.rts_signal());
            }
        }
    }

    fn tx_signal(&self) -> OutputSignal;

    fn rx_signal(&self) -> InputSignal;

    fn cts_signal(&self) -> InputSignal;

    fn rts_signal(&self) -> OutputSignal;
}

impl Instance for UART0 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
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
}

impl Instance for UART1 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
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
}

#[cfg(any(feature = "esp32", feature = "esp32s3"))]
impl Instance for UART2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
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
}

#[cfg(feature = "ufmt")]
impl<T> ufmt_write::uWrite for Serial<T>
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

impl<T> core::fmt::Write for Serial<T>
where
    T: Instance,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

impl<T> embedded_hal::serial::Write<u8> for Serial<T>
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

impl<T> embedded_hal::serial::Read<u8> for Serial<T>
where
    T: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::serial::ErrorType for Serial<T> {
    type Error = Error;
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::serial::nb::Read for Serial<T>
where
    T: Instance,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::serial::nb::Write for Serial<T>
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
