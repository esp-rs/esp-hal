use core::slice::from_raw_parts;

use crate::{
    clock::Clocks,
    pac::twai::RegisterBlock,
    system::PeripheralClockControl,
    types::{InputSignal, OutputSignal},
    InputPin, OutputPin,
};

use embedded_hal::can::{self, ErrorKind, ExtendedId, Frame, StandardId};
use fugit::HertzU32;

use self::filter::{Filter, FilterType};

pub mod bitselector;
pub mod filter;

/// Very basic implementation of the Frame trait.
///
/// TODO: See if data and dlc can be simplified into a slice w/ lifetimes etc.
/// TODO: See if this can be improved.
///
#[derive(Debug)]
pub struct ESPTWAIFrame {
    id: can::Id,
    dlc: usize,
    data: [u8; 8],
    is_remote: bool,
}

impl embedded_hal::can::Frame for ESPTWAIFrame {
    fn new(id: impl Into<can::Id>, data: &[u8]) -> Option<Self> {
        // CAN2.0 frames cannot contain more than 8 bytes of data.
        if data.len() > 8 {
            return None;
        }

        // TODO: See struct docs for TODO list, ideally this copy would be eliminated somehow.
        let mut d: [u8; 8] = Default::default();
        let (left, _unused) = d.split_at_mut(data.len());
        left.clone_from_slice(data);

        Some(ESPTWAIFrame {
            id: id.into(),
            data: d,
            dlc: data.len(),
            is_remote: false,
        })
    }

    fn new_remote(id: impl Into<can::Id>, dlc: usize) -> Option<Self> {
        Some(ESPTWAIFrame {
            id: id.into(),
            data: Default::default(),
            dlc: dlc,
            is_remote: true,
        })
    }

    fn is_extended(&self) -> bool {
        match self.id {
            can::Id::Standard(_) => false,
            can::Id::Extended(_) => true,
        }
    }

    #[inline(always)]
    fn is_remote_frame(&self) -> bool {
        self.is_remote
    }

    #[inline(always)]
    fn id(&self) -> can::Id {
        self.id
    }

    #[inline(always)]
    fn dlc(&self) -> usize {
        self.dlc
    }

    #[inline(always)]
    fn data(&self) -> &[u8] {
        match self.is_remote_frame() {
            true => &[],
            false => &self.data[0..self.dlc],
        }
    }
}

/// The underlying timings for the TWAI peripheral.
pub struct TimingConfig {
    pub baud_rate_prescaler: u16,
    pub sync_jump_width: u8,
    pub tseg_1: u8,
    pub tseg_2: u8,
    pub triple_sample: bool,
}

/// A selection of pre-determined baudrates for the TWAI driver.
pub enum BaudRate {
    B125K,
    B250K,
    B500K,
    B1000K,
    Custom(TimingConfig),
}
impl BaudRate {
    /// Convert the BaudRate into the timings that the peripheral needs.
    // These timings are copied from the ESP IDF C driver.
    // #define TWAI_TIMING_CONFIG_25KBITS()    {.brp = 128, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_50KBITS()    {.brp = 80, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_100KBITS()   {.brp = 40, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_125KBITS()   {.brp = 32, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_250KBITS()   {.brp = 16, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_500KBITS()   {.brp = 8, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_800KBITS()   {.brp = 4, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_1MBITS()     {.brp = 4, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
    fn timing(self) -> TimingConfig {
        match self {
            Self::B125K => TimingConfig {
                baud_rate_prescaler: 32,
                sync_jump_width: 3,
                tseg_1: 15,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::B250K => TimingConfig {
                baud_rate_prescaler: 16,
                sync_jump_width: 3,
                tseg_1: 15,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::B500K => TimingConfig {
                baud_rate_prescaler: 8,
                sync_jump_width: 3,
                tseg_1: 15,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::B1000K => TimingConfig {
                baud_rate_prescaler: 4,
                sync_jump_width: 3,
                tseg_1: 15,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::Custom(timing_config) => timing_config,
        }
    }
}

/// An inactive TWAI peripheral in the "Reset"/configuration state.
pub struct TWAIConfiguration<T> {
    peripheral: T,
}

impl<T> TWAIConfiguration<T>
where
    T: Instance,
{
    pub fn new<TX: OutputPin, RX: InputPin>(
        peripheral: T,
        mut tx_pin: TX,
        mut rx_pin: RX,
        clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
        baud_rate: BaudRate,
    ) -> Self {
        // TODO: Probably should do a low level reset.

        // Enable the peripheral clock for the TWAI peripheral.
        clock_control.enable(crate::system::Peripheral::TWAI);

        // Set up the GPIO pins.
        tx_pin.connect_peripheral_to_output(OutputSignal::TWAI_TX);
        rx_pin.connect_input_to_peripheral(InputSignal::TWAI_RX);

        let mut cfg = TWAIConfiguration {
            peripheral: peripheral,
        };

        cfg.set_baud_rate(baud_rate, clocks);

        cfg
    }

    /// Set the bitrate of the bus.
    ///
    /// Note: The timings currently assume a APB_CLK of 80MHz.
    ///
    fn set_baud_rate(&mut self, baud_rate: BaudRate, clocks: &Clocks) {
        // TWAI is clocked from the APB_CLK according to Table 6-4 [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf)
        // Included timings are all for 80MHz so assert that we are running at 80MHz.
        assert!(clocks.apb_clock == HertzU32::MHz(80));

        // Unpack the baud rate timings and convert them to the values needed for the register.
        // Many of the registers have a minimum value of 1 which is represented by having zero
        // bits set, therefore many values need to have 1 subtracted from them before being
        // stored into the register.
        let timing = baud_rate.timing();

        let prescale = (timing.baud_rate_prescaler / 2) - 1;
        let sjw = timing.sync_jump_width - 1;
        let tseg_1 = timing.tseg_1 - 1;
        let tseg_2 = timing.tseg_2 - 1;
        let triple_sample = timing.triple_sample;

        // Set up the prescaler and sync jump width.
        self.peripheral
            .register_block()
            .bus_timing_0
            .modify(|_, w| {
                w.baud_presc()
                    .variant(prescale)
                    .sync_jump_width()
                    .variant(sjw)
            });

        // Set up the time segment 1, time segment 2, and triple sample.
        self.peripheral
            .register_block()
            .bus_timing_1
            .modify(|_, w| {
                w.time_seg1()
                    .variant(tseg_1)
                    .time_seg2()
                    .variant(tseg_2)
                    .time_samp()
                    .bit(triple_sample)
            });
    }

    /// Set up the acceptance filter on the device to accept the specified filters.
    ///
    /// [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.6)
    pub fn set_filter(&mut self, filter: impl Filter) {
        // Set or clear the rx filter mode bit depending on the filter type.
        let filter_mode_bit = filter.filter_type() == FilterType::Single;
        self.peripheral
            .register_block()
            .mode
            .modify(|_, w| w.rx_filter_mode().bit(filter_mode_bit));

        // Convert the filter into values for the registers and store them to the registers.
        let registers = filter.to_registers();
        // TODO: Use something better for copying, probably something similar to memcpy.
        self.peripheral
            .register_block()
            .data_0
            .write(|w| w.tx_byte_0().variant(registers[0]));
        self.peripheral
            .register_block()
            .data_1
            .write(|w| w.tx_byte_1().variant(registers[1]));
        self.peripheral
            .register_block()
            .data_2
            .write(|w| w.tx_byte_2().variant(registers[2]));
        self.peripheral
            .register_block()
            .data_3
            .write(|w| w.tx_byte_3().variant(registers[3]));
        self.peripheral
            .register_block()
            .data_4
            .write(|w| w.tx_byte_4().variant(registers[4]));
        self.peripheral
            .register_block()
            .data_5
            .write(|w| w.tx_byte_5().variant(registers[5]));
        self.peripheral
            .register_block()
            .data_6
            .write(|w| w.tx_byte_6().variant(registers[6]));
        self.peripheral
            .register_block()
            .data_7
            .write(|w| w.tx_byte_7().variant(registers[7]));
    }

    /// Set the Error warning threshold.
    ///
    /// In the case when any of an error counter value exceeds
    /// the threshold, or all the error counter values are below the threshold, an error warning
    /// interrupt will be triggered (given the enable signal is valid).
    pub fn set_error_warning_limit(&mut self, limit: u8) {
        self.peripheral
            .register_block()
            .err_warning_limit
            .write(|w| w.err_warning_limit().variant(limit));
    }

    /// Put the peripheral into Operation Mode, allowing the transmission and reception of
    /// packets using the new object.
    pub fn start(self) -> TWAI<T> {
        // Put the peripheral into operation mode by clearing the reset mode bit.
        self.peripheral
            .register_block()
            .mode
            .modify(|_, w| w.reset_mode().clear_bit());

        TWAI {
            peripheral: self.peripheral,
        }
    }
}

/// An active TWAI peripheral in Normal Mode.
///
/// According to the [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf) 29.4.1.2 Operation Mode, in "Normal Mode":
/// The TWAI controller can transmit and receive messages including error signals (such as error and overload Frames)
///
///
pub struct TWAI<T> {
    peripheral: T,
}

impl<T> TWAI<T>
where
    T: Instance,
{
    /// Stop the peripheral, putting it into reset mode and enabling reconfiguration.
    ///
    pub fn stop(self) -> TWAIConfiguration<T> {
        // Put the peripheral into reset/configuration mode by setting the reset mode bit.
        self.peripheral
            .register_block()
            .mode
            .modify(|_, w| w.reset_mode().set_bit());

        TWAIConfiguration {
            peripheral: self.peripheral,
        }
    }
    pub fn receive_error_count(&self) -> u8 {
        self.peripheral
            .register_block()
            .rx_err_cnt
            .read()
            .rx_err_cnt()
            .bits()
    }
    pub fn transmit_error_count(&self) -> u8 {
        self.peripheral
            .register_block()
            .tx_err_cnt
            .read()
            .tx_err_cnt()
            .bits()
    }

    /// Test if the transmit buffer is available for writing.
    pub fn transmit_buffer_is_empty(&self) -> bool {
        self.peripheral
            .register_block()
            .status
            .read()
            .tx_buf_st()
            .bit()
    }

    /// Get the number of messages that the peripheral has received.
    ///
    /// Note that this may not be the number of messages in the receive FIFO due to
    /// fifo overflow/overrun.
    pub fn num_messages(&self) -> u8 {
        self.peripheral
            .register_block()
            .rx_message_cnt
            .read()
            .rx_message_counter()
            .bits()
    }
    /// Clear the receive FIFO, discarding any valid, partial, or invalid packets.
    ///
    /// This is typically used to clear an overrun receive FIFO.
    pub fn clear_receive_fifo(&self) {
        while self.num_messages() > 0 {
            self.release_receive_fifo();
        }
    }

    /// Release the message in the buffer. This will decrement the received message
    /// counter and prepare the next message in the FIFO for reading.
    pub fn release_receive_fifo(&self) {
        self.peripheral
            .register_block()
            .cmd
            .write(|w| w.release_buf().set_bit());
    }
}

#[derive(Debug)]
pub enum ESPTWAIError {
    BusOff,
    EmbeddedHAL(embedded_hal::can::ErrorKind),
}
impl embedded_hal::can::Error for ESPTWAIError {
    fn kind(&self) -> can::ErrorKind {
        match self {
            Self::BusOff => can::ErrorKind::Other,
            Self::EmbeddedHAL(kind) => *kind,
        }
    }
}

impl<T> embedded_hal::can::Can for TWAI<T>
where
    T: Instance,
{
    type Frame = ESPTWAIFrame;
    type Error = ESPTWAIError;
    /// Transmit a frame.
    ///
    /// Because of how the TWAI registers are set up, we have to do some assembly of bytes. Note
    /// that these registers serve a filter configuration role when the device is in
    /// configuration mode so patching the svd files to improve this may be non-trivial.
    ///
    /// [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.4.2)
    ///
    /// NOTE: This may not work if using the self reception/self test functionality. See
    /// notes 1 and 2 in the Frame Identifier section of the reference manual.
    ///
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        let status = self.peripheral.register_block().status.read();

        // Check that the peripheral is not already transmitting a packet.
        if !status.tx_buf_st().bit_is_set() {
            return nb::Result::Err(nb::Error::WouldBlock);
        }
        // Check that the peripheral is not in a bus off state.
        if status.bus_off_st().bit_is_set() {
            return nb::Result::Err(nb::Error::Other(ESPTWAIError::BusOff));
        }

        // Assemble the frame information into the data_0 byte.
        let frame_format: u8 = if frame.is_extended() { 0x1 } else { 0x0 };
        let rtr_bit: u8 = if frame.is_remote_frame() { 0x1 } else { 0x0 };
        let dlc_bits: u8 = frame.dlc() as u8 & 0b1111;

        let data_0: u8 = frame_format << 7 | rtr_bit << 6 | dlc_bits;

        self.peripheral
            .register_block()
            .data_0
            .write(|w| w.tx_byte_0().variant(data_0));

        // Assemble the identifier information of the packet.
        match frame.id() {
            can::Id::Standard(id) => {
                let id = id.as_raw();

                self.peripheral
                    .register_block()
                    .data_1
                    .write(|w| w.tx_byte_1().variant((id >> 3) as u8));

                self.peripheral
                    .register_block()
                    .data_2
                    .write(|w| w.tx_byte_2().variant((id << 5) as u8));
            }
            can::Id::Extended(id) => {
                let id = id.as_raw();

                self.peripheral
                    .register_block()
                    .data_1
                    .write(|w| w.tx_byte_1().variant((id >> 21) as u8));
                self.peripheral
                    .register_block()
                    .data_2
                    .write(|w| w.tx_byte_2().variant((id >> 13) as u8));
                self.peripheral
                    .register_block()
                    .data_3
                    .write(|w| w.tx_byte_3().variant((id >> 5) as u8));
                self.peripheral
                    .register_block()
                    .data_4
                    .write(|w| w.tx_byte_4().variant((id << 3) as u8));
            }
        }

        // Store the data portion of the packet into the transmit buffer.
        if frame.is_data_frame() {
            match frame.id() {
                can::Id::Standard(_) => {
                    // TODO: Copy data to the appropriate registers in a better method. Verified in --release asm that this is bad.
                    // Byte 0 of the payload.
                    if frame.dlc() > 0 {
                        let data_byte_1 = &frame.data()[0];
                        self.peripheral
                            .register_block()
                            .data_3
                            .write(|w| w.tx_byte_3().variant(*data_byte_1));
                    }
                    // Byte 1 of the payload.
                    if frame.dlc() > 1 {
                        let data_byte_2 = &frame.data()[1];
                        self.peripheral
                            .register_block()
                            .data_4
                            .write(|w| w.tx_byte_4().variant(*data_byte_2));
                    }
                    // Byte 2 of the payload.
                    if frame.dlc() > 2 {
                        let data_byte_3 = &frame.data()[2];
                        self.peripheral
                            .register_block()
                            .data_5
                            .write(|w| w.tx_byte_5().variant(*data_byte_3));
                    }
                    // Byte 3 of the payload.
                    if frame.dlc() > 3 {
                        let data_byte_4 = &frame.data()[3];
                        self.peripheral
                            .register_block()
                            .data_6
                            .write(|w| w.tx_byte_6().variant(*data_byte_4));
                    }
                    // Byte 4 of the payload.
                    if frame.dlc() > 4 {
                        let data_byte_5 = &frame.data()[4];
                        self.peripheral
                            .register_block()
                            .data_7
                            .write(|w| w.tx_byte_7().variant(*data_byte_5));
                    }
                    // Byte 5 of the payload.
                    if frame.dlc() > 5 {
                        let data_byte_6 = &frame.data()[5];
                        self.peripheral
                            .register_block()
                            .data_8
                            .write(|w| w.tx_byte_8().variant(*data_byte_6));
                    }
                    // Byte 6 of the payload.
                    if frame.dlc() > 6 {
                        let data_byte_7 = &frame.data()[6];
                        self.peripheral
                            .register_block()
                            .data_9
                            .write(|w| w.tx_byte_9().variant(*data_byte_7));
                    }
                    // Byte 7 of the payload.
                    if frame.dlc() > 7 {
                        let data_byte_8 = &frame.data()[7];
                        self.peripheral
                            .register_block()
                            .data_10
                            .write(|w| w.tx_byte_10().variant(*data_byte_8));
                    }
                }
                can::Id::Extended(_) => {
                    // TODO: Copy data to the appropriate registers in a better method. Verified in --release asm that this is bad.
                    // Byte 0 of the payload.
                    if frame.dlc() > 0 {
                        let data_byte_1 = &frame.data()[0];
                        self.peripheral
                            .register_block()
                            .data_5
                            .write(|w| w.tx_byte_5().variant(*data_byte_1));
                    }
                    // Byte 1 of the payload.
                    if frame.dlc() > 1 {
                        let data_byte_2 = &frame.data()[1];
                        self.peripheral
                            .register_block()
                            .data_6
                            .write(|w| w.tx_byte_6().variant(*data_byte_2));
                    }
                    // Byte 2 of the payload.
                    if frame.dlc() > 2 {
                        let data_byte_3 = &frame.data()[2];
                        self.peripheral
                            .register_block()
                            .data_7
                            .write(|w| w.tx_byte_7().variant(*data_byte_3));
                    }
                    // Byte 3 of the payload.
                    if frame.dlc() > 3 {
                        let data_byte_4 = &frame.data()[3];
                        self.peripheral
                            .register_block()
                            .data_8
                            .write(|w| w.tx_byte_8().variant(*data_byte_4));
                    }
                    // Byte 4 of the payload.
                    if frame.dlc() > 4 {
                        let data_byte_5 = &frame.data()[4];
                        self.peripheral
                            .register_block()
                            .data_9
                            .write(|w| w.tx_byte_9().variant(*data_byte_5));
                    }
                    // Byte 5 of the payload.
                    if frame.dlc() > 5 {
                        let data_byte_6 = &frame.data()[5];
                        self.peripheral
                            .register_block()
                            .data_10
                            .write(|w| w.tx_byte_10().variant(*data_byte_6));
                    }
                    // Byte 6 of the payload.
                    if frame.dlc() > 6 {
                        let data_byte_7 = &frame.data()[6];
                        self.peripheral
                            .register_block()
                            .data_11
                            .write(|w| w.tx_byte_11().variant(*data_byte_7));
                    }
                    // Byte 7 of the payload.
                    if frame.dlc() > 7 {
                        let data_byte_8 = &frame.data()[7];
                        self.peripheral
                            .register_block()
                            .data_12
                            .write(|w| w.tx_byte_12().variant(*data_byte_8));
                    }
                }
            }
        } else {
            // Is RTR frame, so no data is included.
        }

        // Set the transmit request command, this will lock the transmit buffer until the
        // transmission is complete or aborted.
        self.peripheral
            .register_block()
            .cmd
            .write(|w| w.tx_req().set_bit());

        nb::Result::Ok(None)
    }
    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        let status = self.peripheral.register_block().status.read();

        // Check that we actually have packets to receive.
        if !status.rx_buf_st().bit_is_set() {
            return nb::Result::Err(nb::Error::WouldBlock);
        }

        // Check if the packet in the receive buffer is valid or overrun.
        if status.miss_st().bit_is_set() {
            return nb::Result::Err(nb::Error::Other(ESPTWAIError::EmbeddedHAL(
                ErrorKind::Overrun,
            )));
        }

        // Read the frame information and extract the frame id format and dlc.
        let data_0 = self
            .peripheral
            .register_block()
            .data_0
            .read()
            .tx_byte_0()
            .bits();

        let is_standard_format = data_0 & 0b1 << 7 == 0;
        let dlc = (data_0 & 0b1111) as usize;

        // Read the payload from the packet and construct a frame.
        let frame = if is_standard_format {
            // Frame uses standard 11 bit id.
            let data_1 = self
                .peripheral
                .register_block()
                .data_1
                .read()
                .tx_byte_1()
                .bits();

            let data_2 = self
                .peripheral
                .register_block()
                .data_1
                .read()
                .tx_byte_1()
                .bits();

            let id = StandardId::new((data_1 as u16) << 3 | (data_2 as u16) >> 5).unwrap();

            // Copy the packet payload from the peripheral into memory.
            // TODO: find a better way of doing this, basically a memcpy, but the
            // destination and source have different strides.
            let raw_payload =
                unsafe { from_raw_parts(self.peripheral.register_block().data_3.as_ptr(), dlc) };

            let mut payload: [u8; 8] = [0; 8];
            for i in 0..dlc {
                payload[i] = raw_payload[i] as u8;
            }

            ESPTWAIFrame::new(id, &payload[..dlc]).unwrap()
        } else {
            // Frame uses extended 29 bit id.
            let data_1 = self
                .peripheral
                .register_block()
                .data_1
                .read()
                .tx_byte_1()
                .bits();

            let data_2 = self
                .peripheral
                .register_block()
                .data_1
                .read()
                .tx_byte_1()
                .bits();

            let data_3 = self
                .peripheral
                .register_block()
                .data_3
                .read()
                .tx_byte_3()
                .bits();

            let data_4 = self
                .peripheral
                .register_block()
                .data_4
                .read()
                .tx_byte_4()
                .bits();

            let id = ExtendedId::new(
                (data_1 as u32) << 21
                    | (data_2 as u32) << 13
                    | (data_3 as u32) << 5
                    | (data_4 as u32) >> 3,
            )
            .unwrap();

            // Copy the packet payload from the peripheral into memory.
            // TODO: find a better way of doing this, basically a memcpy, but the
            // destination and source have different strides.
            let raw_payload =
                unsafe { from_raw_parts(self.peripheral.register_block().data_5.as_ptr(), dlc) };
            let mut payload: [u8; 8] = [0; 8];
            for i in 0..dlc {
                payload[i] = raw_payload[i] as u8;
            }

            ESPTWAIFrame::new(id, &payload[..dlc]).unwrap()
        };

        // Release the packet we read from the FIFO, allowing the peripheral to prepare
        // the next packet.
        self.release_receive_fifo();

        nb::Result::Ok(frame)
    }
}

pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;
}

impl Instance for crate::pac::TWAI {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}
