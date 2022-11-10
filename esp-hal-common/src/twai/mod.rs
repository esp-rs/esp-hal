use core::slice::{from_raw_parts, from_raw_parts_mut};

use crate::{
    clock::Clocks,
    pac::twai::RegisterBlock,
    system::PeripheralClockControl,
    types::{InputSignal, OutputSignal},
    InputPin, OutputPin,
};

#[cfg(feature = "eh1")]
use embedded_can::{nb::Can, Error, ErrorKind, ExtendedId, Frame, Id, StandardId};
#[cfg(not(feature = "eh1"))]
use embedded_hal::can::{Can, Error, ErrorKind, ExtendedId, Frame, Id, StandardId};

use fugit::HertzU32;

use self::filter::{Filter, FilterType};

pub mod bitselector;
pub mod filter;

/// Structure backing the embedded_hal::can::Frame trait.
#[derive(Debug)]
pub struct EspTwaiFrame {
    id: Id,
    dlc: usize,
    data: [u8; 8],
    is_remote: bool,
}

impl EspTwaiFrame {
    /// Make a new frame from an id and a slice of the TWAI_DATA_x_REG registers.
    fn new_from_data_registers(id: impl Into<Id>, data: &[u32]) -> Self {
        let mut d: [u8; 8] = [0; 8];

        // Copy the data from the memory mapped peripheral into actual memory.
        for (src, dest) in data.iter().zip(d.iter_mut()) {
            *dest = *src as u8;
        }

        Self {
            id: id.into(),
            data: d,
            dlc: data.len(),
            is_remote: false,
        }
    }
}

impl Frame for EspTwaiFrame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        // CAN2.0 frames cannot contain more than 8 bytes of data.
        if data.len() > 8 {
            return None;
        }

        let mut d: [u8; 8] = [0; 8];
        let (left, _unused) = d.split_at_mut(data.len());
        left.clone_from_slice(data);

        Some(EspTwaiFrame {
            id: id.into(),
            data: d,
            dlc: data.len(),
            is_remote: false,
        })
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        // CAN2.0 frames cannot have more than 8 bytes.
        if dlc > 8 {
            return None;
        }
        Some(EspTwaiFrame {
            id: id.into(),
            data: [0; 8],
            dlc,
            is_remote: true,
        })
    }

    fn is_extended(&self) -> bool {
        match self.id {
            Id::Standard(_) => false,
            Id::Extended(_) => true,
        }
    }

    fn is_remote_frame(&self) -> bool {
        self.is_remote
    }

    fn id(&self) -> Id {
        self.id
    }

    fn dlc(&self) -> usize {
        self.dlc
    }

    fn data(&self) -> &[u8] {
        // Remote frames do not contain data, yet have a value for the dlc so return
        // an empty slice for remote frames.
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
pub struct TwaiConfiguration<T> {
    peripheral: T,
}

impl<T> TwaiConfiguration<T>
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
        // Enable the peripheral clock for the TWAI peripheral.
        clock_control.enable(crate::system::Peripheral::Twai);

        // Set up the GPIO pins.
        tx_pin.connect_peripheral_to_output(OutputSignal::TWAI_TX);
        rx_pin.connect_input_to_peripheral(InputSignal::TWAI_RX);

        let mut cfg = TwaiConfiguration { peripheral };

        cfg.set_baud_rate(baud_rate, clocks);

        cfg
    }

    /// Set the bitrate of the bus.
    ///
    /// Note: The timings currently assume a APB_CLK of 80MHz.
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

        // Copy the filter to the peripheral.
        unsafe {
            copy_to_data_register(self.peripheral.register_block().data_0.as_ptr(), &registers);
        }
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
    pub fn start(self) -> Twai<T> {
        // Put the peripheral into operation mode by clearing the reset mode bit.
        self.peripheral
            .register_block()
            .mode
            .modify(|_, w| w.reset_mode().clear_bit());

        Twai {
            peripheral: self.peripheral,
        }
    }
}

/// An active TWAI peripheral in Normal Mode.
///
/// According to the [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf) 29.4.1.2 Operation Mode, in "Normal Mode":
/// The TWAI controller can transmit and receive messages including error signals (such as error and overload Frames)
pub struct Twai<T> {
    peripheral: T,
}

impl<T> Twai<T>
where
    T: Instance,
{
    /// Stop the peripheral, putting it into reset mode and enabling reconfiguration.
    pub fn stop(self) -> TwaiConfiguration<T> {
        // Put the peripheral into reset/configuration mode by setting the reset mode bit.
        self.peripheral
            .register_block()
            .mode
            .modify(|_, w| w.reset_mode().set_bit());

        TwaiConfiguration {
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

    /// Check if the controller is in a bus off state.
    pub fn is_bus_off(&self) -> bool {
        self.peripheral
            .register_block()
            .status
            .read()
            .bus_off_st()
            .bit_is_set()
    }

    /// Get the number of messages that the peripheral has available in the receive FIFO.
    ///
    /// Note that this may not be the number of messages in the receive FIFO due to
    /// fifo overflow/overrun.
    pub fn num_available_messages(&self) -> u8 {
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
    ///
    /// TODO: Not sure if this needs to be guarded against Bus Off or other error states.
    pub fn clear_receive_fifo(&self) {
        while self.num_available_messages() > 0 {
            self.release_receive_fifo();
        }
    }

    /// Release the message in the buffer. This will decrement the received message
    /// counter and prepare the next message in the FIFO for reading.
    fn release_receive_fifo(&self) {
        self.peripheral
            .register_block()
            .cmd
            .write(|w| w.release_buf().set_bit());
    }
}

#[derive(Debug)]
pub enum EspTwaiError {
    BusOff,
    EmbeddedHAL(ErrorKind),
}
impl Error for EspTwaiError {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::BusOff => ErrorKind::Other,
            Self::EmbeddedHAL(kind) => *kind,
        }
    }
}

/// Copy data from multiple TWAI_DATA_x_REG registers, packing the source into the destination.
///
/// # Safety
/// This function is marked unsafe because it reads arbitrarily from memory-mapped registers.
/// Specifically, this function is used with the TWAI_DATA_x_REG registers which has different
/// results based on the mode of the peripheral.
unsafe fn _copy_from_data_register(dest: &mut [u8], src: *const u32) {
    let src = from_raw_parts(src, dest.len());

    for (dest, src) in dest.iter_mut().zip(src.iter()) {
        *dest = *src as u8;
    }
}

/// Copy data to multiple TWAI_DATA_x_REG registers, unpacking the source into the destination.
///
/// # Safety
/// This function is marked unsafe because it writes arbitrarily to memory-mapped registers.
/// Specifically, this function is used with the TWAI_DATA_x_REG registers which has different
/// results based on the mode of the peripheral.
unsafe fn copy_to_data_register(dest: *mut u32, src: &[u8]) {
    let dest = from_raw_parts_mut(dest, src.len());

    for (dest, src) in dest.iter_mut().zip(src.iter()) {
        *dest = *src as u32;
    }
}

impl<T> Can for Twai<T>
where
    T: Instance,
{
    type Frame = EspTwaiFrame;
    type Error = EspTwaiError;
    /// Transmit a frame.
    ///
    /// Because of how the TWAI registers are set up, we have to do some assembly of bytes. Note
    /// that these registers serve a filter configuration role when the device is in
    /// configuration mode so patching the svd files to improve this may be non-trivial.
    ///
    /// [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.4.2)
    ///
    /// NOTE: TODO: This may not work if using the self reception/self test functionality. See
    /// notes 1 and 2 in the Frame Identifier section of the reference manual.
    ///
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        let status = self.peripheral.register_block().status.read();

        // Check that the peripheral is not in a bus off state.
        if status.bus_off_st().bit_is_set() {
            return nb::Result::Err(nb::Error::Other(EspTwaiError::BusOff));
        }
        // Check that the peripheral is not already transmitting a packet.
        if !status.tx_buf_st().bit_is_set() {
            return nb::Result::Err(nb::Error::WouldBlock);
        }

        // Assemble the frame information into the data_0 byte.
        let frame_format: u8 = frame.is_extended() as u8;
        let rtr_bit: u8 = frame.is_remote_frame() as u8;
        let dlc_bits: u8 = frame.dlc() as u8 & 0b1111;

        let data_0: u8 = frame_format << 7 | rtr_bit << 6 | dlc_bits;

        self.peripheral
            .register_block()
            .data_0
            .write(|w| w.tx_byte_0().variant(data_0));

        // Assemble the identifier information of the packet.
        match frame.id() {
            Id::Standard(id) => {
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
            Id::Extended(id) => {
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
                Id::Standard(_) => unsafe {
                    copy_to_data_register(
                        self.peripheral.register_block().data_3.as_ptr(),
                        frame.data(),
                    )
                },
                Id::Extended(_) => unsafe {
                    copy_to_data_register(
                        self.peripheral.register_block().data_5.as_ptr(),
                        frame.data(),
                    )
                },
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

        // Check that the peripheral is not in a bus off state.
        if status.bus_off_st().bit_is_set() {
            return nb::Result::Err(nb::Error::Other(EspTwaiError::BusOff));
        }

        // Check that we actually have packets to receive.
        if !status.rx_buf_st().bit_is_set() {
            return nb::Result::Err(nb::Error::WouldBlock);
        }

        // Check if the packet in the receive buffer is valid or overrun.
        if status.miss_st().bit_is_set() {
            return nb::Result::Err(nb::Error::Other(EspTwaiError::EmbeddedHAL(
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
        let is_data_frame = data_0 & 0b1 << 6 == 0;
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
                .data_2
                .read()
                .tx_byte_2()
                .bits();

            let raw_id: u16 = ((data_1 as u16) << 3) | ((data_2 as u16) >> 5);

            let id = StandardId::new(raw_id).unwrap();

            if is_data_frame {
                // Create a new frame from the contents of the appropriate TWAI_DATA_x_REG registers.
                let register_data = unsafe {
                    from_raw_parts(self.peripheral.register_block().data_3.as_ptr(), dlc)
                };
                EspTwaiFrame::new_from_data_registers(id, register_data)
            } else {
                EspTwaiFrame::new_remote(id, dlc).unwrap()
            }
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
                .data_2
                .read()
                .tx_byte_2()
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

            let raw_id: u32 = (data_1 as u32) << 21
                | (data_2 as u32) << 13
                | (data_3 as u32) << 5
                | (data_4 as u32) >> 3;

            let id = ExtendedId::new(raw_id).unwrap();

            if is_data_frame {
                let register_data = unsafe {
                    from_raw_parts(self.peripheral.register_block().data_5.as_ptr(), dlc)
                };
                EspTwaiFrame::new_from_data_registers(id, register_data)
            } else {
                EspTwaiFrame::new_remote(id, dlc).unwrap()
            }
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
