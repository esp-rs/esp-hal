//! # RISC-­V Trace Encoder (TRACE)
//!
//! ## Overview
//! The high-performance CPU supports instruction trace interface through the
//! trace encoder. The trace encoder connects to HP CPU’s instruction trace
//! interface, compresses the information into smaller packets, and then stores
//! the packets in internal SRAM.
//!
//! In complex systems, understanding program execution flow is not
//! straightforward. This may be due to a number of factors, for example,
//! interactions with other cores, peripherals, real-time events, poor
//! implementations, or some combination of all of the above.
//!
//! It is hard to use a debugger to monitor the program execution flow of a
//! running system in real time, as this is intrusive and might affect the
//! running state. But providing visibility of program execution is important.
//!
//! That is where instruction trace comes in, which provides trace of the
//! program execution.
//!
//! ## Examples
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::trace::Trace;
//! let mut trace = Trace::new(peripherals.TRACE0);
//! let mut buffer = [0_u8; 1024];
//! trace.start_trace(&mut buffer);
//! // traced code
//!
//! // end traced code
//! let res = trace.stop_trace().unwrap();
//! // transfer the trace result to the host and decode it there
//! # }
//! ```

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::trace::RegisterBlock,
    system::PeripheralClockControl,
};

/// Errors returned from [Trace::stop_trace]
#[derive(Debug, Clone, Copy)]
pub enum Error {
    /// Attempted to stop a trace which had not been started yet
    NotStarted,
}

/// Returned by [Trace::stop_trace]
#[derive(Debug, Clone, Copy)]
pub struct TraceResult {
    /// Start index of the valid data
    pub valid_start_index: usize,
    /// Length of the valid data
    pub valid_length: usize,
}

/// TRACE Encoder Instance
pub struct Trace<'d, T> {
    peripheral: PeripheralRef<'d, T>,
    buffer: Option<&'d mut [u8]>,
}

impl<'d, T> Trace<'d, T>
where
    T: Instance,
{
    /// Construct a new instance
    pub fn new(peripheral: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(peripheral);

        PeripheralClockControl::reset(crate::system::Peripheral::Trace0);
        PeripheralClockControl::enable(crate::system::Peripheral::Trace0);

        Self {
            peripheral,
            buffer: None,
        }
    }

    /// Start tracing, writing data into the `buffer`
    pub fn start_trace(&mut self, buffer: &'d mut [u8]) {
        let reg_block = self.peripheral.register_block();

        reg_block
            .mem_start_addr()
            .modify(|_, w| unsafe { w.mem_start_addr().bits(buffer.as_ptr() as *const _ as u32) });
        reg_block.mem_end_addr().modify(|_, w| unsafe {
            w.mem_end_addr()
                .bits((buffer.as_ptr() as *const _ as u32) + (buffer.len() as u32))
        });
        reg_block
            .mem_addr_update()
            .write(|w| w.mem_current_addr_update().set_bit());

        // won't set bit in int-raw without enabling
        reg_block
            .intr_ena()
            .modify(|_, w| w.mem_full_intr_ena().set_bit());

        // for now always use looping mode
        reg_block
            .trigger()
            .write(|w| w.mem_loop().set_bit().restart_ena().set_bit());

        reg_block.intr_clr().write(|w| {
            w.fifo_overflow_intr_clr()
                .set_bit()
                .mem_full_intr_clr()
                .set_bit()
        });

        self.buffer.replace(buffer);
        reg_block.trigger().write(|w| w.on().set_bit());
    }

    /// Stop tracing
    ///
    /// Be aware that valid data might not start at index 0 and you need to
    /// account for wrapping when reading the data.
    pub fn stop_trace(&mut self) -> Result<TraceResult, Error> {
        let reg_block = self.peripheral.register_block();

        reg_block
            .trigger()
            .write(|w| w.off().set_bit().restart_ena().clear_bit());

        if self.buffer.is_none() {
            return Err(Error::NotStarted);
        }

        let buffer = self.buffer.take().unwrap();

        while !reg_block.fifo_status().read().fifo_empty().bit_is_set() {}

        let overflow = reg_block.intr_raw().read().mem_full_intr_raw().bit();
        let idx = if overflow {
            reg_block
                .mem_current_addr()
                .read()
                .mem_current_addr()
                .bits()
                - &buffer as *const _ as u32
        } else {
            0
        };

        let len = if overflow {
            buffer.len()
        } else {
            reg_block
                .mem_current_addr()
                .read()
                .mem_current_addr()
                .bits() as usize
                - buffer.as_ptr() as *const _ as usize
                - 14 // there will be 14 zero bytes at the start
        };

        let mut valid = false;
        let mut fourteen_zeroes = false;
        let mut zeroes = 0;
        let start_index = if !valid {
            let mut i = 0;
            loop {
                let b = unsafe {
                    buffer
                        .as_ptr()
                        .add((i + idx as usize) % buffer.len())
                        .read_volatile()
                };

                if !valid {
                    if b == 0 {
                        zeroes += 1;
                    } else {
                        zeroes = 0;
                    }

                    if zeroes >= 14 {
                        fourteen_zeroes = true;
                    }

                    if fourteen_zeroes && b != 0 {
                        valid = true;
                    }
                }

                if valid {
                    break i;
                }

                i += 1;
            }
        } else {
            0
        };

        Ok(TraceResult {
            valid_start_index: start_index,
            valid_length: len,
        })
    }
}

/// Trace peripheral instance
pub trait Instance: crate::private::Sealed {
    /// Get a reference to the peripheral's underlying register block
    fn register_block(&self) -> &RegisterBlock;
}

impl Instance for crate::peripherals::TRACE0 {
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}
