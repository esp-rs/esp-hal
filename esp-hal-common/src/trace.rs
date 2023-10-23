//! # RISC­V Trace Encoder (TRACE)
//!
//! ## Overview
//!
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
//! ## Example
//! ```no_run
//! let mut trace = Trace::new(peripherals.TRACE);
//! let buffer = unsafe { &mut BUFFER[..] };
//! trace.start_trace(buffer);
//! // traced code
//! println!("Hello");
//! // end traced code
//! let res = trace.stop_trace().unwrap();
//! // transfer the trace result to the host and decode it there
//! ```

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralClockControl,
};

/// Errors returned from [Trace::stop_trace]
#[derive(Debug, Clone, Copy)]
pub enum Error {
    NotStarted,
}

/// Returned by [Trace::stop_trace]
#[derive(Debug, Clone, Copy)]
pub struct TraceResult {
    pub valid_start_index: usize,
    pub valid_length: usize,
}

/// TRACE Encoder Instance
pub struct Trace<'d> {
    peripheral: PeripheralRef<'d, crate::peripherals::TRACE>,
    buffer: Option<&'d mut [u8]>,
}

impl<'d> Trace<'d> {
    /// Construct a new instance
    pub fn new(peripheral: impl Peripheral<P = crate::peripherals::TRACE> + 'd) -> Self {
        crate::into_ref!(peripheral);

        PeripheralClockControl::enable(crate::system::Peripheral::Trace);

        Self {
            peripheral,
            buffer: None,
        }
    }

    /// Start tracing, writing data into the `buffer`
    pub fn start_trace(&mut self, buffer: &'d mut [u8]) {
        self.peripheral.mem_start_addr.modify(|_, w| {
            w.mem_staet_addr()
                .variant(buffer.as_ptr() as *const _ as u32)
        });
        self.peripheral.mem_end_addr.modify(|_, w| {
            w.mem_end_addr()
                .variant((buffer.as_ptr() as *const _ as u32) + (buffer.len() as u32))
        });
        self.peripheral
            .mem_addr_update
            .write(|w| w.mem_current_addr_update().set_bit());

        // won't set bit in int-raw without enabling
        self.peripheral
            .intr_ena
            .modify(|_, w| w.mem_full_intr_ena().set_bit());

        // for now always use looping mode
        self.peripheral
            .trigger
            .write(|w| w.mem_loop().set_bit().restart_ena().set_bit());

        self.peripheral.intr_clr.write(|w| {
            w.fifo_overflow_intr_clr()
                .set_bit()
                .mem_full_intr_clr()
                .set_bit()
        });

        self.buffer.replace(buffer);
        self.peripheral.trigger.write(|w| w.on().set_bit());
    }

    /// Stop tracing
    ///
    /// Be aware that valid data might not start at index 0 and you need to
    /// account for wrapping when reading the data.
    pub fn stop_trace(&mut self) -> Result<TraceResult, Error> {
        self.peripheral
            .trigger
            .write(|w| w.off().set_bit().restart_ena().clear_bit());

        if self.buffer.is_none() {
            return Err(Error::NotStarted);
        }

        let buffer = self.buffer.take().unwrap();

        loop {
            if self.peripheral.fifo_status.read().fifo_empty().bit_is_set() {
                break;
            }
        }

        let overflow = self.peripheral.intr_raw.read().mem_full_intr_raw().bit();
        let idx = if overflow {
            self.peripheral
                .mem_current_addr
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
            self.peripheral
                .mem_current_addr
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
                    (buffer.as_ptr() as *const _ as *const u8)
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
