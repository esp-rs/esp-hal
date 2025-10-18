use core::ops::ControlFlow;

#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

use super::{DynChannelAccess, Error, PulseCode, Tx};

/// Access to a free slot in an RMT channel hardware buffer.
///
/// An `RmtSlot` indicates a guarantee that there's a free slot in memory
/// that can immediately be written to. If this slot isn't used, the
/// next call to `RmtWriter.next` will use the same slot again (i.e.
/// there will be no garbage data due to skipping unused slots).
///
/// See [`Encoder::encode`] and [`RmtWriter::next`] for more details.
#[derive(Debug)]
pub struct RmtSlot<'a> {
    writer: &'a mut RmtWriter,
}

impl<'a> RmtSlot<'a> {
    /// Append a `PulseCode` to the RMT hardware buffer.
    #[inline(always)]
    pub fn write(self, code: PulseCode) {
        let writer = self.writer;

        debug_assert!(writer.ptr < writer.end);

        unsafe { writer.ptr.write_volatile(code) }

        writer.ptr = unsafe { writer.ptr.add(1) };
        writer.last_code = code;
    }
}

/// Provides methods to fill (part of) the RMT channel hardware buffer.
///
/// An `RmtWriter` is provided to [`Encoder::encode`], see there for more details.
pub struct RmtWriter {
    // Most recently written PulseCode
    last_code: PulseCode,
    // Current write pointer
    ptr: *mut PulseCode,
    // Start of the slice to be written
    start: *mut PulseCode,
    // End of the slice to be written
    end: *mut PulseCode,
}

// Given a pointer to RMT RAM, return the channel it belongs to
fn get_ch(ptr: *mut PulseCode) -> i32 {
    let diff = unsafe { ptr.offset_from(property!("rmt.ram_start") as *mut PulseCode) };
    (diff / property!("rmt.channel_ram_size")) as i32
}

// Given a pointer to RMT RAM, return the offset relative to start of the channel's RAM
fn get_offset(ptr: *mut PulseCode) -> i32 {
    let diff = unsafe { ptr.offset_from(property!("rmt.ram_start") as *mut PulseCode) };
    (diff % property!("rmt.channel_ram_size")) as i32
}

impl core::fmt::Debug for RmtWriter {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "RmtWriter {{ last_code: {:?}, ptr: CH{} + {:#x}, start: CH{} + {:#x}, end: CH{} + {:#x} }}",
            self.last_code,
            get_ch(self.ptr),
            get_offset(self.ptr),
            get_ch(self.start),
            get_offset(self.start),
            get_ch(self.end),
            get_offset(self.end),
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for RmtWriter {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "RmtWriter {{ last_code: {:?}, ptr: CH{} + {:#x}, start: CH{} + {:#x}, end: CH{} + {:#x} }}",
            self.last_code,
            get_ch(self.ptr),
            get_offset(self.ptr),
            get_ch(self.start),
            get_offset(self.start),
            get_ch(self.end),
            get_offset(self.end),
        )
    }
}

impl RmtWriter {
    /// Try to obtain a slot in the channel's hardware buffer.
    ///
    /// Returns `None` if the buffer is full; in this case [`Encoder::encode`] must return.
    #[inline(always)]
    pub fn next<'s>(&'s mut self) -> Option<RmtSlot<'s>> {
        if self.ptr >= self.end {
            return None;
        }

        Some(RmtSlot { writer: self })
    }

    /// Write several pulse codes to the channel's hardware buffer.
    ///
    /// `f` will be called up to `count` times to obtain pulse codes. Returns the actual number of
    /// codes written. If less than `count`, the buffer is full and [`Encoder::encode`] must
    /// return.
    ///
    /// This explicitly supports `count = 0`, thus callers can avoid to check for that condition.
    ///
    /// This can lead to more efficient code compared to repeated use of [`RmtWriter::next`] when
    /// the caller can provide an infallible function that yields `count` pulse codes.
    #[inline(always)]
    pub fn write_many<F>(&mut self, count: usize, mut f: F) -> usize
    where
        F: FnMut() -> PulseCode,
    {
        let count = count.min(unsafe { self.end.offset_from(self.ptr) } as usize);

        let mut last_code = self.last_code;
        let mut ptr = self.ptr;
        let end = unsafe { self.ptr.add(count) };

        while ptr < end {
            last_code = f();
            unsafe { ptr.write_volatile(last_code) };
            ptr = unsafe { ptr.add(1) };
        }

        debug_assert!(self.ptr <= self.end);

        self.ptr = ptr;
        self.last_code = last_code;

        count
    }
}

/// A trait to convert arbitrary data to RMT [`PulseCode`]s.
///
/// Implementations need to provide the single `encode` method, which transmit methods will
/// (potentially repeatedly) call to obtain pulse codes.
pub trait Encoder {
    /// (Re)Fill an RMT channel hardware buffer.
    ///
    /// Implementations can provide arbitrary conversions of data types, which will be performed
    /// just before writing to the hardware, without intermediate buffering of `PulseCode`s.
    /// This should mainly be used for low-cost operations (e.g. simple state machines and shift
    /// out bits), since it needs to be fast enough to refill hardware buffers before transmission
    /// exhausts them. If this cannot be guaranteed reliably, consider performing conversion to
    /// `PulseCode`s upfront, before calling the channel's transmit methods.
    ///
    /// This function must write pulse codes via the `writer` until the latter indicates that the
    /// buffer is full. Otherwise, the `writer` will consider the `Encoder` to be exhausted and not
    /// call it again to check for more data. Upon exhaustion of the `Encoder` the last pulse code
    /// written should contain an end marker. If the `Encoder` fails to provide one, the driver
    /// will still ensure that transmission is never started or eventually stops, and return
    /// `Error::EndMarkerMissing` from the transmit method. However, there are no further
    /// guarantees: In particular, the current driver code might lead to truncated transmission in
    /// this error case.
    ///
    /// In addition to writing less codes than the `writer` would allow for, implementations can
    /// indicate that they have completed by returning `ControlFlow::Break`. This leads to a slight
    /// optimization in case that the encoding ended just at the end of the hardware buffer, and
    /// would otherwise only be detected when the subsequent call to `encode` would write zero
    /// symbols.
    ///
    /// While this will presently not be called again after being exhausted, implementations must
    /// not rely on this for safety or correctness. Rather, they should return without writing any
    /// pulse codes in that case.
    ///
    /// For best performance, this should be inlined. It is likely that the compiler does so
    /// automatically (since the driver has only a single, non-generic function that calls this
    /// method). To be extra sure, consider marking your implementation of this method as
    /// `#[inline(always)]`. This also applies when you're using the `PLACE_RMT_DRIVER_IN_RAM`
    /// configuration: `encode` will be inlined in a driver-internal function, which in turn will
    /// be placed in RAM. Annotating this method with `#[ram]` would prevent inlining and likely
    /// impair performance.
    fn encode(&mut self, writer: &mut RmtWriter) -> ControlFlow<()>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) enum WriterState {
    // There's still data left to write
    Active,

    // An error occurred, and either
    // - no data was written (and transmission was never started), or
    // - some data was written and the last code written is an end marker, such that transmission
    //   will eventually stop
    Error(Error),

    // Completed without errors
    Done,
}

impl WriterState {
    pub(super) fn is_ok(self) -> bool {
        !matches!(self, Self::Error(_))
    }

    pub(super) fn to_result(self) -> Result<(), Error> {
        match self {
            // tx ended but writer wasn't done yet
            Self::Active => Err(Error::TransmissionError),
            Self::Error(e) => Err(e),
            Self::Done => Ok(()),
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) struct WriterContext {
    // The position in channel RAM to continue writing at; must be either
    // 0 or half the available RAM size if there's further data.
    // The position may be invalid if there's no data left.
    offset: u16,

    written: usize,

    last_code: PulseCode,

    state: WriterState,
}

impl WriterContext {
    #[inline]
    pub(super) fn new() -> Self {
        Self {
            offset: 0,
            written: 0,
            last_code: PulseCode::default(),
            state: WriterState::Active,
        }
    }

    #[inline]
    pub(super) fn state(&self) -> WriterState {
        self.state
    }
}

// Do not add an EncoderExt: Encoder bound here: That would add the Encoder::encode method to the
// vtable, preventing optimizing it away (it should always be inlined in EncoderExt::write).
pub(super) trait EncoderExt {
    fn write(&mut self, writer: &mut WriterContext, raw: DynChannelAccess<Tx>, initial: bool);
}

impl<E: Encoder + ?Sized> EncoderExt for E {
    // Copy from `data` to the hardware buffer, advancing the `data` slice accordingly.
    //
    // If `initial` is set, fill the entire buffer. Otherwise, append half the buffer's length from
    // `data`.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn write(&mut self, context: &mut WriterContext, raw: DynChannelAccess<Tx>, initial: bool) {
        if context.state != WriterState::Active {
            return;
        }

        debug_assert!(!initial || context.offset == 0);

        let ram_start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();

        let max_count = if initial { memsize } else { memsize / 2 };

        let start = unsafe { ram_start.add(context.offset as usize) };
        let mut writer = RmtWriter {
            last_code: context.last_code,
            ptr: start,
            start,
            end: unsafe { start.add(max_count) },
        };

        let done = self.encode(&mut writer);
        context.last_code = writer.last_code;
        context.written += unsafe { writer.ptr.offset_from(writer.start) } as usize;

        debug_assert!(writer.ptr.addr() >= writer.start.addr());
        debug_assert!(writer.ptr.addr() <= writer.end.addr());

        // If the input data was not exhausted, update offset as
        //
        // | initial | offset      | max_count   | new offset  |
        // | ------- + ----------- + ----------- + ----------- |
        // | true    | 0           | memsize     | 0           |
        // | false   | 0           | memsize / 2 | memsize / 2 |
        // | false   | memsize / 2 | memsize / 2 | 0           |
        //
        // Otherwise, the new position is invalid but the new slice is empty and we won't use the
        // offset again. In either case, the unsigned subtraction will not underflow.
        context.offset = memsize as u16 - max_count as u16 - context.offset;

        if writer.ptr < writer.end || done.is_break() {
            context.state = if context.written == 0 {
                // The encoder was empty
                WriterState::Error(Error::InvalidArgument)
            } else if writer.last_code.is_end_marker() {
                // Do not check for end markers in the inner loop in Encoder::encode() since this
                // would substantially increase the instruction count there. Instead, only check
                // the last code to report on error.
                WriterState::Done
            } else {
                // Write an extra end marker to prevent looping forever with wrapping tx.
                if writer.ptr < writer.end {
                    unsafe { writer.ptr.write_volatile(PulseCode::end_marker()) };
                    WriterState::Error(Error::EndMarkerMissing)
                } else {
                    // // The buffer is full, and we would need to wait for the next
                    // // Event::Threshold before writing an end marker.
                    // // However, the data was invalid by missing an end marker,
                    // // and the only guarantee that we provide is that transmission will
                    // // eventually stop and return an error. Thus, to avoid the logic to implement
                    // // this, simply modify the last code that was written to
                    // // contain an end marker.
                    // unsafe {
                    //     writer
                    //         .ptr
                    //         .sub(1)
                    //         .write_volatile(writer.last_code.with_length2(0).unwrap())
                    // };

                    // The buffer is full, and we can't easily write an end marker. (Short of
                    // overwriting some other code, which might be ok since hitting this error case
                    // always indicates a bug in user code.)
                    // Thus, remain in Active state. On the next Event::Threshold, this function
                    // will be called again, with data already exhausted, and hit the other arm of
                    // this conditional.
                    WriterState::Active
                }
            };
        }

        debug_assert!(context.offset == 0 || context.offset as usize == memsize / 2);
    }
}

/// An [`Encoder`] that simply copies a slice of `PulseCode`s to the hardware.
#[derive(Clone, Debug)]
pub struct CopyEncoder<'a> {
    data: &'a [PulseCode],
}

impl<'a> CopyEncoder<'a> {
    /// Create a new instance that transmits the provided `data`.
    pub fn new(data: &'a [PulseCode]) -> Self {
        Self { data }
    }
}

impl<'a> Encoder for CopyEncoder<'a> {
    #[inline(always)]
    fn encode(&mut self, writer: &mut RmtWriter) -> ControlFlow<()> {
        let len = self.data.len();
        let mut ptr = self.data.as_ptr();
        let end = unsafe { ptr.add(len) };

        let written = writer.write_many(len, || {
            // SAFETY: write_many() guarantees that this will be called at most
            // `len` times, such the pointer will remain in-bounds.
            // FIXME: Check whether we can use safe indexing here and the compiler is smart enough
            // to optimize the bounds check.
            let code = unsafe { ptr.read() };
            ptr = unsafe { ptr.add(1) };
            code
        });

        debug_assert!(ptr <= end);

        self.data.split_off(..written).unwrap();

        if self.data.is_empty() {
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    }
}
