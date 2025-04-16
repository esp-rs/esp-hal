//! defmt global logger implementation.
// Implementation taken from defmt-rtt, with a custom framing prefix

#[cfg(feature = "critical-section")]
use critical_section::RestoreState;

use super::{LockToken, PrinterImpl};

/// Global logger lock.
#[cfg(feature = "critical-section")]
static mut TAKEN: bool = false;
#[cfg(feature = "critical-section")]
static mut CS_RESTORE: RestoreState = RestoreState::invalid();
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();

#[defmt::global_logger]
pub struct Logger;
unsafe impl defmt::Logger for Logger {
    fn acquire() {
        #[cfg(feature = "critical-section")]
        unsafe {
            // safety: Must be paired with corresponding call to release(), see below
            let restore = critical_section::acquire();

            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            if TAKEN {
                panic!("defmt logger taken reentrantly")
            }

            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            TAKEN = true;

            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            CS_RESTORE = restore;
        }

        // If not disabled, write a non-UTF8 sequence to indicate the start of a defmt
        // frame. We need this to distinguish defmt frames from other data that
        // might be written to the printer.
        do_write(&[0xFF, 0x00]);

        // safety: accessing the `static mut` is OK because we have acquired a critical
        // section.
        unsafe { ENCODER.start_frame(do_write) }
    }

    unsafe fn release() {
        unsafe {
            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            ENCODER.end_frame(do_write);

            Self::flush();

            #[cfg(feature = "critical-section")]
            {
                // We don't need to write a custom end-of-frame sequence because:
                //  - using `defmt`, the rzcobs encoding already includes a terminating zero
                //  - using `defmt-raw`, we don't add any additional framing data

                // safety: accessing the `static mut` is OK because we have acquired a critical
                // section.
                TAKEN = false;

                // safety: accessing the `static mut` is OK because we have acquired a critical
                // section.
                let restore = CS_RESTORE;

                // safety: Must be paired with corresponding call to acquire(), see above
                critical_section::release(restore);
            }
        }
    }

    unsafe fn flush() {
        let token = unsafe {
            // Safety: the implementation ensures this is only called in a critical
            // section.
            LockToken::conjure()
        };
        PrinterImpl::flush(token);
    }

    unsafe fn write(bytes: &[u8]) {
        unsafe {
            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            ENCODER.write(bytes, do_write);
        }
    }
}

fn do_write(bytes: &[u8]) {
    let token = unsafe {
        // Safety: the above implementation ensures this is only called in a critical
        // section.
        LockToken::conjure()
    };
    PrinterImpl::write_bytes_in_cs(bytes, token)
}
