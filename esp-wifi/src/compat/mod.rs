pub mod common;
pub mod malloc;
pub mod misc;
#[cfg(any(feature = "wifi-logs", nightly))]
pub mod syslog;
pub mod timer_compat;

pub mod queue {
    pub use heapless::spsc::Queue as SimpleQueue;
}

#[no_mangle]
unsafe extern "C" fn _putchar(c: u8) {
    static mut BUFFER: [u8; 256] = [0u8; 256];
    static mut IDX: usize = 0;

    if c == 0 || c == b'\n' || IDX == BUFFER.len() - 1 {
        if c != 0 {
            BUFFER[IDX] = c;
        } else if IDX > 0 {
            IDX -= 1;
        }

        info!("{}", core::str::from_utf8_unchecked(&BUFFER[..IDX]));
        IDX = 0;
    } else {
        BUFFER[IDX] = c;
        IDX += 1;
    }
}
