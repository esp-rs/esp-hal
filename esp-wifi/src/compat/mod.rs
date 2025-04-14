pub mod common;
pub mod malloc;
pub mod misc;
pub mod timer_compat;

#[unsafe(no_mangle)]
unsafe extern "C" fn _putchar(c: u8) { unsafe {
    static mut BUFFER: [u8; 256] = [0u8; 256];
    static mut IDX: usize = 0;

    let buffer = core::ptr::addr_of_mut!(BUFFER);
    if c == 0 || c == b'\n' || IDX == (*buffer).len() - 1 {
        if c != 0 {
            BUFFER[IDX] = c;
        } else {
            IDX = IDX.saturating_sub(1);
        }

        info!("{}", core::str::from_utf8_unchecked(&BUFFER[..IDX]));
        IDX = 0;
    } else {
        BUFFER[IDX] = c;
        IDX += 1;
    }
}}
