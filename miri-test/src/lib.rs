#![no_std]
#![allow(internal_features)]
#![feature(core_intrinsics)]

// see https://github.com/rust-lang/miri/blob/master/tests/utils/miri_extern.rs
extern "Rust" {
    pub fn miri_write_to_stdout(bytes: &[u8]);
}

#[panic_handler]
fn panic<'a, 'b>(pi: &'a core::panic::PanicInfo<'b>) -> ! {
    println!("{:?}", pi);
    core::intrinsics::abort();
}

#[macro_export]
macro_rules! miri_init {
    () => {
        #[cfg(miri)]
        #[no_mangle]
        fn miri_start(argc: isize, argv: *const *const u8) -> isize {
            main();
            0
        }

        #[cfg(not(miri))]
        pub fn _main() {
            main();
        }
    };
}

#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {{
        {
            use core::fmt::Write;
            writeln!($crate::Printer, $($arg)*).ok();
        }
    }};
}

pub struct Printer;

impl core::fmt::Write for Printer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        unsafe { miri_write_to_stdout(s.as_bytes()) };
        Ok(())
    }
}
