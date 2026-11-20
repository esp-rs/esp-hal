use std::sync::Mutex;

/// What the test writer received. Tests run with `--test-threads=1`, because the logger state is
/// global.
static OUTPUT: Mutex<Vec<u8>> = Mutex::new(Vec::new());

/// A writer that appends to [`OUTPUT`], and can run a callback in the middle of a write to simulate
/// a panic that interrupts it.
struct TestWriter {
    interrupt: Option<fn()>,
}

impl embedded_io::ErrorType for TestWriter {
    type Error = core::convert::Infallible;
}

impl embedded_io::Write for TestWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let Some((first, rest)) = buf.split_first() else {
            return Ok(0);
        };
        OUTPUT.lock().unwrap().push(*first);
        if let Some(interrupt) = self.interrupt.take() {
            interrupt();
        }
        OUTPUT.lock().unwrap().extend_from_slice(rest);
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn install(interrupt: Option<fn()>) {
    OUTPUT.lock().unwrap().clear();
    // Reset the takeover state of a previous test.
    critical_section::with(|_| unsafe {
        let state = &mut *crate::STATE.0.get();
        state.taken = false;
        state.taken_over = false;
    });
    crate::set_writer(Box::leak(Box::new(TestWriter { interrupt })));
}

fn output() -> Vec<u8> {
    OUTPUT.lock().unwrap().clone()
}

#[cfg(feature = "timestamp")]
#[unsafe(no_mangle)]
extern "Rust" fn _esp_log_timestamp() -> u64 {
    1234
}

#[test]
fn nothing_is_written_without_a_writer() {
    OUTPUT.lock().unwrap().clear();
    critical_section::with(|_| unsafe { (*crate::STATE.0.get()).writer = None });
    crate::with_writer(|writer, _| writer.write_bytes(b"lost"));
    assert!(output().is_empty());
}

#[test]
fn interrupted_write_is_taken_over() {
    install(Some(|| {
        crate::with_writer(|writer, takeover| {
            assert!(takeover);
            writer.write_bytes(b"PANIC");
        })
    }));

    crate::with_writer(|writer, takeover| {
        assert!(!takeover);
        writer.write_bytes(b"hello");
    });
    assert_eq!(output(), b"hPANICello");

    // After the takeover, output keeps flowing, and isn't reported as another takeover.
    crate::with_writer(|writer, takeover| {
        assert!(!takeover);
        writer.write_bytes(b"!");
    });
    assert_eq!(output(), b"hPANICello!");
}

#[cfg(feature = "log-04")]
mod log {
    use log_04::{Level, LevelFilter, Record};

    use super::*;
    use crate::logger::{Filter, write_record};

    fn format(level: Level, args: core::fmt::Arguments<'_>) -> String {
        install(None);
        crate::with_writer(|writer, _| {
            write_record(writer, &Record::builder().level(level).args(args).build());
        });
        String::from_utf8(output()).unwrap()
    }

    #[test]
    fn formats_records() {
        let line = format(Level::Warn, format_args!("value = {}", 42));

        let timestamp = if cfg!(feature = "timestamp") {
            " (1234)"
        } else {
            ""
        };
        let (color, reset) = if cfg!(feature = "colors") {
            ("\u{001B}[33m", "\u{001B}[0m")
        } else {
            ("", "")
        };
        assert_eq!(
            line,
            format!("{color}WARN{timestamp} - value = 42{reset}\n")
        );
    }

    #[test]
    fn takeover_starts_a_new_line() {
        install(Some(|| {
            log_04::Log::log(
                &LevelLoggerProbe,
                &Record::builder()
                    .level(Level::Error)
                    .args(format_args!("panic"))
                    .build(),
            )
        }));
        crate::with_writer(|writer, _| writer.write_bytes(b"partial"));

        let out = String::from_utf8(output()).unwrap();
        assert!(out.starts_with("p\n"), "{out:?}");
        assert!(out.contains("ERROR"), "{out:?}");
        assert!(out.ends_with("artial"), "{out:?}");
    }

    /// Calls the same path as the installed loggers, without registering a global logger.
    struct LevelLoggerProbe;

    impl log_04::Log for LevelLoggerProbe {
        fn enabled(&self, _: &log_04::Metadata<'_>) -> bool {
            true
        }

        fn log(&self, record: &Record<'_>) {
            crate::logger::print_record(record);
        }

        fn flush(&self) {}
    }

    #[test]
    fn filters_by_target() {
        let filter = Filter {
            directives: &[("a::b", LevelFilter::Debug), ("a", LevelFilter::Error)],
            global: Some(LevelFilter::Warn),
        };
        assert!(filter.is_enabled(Level::Debug, "a::b::c"));
        assert!(!filter.is_enabled(Level::Trace, "a::b"));
        assert!(!filter.is_enabled(Level::Warn, "a::x"));
        assert!(filter.is_enabled(Level::Error, "a"));
        assert!(filter.is_enabled(Level::Warn, "other"));
        assert!(!filter.is_enabled(Level::Info, "other"));

        let no_global = Filter {
            directives: &[("a", LevelFilter::Info)],
            global: None,
        };
        assert!(!no_global.is_enabled(Level::Error, "other"));
    }
}

/// The defmt framing, with a stand-in for `defmt::Encoder`. defmt itself does not link on the host:
/// its marker symbols come from the target linker script.
mod frames {
    use core::cell::UnsafeCell;

    use super::*;
    use crate::frame::{FRAME_ABORT, FRAME_START, FrameEncoder, Framer};

    /// Wraps each frame in `<` and `>`, and passes the payload through.
    struct MarkerEncoder;

    impl FrameEncoder for MarkerEncoder {
        fn new() -> Self {
            MarkerEncoder
        }

        fn start_frame(&mut self, mut write: impl FnMut(&[u8])) {
            write(b"<");
        }

        fn write(&mut self, data: &[u8], mut write: impl FnMut(&[u8])) {
            write(data);
        }

        fn end_frame(&mut self, mut write: impl FnMut(&[u8])) {
            write(b">");
        }
    }

    struct TestFramer(UnsafeCell<Framer<MarkerEncoder>>);
    unsafe impl Sync for TestFramer {}
    static FRAMER: TestFramer = TestFramer(UnsafeCell::new(Framer::new(MarkerEncoder)));

    fn log_frame(payload: &[u8]) {
        let framer = unsafe { &mut *FRAMER.0.get() };
        unsafe {
            framer.acquire();
            framer.write(payload);
            framer.flush();
            framer.release();
        }
    }

    #[test]
    fn frames_are_prefixed() {
        install(None);
        log_frame(b"abc");
        assert_eq!(output(), [&FRAME_START[..], b"<abc>"].concat());
    }

    #[test]
    fn interrupted_frame_is_terminated() {
        install(Some(|| log_frame(b"panic")));
        log_frame(b"hi");

        let expected = [
            // The first byte of the interrupted frame.
            &FRAME_START[..1],
            // The panic report closes it and starts its own frame.
            &FRAME_ABORT,
            &FRAME_START,
            b"<panic>",
            // The interrupted frame continues (only on the host, a real panic never returns).
            &FRAME_START[1..],
            b"<hi>",
        ]
        .concat();
        assert_eq!(output(), expected);
    }
}
