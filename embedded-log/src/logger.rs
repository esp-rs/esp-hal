//! `log` 0.4 logger.

use core::fmt::Write as _;

use log_04 as log;

use crate::ErasedWrite;

include!(concat!(env!("OUT_DIR"), "/log_filter.rs"));

/// Installs `writer` and a logger filtered by the `ESP_LOG` environment variable at build time.
///
/// Calling this again replaces the writer, but not the logger.
pub fn init<W>(writer: &'static mut W)
where
    W: embedded_io::Write + Send,
{
    crate::set_writer(writer);
    // Safety: `log` is not used concurrently during initialization.
    unsafe {
        _ = log::set_logger_racy(&EnvLogger);
        log::set_max_level_racy(FILTER_MAX);
    }
}

/// Installs `writer` and a logger that prints records up to `level`, ignoring `ESP_LOG`.
///
/// Calling this again replaces the writer and the level, but not the logger.
pub fn init_with_level<W>(writer: &'static mut W, level: log::LevelFilter)
where
    W: embedded_io::Write + Send,
{
    crate::set_writer(writer);
    // Safety: `log` is not used concurrently during initialization.
    unsafe {
        _ = log::set_logger_racy(&LevelLogger);
        log::set_max_level_racy(level);
    }
}

/// A target filter parsed from `ESP_LOG`.
pub(crate) struct Filter {
    /// Target prefixes and their levels, longest prefix first.
    pub(crate) directives: &'static [(&'static str, log::LevelFilter)],
    /// The level of targets that match no prefix.
    pub(crate) global: Option<log::LevelFilter>,
}

impl Filter {
    pub(crate) fn is_enabled(&self, level: log::Level, target: &str) -> bool {
        for (prefix, max) in self.directives {
            if target.starts_with(prefix) {
                return level <= *max;
            }
        }
        self.global.is_some_and(|max| level <= max)
    }
}

struct LevelLogger;

impl log::Log for LevelLogger {
    fn enabled(&self, _: &log::Metadata) -> bool {
        // `log` already filtered by the max level.
        true
    }

    fn log(&self, record: &log::Record) {
        print_record(record);
    }

    fn flush(&self) {}
}

struct EnvLogger;

impl log::Log for EnvLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        FILTER.is_enabled(metadata.level(), metadata.target())
    }

    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            print_record(record);
        }
    }

    fn flush(&self) {}
}

pub(crate) fn print_record(record: &log::Record) {
    crate::with_writer(|writer, takeover| {
        if takeover {
            // The interrupted record may have stopped mid-line.
            writer.write_bytes(b"\n");
        }
        write_record(writer, record);
        writer.flush_bytes();
    });
}

pub(crate) fn write_record(writer: &mut dyn ErasedWrite, record: &log::Record) {
    let (color, reset) = if cfg!(feature = "colors") {
        let color = match record.level() {
            log::Level::Error => "\u{001B}[31m",
            log::Level::Warn => "\u{001B}[33m",
            log::Level::Info => "\u{001B}[32m",
            log::Level::Debug => "\u{001B}[34m",
            log::Level::Trace => "\u{001B}[35m",
        };
        (color, "\u{001B}[0m")
    } else {
        ("", "")
    };

    let mut out = FmtWriter(writer);

    #[cfg(feature = "timestamp")]
    let result = writeln!(
        out,
        "{color}{} ({}) - {}{reset}",
        record.level(),
        crate::timestamp(),
        record.args(),
    );
    #[cfg(not(feature = "timestamp"))]
    let result = writeln!(out, "{color}{} - {}{reset}", record.level(), record.args());
    // `FmtWriter` never fails, but a `Display` impl in the arguments may.
    _ = result;
}

struct FmtWriter<'a>(&'a mut dyn ErasedWrite);

impl core::fmt::Write for FmtWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0.write_bytes(s.as_bytes());
        Ok(())
    }
}
