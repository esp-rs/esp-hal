use core::str::FromStr;

use log::LevelFilter;

use super::println;

const LOG_TARGETS: Option<&'static str> = option_env!("ESP_LOGTARGETS");

pub fn init_logger(level: log::LevelFilter) {
    unsafe {
        log::set_logger_racy(&EspLogger).unwrap();
        log::set_max_level_racy(level);
    }
}

pub fn init_logger_from_env() {
    unsafe {
        log::set_logger_racy(&EspLogger).unwrap();
    }

    const LEVEL: Option<&'static str> = option_env!("ESP_LOGLEVEL");

    if let Some(lvl) = LEVEL {
        let level = LevelFilter::from_str(lvl).unwrap_or_else(|_| LevelFilter::Off);
        unsafe { log::set_max_level_racy(level) };
    }
}

struct EspLogger;

impl log::Log for EspLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    #[allow(unused)]
    fn log(&self, record: &log::Record) {
        // check enabled log targets if any
        if let Some(targets) = LOG_TARGETS {
            if targets
                .split(",")
                .find(|v| record.target().starts_with(v))
                .is_none()
            {
                return;
            }
        }

        const RESET: &str = "\u{001B}[0m";
        const RED: &str = "\u{001B}[31m";
        const GREEN: &str = "\u{001B}[32m";
        const YELLOW: &str = "\u{001B}[33m";
        const BLUE: &str = "\u{001B}[34m";
        const CYAN: &str = "\u{001B}[35m";

        #[cfg(feature = "colors")]
        let color = match record.level() {
            log::Level::Error => RED,
            log::Level::Warn => YELLOW,
            log::Level::Info => GREEN,
            log::Level::Debug => BLUE,
            log::Level::Trace => CYAN,
        };
        #[cfg(feature = "colors")]
        let reset = RESET;

        #[cfg(not(feature = "colors"))]
        let color = "";
        #[cfg(not(feature = "colors"))]
        let reset = "";

        println!("{}{} - {}{}", color, record.level(), record.args(), reset);
    }

    fn flush(&self) {}
}
