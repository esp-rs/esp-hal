#[cfg(any(
    feature = "esp32",
    feature = "esp32c2",
    feature = "esp32c3",
    feature = "esp32c6",
    feature = "esp32s2",
    feature = "esp32s3",
))]
fn main() -> Result<(), String> {
    match std::env::var("OPT_LEVEL") {
        Ok(level) => {
            if level != "2" && level != "3" {
                let message = format!(
                    "esp-wifi should be built with optimization level 2 or 3 - yours is {}. 
                    See https://github.com/esp-rs/esp-wifi",
                    level
                )
                .to_string();
                println!("cargo:warning={}", message);
            }
        }
        Err(_err) => (),
    }

    #[cfg(feature = "esp32")]
    println!("cargo:rustc-cfg=esp32");

    #[cfg(feature = "esp32c2")]
    println!("cargo:rustc-cfg=esp32c2");

    #[cfg(feature = "esp32c3")]
    println!("cargo:rustc-cfg=esp32c3");

    #[cfg(feature = "esp32c6")]
    println!("cargo:rustc-cfg=esp32c6");

    #[cfg(feature = "esp32s2")]
    println!("cargo:rustc-cfg=esp32s2");

    #[cfg(feature = "esp32s3")]
    println!("cargo:rustc-cfg=esp32s3");

    #[cfg(feature = "coex")]
    println!("cargo:rustc-cfg=coex");

    let version_output = std::process::Command::new(
        std::env::var_os("RUSTC").unwrap_or_else(|| std::ffi::OsString::from("rustc")),
    )
    .arg("-V")
    .output()
    .unwrap()
    .stdout;
    let version_string = String::from_utf8_lossy(&version_output);

    // HACK: we detect the xtensa-enabled compiler by existence of the second version string in parens
    // - upstream output format: rustc 1.75.0-nightly (cae0791da 2023-10-05)
    // - xtensa output format: rustc 1.73.0-nightly (9163a2087 2023-10-03) (1.73.0.0)
    if version_string.chars().filter(|&c| c == '(').count() == 2 {
        let version = version_string
            .split('(')
            .last()
            .unwrap()
            .split(')')
            .next()
            .unwrap();

        let mut version = version.split('.');

        let major = version.next().unwrap().parse::<u32>().unwrap();
        let minor = version.next().unwrap().parse::<u32>().unwrap();
        let patch = version.next().unwrap().parse::<u32>().unwrap();
        let release = version.next().unwrap().parse::<u32>().unwrap();

        let version = Version4(major, minor, patch, release);

        if version >= Version4(1, 73, 0, 1) {
            println!("cargo:rustc-cfg=xtensa_has_vaarg");
        }
    }

    Ok(())
}

#[cfg(not(any(
    feature = "esp32",
    feature = "esp32c2",
    feature = "esp32c3",
    feature = "esp32c6",
    feature = "esp32s2",
    feature = "esp32s3",
)))]
fn main() {
    panic!("Select a chip via it's cargo feature");
}

use std::cmp::Ordering;

#[derive(Debug, Clone, Copy, PartialEq)]
struct Version4(u32, u32, u32, u32);

impl PartialOrd for Version4 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match self.0.partial_cmp(&other.0) {
            Some(Ordering::Equal) => {}
            ord => return ord,
        }
        match self.1.partial_cmp(&other.1) {
            Some(Ordering::Equal) => {}
            ord => return ord,
        }
        match self.2.partial_cmp(&other.2) {
            Some(Ordering::Equal) => {}
            ord => return ord,
        }
        self.3.partial_cmp(&other.3)
    }
}
