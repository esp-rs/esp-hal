use std::{error::Error, path::Path};

use esp_metadata::{Chip, Config};

#[macro_export]
macro_rules! assert_unique_features {
    ($($feature:literal),+ $(,)?) => {
        assert!(
            (0 $(+ cfg!(feature = $feature) as usize)+ ) <= 1,
            "Exactly zero or one of the following features must be enabled: {}",
            [$($feature),+].join(", ")
        );
    };
}

fn main() -> Result<(), Box<dyn Error>> {
    let self_version = std::env::var("CARGO_PKG_VERSION")?;
    let self_version: Vec<&str> = self_version.split('.').collect();
    if self_version[0] != "0" || self_version[1] != "0" && self_version[1] != "1" {
        panic!("The 'esp-rom-sys' crate is not allowed to get bumped to anything above 0.1.x");
    }

    // Ensure that exactly one chip has been specified:
    let chip = Chip::from_cargo_feature()?;

    // Log and defmt are mutually exclusive features. The main technical reason is
    // that allowing both would make the exact panicking behaviour a fragile
    // implementation detail.
    assert_unique_features!("log-04", "defmt");

    // Load the configuration file for the configured device:
    let config = Config::for_chip(&chip);

    // Define all necessary configuration symbols for the configured device:
    config.define_symbols();
    config.generate_metadata();

    let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    copy_dir_all(format!("./ld/{chip}/"), &out)?;
    copy_dir_all(format!("./libs/{chip}/"), &out)?;

    include_libs(format!("./libs/{chip}/"))?;

    // exploit the fact that linkers treat an unknown library format as a linker
    // script
    println!("cargo:rustc-link-lib=esp_rom_sys");

    Ok(())
}

fn copy_dir_all(src: impl AsRef<Path>, dst: impl AsRef<Path>) -> std::io::Result<()> {
    std::fs::create_dir_all(&dst)?;
    for entry in std::fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        if ty.is_dir() {
            copy_dir_all(entry.path(), dst.as_ref().join(entry.file_name()))?;
        } else {
            std::fs::copy(entry.path(), dst.as_ref().join(entry.file_name()))?;
            println!("cargo:rerun-if-changed={}", entry.path().display());
        }
    }
    Ok(())
}

fn include_libs(path: impl AsRef<Path>) -> std::io::Result<()> {
    for entry in std::fs::read_dir(path.as_ref())? {
        let file_name = entry?.file_name().into_string().unwrap();
        if let Some(lib_name) = file_name
            .strip_prefix("lib")
            .and_then(|f| f.strip_suffix(".a"))
        {
            println!("cargo:rustc-link-lib=static={lib_name}");
        }
    }

    let add_rwtext_file = std::path::PathBuf::from(path.as_ref()).join("add_rwtext");
    if std::fs::exists(&add_rwtext_file)? {
        let contents = std::fs::read_to_string(add_rwtext_file)?.replace("\n", "\\n");
        println!("cargo::metadata=RWTEXT_ROM_FUNCTIONS={contents}");
    }
    Ok(())
}
