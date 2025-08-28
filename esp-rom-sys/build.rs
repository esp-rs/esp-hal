use std::{error::Error, path::Path};

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
    if self_version[0] != "0" || self_version[1] != "1" {
        panic!("The 'esp-rom-sys' crate is not allowed to get bumped to anything above 0.1.x");
    }

    let chip = esp_metadata_generated::Chip::from_cargo_feature()?;

    // Define all necessary configuration symbols for the configured device:
    chip.define_cfgs();

    let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    copy_dir_all(format!("./ld/{}/", chip.name()), &out)?;
    copy_dir_all(format!("./libs/{}/", chip.name()), &out)?;

    include_libs(format!("./libs/{}/", chip.name()))?;

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
    Ok(())
}
