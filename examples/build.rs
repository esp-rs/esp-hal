fn main() {
    if cfg!(feature = "esp32c6") || cfg!(feature = "esp32h2") {
        // TODO this shouldn't be required to be done by the user but done in `linkall.x`
        println!("cargo::rustc-link-arg=-Trom_coexist.x");
        println!("cargo::rustc-link-arg=-Trom_functions.x");
        println!("cargo::rustc-link-arg=-Trom_phy.x");
    }

    // Allow building examples in CI in debug mode
    println!("cargo:rustc-check-cfg=cfg(is_not_release)");
    println!("cargo:rerun-if-env-changed=CI");
    #[cfg(debug_assertions)]
    if std::env::var("CI").is_err() {
        println!("cargo::rustc-cfg=is_not_release");
    }
}
