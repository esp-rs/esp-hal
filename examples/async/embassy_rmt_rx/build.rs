fn main() {
    // Allow building examples in CI in debug mode
    println!("cargo:rustc-check-cfg=cfg(is_not_release)");
    println!("cargo:rerun-if-env-changed=CI");
    if std::env::var("CI").is_err() {
        if let Ok(level) = std::env::var("OPT_LEVEL") {
            if level == "0" || level == "1" {
                println!("cargo::rustc-cfg=is_not_release");
            }
        }
    }
}
