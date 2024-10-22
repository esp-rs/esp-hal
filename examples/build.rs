fn main() {
    // Allow building examples in CI in debug mode
    println!("cargo:rustc-check-cfg=cfg(is_not_release)");
    println!("cargo:rerun-if-env-changed=CI");
    #[cfg(debug_assertions)]
    if std::env::var("CI").is_err() {
        println!("cargo::rustc-cfg=is_not_release");
    }
}
