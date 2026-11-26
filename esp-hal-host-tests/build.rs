fn main() {
    // The esp-rtos sources under test target a dual-core chip.
    println!("cargo::rustc-check-cfg=cfg(multi_core)");
    println!("cargo::rustc-cfg=multi_core");
}
