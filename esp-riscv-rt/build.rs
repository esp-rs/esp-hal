fn main() {
    // required for `riscv-rt-macros` to actually generate code
    println!("cargo:rustc-env=RISCV_RT_BASE_ISA=rv32i");

    // `target_feature = "f"` is unstable on stable Rust, so detect the F
    // extension from the target name instead (e.g. "riscv32imafc-unknown-none-elf").
    println!("cargo::rustc-check-cfg=cfg(riscv_has_f)");
    println!("cargo::rerun-if-changed=build.rs");

    let target = std::env::var("TARGET").unwrap_or_default();
    let arch = target.split('-').next().unwrap_or("");
    let exts = arch
        .trim_start_matches("riscv32")
        .trim_start_matches("riscv64")
        .split('_') // ignore Z* extensions such as zfinx
        .next()
        .unwrap_or("");

    // 'g' = imafd; 'f' or 'd' implies FP registers
    if exts.contains('f') || exts.contains('d') || exts.contains('g') {
        println!("cargo::rustc-cfg=riscv_has_f");
    }
}
