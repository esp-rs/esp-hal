fn main() {
    // required for `riscv-rt-macros` to actually generate code
    println!("cargo:rustc-env=RISCV_RT_BASE_ISA=rv32i");

    println!("cargo::rustc-check-cfg=cfg(riscv_has_f)");
    println!("cargo::rerun-if-changed=build.rs");

    let has_f = has_f_extension();
    if has_f {
        println!("cargo::rustc-cfg=riscv_has_f");
    }

    // Crates that depend directly on `esp-riscv-rt` read these values as
    // `DEP_ESP_RISCV_RT_FLOAT_SAVE_RESTORE` and `DEP_ESP_RISCV_RT_FPU_DISABLED_IN_INTERRUPTS`.
    if has_f {
        if std::env::var_os("CARGO_FEATURE_FLOAT_SAVE_RESTORE").is_some() {
            println!("cargo::metadata=float_save_restore=1");
        } else {
            println!("cargo::metadata=fpu_disabled_in_interrupts=1");
        }
    }
}

fn has_f_extension() -> bool {
    // Stable rustc does not report the unstable `f` target feature. The ISA string in the target
    // name shows the F extension.
    let target = std::env::var("TARGET").unwrap_or_default();
    let mut has_f = target
        .split('-')
        .next()
        .and_then(|arch| arch.strip_prefix("riscv32"))
        .is_some_and(|isa| isa.contains('f'));

    let rustflags = std::env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    let target_features = rustflags
        .split('\x1f')
        .map(|flag| flag.strip_prefix("-C").unwrap_or(flag))
        .filter_map(|flag| flag.strip_prefix("target-feature="))
        .flat_map(|flag| flag.split(','));
    for feature in target_features {
        match feature.trim() {
            "+f" => has_f = true,
            "-f" => has_f = false,
            _ => {}
        }
    }

    has_f
}
