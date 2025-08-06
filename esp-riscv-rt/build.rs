fn main() {
    // required for `riscv-rt-macros` to actually generate code
    println!("cargo:rustc-env=RISCV_RT_BASE_ISA=rv32i");
}
