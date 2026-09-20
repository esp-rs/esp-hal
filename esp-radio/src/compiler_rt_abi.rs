//! Integer builtins the Wi-Fi/BLE blobs call, defined here so that
//! `compiler_builtins` is never asked for them.
//!
//! `riscv32imafc-unknown-none-elf` is a hard-float (`ilp32f`) target, and so
//! is every object in the blobs. Rust, however, ships `compiler_builtins`
//! with `optimized-compiler-builtins` enabled, which builds compiler-rt's C
//! routines through cc-rs, and cc-rs passes `-mabi=ilp32` for every non-Linux
//! `riscv32` target regardless of the `f` in the triple
//! (<https://github.com/rust-lang/cc-rs/issues/795>). 36 objects in the
//! shipped rlib are soft-float as a result, and pulling one in fails the link
//! with "cannot link object files with different floating-point ABI".
//!
//! Defining the symbols here means lld resolves them before it reaches those
//! archive members. Plain Rust cannot hit this - LLVM inlines `ctpop` and
//! `bswap` for this target - so only the GCC-built blobs need it, which is why
//! this lives in esp-radio and not in esp-hal. It can go once a Rust release
//! ships a `compiler_builtins` built by a fixed cc-rs.

#[unsafe(no_mangle)]
extern "C" fn __popcountsi2(x: u32) -> i32 {
    x.count_ones() as i32
}

#[unsafe(no_mangle)]
extern "C" fn __popcountdi2(x: u64) -> i32 {
    x.count_ones() as i32
}

#[unsafe(no_mangle)]
extern "C" fn __bswapsi2(x: u32) -> u32 {
    x.swap_bytes()
}

#[unsafe(no_mangle)]
extern "C" fn __bswapdi2(x: u64) -> u64 {
    x.swap_bytes()
}
