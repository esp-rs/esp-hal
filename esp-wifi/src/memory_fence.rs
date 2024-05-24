pub(crate) fn memory_fence() {
    #[cfg(target_arch = "xtensa")]
    unsafe {
        core::arch::asm!("memw");
    }

    #[cfg(target_arch = "riscv")]
    unsafe {
        core::arch::asm!("fence");
    }
}
