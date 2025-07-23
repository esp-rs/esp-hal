pub(crate) fn memory_fence() {
    #[cfg(xtensa)]
    unsafe {
        core::arch::asm!("memw");
    }

    #[cfg(riscv)]
    unsafe {
        core::arch::asm!("fence");
    }
}
