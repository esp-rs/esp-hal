cfg_if::cfg_if! {
    if #[cfg(feature="interrupts")]
    {
        // RISCV ULP specific interrupt handlers, critical section
        #[cfg(any(esp32s2, esp32s3))]
        pub mod riscv_ulp;
        #[cfg(any(esp32s2, esp32s3))]
        pub use riscv_ulp::*;
        #[cfg(any(esp32s2, esp32s3))]
        mod critical_section;

        /// Portable interrupt binding and handling code
        #[cfg(any(esp32s2, esp32s3))]
        pub mod generic;
        #[cfg(any(esp32s2, esp32s3))]
        pub use generic::*;
    } else {
        /// If interrupt handling is not enabled, need to provide a stubs functions for
        /// start_trap_rust and DefaultHandler.
        #[doc(hidden)]
        #[allow(dead_code)]
        #[unsafe(link_section = ".trap.rust")]
        #[unsafe(export_name = "_start_trap_rust")]
        pub extern "C" fn stub_start_trap_rust(_trap_frame: *const u32, _irqs: u32) {}

        #[doc(hidden)]
        #[allow(dead_code)]
        #[unsafe(export_name = "DefaultHandler")]
        pub fn stub_default_handler() {}
    }
}
