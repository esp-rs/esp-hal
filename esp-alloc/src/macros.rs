//! Macros provided for convenience

/// Create a heap allocator providing a heap of the given size in bytes
///
/// You can only have ONE allocator at most
#[macro_export]
macro_rules! heap_allocator {
    ($size:expr) => {{
        #[global_allocator]
        static ALLOCATOR: $crate::EspHeap = $crate::EspHeap::empty();
        static mut HEAP: core::mem::MaybeUninit<[u8; $size]> = core::mem::MaybeUninit::uninit();

        unsafe {
            ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, $size);
        }
    }};
}

/// Create a heap allocator backed by PSRAM
///
/// You can only have ONE allocator at most. You need a SoC which supports PSRAM
/// and activate the feature to enable it. You need to pass the PSRAM peripheral
/// and the psram module path.
///
/// # Usage
/// ```no_run
/// esp_alloc::psram_allocator!(peripherals.PSRAM, hal::psram);
/// ```
#[macro_export]
macro_rules! psram_allocator {
    ($peripheral:expr,$psram_module:path) => {{
        #[global_allocator]
        static ALLOCATOR: $crate::EspHeap = $crate::EspHeap::empty();

        use $psram_module as _psram;
        _psram::init_psram($peripheral);
        unsafe {
            ALLOCATOR.init(_psram::psram_vaddr_start() as *mut u8, _psram::PSRAM_BYTES);
        }
    }};
}
