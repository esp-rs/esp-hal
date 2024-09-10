//! Macros provided for convenience

/// Initialize a global heap allocator providing a heap of the given size in
/// bytes
#[macro_export]
macro_rules! heap_allocator {
    ($size:expr) => {{
        static mut HEAP: core::mem::MaybeUninit<[u8; $size]> = core::mem::MaybeUninit::uninit();

        unsafe {
            $crate::HEAP.add_region($crate::HeapRegion::new(
                HEAP.as_mut_ptr() as *mut u8,
                $size,
                $crate::MemoryCapability::Internal.into(),
            ));
        }
    }};
}

/// Initialize a global heap allocator backed by PSRAM
///
/// You need a SoC which supports PSRAM
/// and activate the feature to enable it. You need to pass the PSRAM peripheral
/// and the psram module path.
///
/// # Usage
/// ```rust, no_run
/// esp_alloc::psram_allocator!(peripherals.PSRAM, hal::psram);
/// ```
#[macro_export]
macro_rules! psram_allocator {
    ($peripheral:expr,$psram_module:path) => {{
        use $psram_module as _psram;
        _psram::init_psram($peripheral);
        unsafe {
            $crate::HEAP.add_region($crate::HeapRegion::new(
                _psram::psram_vaddr_start() as *mut u8,
                _psram::PSRAM_BYTES,
                $crate::MemoryCapability::External.into(),
            ));
        }
    }};
}

/// Gets the usage stats for the current memory heap
///
/// # Usage
/// ```rust, no_run
/// esp_alloc::get_info!();
/// ```
#[macro_export]
macro_rules! get_info {
    () => {{
        $crate::HEAP.stats()
    }};
}
