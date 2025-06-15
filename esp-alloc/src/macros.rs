//! Macros provided for convenience

/// Initialize a global heap allocator providing a heap of the given size in
/// bytes. This supports attributes.
///
/// # Usage
/// ```rust, no_run
/// // Use 64kB in the same region stack uses (dram_seg), for the heap.
/// heap_allocator!(size: 64000);
/// // Use 64kB in dram2_seg for the heap, which is otherwise unused.
/// heap_allocator!(#[link_section = ".dram2_uninit"] size: 64000);
/// ```
#[macro_export]
macro_rules! heap_allocator {
    ($(#[$m:meta])* size: $size:expr) => {{
        $(#[$m])*
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
/// You need a SoC which supports PSRAM and activate the feature to enable
/// it. You need to pass the PSRAM peripheral and the psram module path.
///
/// # Usage
///
/// ```rust, no_run
/// esp_alloc::psram_allocator!(peripherals.PSRAM, hal::psram);
/// ```
///
/// # ⚠️ Limitations
///
/// On ESP32, ESP32-S2 and ESP32-S3 the atomic instructions do not work
/// correctly when the memory they access is located in PSRAM. This means that
/// the allocator must not be used to allocate `Atomic*` types - either directly
/// or indirectly. Be very careful when using PSRAM in your global allocator.
#[macro_export]
macro_rules! psram_allocator {
    ($peripheral:expr, $psram_module:path) => {{
        use $psram_module as _psram;
        let (start, size) = _psram::psram_raw_parts(&$peripheral);
        unsafe {
            $crate::HEAP.add_region($crate::HeapRegion::new(
                start,
                size,
                $crate::MemoryCapability::External.into(),
            ));
        }
    }};
}
