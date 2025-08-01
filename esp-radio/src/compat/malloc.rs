//! These symbols are expected to be present.
//!
//! `esp-alloc` will provide them. The user is expected to provide implementation if
//! they don't want to use `esp-alloc`

unsafe extern "C" {
    pub fn malloc(size: usize) -> *mut u8;

    #[cfg(any(feature = "wifi", not(npl)))]
    pub fn malloc_internal(size: usize) -> *mut u8;

    pub fn free(ptr: *mut u8);

    #[cfg(any(feature = "wifi", all(feature = "ble", npl)))]
    pub fn calloc(number: u32, size: usize) -> *mut u8;

    #[cfg(feature = "wifi")]
    pub fn calloc_internal(number: u32, size: usize) -> *mut u8;

    #[cfg(feature = "wifi")]
    pub fn get_free_internal_heap_size() -> usize;
}
