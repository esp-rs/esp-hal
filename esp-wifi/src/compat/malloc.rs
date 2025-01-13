use core::{alloc::Layout, mem::MaybeUninit, ptr::NonNull};

#[no_mangle]
pub unsafe extern "C" fn malloc(size: usize) -> *mut u8 {
    trace!("alloc {}", size);

    let total_size = size + 4;

    extern "C" {
        fn esp_wifi_allocate_from_internal_ram(size: usize) -> *mut u8;
    }

    let ptr = unsafe { esp_wifi_allocate_from_internal_ram(total_size) };

    if ptr.is_null() {
        warn!("Unable to allocate {} bytes", size);
        return ptr;
    }

    *(ptr as *mut usize) = total_size;
    ptr.offset(4)
}

#[no_mangle]
pub unsafe extern "C" fn free(ptr: *mut u8) {
    trace!("free {:?}", ptr);

    if ptr.is_null() {
        return;
    }

    let ptr = ptr.offset(-4);
    let total_size = *(ptr as *const usize);

    let layout = Layout::from_size_align_unchecked(total_size, 4);
    alloc::alloc::dealloc(ptr, layout);
}

#[no_mangle]
pub unsafe extern "C" fn calloc(number: u32, size: usize) -> *mut u8 {
    trace!("calloc {} {}", number, size);

    let total_size = number as usize * size;
    let ptr = malloc(total_size);

    if !ptr.is_null() {
        for i in 0..total_size as isize {
            ptr.offset(i).write_volatile(0);
        }
    }

    ptr
}

#[no_mangle]
unsafe extern "C" fn realloc(ptr: *mut u8, new_size: usize) -> *mut u8 {
    trace!("realloc {:?} {}", ptr, new_size);

    extern "C" {
        fn memcpy(d: *mut u8, s: *const u8, l: usize);
    }

    unsafe {
        let p = malloc(new_size);
        if !p.is_null() && !ptr.is_null() {
            let len = usize::min(
                (ptr as *const u32).sub(1).read_volatile() as usize,
                new_size,
            );
            memcpy(p, ptr, len);
            free(ptr);
        }
        p
    }
}

#[cfg(feature = "esp-alloc")]
#[doc(hidden)]
#[no_mangle]
pub extern "C" fn esp_wifi_free_internal_heap() -> usize {
    esp_alloc::HEAP.free_caps(esp_alloc::MemoryCapability::Internal.into())
}

#[cfg(feature = "esp-alloc")]
#[doc(hidden)]
#[no_mangle]
pub extern "C" fn esp_wifi_allocate_from_internal_ram(size: usize) -> *mut u8 {
    unsafe {
        esp_alloc::HEAP.alloc_caps(
            esp_alloc::MemoryCapability::Internal.into(),
            core::alloc::Layout::from_size_align_unchecked(size, 4),
        )
    }
}

struct Mallocator;

impl Mallocator {
    fn allocate(layout: Layout) -> *mut u8 {
        extern "C" {
            fn esp_wifi_allocate_from_internal_ram(size: usize) -> *mut u8;
        }
        unsafe { esp_wifi_allocate_from_internal_ram(layout.size()) }
    }

    fn deallocate(ptr: *mut u8, layout: Layout) {
        unsafe { alloc::alloc::dealloc(ptr, layout) }
    }
}

pub(crate) struct MallocBox<T> {
    ptr: NonNull<T>,
}

impl<T> MallocBox<T> {
    #[inline(always)]
    pub fn try_new_uninit() -> Option<MallocBox<MaybeUninit<T>>> {
        let ptr = if core::mem::size_of::<T>() == 0 {
            NonNull::dangling()
        } else {
            let layout = Layout::new::<MaybeUninit<T>>();
            let ptr = Mallocator::allocate(layout).cast::<MaybeUninit<T>>();
            NonNull::new(ptr)?
        };

        Some(MallocBox { ptr })
    }

    #[inline(always)]
    pub fn new_uninit() -> MallocBox<MaybeUninit<T>> {
        unwrap!(
            Self::try_new_uninit(),
            "Failed to allocate {} bytes",
            core::mem::size_of::<T>()
        )
    }

    #[inline(always)]
    pub fn new(value: T) -> MallocBox<T> {
        let mut this = Self::new_uninit();
        unsafe { this.ptr.as_mut().write(value) };
        this.assume_init()
    }
}

impl<T> MallocBox<MaybeUninit<T>> {
    #[inline(always)]
    pub fn assume_init(self) -> MallocBox<T> {
        let ptr = self.ptr.cast::<T>();
        core::mem::forget(self);
        MallocBox { ptr }
    }
}

impl<T> core::ops::Deref for MallocBox<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { self.ptr.as_ref() }
    }
}

impl<T> core::ops::DerefMut for MallocBox<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.ptr.as_mut() }
    }
}

impl<T> Drop for MallocBox<T> {
    fn drop(&mut self) {
        Mallocator::deallocate(self.ptr.as_ptr().cast(), Layout::new::<T>());
    }
}
