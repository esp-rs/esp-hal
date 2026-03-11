use core::{alloc::Layout, ptr::NonNull};

use linked_list_allocator::Heap;

pub(crate) struct LlffHeap {
    heap: Heap,
}

impl LlffHeap {
    pub unsafe fn new(heap_bottom: *mut u8, size: usize) -> Self {
        let mut heap = Heap::empty();
        unsafe { heap.init(heap_bottom, size) };
        Self { heap }
    }

    pub fn size(&self) -> usize {
        self.heap.size()
    }

    pub fn used(&self) -> usize {
        self.heap.used()
    }

    pub fn free(&self) -> usize {
        self.heap.free()
    }

    pub fn allocate(&mut self, layout: Layout) -> Option<NonNull<u8>> {
        self.heap.allocate_first_fit(layout).ok()
    }

    pub(crate) unsafe fn try_deallocate(&mut self, ptr: NonNull<u8>, layout: Layout) -> bool {
        if self.heap.bottom() <= ptr.as_ptr() && self.heap.top() >= ptr.as_ptr() {
            unsafe { self.heap.deallocate(ptr, layout) };
            true
        } else {
            false
        }
    }
}
