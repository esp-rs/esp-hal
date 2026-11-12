use core::{alloc::Layout, ptr::NonNull};

use rlsf::{GRANULARITY, Tlsf};

// The largest block, `(GRANULARITY << FLLEN) - GRANULARITY` bytes, is just below `isize::MAX`, the
// largest size a `Layout` allows.
const FLLEN: usize = (usize::BITS - GRANULARITY.trailing_zeros() - 1) as usize;

type Heap = Tlsf<'static, usize, usize, FLLEN, { usize::BITS as usize }>;

pub(crate) struct TlsfHeap {
    // The control structure is several KB. It lives at the start of the region it manages, so
    // unused region slots do not cost static memory.
    heap: &'static mut Heap,
    pool_start: usize,
    pool_end: usize,
}

impl TlsfHeap {
    pub unsafe fn new(heap_bottom: *mut u8, size: usize) -> Self {
        let heap_offset = heap_bottom.align_offset(align_of::<Heap>());
        let pool_offset = heap_offset + size_of::<Heap>();
        assert!(
            pool_offset < size,
            "The heap region is too small for the TLSF control structure"
        );

        let heap = unsafe {
            let heap = heap_bottom.add(heap_offset).cast::<Heap>();
            heap.write(Heap::new());
            &mut *heap
        };

        let pool_bottom = unsafe { heap_bottom.add(pool_offset) };
        let block = unsafe {
            NonNull::slice_from_raw_parts(NonNull::new_unchecked(pool_bottom), size - pool_offset)
        };
        let actual_size = unsafe { heap.insert_free_block_ptr(block).unwrap() };

        Self {
            heap,
            pool_start: pool_bottom as usize,
            pool_end: pool_bottom as usize + actual_size.get(),
        }
    }

    pub fn size(&self) -> usize {
        self.pool_end - self.pool_start
    }

    pub fn used(&self) -> usize {
        let mut used = 0;
        let pool =
            unsafe { core::slice::from_raw_parts(self.pool_start as *const u8, self.size()) };
        for block in unsafe { self.heap.iter_blocks(NonNull::from(pool)) } {
            if block.is_occupied() {
                used += block.size();
            }
        }
        used
    }

    pub fn free(&self) -> usize {
        let mut free = 0;
        let pool =
            unsafe { core::slice::from_raw_parts(self.pool_start as *const u8, self.size()) };
        for block in unsafe { self.heap.iter_blocks(NonNull::from(pool)) } {
            if !block.is_occupied() {
                free += block.max_payload_size();
            }
        }
        free
    }

    pub fn allocate(&mut self, layout: Layout) -> Option<NonNull<u8>> {
        self.heap.allocate(layout)
    }

    pub(crate) unsafe fn try_deallocate(&mut self, ptr: NonNull<u8>, layout: Layout) -> bool {
        let addr = ptr.addr().get();
        if self.pool_start <= addr && self.pool_end > addr {
            unsafe { self.heap.deallocate(ptr, layout.align()) };
            true
        } else {
            false
        }
    }
}
