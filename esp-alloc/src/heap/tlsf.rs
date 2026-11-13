use core::{alloc::Layout, ptr::NonNull};

use esp_config::esp_config_int;
use rlsf::{GRANULARITY, Tlsf};

const MAX_ALLOCATION: usize = esp_config_int!(usize, "ESP_ALLOC_CONFIG_TLSF_MAX_ALLOCATION");

const SLLEN: usize = usize::BITS as usize;

// `Tlsf::allocate` searches for the allocation plus the block header, rounded up to
// `GRANULARITY`. It then rounds that size up to the start of the next second-level list, which can
// be up to `1 / SLLEN` larger. That list's first level must be below `FLLEN`.
const SEARCH_SIZE: usize = (MAX_ALLOCATION + GRANULARITY / 2).next_multiple_of(GRANULARITY);
const LIST_SIZE: usize =
    SEARCH_SIZE.next_multiple_of(1 << SEARCH_SIZE.ilog2().saturating_sub(SLLEN.ilog2()));
const FLLEN: usize = (LIST_SIZE.ilog2() - GRANULARITY.ilog2() + 1) as usize;

type Heap = Tlsf<'static, usize, usize, FLLEN, SLLEN>;

pub(crate) struct TlsfHeap {
    heap: Heap,
    pool_start: usize,
    pool_end: usize,
}

impl TlsfHeap {
    pub unsafe fn new(heap_bottom: *mut u8, size: usize) -> Self {
        let mut heap = Heap::new();

        let block = unsafe { core::slice::from_raw_parts(heap_bottom, size) };
        let actual_size = unsafe { heap.insert_free_block_ptr(block.into()).unwrap() };

        Self {
            heap,
            pool_start: heap_bottom as usize,
            pool_end: heap_bottom as usize + actual_size.get(),
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
