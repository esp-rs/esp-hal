//! A `no_std` heap allocator for RISC-V and Xtensa processors from
//! Espressif. Supports all currently available ESP32 devices.
//!
//! **NOTE:** using this as your global allocator requires using Rust 1.68 or
//! greater, or the `nightly` release channel.
//!
//! # Using this as your Global Allocator
//!
//! ```rust
//! use esp_alloc as _;
//!
//! fn init_heap() {
//!     const HEAP_SIZE: usize = 32 * 1024;
//!     static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();
//!
//!     unsafe {
//!         esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
//!             HEAP.as_mut_ptr() as *mut u8,
//!             HEAP_SIZE,
//!             esp_alloc::MemoryCapability::Internal.into(),
//!         ));
//!     }
//! }
//! ```
//!
//! # Using this with the nightly `allocator_api`-feature
//! Sometimes you want to have more control over allocations.
//!
//! For that, it's convenient to use the nightly `allocator_api`-feature,
//! which allows you to specify an allocator for single allocations.
//!
//! **NOTE:** To use this, you have to enable the crate's `nightly` feature
//! flag.
//!
//! Create and initialize an allocator to use in single allocations:
//! ```rust
//! static PSRAM_ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
//!
//! fn init_psram_heap() {
//!     unsafe {
//!         PSRAM_ALLOCATOR.add_region(esp_alloc::HeapRegion::new(
//!             psram::psram_vaddr_start() as *mut u8,
//!             psram::PSRAM_BYTES,
//!             esp_alloc::MemoryCapability::Internal.into(),
//!         ));
//!     }
//! }
//! ```
//!
//! And then use it in an allocation:
//! ```rust
//! let large_buffer: Vec<u8, _> = Vec::with_capacity_in(1048576, &PSRAM_ALLOCATOR);
//! ```
//!
//! You can also gets stats about the heap usage at anytime with:
//! ```rust
//! let stats: HeapStats = esp_alloc::HEAP.stats();
//! // HeapStats implements the Display trait, so you can pretty print the heap stats.
//! println!("{}", stats);
//! ```
//!
//! ```txt
//! HEAP INFO
//! Size: 2097152
//! Current usage: 512028
//! Max usage: 512028
//! Total freed: 0
//! Total allocated: 512028
//! Memory Layout:
//! External | ████████░░░░░░░░░░░░░░░░░░░░░░░░░░░ | Used: 512028 / Total: 2097152 (Free: 1585124)
//! Unused   | ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ |
//! Unused   | ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ |
//! ```
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![no_std]
#![cfg_attr(feature = "nightly", feature(allocator_api))]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

mod macros;

#[cfg(feature = "nightly")]
use core::alloc::{AllocError, Allocator};
use core::{
    alloc::{GlobalAlloc, Layout},
    cell::RefCell,
    fmt::Display,
    ptr::{self, NonNull},
};

use critical_section::Mutex;
use enumset::{EnumSet, EnumSetType};
use linked_list_allocator::Heap;

/// The global allocator instance
#[global_allocator]
pub static HEAP: EspHeap = EspHeap::empty();

const NON_REGION: Option<HeapRegion> = None;

const BAR_WIDTH: usize = 35;

#[derive(EnumSetType, Debug)]
/// Describes the properties of a memory region
pub enum MemoryCapability {
    /// Memory must be internal; specifically it should not disappear when
    /// flash/spiram cache is switched off
    Internal,
    /// Memory must be in SPI RAM
    External,
}

/// Stats for a heap region
#[derive(Debug)]
pub struct RegionStats {
    /// Total usable size of the heap region in bytes.
    size: usize,

    /// Currently used size of the heap region in bytes.
    used: usize,

    /// Free size of the heap region in bytes.
    free: usize,

    /// Capabilities of the memory region.
    capabilities: EnumSet<MemoryCapability>,
}

impl Display for RegionStats {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let used_blocks = BAR_WIDTH * self.used / self.size;
        let free_blocks = BAR_WIDTH - used_blocks;

        // Display Memory type
        if self.capabilities.contains(MemoryCapability::Internal) {
            write!(f, "Internal")?;
        } else if self.capabilities.contains(MemoryCapability::External) {
            write!(f, "External")?;
        } else {
            write!(f, "Unknown")?;
        }

        write!(f, " | ")?;

        // Display usage of the memory region using a bar graph.
        for _ in 0..used_blocks {
            write!(f, "█")?;
        }
        for _ in 0..free_blocks {
            write!(f, "░")?;
        }

        write!(
            f,
            " | Used: {} / Total: {} (Free: {})",
            self.used, self.size, self.free
        )
    }
}

/// A memory region to be used as heap memory
pub struct HeapRegion {
    heap: Heap,
    capabilities: EnumSet<MemoryCapability>,
}

impl HeapRegion {
    /// Create a new [HeapRegion] with the given capabilities
    ///
    /// # Safety
    ///
    /// - The supplied memory region must be available for the entire program
    ///   (`'static`).
    /// - The supplied memory region must be exclusively available to the heap
    ///   only, no aliasing.
    /// - `size > 0`.
    pub unsafe fn new(
        heap_bottom: *mut u8,
        size: usize,
        capabilities: EnumSet<MemoryCapability>,
    ) -> Self {
        let mut heap = Heap::empty();
        heap.init(heap_bottom, size);

        Self { heap, capabilities }
    }

    /// Return stats for the current memory region
    pub fn stats(&self) -> RegionStats {
        RegionStats {
            size: self.heap.size(),
            used: self.heap.used(),
            free: self.heap.free(),
            capabilities: self.capabilities,
        }
    }
}

/// Stats for a heap allocator
///
/// Enable the "internal-heap-stats" feature if you want collect additional heap
/// informations at the cost of extra cpu time during every alloc/dealloc.
#[derive(Debug)]
pub struct HeapStats {
    /// Granular stats for all the configured memory regions.
    region_stats: [Option<RegionStats>; 3],

    /// Total size of all combined heap regions in bytes.
    size: usize,

    /// Current usage of the heap across all configured regions in bytes.
    current_usage: usize,

    /// Estimation of the max used heap in bytes.
    #[cfg(feature = "internal-heap-stats")]
    max_usage: usize,

    /// Estimation of the total allocated bytes since initialization.
    #[cfg(feature = "internal-heap-stats")]
    total_allocated: usize,

    /// Estimation of the total freed bytes since initialization.
    #[cfg(feature = "internal-heap-stats")]
    total_freed: usize,
}

impl Display for HeapStats {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "HEAP INFO")?;
        writeln!(f, "Size: {}", self.size)?;
        writeln!(f, "Current usage: {}", self.current_usage)?;
        #[cfg(feature = "internal-heap-stats")]
        {
            writeln!(f, "Max usage: {}", self.max_usage)?;
            writeln!(f, "Total freed: {}", self.total_freed)?;
            writeln!(f, "Total allocated: {}", self.total_allocated)?;
        }
        writeln!(f, "Memory Layout: ")?;
        for region in self.region_stats.iter() {
            if let Some(region) = region.as_ref() {
                region.fmt(f)?;
                writeln!(f)?;
            } else {
                // Display unused memory regions
                write!(f, "Unused   | ")?;
                for _ in 0..BAR_WIDTH {
                    write!(f, "░")?;
                }
                writeln!(f, " |")?;
            }
        }
        Ok(())
    }
}

/// Internal stats to keep track across multiple regions.
#[cfg(feature = "internal-heap-stats")]
struct InternalHeapStats {
    max_usage: usize,
    total_allocated: usize,
    total_freed: usize,
}

/// A memory allocator
///
/// In addition to what Rust's memory allocator can do it allows to allocate
/// memory in regions satisfying specific needs.
pub struct EspHeap {
    heap: Mutex<RefCell<[Option<HeapRegion>; 3]>>,
    #[cfg(feature = "internal-heap-stats")]
    internal_heap_stats: Mutex<RefCell<InternalHeapStats>>,
}

impl EspHeap {
    /// Crate a new UNINITIALIZED heap allocator
    pub const fn empty() -> Self {
        EspHeap {
            heap: Mutex::new(RefCell::new([NON_REGION; 3])),
            #[cfg(feature = "internal-heap-stats")]
            internal_heap_stats: Mutex::new(RefCell::new(InternalHeapStats {
                max_usage: 0,
                total_allocated: 0,
                total_freed: 0,
            })),
        }
    }

    /// Add a memory region to the heap
    ///
    /// `heap_bottom` is a pointer to the location of the bottom of the heap.
    ///
    /// `size` is the size of the heap in bytes.
    ///
    /// You can add up to three regions per allocator.
    ///
    /// Note that:
    ///
    /// - Memory is allocated from the first suitable memory region first
    ///
    /// - The heap grows "upwards", towards larger addresses. Thus `end_addr`
    ///   must be larger than `start_addr`
    ///
    /// - The size of the heap is `(end_addr as usize) - (start_addr as usize)`.
    ///   The allocator won't use the byte at `end_addr`.
    ///
    /// # Safety
    ///
    /// - The supplied memory region must be available for the entire program (a
    ///   `'static` lifetime).
    /// - The supplied memory region must be exclusively available to the heap
    ///   only, no aliasing.
    /// - `size > 0`.
    pub unsafe fn add_region(&self, region: HeapRegion) {
        critical_section::with(|cs| {
            let mut regions = self.heap.borrow_ref_mut(cs);
            let free = regions
                .iter()
                .enumerate()
                .find(|v| v.1.is_none())
                .map(|v| v.0);

            if let Some(free) = free {
                regions[free] = Some(region);
            } else {
                panic!(
                    "Exceeded the maximum of {} heap memory regions",
                    regions.len()
                );
            }
        });
    }

    /// Returns an estimate of the amount of bytes in use in all memory regions.
    pub fn used(&self) -> usize {
        critical_section::with(|cs| {
            let regions = self.heap.borrow_ref(cs);
            let mut used = 0;
            for region in regions.iter() {
                if let Some(region) = region.as_ref() {
                    used += region.heap.used();
                }
            }
            used
        })
    }

    /// Return usage stats for the [Heap].
    ///
    /// Note:
    /// [HeapStats] directly implements [Display], so this function can be
    /// called from within `println!()` to pretty-print the usage of the
    /// heap.
    pub fn stats(&self) -> HeapStats {
        const EMPTY_REGION_STAT: Option<RegionStats> = None;
        let mut region_stats: [Option<RegionStats>; 3] = [EMPTY_REGION_STAT; 3];

        critical_section::with(|cs| {
            let mut used = 0;
            let mut free = 0;
            let regions = self.heap.borrow_ref(cs);
            for (id, region) in regions.iter().enumerate() {
                if let Some(region) = region.as_ref() {
                    let stats = region.stats();
                    free += stats.free;
                    used += stats.used;
                    region_stats[id] = Some(region.stats());
                }
            }

            cfg_if::cfg_if! {
                if #[cfg(feature = "internal-heap-stats")] {
                    let internal_heap_stats = self.internal_heap_stats.borrow_ref(cs);
                    HeapStats {
                        region_stats,
                        size: free + used,
                        current_usage: used,
                        max_usage: internal_heap_stats.max_usage,
                        total_allocated: internal_heap_stats.total_allocated,
                        total_freed: internal_heap_stats.total_freed,
                    }
                } else {
                    HeapStats {
                        region_stats,
                        size: free + used,
                        current_usage: used,
                    }
                }
            }
        })
    }

    /// Returns an estimate of the amount of bytes available.
    pub fn free(&self) -> usize {
        self.free_caps(EnumSet::empty())
    }

    /// The free heap satisfying the given requirements
    pub fn free_caps(&self, capabilities: EnumSet<MemoryCapability>) -> usize {
        critical_section::with(|cs| {
            let regions = self.heap.borrow_ref(cs);
            let mut free = 0;
            for region in regions.iter().filter(|region| {
                if region.is_some() {
                    region
                        .as_ref()
                        .unwrap()
                        .capabilities
                        .is_superset(capabilities)
                } else {
                    false
                }
            }) {
                if let Some(region) = region.as_ref() {
                    free += region.heap.free();
                }
            }
            free
        })
    }

    /// Allocate memory in a region satisfying the given requirements.
    ///
    /// # Safety
    ///
    /// This function is unsafe because undefined behavior can result
    /// if the caller does not ensure that `layout` has non-zero size.
    ///
    /// The allocated block of memory may or may not be initialized.
    pub unsafe fn alloc_caps(
        &self,
        capabilities: EnumSet<MemoryCapability>,
        layout: Layout,
    ) -> *mut u8 {
        critical_section::with(|cs| {
            #[cfg(feature = "internal-heap-stats")]
            let before = self.used();
            let mut regions = self.heap.borrow_ref_mut(cs);
            let mut iter = (*regions).iter_mut().filter(|region| {
                if region.is_some() {
                    region
                        .as_ref()
                        .unwrap()
                        .capabilities
                        .is_superset(capabilities)
                } else {
                    false
                }
            });

            let res = loop {
                if let Some(Some(region)) = iter.next() {
                    let res = region.heap.allocate_first_fit(layout);
                    if let Ok(res) = res {
                        break Some(res);
                    }
                } else {
                    break None;
                }
            };

            res.map_or(ptr::null_mut(), |allocation| {
                #[cfg(feature = "internal-heap-stats")]
                {
                    let mut internal_heap_stats = self.internal_heap_stats.borrow_ref_mut(cs);
                    drop(regions);
                    // We need to call used because [linked_list_allocator::Heap] does internal size
                    // alignment so we cannot use the size provided by the layout.
                    let used = self.used();

                    internal_heap_stats.total_allocated += used - before;
                    internal_heap_stats.max_usage =
                        core::cmp::max(internal_heap_stats.max_usage, used);
                }

                allocation.as_ptr()
            })
        })
    }
}

unsafe impl GlobalAlloc for EspHeap {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        self.alloc_caps(EnumSet::empty(), layout)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        if ptr.is_null() {
            return;
        }

        critical_section::with(|cs| {
            #[cfg(feature = "internal-heap-stats")]
            let before = self.used();
            let mut regions = self.heap.borrow_ref_mut(cs);
            let mut iter = (*regions).iter_mut();

            while let Some(Some(region)) = iter.next() {
                if region.heap.bottom() <= ptr && region.heap.top() >= ptr {
                    region.heap.deallocate(NonNull::new_unchecked(ptr), layout);
                }
            }

            #[cfg(feature = "internal-heap-stats")]
            {
                let mut internal_heap_stats = self.internal_heap_stats.borrow_ref_mut(cs);
                drop(regions);
                // We need to call used because [linked_list_allocator::Heap] does internal size
                // alignment so we cannot use the size provided by the layout.
                internal_heap_stats.total_freed += before - self.used();
            }
        })
    }
}

#[cfg(feature = "nightly")]
unsafe impl Allocator for EspHeap {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        let raw_ptr = unsafe { self.alloc(layout) };

        if raw_ptr.is_null() {
            return Err(AllocError);
        }

        let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
        Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        self.dealloc(ptr.as_ptr(), layout);
    }
}
