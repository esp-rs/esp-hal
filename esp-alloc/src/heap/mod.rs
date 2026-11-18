// The heap algorithm is an esp-config option, not a Cargo feature, so both
// allocator crates are always dependencies.
#[cfg(heap_algorithm_tlsf)]
use linked_list_allocator as _;
#[cfg(heap_algorithm_llff)]
use rlsf as _;

#[cfg(heap_algorithm_llff)]
mod llff;
#[cfg(heap_algorithm_tlsf)]
mod tlsf;

#[cfg(heap_algorithm_llff)]
pub(crate) use llff::LlffHeap as Heap;
#[cfg(heap_algorithm_tlsf)]
pub(crate) use tlsf::TlsfHeap as Heap;
