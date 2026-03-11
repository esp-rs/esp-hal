#[cfg(heap_algorithm_llff)]
mod llff;
#[cfg(heap_algorithm_tlsf)]
mod tlsf;

#[cfg(heap_algorithm_llff)]
pub(crate) use llff::LlffHeap as Heap;
#[cfg(heap_algorithm_tlsf)]
pub(crate) use tlsf::TlsfHeap as Heap;
