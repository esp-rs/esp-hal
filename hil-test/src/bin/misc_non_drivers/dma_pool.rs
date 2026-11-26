//! The functions return prefixes of one static buffer and one descriptor list. The DMA tests
//! share them, so the binary needs memory for the largest test only.
//!
//! A second call panics. Call one function once after each reset.

use esp_hal::{
    dma::{DmaDescriptor, aligned::DmaAlignedMut, descriptor_count},
    dma_burst_aligned_buffer,
    dma_descriptors_impl,
};

const POOL_SIZE: usize = 20 * 1024;
const POOL_DESCRIPTORS: usize = 8;

// Size128 is the largest burst. The static and the second buffer both start on that boundary.
#[cfg(dma_supports_mem2mem)]
const POOL_ALIGN: usize = esp_hal::dma::BurstSize::Size128.bytes();

fn pool() -> (&'static mut [u8], &'static mut [DmaDescriptor]) {
    let buffer = dma_burst_aligned_buffer!(Size128, POOL_SIZE);
    let descriptors = dma_descriptors_impl!(POOL_SIZE, POOL_SIZE.div_ceil(POOL_DESCRIPTORS));
    (buffer.into_inner(), descriptors.into_inner())
}

pub fn dma_buffer(
    size: usize,
    chunk_size: usize,
) -> (
    DmaAlignedMut<'static, [u8]>,
    DmaAlignedMut<'static, [DmaDescriptor]>,
) {
    let (buffer, descriptors) = pool();
    let buffer = &mut buffer[..size];
    let descriptors = &mut descriptors[..descriptor_count(size, chunk_size)];
    // SAFETY: Both slices start at DMA-aligned statics. The bytes after each slice stay unused.
    unsafe {
        (
            DmaAlignedMut::new_unchecked(buffer),
            DmaAlignedMut::new_unchecked(descriptors),
        )
    }
}

#[cfg(dma_supports_mem2mem)]
pub fn dma_buffers(
    rx_size: usize,
    tx_size: usize,
    chunk_size: usize,
) -> (
    &'static mut [u8],
    &'static mut [DmaDescriptor],
    &'static mut [u8],
    &'static mut [DmaDescriptor],
) {
    let (buffer, descriptors) = pool();
    let (rx_buffer, tx_buffer) = buffer.split_at_mut(rx_size.next_multiple_of(POOL_ALIGN));
    let (rx_descriptors, tx_descriptors) =
        descriptors.split_at_mut(descriptor_count(rx_size, chunk_size));
    (
        &mut rx_buffer[..rx_size],
        rx_descriptors,
        &mut tx_buffer[..tx_size],
        &mut tx_descriptors[..descriptor_count(tx_size, chunk_size)],
    )
}
