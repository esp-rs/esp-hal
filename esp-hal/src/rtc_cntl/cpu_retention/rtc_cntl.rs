//! CPU retention through the RTC_CNTL retention DMA.

const BUFFER_SIZE: usize = super::memory::buffer_size();

/// Bytes the DMA descriptor takes at the head of a retention buffer.
pub(crate) const DMA_LINK_SIZE: usize = 16;

/// Bytes the DMA moves out of a CPU retention buffer, not counting the descriptor.
pub(crate) const fn payload_size() -> usize {
    BUFFER_SIZE - DMA_LINK_SIZE
}

/// RTC_CNTL retention DMA link node. Matches `lldesc_t` in `esp_rom_lldesc.h`.
#[repr(C)]
struct RtcCntlDmaLink {
    word0: u32,
    buf: *mut u8,
    next: u32,
}

/// Writes the descriptor at the head of the buffer and returns the payload that follows it.
///
/// # Safety
///
/// `buffer` must be valid for `DMA_LINK_SIZE + payload_size` bytes, and aligned as the DMA needs.
pub(crate) unsafe fn init_link(buffer: *mut u8, payload_size: usize) -> *mut u8 {
    unsafe {
        let link = buffer.cast::<RtcCntlDmaLink>();
        let payload = buffer.add(DMA_LINK_SIZE);
        let units = (payload_size >> 4) as u32;

        // `lldesc_t` first word, from `rom/lldesc.h`: size in bits 0..12, length in 12..24, `eof`
        // at bit 30 for the only node in the list, `owner` at bit 31 for the DMA. Both counts are
        // in 16-byte units, as `rtc_cntl_hal_dma_link_init` writes them.
        let word0 = units | (units << 12) | (1 << 30) | (1 << 31);

        core::ptr::write_volatile(&raw mut (*link).word0, word0);
        core::ptr::write_volatile(&raw mut (*link).buf, payload);
        core::ptr::write_volatile(&raw mut (*link).next, 0);

        payload
    }
}

/// Writes the descriptor and the four configuration words that the CPU frames begin with.
///
/// The chips agree on every word but the last, which `config_word3` supplies.
///
/// # Safety
///
/// `buffer` must be the installed CPU retention buffer.
pub(crate) unsafe fn init_cpu_dma_link(buffer: *mut u8, config_word3: u32) {
    unsafe {
        let cfg = init_link(buffer, payload_size()).cast::<u32>();

        core::ptr::write_volatile(cfg, 0);
        core::ptr::write_volatile(cfg.add(1), 0);
        core::ptr::write_volatile(cfg.add(2), 0);
        core::ptr::write_volatile(cfg.add(3), config_word3);
    }
}
