//! Supply window passed to the SDIO crate's CMD5 constructor.

/// The constructor shifts its 9-bit voltage argument into OCR bits 15..23.
pub(super) const fn voltage_window(card_ocr: u32) -> u16 {
    const SUPPORTED: u32 = (1 << 20) | (1 << 21);
    ((card_ocr & SUPPORTED) >> 15) as u16
}
