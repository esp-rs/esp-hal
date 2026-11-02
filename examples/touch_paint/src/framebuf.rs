//! The RGB565 framebuffer as an `embedded-graphics` draw target.

use core::ops::Range;

use embedded_graphics::{
    pixelcolor::{Rgb565, raw::RawU16},
    prelude::*,
};
use embedded_graphics_framebuf::{FrameBuf, backends::FrameBufferBackend};

use crate::display::{HEIGHT, WIDTH};

pub type Frame<'a> = FrameBuf<Rgb565, Pixels<'a>>;

pub fn frame(bytes: &mut [u8]) -> Frame<'_> {
    let pixels = Pixels {
        pixels: bytes.as_chunks_mut().0,
        first: usize::MAX,
        end: 0,
    };
    FrameBuf::new(pixels, WIDTH, HEIGHT)
}

/// Little-endian RGB565 pixels. They record which pixels changed, so that only
/// those rows need a cache writeback.
pub struct Pixels<'a> {
    pixels: &'a mut [[u8; 2]],
    first: usize,
    end: usize,
}

impl Pixels<'_> {
    /// Rows written since the frame was created.
    pub fn dirty_rows(&self) -> Range<usize> {
        if self.first >= self.end {
            return 0..0;
        }
        self.first / WIDTH..(self.end - 1) / WIDTH + 1
    }
}

impl FrameBufferBackend for Pixels<'_> {
    type Color = Rgb565;

    fn set(&mut self, index: usize, color: Rgb565) {
        self.pixels[index] = color.into_storage().to_le_bytes();
        self.first = self.first.min(index);
        self.end = self.end.max(index + 1);
    }

    fn get(&self, index: usize) -> Rgb565 {
        RawU16::new(u16::from_le_bytes(self.pixels[index])).into()
    }

    fn nr_elements(&self) -> usize {
        self.pixels.len()
    }
}
