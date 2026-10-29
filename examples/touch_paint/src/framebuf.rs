//! Drawing into the RGB565 framebuffer.

use core::ops::Range;

use embedded_graphics::{pixelcolor::Rgb565, prelude::*, primitives::Rectangle};

use crate::{
    display::{HEIGHT, WIDTH},
    ui::TOOLBAR_HEIGHT,
};

/// The framebuffer as a drawing target. It records which rows it changed, so that
/// only those rows need a cache writeback.
pub struct FrameBuf<'a> {
    bytes: &'a mut [u8],
    dirty: Range<usize>,
}

impl<'a> FrameBuf<'a> {
    pub fn new(bytes: &'a mut [u8]) -> Self {
        Self { bytes, dirty: 0..0 }
    }

    /// Rows written since this `FrameBuf` was created.
    pub fn dirty_rows(&self) -> Range<usize> {
        self.dirty.clone()
    }

    fn mark_dirty(&mut self, rows: Range<usize>) {
        if rows.is_empty() {
            return;
        }
        self.dirty = if self.dirty.is_empty() {
            rows
        } else {
            self.dirty.start.min(rows.start)..self.dirty.end.max(rows.end)
        };
    }

    fn set_pixel(&mut self, x: i32, y: i32, color: Rgb565) {
        if x < 0 || x >= WIDTH as i32 || y < 0 || y >= HEIGHT as i32 {
            return;
        }
        let offset = (y as usize * WIDTH + x as usize) * 2;
        self.bytes[offset..offset + 2].copy_from_slice(&color.into_storage().to_le_bytes());
        self.mark_dirty(y as usize..y as usize + 1);
    }

    pub fn fill_rect(&mut self, x: i32, y: i32, width: i32, height: i32, color: Rgb565) {
        let x0 = x.clamp(0, WIDTH as i32) as usize;
        let y0 = y.clamp(0, HEIGHT as i32) as usize;
        let x1 = (x + width).clamp(0, WIDTH as i32) as usize;
        let y1 = (y + height).clamp(0, HEIGHT as i32) as usize;
        if x0 < x1 {
            self.mark_dirty(y0..y1);
        }
        let raw = color.into_storage();
        let word = raw as u32 | ((raw as u32) << 16);

        for row in y0..y1 {
            let row = &mut self.bytes[(row * WIDTH + x0) * 2..(row * WIDTH + x1) * 2];
            let (prefix, middle, suffix) = unsafe { row.align_to_mut::<u32>() };
            prefix.as_chunks_mut::<2>().0.fill(raw.to_le_bytes());
            middle.fill(word);
            suffix.as_chunks_mut::<2>().0.fill(raw.to_le_bytes());
        }
    }

    pub fn stroke_rect(&mut self, x: i32, y: i32, width: i32, height: i32, color: Rgb565) {
        self.fill_rect(x, y, width, 2, color);
        self.fill_rect(x, y + height - 2, width, 2, color);
        self.fill_rect(x, y, 2, height, color);
        self.fill_rect(x + width - 2, y, 2, height, color);
    }

    pub fn draw_brush(&mut self, x: i32, y: i32, radius: i32, color: Rgb565) {
        for dy in -radius..=radius {
            // Brush strokes stay below the toolbar.
            if y + dy < TOOLBAR_HEIGHT {
                continue;
            }
            let half_width = libm::sqrtf((radius * radius - dy * dy) as f32) as i32;
            self.fill_rect(x - half_width, y + dy, half_width * 2 + 1, 1, color);
        }
    }

    pub fn draw_line(&mut self, from: (i32, i32), to: (i32, i32), radius: i32, color: Rgb565) {
        let dx = to.0 - from.0;
        let dy = to.1 - from.1;
        let steps = dx.abs().max(dy.abs()).max(1);
        for step in 0..=steps {
            let x = from.0 + dx * step / steps;
            let y = from.1 + dy * step / steps;
            self.draw_brush(x, y, radius, color);
        }
    }

    pub fn fill_triangle(
        &mut self,
        center_x: i32,
        peak_y: i32,
        half_base: i32,
        height: i32,
        color: Rgb565,
    ) {
        for dy in 0..height {
            let half_width = half_base * dy / height;
            self.fill_rect(
                center_x - half_width,
                peak_y + dy,
                half_width * 2 + 1,
                1,
                color,
            );
        }
    }
}

impl OriginDimensions for FrameBuf<'_> {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for FrameBuf<'_> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            self.set_pixel(point.x, point.y, color);
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_rect(
            area.top_left.x,
            area.top_left.y,
            area.size.width as i32,
            area.size.height as i32,
            color,
        );
        Ok(())
    }
}
