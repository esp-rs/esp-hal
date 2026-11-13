//! Drawing primitives for panel rows: boxes with round corners and text masks. All of
//! them work on one row of RGB565 pixels at a time, with the high byte first.

use alloc::{vec, vec::Vec};

use embedded_graphics::{
    Pixel,
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};
use esp_hal::ram;

use crate::display::{HEIGHT, WIDTH};

pub const WHITE: u16 = rgb565(255, 255, 255);
pub const BLACK: u16 = 0;

/// A glass box with round corners: each pixel becomes half the box's color and half the
/// pixel under it. A circle is a square box with a corner radius of half its size. With
/// a border above zero, only a ring of that width along the edge is painted.
#[derive(Clone, Copy)]
pub struct Shape {
    pub left: i32,
    pub top: i32,
    /// Exclusive.
    pub right: i32,
    /// Exclusive.
    pub bottom: i32,
    pub radius: i32,
    pub border: i32,
    pub color: u16,
}

impl Shape {
    pub const fn rounded(x: i32, y: i32, width: i32, height: i32, color: u16) -> Self {
        Self {
            left: x,
            top: y,
            right: x + width,
            bottom: y + height,
            radius: if width < height { width } else { height } / 2,
            border: 0,
            color,
        }
    }

    pub const fn circle(x: i32, y: i32, radius: i32, color: u16) -> Self {
        Self::rounded(
            x - radius,
            y - radius,
            2 * radius + 1,
            2 * radius + 1,
            color,
        )
    }

    pub const fn with_border(self, border: i32) -> Self {
        Self { border, ..self }
    }

    /// Whether the point is in the bounding box, or at most `slack` pixels outside it.
    pub fn hit(&self, x: i32, y: i32, slack: i32) -> bool {
        (self.left - slack..self.right + slack).contains(&x)
            && (self.top - slack..self.bottom + slack).contains(&y)
    }

    /// Paints the shape's part of panel row `y`.
    #[ram]
    pub fn draw_row(&self, y: i32, row: &mut [u16]) {
        let Some((x0, x1)) = span(self.left, self.top, self.right, self.bottom, self.radius, y)
        else {
            return;
        };
        let b = self.border;
        let hole = if b > 0 {
            span(
                self.left + b,
                self.top + b,
                self.right - b,
                self.bottom - b,
                (self.radius - b).max(0),
                y,
            )
        } else {
            None
        };
        match hole {
            Some((h0, h1)) => {
                self.paint_span(row, x0, h0);
                self.paint_span(row, h1, x1);
            }
            None => self.paint_span(row, x0, x1),
        }
    }

    fn paint_span(&self, row: &mut [u16], x0: i32, x1: i32) {
        let x0 = x0.clamp(0, WIDTH as i32) as usize;
        let x1 = x1.clamp(0, WIDTH as i32) as usize;
        if x0 < x1 {
            for pixel in &mut row[x0..x1] {
                *pixel = half_mix(*pixel, self.color);
            }
        }
    }
}

/// The columns that a box with round corners covers in row `y`.
#[ram]
fn span(left: i32, top: i32, right: i32, bottom: i32, radius: i32, y: i32) -> Option<(i32, i32)> {
    if y < top || y >= bottom || left >= right {
        return None;
    }
    let dy = if y < top + radius {
        top + radius - y
    } else if y >= bottom - radius {
        y - (bottom - radius - 1)
    } else {
        0
    };
    let inset = radius - isqrt(radius * radius - dy * dy);
    Some((left + inset, right - inset))
}

/// A one-color picture with one bit per pixel, for text.
pub struct Mask {
    x: i32,
    y: i32,
    width: i32,
    height: i32,
    words_per_row: usize,
    bits: Vec<u32>,
    color: u16,
}

impl Mask {
    /// An empty mask that covers the given panel area.
    pub fn new(x: i32, y: i32, width: i32, height: i32, color: u16) -> Self {
        let words_per_row = (width as usize).div_ceil(32);
        Self {
            x,
            y,
            width,
            height,
            words_per_row,
            bits: vec![0; words_per_row * height as usize],
            color,
        }
    }

    /// A mask with one line of text, centered on the point.
    pub fn text(text: &str, x: i32, y: i32, color: u16) -> Self {
        let (width, height) = text_size(text);
        let mut mask = Self::new(
            x - width / 2 - 1,
            y - height / 2 - 1,
            width + 2,
            height + 2,
            color,
        );
        mask.write(text, x, y, Alignment::Center);
        mask
    }

    /// Writes one line of text, aligned on `x` and centered on `y`.
    pub fn write(&mut self, text: &str, x: i32, y: i32, alignment: Alignment) {
        let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let text_style = TextStyleBuilder::new()
            .alignment(alignment)
            .baseline(Baseline::Middle)
            .build();
        let _ = Text::with_text_style(text, Point::new(x, y), style, text_style).draw(self);
    }

    /// Paints the mask's part of panel row `y`.
    #[ram]
    pub fn draw_row(&self, y: i32, row: &mut [u16]) {
        let line = y - self.y;
        if !(0..self.height).contains(&line) {
            return;
        }
        let words = &self.bits[line as usize * self.words_per_row..][..self.words_per_row];
        for (index, &word) in words.iter().enumerate() {
            let mut word = word;
            while word != 0 {
                let x = self.x + (index * 32) as i32 + word.trailing_zeros() as i32;
                if (0..WIDTH as i32).contains(&x) {
                    row[x as usize] = self.color;
                }
                word &= word - 1;
            }
        }
    }
}

/// Draws in panel coordinates. Pixels outside the mask are dropped.
impl DrawTarget for Mask {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            let (x, y) = (point.x - self.x, point.y - self.y);
            if color.is_on() && (0..self.width).contains(&x) && (0..self.height).contains(&y) {
                let index = y as usize * self.words_per_row + x as usize / 32;
                self.bits[index] |= 1 << (x % 32);
            }
        }
        Ok(())
    }
}

impl OriginDimensions for Mask {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

/// White text on a dark glass pill, centered on the point.
pub struct Label {
    backdrop: Shape,
    text: Mask,
}

impl Label {
    pub fn new(text: &str, x: i32, y: i32) -> Self {
        let (width, height) = text_size(text);
        let (width, height) = (width + 28, height + 12);
        Self {
            backdrop: Shape::rounded(x - width / 2, y - height / 2, width, height, BLACK),
            text: Mask::text(text, x, y, WHITE),
        }
    }

    #[ram]
    pub fn draw_row(&self, y: i32, row: &mut [u16]) {
        self.backdrop.draw_row(y, row);
        self.text.draw_row(y, row);
    }
}

/// The size of one line of text in pixels.
fn text_size(text: &str) -> (i32, i32) {
    let size = FONT_10X20.character_size;
    (text.len() as i32 * size.width as i32, size.height as i32)
}

/// The RGB565 pixel of an 8-bit-per-channel color, with the high byte first.
pub const fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    u16::to_be(((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3))
}

/// The average of two RGB565 pixels.
fn half_mix(a: u16, b: u16) -> u16 {
    let (a, b) = (u16::from_be(a), u16::from_be(b));
    u16::to_be(((a & 0xf7de) >> 1) + ((b & 0xf7de) >> 1))
}

/// The integer square root. The scanout interrupt runs it, so it must not read a
/// lookup table from flash, as `i32::isqrt` does.
#[ram]
fn isqrt(value: i32) -> i32 {
    if value <= 0 {
        return 0;
    }
    // A power of two above the root, so that Newton's method needs few steps.
    let mut root = 1 << (32 - value.leading_zeros()).div_ceil(2);
    loop {
        let next = (root + value / root) / 2;
        if next >= root {
            return root;
        }
        root = next;
    }
}
