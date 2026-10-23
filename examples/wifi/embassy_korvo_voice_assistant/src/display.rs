//! The 800x480 RGB panel of the LCD sub-board and the screen drawn on it.
//!
//! The panel has no frame memory, so the LCD peripheral sends it the frame buffer in PSRAM over and
//! over: the DMA descriptors of the frame buffer form a ring. The refresh stops while a new
//! [`Screen`] is drawn.

use alloc::string::String;
use core::convert::Infallible;

use allocator_api2::vec::Vec;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_graphics::{
    mono_font::{MonoTextStyle, iso_8859_1::FONT_10X20},
    pixelcolor::{Rgb565, raw::RawU16},
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle, RoundedRectangle},
    text::{Alignment, Text},
};
use embedded_text::{
    TextBox,
    style::{HeightMode, TextBoxStyleBuilder, VerticalOverdraw},
};
use esp_alloc::DmaCompatibleExternalMemory;
use esp_hal::{
    Blocking,
    dma::{
        BurstConfig,
        DmaDescriptor,
        DmaTxBuffer,
        ExternalBurstConfig,
        Owner,
        Preparation,
        aligned::DmaAlignedMut,
    },
    dma_descriptors_impl,
    lcd_cam::lcd::dpi::Dpi,
};

pub const WIDTH: u32 = 800;
pub const HEIGHT: u32 = 480;
const FRAME_BYTES: usize = (WIDTH * HEIGHT * 2) as usize;

/// The area of the talk button, for drawing and for touch hit tests.
pub const BUTTON: Rectangle = Rectangle::new(Point::new(250, 384), Size::new(300, 72));

const BACKGROUND: Rgb565 = Rgb565::new(2, 4, 6);
const FOREGROUND: Rgb565 = Rgb565::WHITE;
const ACCENT: Rgb565 = Rgb565::new(31, 40, 0);

/// What the screen shows.
#[derive(Clone)]
pub struct Screen {
    pub status: String,
    pub button: Button,
    pub you: String,
    pub assistant: String,
}

/// The state of the talk button.
#[derive(Clone, Copy)]
pub enum Button {
    Talk,
    Stop,
    Busy,
}

static SCREEN: Signal<CriticalSectionRawMutex, Screen> = Signal::new();

/// Replaces the screen content from the next frame on.
pub fn show(screen: &Screen) {
    SCREEN.signal(screen.clone());
}

#[embassy_executor::task]
pub async fn run(mut dpi: Dpi<'static, Blocking>) {
    let mut frame = FrameRing::new();
    loop {
        let Ok(transfer) = dpi.send(true, frame) else {
            panic!("Failed to start the display refresh");
        };
        let screen = SCREEN.wait().await;
        // A bus stall (Wi-Fi starting, for example) can starve the LCD, which then keeps going one
        // frame offset from the data. Restarting the refresh realigns the picture.
        (dpi, frame) = transfer.stop();
        draw(&mut FrameBuffer(frame.buffer.as_mut()), &screen);
        frame.buffer.writeback();
    }
}

/// A black frame buffer in PSRAM, with DMA descriptors that link back to the start.
struct FrameRing {
    descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
    buffer: DmaAlignedMut<'static, [u8]>,
}

const BURST: ExternalBurstConfig = ExternalBurstConfig::Size64;
/// The largest DMA chunk that keeps every chunk aligned to the burst size.
const CHUNK_SIZE: usize = 4096 - BURST as usize;

impl FrameRing {
    fn new() -> Self {
        let mut buffer = Vec::with_capacity_in(FRAME_BYTES, DmaCompatibleExternalMemory);
        buffer.resize(FRAME_BYTES, 0);
        let mut buffer = DmaAlignedMut::new(buffer.leak()).unwrap();
        let mut descriptors = dma_descriptors_impl!(FRAME_BYTES, CHUNK_SIZE);

        let first: *mut DmaDescriptor = &mut descriptors[0];
        let count = descriptors.len();
        for (index, (descriptor, chunk)) in descriptors
            .iter_mut()
            .zip(buffer.chunks_mut(CHUNK_SIZE))
            .enumerate()
        {
            descriptor.set_owner(Owner::Dma);
            descriptor.set_suc_eof(false);
            descriptor.set_size(chunk.len());
            descriptor.set_length(chunk.len());
            descriptor.buffer = chunk.as_mut_ptr();
            descriptor.next = if index + 1 == count {
                first
            } else {
                (descriptor as *mut DmaDescriptor).wrapping_add(1)
            };
        }
        buffer.writeback();

        Self {
            descriptors,
            buffer,
        }
    }
}

unsafe impl DmaTxBuffer for FrameRing {
    type View = FrameRing;
    type Final = FrameRing;

    fn prepare(&mut self) -> Preparation {
        Preparation {
            start: self.descriptors.as_mut_ptr(),
            accesses_psram: true,
            burst_transfer: BurstConfig::from(BURST),
            // The DMA never hands the descriptors back, so it must not check their owner.
            check_owner: Some(false),
            auto_write_back: false,
        }
    }

    fn into_view(self) -> FrameRing {
        self
    }

    fn from_view(view: FrameRing) -> FrameRing {
        view
    }
}

fn draw(frame: &mut FrameBuffer<'_>, screen: &Screen) {
    let text = MonoTextStyle::new(&FONT_10X20, FOREGROUND);
    let accent = MonoTextStyle::new(&FONT_10X20, ACCENT);

    frame.clear(BACKGROUND).unwrap();

    Text::new("esp-hal voice assistant", Point::new(24, 36), accent)
        .draw(frame)
        .unwrap();
    Text::with_alignment(&screen.status, Point::new(776, 36), text, Alignment::Right)
        .draw(frame)
        .unwrap();

    draw_message(frame, "You", &screen.you, 64, 80);
    draw_message(frame, "Assistant", &screen.assistant, 168, 200);

    let (label, color) = match screen.button {
        Button::Talk => ("Tap to talk", Rgb565::new(0, 40, 12)),
        Button::Stop => ("Tap to stop", Rgb565::new(28, 8, 4)),
        Button::Busy => ("Please wait", Rgb565::new(10, 20, 10)),
    };
    RoundedRectangle::with_equal_corners(BUTTON, Size::new(24, 24))
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(frame)
        .unwrap();
    Text::with_alignment(
        label,
        BUTTON.center() + Point::new(0, 6),
        text,
        Alignment::Center,
    )
    .draw(frame)
    .unwrap();
}

/// Draws a title and a word-wrapped message below it, cut off at `height` pixels.
fn draw_message(frame: &mut FrameBuffer<'_>, title: &str, message: &str, top: i32, height: u32) {
    let accent = MonoTextStyle::new(&FONT_10X20, ACCENT);
    Text::new(title, Point::new(24, top + 16), accent)
        .draw(frame)
        .unwrap();

    let message = to_latin1(message);
    let area = Rectangle::new(Point::new(24, top + 24), Size::new(WIDTH - 48, height));
    let style = TextBoxStyleBuilder::new()
        .height_mode(HeightMode::Exact(VerticalOverdraw::FullRowsOnly))
        .build();
    TextBox::with_textbox_style(
        &message,
        area,
        MonoTextStyle::new(&FONT_10X20, FOREGROUND),
        style,
    )
    .draw(frame)
    .unwrap();
}

/// Replaces the typographic characters the model likes to use with ones the font has.
fn to_latin1(text: &str) -> String {
    text.chars()
        .map(|c| match c {
            '\u{2018}' | '\u{2019}' => '\'',
            '\u{201C}' | '\u{201D}' => '"',
            '\u{2013}' | '\u{2014}' => '-',
            c if u32::from(c) <= 0xFF => c,
            _ => '?',
        })
        .collect()
}

/// RGB565 pixels in little-endian byte order, as the panel's data lines expect them.
struct FrameBuffer<'a>(&'a mut [u8]);

impl FrameBuffer<'_> {
    fn pixel_index(x: u32, y: u32) -> usize {
        ((y * WIDTH + x) * 2) as usize
    }
}

impl OriginDimensions for FrameBuffer<'_> {
    fn size(&self) -> Size {
        Size::new(WIDTH, HEIGHT)
    }
}

impl DrawTarget for FrameBuffer<'_> {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Infallible>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(point, color) in pixels {
            if let (Ok(x), Ok(y)) = (u32::try_from(point.x), u32::try_from(point.y))
                && x < WIDTH
                && y < HEIGHT
            {
                let index = Self::pixel_index(x, y);
                let raw = RawU16::from(color).into_inner();
                self.0[index..index + 2].copy_from_slice(&raw.to_le_bytes());
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Rgb565) -> Result<(), Infallible> {
        let area = area.intersection(&self.bounding_box());
        let Some(bottom_right) = area.bottom_right() else {
            return Ok(());
        };
        let raw = RawU16::from(color).into_inner().to_le_bytes();
        for y in area.top_left.y..=bottom_right.y {
            let start = Self::pixel_index(area.top_left.x as u32, y as u32);
            let end = Self::pixel_index(bottom_right.x as u32, y as u32) + 2;
            self.0[start..end].as_chunks_mut::<2>().0.fill(raw);
        }
        Ok(())
    }
}
