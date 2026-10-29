//! The 800x480 RGB panel of the LCD sub-board and the screen drawn on it.
//!
//! The panel has no frame memory, so the LCD peripheral refreshes it all the time from a frame
//! buffer in PSRAM. Speech recognition on the second core takes so much PSRAM bandwidth that a DMA
//! reading the frame buffer falls behind, and the picture slips sideways. So, as ESP-IDF's RGB LCD
//! driver does with bounce buffers, the DMA sends two buffers in internal RAM in turns, and an
//! interrupt copies the next lines of the frame buffer into each buffer while the DMA sends the
//! other. The refresh stops while a new [`Screen`] is drawn.

use alloc::string::String;
use core::{
    cell::UnsafeCell,
    convert::Infallible,
    sync::atomic::{AtomicBool, Ordering},
};

use allocator_api2::vec::Vec;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, with_timeout};
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
use esp_alloc::ExternalMemory;
use esp_hal::{
    Blocking,
    dma::{
        BurstConfig,
        DmaDescriptor,
        DmaTxBuffer,
        Owner,
        Preparation,
        aligned::{DmaAlignedMut, InternalMemory},
    },
    dma_descriptors_impl,
    handler,
    interrupt::{self, Priority},
    lcd_cam::lcd::dpi::Dpi,
    peripherals::Interrupt,
};
use static_cell::ConstStaticCell;

pub const WIDTH: u32 = 800;
pub const HEIGHT: u32 = 480;
const LINE_BYTES: usize = WIDTH as usize * 2;
const FRAME_BYTES: usize = LINE_BYTES * HEIGHT as usize;

/// The lines in a bounce buffer. A refill must be done before the DMA comes back to the buffer,
/// within about 14 lines of 928 pixel clocks at 18 MHz, about 0.7 ms.
const BOUNCE_LINES: usize = 16;
const BOUNCE_BYTES: usize = BOUNCE_LINES * LINE_BYTES;
const _: () = assert!(
    (HEIGHT as usize).is_multiple_of(2 * BOUNCE_LINES),
    "Each frame must start with the first bounce buffer"
);
const REFILLS_PER_FRAME: usize = HEIGHT as usize / BOUNCE_LINES;
/// Two lines per DMA descriptor, which holds at most 4095 bytes.
const DESCRIPTOR_BYTES: usize = 2 * LINE_BYTES;
const DESCRIPTORS_PER_BUFFER: usize = BOUNCE_BYTES / DESCRIPTOR_BYTES;

// Registers that esp-hal does not expose. The DMA ones are those of `DMA_AXI_CH0`, which `main`
// gives the display.
/// LCD_CAM `LC_DMA_INT_RAW` and `LC_DMA_INT_CLR`.
const LCD_INT_RAW: *mut u32 = 0x2039_6068 as _;
const LCD_INT_CLR: *mut u32 = 0x2039_6070 as _;
/// The LCD finished a frame.
const LCD_VSYNC: u32 = 1 << 0;
/// AXI-DMA `OUT_INT_ENA_CH0`, `OUT_INT_CLR_CH0` and `OUT_DSCR_CH0`, the descriptor the DMA reads.
const DMA_OUT_INT_ENA: *mut u32 = 0x2034_8140 as _;
const DMA_OUT_INT_CLR: *mut u32 = 0x2034_8144 as _;
const DMA_OUT_DESCRIPTOR: *mut u32 = 0x2034_816C as _;
/// The DMA has read a descriptor. Refills do not use the EOF flag of descriptors, because each EOF
/// makes the LCD pause, and the LCD then gets a third of the pixel data it needs.
const DMA_OUT_DONE: u32 = 1 << 0;

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
    let mut frame = Vec::with_capacity_in(FRAME_BYTES, ExternalMemory);
    frame.resize(FRAME_BYTES, 0);
    let frame = frame.leak();
    let mut ring = BounceRing::new();
    interrupt::bind_handler(Interrupt::AXI_PDMA_OUT_CH0, refill);
    loop {
        // SAFETY: The DMA is stopped and the refill interrupt is off.
        unsafe { REFILL.get().start(frame) };
        let Ok(transfer) = dpi.send(true, ring) else {
            panic!("Failed to start the display refresh");
        };
        listen_for_refills(true);
        let screen = next_screen().await;
        listen_for_refills(false);
        (dpi, ring) = transfer.stop();
        if let Some(screen) = screen {
            draw(&mut FrameBuffer(frame), &screen);
        }
    }
}

/// Set when a frame had too few refills and the picture is out of step with the bounce buffers.
/// This happens when the refill interrupt waits longer than a bounce buffer lasts, for example
/// while Wi-Fi starts.
static OUT_OF_STEP: AtomicBool = AtomicBool::new(false);

/// Waits for a new screen, or returns `None` when the picture is out of step. Restarting the
/// refresh realigns it.
async fn next_screen() -> Option<Screen> {
    loop {
        if let Ok(screen) = with_timeout(Duration::from_millis(100), SCREEN.wait()).await {
            return Some(screen);
        }
        if OUT_OF_STEP.load(Ordering::Relaxed) {
            return None;
        }
    }
}

fn listen_for_refills(enable: bool) {
    // SAFETY: Only the display uses the interrupts of its DMA channel.
    unsafe {
        DMA_OUT_INT_CLR.write_volatile(DMA_OUT_DONE);
        let enabled = DMA_OUT_INT_ENA.read_volatile() & !DMA_OUT_DONE;
        DMA_OUT_INT_ENA.write_volatile(enabled | if enable { DMA_OUT_DONE } else { 0 });
    }
}

#[handler(priority = Priority::max())]
fn refill() {
    // SAFETY: Clears the flags this interrupt handles, and reads the position of the DMA.
    let (descriptor, vsync) = unsafe {
        DMA_OUT_INT_CLR.write_volatile(DMA_OUT_DONE);
        let vsync = LCD_INT_RAW.read_volatile() & LCD_VSYNC != 0;
        if vsync {
            LCD_INT_CLR.write_volatile(LCD_VSYNC);
        }
        (DMA_OUT_DESCRIPTOR.read_volatile() as usize, vsync)
    };
    // SAFETY: The interrupt only runs while the refresh does.
    unsafe { REFILL.get().advance(descriptor, vsync) };
}

/// The two bounce buffers and what goes into them next.
struct Refill {
    buffers: Option<DmaAlignedMut<'static, [[u8; BOUNCE_BYTES]; 2]>>,
    first_descriptor: usize,
    frame: *const u8,
    /// The frame line that goes into each buffer next.
    next_lines: [usize; 2],
    /// The buffer the DMA reads.
    reading: usize,
    /// The refills since the last VSYNC, from the first VSYNC on.
    refills: Option<usize>,
}

/// The refill state, which the display task uses while the refresh is stopped, and the refill
/// interrupt while it runs.
struct RefillCell(UnsafeCell<Refill>);

// SAFETY: See `get`.
unsafe impl Sync for RefillCell {}

impl RefillCell {
    /// # Safety
    ///
    /// Only the display task while the refresh is stopped, or the refill interrupt, may call this.
    #[allow(clippy::mut_from_ref)]
    unsafe fn get(&self) -> &mut Refill {
        unsafe { &mut *self.0.get() }
    }
}

static REFILL: RefillCell = RefillCell(UnsafeCell::new(Refill {
    buffers: None,
    first_descriptor: 0,
    frame: core::ptr::null(),
    next_lines: [0; 2],
    reading: 0,
    refills: None,
}));

impl Refill {
    /// Fills both buffers with the first lines of `frame`.
    fn start(&mut self, frame: &[u8]) {
        self.frame = frame.as_ptr();
        self.next_lines = [0, BOUNCE_LINES];
        self.fill(0);
        self.fill(1);
        self.reading = 0;
        self.refills = None;
        OUT_OF_STEP.store(false, Ordering::Relaxed);
        // SAFETY: Clears a stale VSYNC flag of the previous refresh.
        unsafe { LCD_INT_CLR.write_volatile(LCD_VSYNC) };
    }

    /// Refills the buffer the DMA has left, once the DMA reads `descriptor`.
    fn advance(&mut self, descriptor: usize, vsync: bool) {
        if vsync {
            if self
                .refills
                .is_some_and(|refills| refills < REFILLS_PER_FRAME)
            {
                OUT_OF_STEP.store(true, Ordering::Relaxed);
            }
            self.refills = Some(0);
        }

        let index = descriptor.wrapping_sub(self.first_descriptor) / size_of::<DmaDescriptor>();
        let buffer = index / DESCRIPTORS_PER_BUFFER;
        // The DMA may still read the descriptor before the one it reports, so a buffer is free once
        // the DMA reads the second descriptor of the other buffer.
        if buffer < 2 && buffer != self.reading && index % DESCRIPTORS_PER_BUFFER >= 1 {
            self.fill(self.reading);
            self.reading = buffer;
            if let Some(refills) = &mut self.refills {
                *refills += 1;
            }
        }
    }

    fn fill(&mut self, buffer: usize) {
        let Some(buffers) = &mut self.buffers else {
            return;
        };
        let line = self.next_lines[buffer];
        // SAFETY: `frame` points to a whole frame, which is not drawn into while the refresh runs.
        let lines =
            unsafe { core::slice::from_raw_parts(self.frame.add(line * LINE_BYTES), BOUNCE_BYTES) };
        buffers[buffer].copy_from_slice(lines);
        buffers.writeback();
        self.next_lines[buffer] = (line + 2 * BOUNCE_LINES) % HEIGHT as usize;
    }
}

/// The DMA descriptors of the two bounce buffers, which link into a ring.
struct BounceRing {
    descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
}

impl BounceRing {
    fn new() -> Self {
        static BUFFERS: ConstStaticCell<InternalMemory<[[u8; BOUNCE_BYTES]; 2]>> =
            ConstStaticCell::new(InternalMemory::new([[0; BOUNCE_BYTES]; 2]));
        let mut buffers = BUFFERS.take().get_mut();
        let mut descriptors = dma_descriptors_impl!(2 * BOUNCE_BYTES, DESCRIPTOR_BYTES);

        let first: *mut DmaDescriptor = &mut descriptors[0];
        let count = descriptors.len();
        for (index, (descriptor, chunk)) in descriptors
            .iter_mut()
            .zip(buffers.as_flattened_mut().chunks_mut(DESCRIPTOR_BYTES))
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

        // SAFETY: The refresh has not started yet.
        let state = unsafe { REFILL.get() };
        state.buffers = Some(buffers);
        state.first_descriptor = first as usize;
        Self { descriptors }
    }
}

unsafe impl DmaTxBuffer for BounceRing {
    type View = BounceRing;
    type Final = BounceRing;

    fn prepare(&mut self) -> Preparation {
        Preparation {
            start: self.descriptors.as_mut_ptr(),
            accesses_psram: false,
            burst_transfer: BurstConfig::default(),
            // The DMA never hands the descriptors back, so it must not check their owner.
            check_owner: Some(false),
            auto_write_back: false,
        }
    }

    fn into_view(self) -> BounceRing {
        self
    }

    fn from_view(view: BounceRing) -> BounceRing {
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
