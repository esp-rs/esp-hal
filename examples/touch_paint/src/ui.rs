//! The toolbar, the brush strokes and the demo picture.

use embedded_graphics::{
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_10X20},
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{
        Circle,
        Line,
        PrimitiveStyle,
        PrimitiveStyleBuilder,
        Rectangle,
        StrokeAlignment,
        Triangle,
    },
    text::{Alignment, Text},
};
use embedded_gui::{
    Border,
    EdgeInsets,
    FontId,
    GuiContext,
    InputEvent,
    PointerButton,
    PointerState,
    Rect,
    Style,
    UiEvent,
    WidgetId,
    WidgetStyle,
};

use crate::{
    display::{HEIGHT, WIDTH},
    framebuf::Frame,
};

pub const TOOLBAR_HEIGHT: i32 = 68;
const CANVAS: Rectangle = Rectangle::new(
    Point::new(0, TOOLBAR_HEIGHT),
    Size::new(WIDTH as u32, HEIGHT as u32 - TOOLBAR_HEIGHT as u32),
);
const CANVAS_COLOR: Rgb565 = Rgb565::WHITE;
const PALETTE: [Rgb565; 8] = [
    Rgb565::BLACK,
    Rgb565::RED,
    Rgb565::new(31, 32, 0),
    Rgb565::YELLOW,
    Rgb565::GREEN,
    Rgb565::CYAN,
    Rgb565::BLUE,
    Rgb565::MAGENTA,
];
/// The label and brush radius of the S, M and L buttons.
const BRUSH_SIZES: [(&str, u32); 3] = [("S", 3), ("M", 7), ("L", 13)];

const TOOLBAR_COLOR: Rgb565 = Rgb565::new(3, 6, 8);
const TOOLBAR_BORDER: Rgb565 = Rgb565::new(10, 24, 24);
const SELECTED_COLOR: Rgb565 = Rgb565::new(0, 55, 31);
const BUTTON_COLOR: Rgb565 = Rgb565::new(6, 10, 12);

/// One slot per toolbar widget. Restyling all buttons marks each of them dirty.
type Gui = GuiContext<'static, 16, 16, 16>;

#[derive(Clone, Copy)]
enum Tool {
    Color(usize),
    Size(u32),
    Eraser,
    Clear,
}

/// The toolbar buttons and the brush that they select.
pub struct Toolbar {
    gui: Gui,
    buttons: [(WidgetId, Tool); 13],
    color: usize,
    radius: u32,
    erasing: bool,
}

impl Toolbar {
    pub fn new() -> Self {
        let (width, height) = (WIDTH as u32, TOOLBAR_HEIGHT as u32);
        let mut gui = Gui::new(Rect::new(0, 0, width, height));
        let background = Rect::new(0, 0, width, height - 2);
        gui.add_panel(background, Style::new().with_bg(TOOLBAR_COLOR))
            .unwrap();
        let border = Rect::new(0, TOOLBAR_HEIGHT - 2, width, 2);
        gui.add_panel(border, Style::new().with_bg(TOOLBAR_BORDER))
            .unwrap();

        let buttons = core::array::from_fn(|index| {
            let (rect, label, tool) = match index {
                0..8 => {
                    let x = 5 + index as i32 * 53;
                    (Rect::new(x, 7, 49, 49), "", Tool::Color(index))
                }
                8..11 => {
                    let x = 448 + (index as i32 - 8) * 52;
                    let (label, radius) = BRUSH_SIZES[index - 8];
                    (Rect::new(x, 10, 43, 43), label, Tool::Size(radius))
                }
                11 => (Rect::new(608, 10, 82, 43), "ERASER", Tool::Eraser),
                _ => (Rect::new(704, 10, 88, 43), "CLEAR", Tool::Clear),
            };
            (gui.add_button(rect, label, Style::new()).unwrap(), tool)
        });

        let mut toolbar = Self {
            gui,
            buttons,
            color: 1,
            radius: BRUSH_SIZES[1].1,
            erasing: false,
        };
        toolbar.restyle();
        toolbar
    }

    /// The color and radius of the brush.
    pub fn brush(&self) -> (Rgb565, u32) {
        let color = if self.erasing {
            CANVAS_COLOR
        } else {
            PALETTE[self.color]
        };
        (color, self.radius)
    }

    /// Handles a finger that lands on the toolbar. Returns `true` when it pressed CLEAR.
    pub fn press(&mut self, x: i32, y: i32) -> bool {
        let mut clear = false;
        let _ = self.gui.handle_input(pointer(x, y, PointerState::Pressed));
        while let Some(event) = self.gui.pop_event() {
            let UiEvent::Clicked(id) = event else {
                continue;
            };
            match self.buttons.iter().find(|(button, _)| *button == id) {
                Some((_, Tool::Color(index))) => {
                    self.color = *index;
                    self.erasing = false;
                }
                Some((_, Tool::Size(radius))) => self.radius = *radius,
                Some((_, Tool::Eraser)) => self.erasing = true,
                Some((_, Tool::Clear)) => clear = true,
                None => {}
            }
        }
        self.restyle();
        clear
    }

    /// Handles a finger that leaves the display.
    pub fn release(&mut self) {
        let _ = self.gui.handle_input(pointer(0, 0, PointerState::Released));
        while self.gui.pop_event().is_some() {}
    }

    /// Draws the parts of the toolbar that changed.
    pub fn draw(&mut self, frame: &mut Frame<'_>) {
        let _ = self.gui.render_dirty(frame);
        self.gui.clear_dirty();
    }

    fn restyle(&mut self) {
        for (id, tool) in self.buttons {
            let (background, border) = match tool {
                Tool::Color(index) if !self.erasing && index == self.color => {
                    (PALETTE[index], SELECTED_COLOR)
                }
                Tool::Color(index) => (PALETTE[index], TOOLBAR_BORDER),
                Tool::Size(radius) if radius == self.radius => {
                    (Rgb565::new(0, 24, 20), SELECTED_COLOR)
                }
                Tool::Eraser if self.erasing => (Rgb565::new(18, 14, 3), Rgb565::YELLOW),
                Tool::Size(_) | Tool::Eraser => (BUTTON_COLOR, TOOLBAR_BORDER),
                Tool::Clear => (Rgb565::new(18, 4, 4), Rgb565::RED),
            };
            let _ = self
                .gui
                .set_widget_style(id, button_style(background, border));
        }
    }
}

fn pointer(x: i32, y: i32, state: PointerState) -> InputEvent {
    InputEvent::Pointer {
        x,
        y,
        state,
        button: PointerButton::Primary,
    }
}

/// A button that looks the same when it is pressed or has the focus.
fn button_style(background: Rgb565, border: Rgb565) -> WidgetStyle {
    // embedded-gui draws a `MonoFont` label higher than its box, by the font's baseline.
    // Top padding of twice the baseline centers the label again.
    let label_offset = FONT_6X10.baseline as i16 * 2;
    let style = Style::new()
        .with_bg(background)
        .with_border(Border {
            color: border,
            width: 2,
        })
        .with_padding(EdgeInsets::new(label_offset, 0, 0, 0))
        .with_font_id(FontId::MonoFont(&FONT_6X10));
    WidgetStyle::new(style)
        .with_focused(style)
        .with_pressed(style)
}

pub fn clear_canvas(frame: &mut Frame<'_>) {
    let _ = frame.fill_solid(&CANVAS, CANVAS_COLOR);
}

/// Draws a brush stroke with round ends.
pub fn draw_stroke(frame: &mut Frame<'_>, from: Point, to: Point, color: Rgb565, radius: u32) {
    let canvas = &mut frame.clipped(&CANVAS);
    let diameter = radius * 2 + 1;
    if from != to {
        let _ = Line::new(from, to)
            .into_styled(PrimitiveStyle::with_stroke(color, diameter))
            .draw(canvas);
    }
    let _ = Circle::with_center(to, diameter)
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(canvas);
}

pub fn draw_demo_picture(frame: &mut Frame<'_>) {
    const SKY: Rgb565 = Rgb565::new(8, 42, 31);
    const GRASS: Rgb565 = Rgb565::new(5, 38, 10);
    const MOUNTAIN_BACK: Rgb565 = Rgb565::new(11, 25, 20);
    const MOUNTAIN_FRONT: Rgb565 = Rgb565::new(9, 18, 13);
    const TRUNK: Rgb565 = Rgb565::new(16, 24, 5);
    const LEAVES: Rgb565 = Rgb565::new(3, 31, 7);

    let canvas = &mut frame.clipped(&CANVAS);
    let fill = PrimitiveStyle::with_fill;
    let circle = |x, y, radius: u32| Circle::with_center(Point::new(x, y), radius * 2 + 1);
    let mountain = |x, y, half_base, height| {
        Triangle::new(
            Point::new(x, y),
            Point::new(x - half_base, y + height),
            Point::new(x + half_base, y + height),
        )
    };

    let _ = canvas.clear(SKY);
    let _ = circle(675, 132, 38)
        .into_styled(fill(Rgb565::YELLOW))
        .draw(canvas);
    for (x, y, radius) in [(105, 125, 24), (135, 112, 31), (170, 126, 25)] {
        let _ = circle(x, y, radius)
            .into_styled(fill(Rgb565::WHITE))
            .draw(canvas);
    }

    for (x, y, half_base, height, color) in [
        (210, 150, 230, 220, MOUNTAIN_BACK),
        (520, 135, 285, 235, MOUNTAIN_FRONT),
        (520, 135, 55, 46, Rgb565::WHITE),
        (210, 150, 45, 42, Rgb565::WHITE),
    ] {
        let _ = mountain(x, y, half_base, height)
            .into_styled(fill(color))
            .draw(canvas);
    }

    let _ = canvas.fill_solid(
        &Rectangle::new(Point::new(0, 350), Size::new(WIDTH as u32, 130)),
        GRASS,
    );
    let _ = canvas.fill_solid(
        &Rectangle::new(Point::new(116, 316), Size::new(25, 105)),
        TRUNK,
    );
    for (x, y, radius) in [
        (128, 290, 48),
        (91, 310, 38),
        (164, 308, 40),
        (128, 255, 38),
    ] {
        let _ = circle(x, y, radius).into_styled(fill(LEAVES)).draw(canvas);
    }

    let title_box = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::new(2, 9, 12))
        .stroke_color(Rgb565::CYAN)
        .stroke_width(2)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();
    let _ = Rectangle::new(Point::new(270, 378), Size::new(330, 60))
        .into_styled(title_box)
        .draw(canvas);
    let title = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    let subtitle = MonoTextStyle::new(&FONT_6X10, Rgb565::CYAN);
    let _ = Text::with_alignment(
        "ESP32-S31 TOUCH PAINT",
        Point::new(435, 408),
        title,
        Alignment::Center,
    )
    .draw(canvas);
    let _ = Text::with_alignment(
        "Draw over me - SET restores this picture",
        Point::new(435, 427),
        subtitle,
        Alignment::Center,
    )
    .draw(canvas);
}
