//! The toolbar and the demo picture.

use embedded_graphics::{
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_10X20},
    },
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};

use crate::{
    display::{HEIGHT, WIDTH},
    framebuf::FrameBuf,
};

pub const TOOLBAR_HEIGHT: i32 = 68;
pub const CANVAS_COLOR: Rgb565 = Rgb565::WHITE;
pub const PALETTE: [Rgb565; 8] = [
    Rgb565::BLACK,
    Rgb565::RED,
    Rgb565::new(31, 32, 0),
    Rgb565::YELLOW,
    Rgb565::GREEN,
    Rgb565::CYAN,
    Rgb565::BLUE,
    Rgb565::MAGENTA,
];

const TOOLBAR_COLOR: Rgb565 = Rgb565::new(3, 6, 8);
const TOOLBAR_BORDER: Rgb565 = Rgb565::new(10, 24, 24);
const SELECTED_COLOR: Rgb565 = Rgb565::new(0, 55, 31);

/// The x position, label and brush radius of the S, M and L buttons.
const BRUSH_SIZES: [(i32, &str, i32); 3] = [(448, "S", 3), (500, "M", 7), (552, "L", 13)];

pub enum ToolbarButton {
    Color(usize),
    BrushSize(i32),
    Eraser,
    Clear,
}

/// The toolbar button at `x`, if any.
pub fn toolbar_button(x: i32) -> Option<ToolbarButton> {
    match x {
        ..432 => Some(ToolbarButton::Color(
            ((x - 8).max(0) / 53).clamp(0, 7) as usize
        )),
        448..=490 => Some(ToolbarButton::BrushSize(BRUSH_SIZES[0].2)),
        491..=542 => Some(ToolbarButton::BrushSize(BRUSH_SIZES[1].2)),
        543..595 => Some(ToolbarButton::BrushSize(BRUSH_SIZES[2].2)),
        608..690 => Some(ToolbarButton::Eraser),
        704.. => Some(ToolbarButton::Clear),
        _ => None,
    }
}

pub fn clear_canvas(frame: &mut FrameBuf<'_>) {
    frame.fill_rect(
        0,
        TOOLBAR_HEIGHT,
        WIDTH as i32,
        HEIGHT as i32 - TOOLBAR_HEIGHT,
        CANVAS_COLOR,
    );
}

pub fn draw_toolbar(frame: &mut FrameBuf<'_>, selected: usize, radius: i32, eraser: bool) {
    frame.fill_rect(0, 0, WIDTH as i32, TOOLBAR_HEIGHT, TOOLBAR_COLOR);
    frame.fill_rect(0, TOOLBAR_HEIGHT - 2, WIDTH as i32, 2, TOOLBAR_BORDER);

    for (index, color) in PALETTE.iter().copied().enumerate() {
        let x = 8 + index as i32 * 53;
        frame.fill_rect(x, 10, 43, 43, color);
        frame.stroke_rect(
            x - 3,
            7,
            49,
            49,
            if !eraser && selected == index {
                SELECTED_COLOR
            } else {
                TOOLBAR_BORDER
            },
        );
    }

    let text = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    for (x, label, size) in BRUSH_SIZES {
        frame.fill_rect(
            x,
            10,
            43,
            43,
            if radius == size {
                Rgb565::new(0, 24, 20)
            } else {
                Rgb565::new(6, 10, 12)
            },
        );
        frame.stroke_rect(
            x,
            10,
            43,
            43,
            if radius == size {
                SELECTED_COLOR
            } else {
                TOOLBAR_BORDER
            },
        );
        let _ = Text::new(label, Point::new(x + 18, 37), text).draw(frame);
    }

    frame.fill_rect(
        608,
        10,
        82,
        43,
        if eraser {
            Rgb565::new(18, 14, 3)
        } else {
            Rgb565::new(6, 10, 12)
        },
    );
    frame.stroke_rect(
        608,
        10,
        82,
        43,
        if eraser {
            Rgb565::YELLOW
        } else {
            TOOLBAR_BORDER
        },
    );
    let _ = Text::new("ERASER", Point::new(629, 37), text).draw(frame);

    frame.fill_rect(704, 10, 88, 43, Rgb565::new(18, 4, 4));
    frame.stroke_rect(704, 10, 88, 43, Rgb565::RED);
    let _ = Text::new("CLEAR", Point::new(733, 37), text).draw(frame);
}

pub fn draw_demo_picture(frame: &mut FrameBuf<'_>) {
    const SKY: Rgb565 = Rgb565::new(8, 42, 31);
    const GRASS: Rgb565 = Rgb565::new(5, 38, 10);
    const MOUNTAIN_BACK: Rgb565 = Rgb565::new(11, 25, 20);
    const MOUNTAIN_FRONT: Rgb565 = Rgb565::new(9, 18, 13);

    frame.fill_rect(
        0,
        TOOLBAR_HEIGHT,
        WIDTH as i32,
        HEIGHT as i32 - TOOLBAR_HEIGHT,
        SKY,
    );
    frame.draw_brush(675, 132, 38, Rgb565::YELLOW);

    for (x, y, radius) in [(105, 125, 24), (135, 112, 31), (170, 126, 25)] {
        frame.draw_brush(x, y, radius, Rgb565::WHITE);
    }

    frame.fill_triangle(210, 150, 230, 220, MOUNTAIN_BACK);
    frame.fill_triangle(520, 135, 285, 235, MOUNTAIN_FRONT);
    frame.fill_triangle(520, 135, 55, 46, Rgb565::WHITE);
    frame.fill_triangle(210, 150, 45, 42, Rgb565::WHITE);

    frame.fill_rect(0, 350, WIDTH as i32, 130, GRASS);
    frame.fill_rect(116, 316, 25, 105, Rgb565::new(16, 24, 5));
    for (x, y, radius) in [
        (128, 290, 48),
        (91, 310, 38),
        (164, 308, 40),
        (128, 255, 38),
    ] {
        frame.draw_brush(x, y, radius, Rgb565::new(3, 31, 7));
    }

    frame.fill_rect(270, 378, 330, 60, Rgb565::new(2, 9, 12));
    frame.stroke_rect(270, 378, 330, 60, Rgb565::CYAN);
    let title = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    let subtitle = MonoTextStyle::new(&FONT_6X10, Rgb565::CYAN);
    let _ = Text::with_alignment(
        "ESP32-S31 TOUCH PAINT",
        Point::new(435, 408),
        title,
        Alignment::Center,
    )
    .draw(frame);
    let _ = Text::with_alignment(
        "Draw over me - SET restores this picture",
        Point::new(435, 427),
        subtitle,
        Alignment::Center,
    )
    .draw(frame);
}
