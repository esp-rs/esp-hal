//! The on-screen buttons of the camera and the gallery screens.

use alloc::{boxed::Box, vec::Vec};

use crate::{
    canvas::{BLACK, Mask, Shape, WHITE},
    scene::Layer,
};

/// Extra pixels around a button that still count as a hit.
const SLACK: i32 = 16;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Action {
    OpenGallery,
    OpenCamera,
    Previous,
    Next,
    Delete,
}

struct Button {
    action: Action,
    shape: Shape,
    label: &'static str,
}

impl Button {
    const fn pill(action: Action, x: i32, y: i32, label: &'static str) -> Self {
        Self {
            action,
            shape: Shape::rounded(x, y, 150, 56, BLACK),
            label,
        }
    }

    const fn round(action: Action, x: i32, y: i32, label: &'static str) -> Self {
        Self {
            action,
            shape: Shape::circle(x, y, 34, BLACK),
            label,
        }
    }
}

pub struct Screen {
    pub layer: &'static Layer,
    buttons: Vec<(Action, Shape)>,
}

impl Screen {
    fn new(buttons: &[Button]) -> Self {
        let mut shapes = Vec::new();
        let mut labels = Vec::new();
        for button in buttons {
            let shape = button.shape;
            shapes.push(shape);
            shapes.push(Shape {
                color: WHITE,
                ..shape.with_border(3)
            });
            labels.push(Mask::text(
                button.label,
                (shape.left + shape.right) / 2,
                (shape.top + shape.bottom) / 2,
                WHITE,
            ));
        }
        Self {
            layer: Box::leak(Box::new(Layer { shapes, labels })),
            buttons: buttons
                .iter()
                .map(|button| (button.action, button.shape))
                .collect(),
        }
    }

    pub fn camera() -> Self {
        Self::new(&[Button::pill(Action::OpenGallery, 626, 398, "GALLERY")])
    }

    pub fn gallery() -> Self {
        Self::new(&[
            Button::pill(Action::OpenCamera, 24, 398, "CAMERA"),
            Button::pill(Action::Delete, 626, 398, "DELETE"),
            Button::round(Action::Previous, 60, 240, "<"),
            Button::round(Action::Next, 740, 240, ">"),
        ])
    }

    /// The button under the point.
    pub fn action_at(&self, x: i32, y: i32) -> Option<Action> {
        self.buttons
            .iter()
            .find(|(_, shape)| shape.hit(x, y, SLACK))
            .map(|&(action, _)| action)
    }
}
