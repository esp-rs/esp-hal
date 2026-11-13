//! What the panel shows, and how it gets from the main loop to the scanout
//! interrupt.
//!
//! The main loop prepares a [`Scene`] and posts it. At the start of each frame, the
//! interrupt takes the newest posted scene, and hands the one that it showed before
//! back to the main loop, which drops it. The interrupt never drops anything.

use alloc::{boxed::Box, sync::Arc, vec::Vec};
use core::{
    ptr,
    sync::atomic::{AtomicPtr, Ordering},
};

use esp_hal::ram;

use crate::{
    canvas::{Label, Mask, Shape},
    display::{HEIGHT, WIDTH},
    picture::{self, Picture},
};

/// The picture fills the panel's width, with black bars above and below.
pub const PICTURE_HEIGHT: usize = picture::HEIGHT * WIDTH / picture::WIDTH;
pub const PICTURE_Y: usize = (HEIGHT - PICTURE_HEIGHT) / 2;
/// Every 8 picture pixels make 5 panel pixels, from these columns of the 8.
const COLUMNS: [usize; 5] = [0, 1, 3, 4, 6];
const _: () = assert!(picture::WIDTH * 5 == WIDTH * 8);

/// The buttons of a screen. They never change.
pub struct Layer {
    pub shapes: Vec<Shape>,
    pub labels: Vec<Mask>,
}

#[derive(Default)]
pub struct Scene {
    pub picture: Option<Arc<Picture>>,
    /// Shows the picture flipped left to right, like a mirror.
    pub mirror: bool,
    pub layer: Option<&'static Layer>,
    /// The status line and messages, over everything else. Scenes share them.
    pub labels: Vec<Arc<Label>>,
}

impl Scene {
    fn draw_overlays(&self, y: i32, row: &mut [u16]) {
        if let Some(layer) = self.layer {
            for shape in &layer.shapes {
                shape.draw_row(y, row);
            }
            for label in &layer.labels {
                label.draw_row(y, row);
            }
        }
        for label in &self.labels {
            label.draw_row(y, row);
        }
    }
}

/// The newest scene from the main loop.
static NEXT: AtomicPtr<Scene> = AtomicPtr::new(ptr::null_mut());
/// The scene that the interrupt is showing. Only the interrupt uses it.
static CURRENT: AtomicPtr<Scene> = AtomicPtr::new(ptr::null_mut());
/// The scene that the interrupt showed before, for the main loop to drop.
static DONE: AtomicPtr<Scene> = AtomicPtr::new(ptr::null_mut());

/// Shows the scene from the next frame on, as soon as the panel is done with the scene
/// before. Drops an older scene that was posted but never shown.
pub fn post(scene: Scene) {
    collect();
    let old = NEXT.swap(Box::into_raw(Box::new(scene)), Ordering::AcqRel);
    if !old.is_null() {
        // SAFETY: only `Box::into_raw` puts non-null pointers into the slots.
        drop(unsafe { Box::from_raw(old) });
    }
}

/// Drops the scene that the panel stopped showing. The panel takes the next scene only
/// after this, so call it often.
pub fn collect() {
    let done = DONE.swap(ptr::null_mut(), Ordering::AcqRel);
    if !done.is_null() {
        // SAFETY: as in `post`.
        drop(unsafe { Box::from_raw(done) });
    }
}

/// Draws panel rows from `first_row` on into `rows`. Called by the scanout interrupt.
#[ram]
pub fn draw_rows(first_row: usize, rows: &mut [u16]) {
    if first_row == 0 {
        // Until the main loop drops the scene before, keep showing the current one.
        if DONE.load(Ordering::Acquire).is_null() {
            let next = NEXT.swap(ptr::null_mut(), Ordering::AcqRel);
            if !next.is_null() {
                let shown = CURRENT.swap(next, Ordering::AcqRel);
                DONE.store(shown, Ordering::Release);
            }
        }
    }

    // SAFETY: the main loop does not touch the current scene, and only this
    // interrupt replaces it.
    let Some(scene) = (unsafe { CURRENT.load(Ordering::Acquire).as_ref() }) else {
        rows.fill(0);
        return;
    };

    let picture_rows = PICTURE_Y..PICTURE_Y + PICTURE_HEIGHT;
    for (i, row) in rows.as_chunks_mut::<WIDTH>().0.iter_mut().enumerate() {
        let y = first_row + i;
        match &scene.picture {
            Some(picture) if picture_rows.contains(&y) => {
                let line = (y - PICTURE_Y) * picture::HEIGHT / PICTURE_HEIGHT;
                let source = &picture.pixels()[line * picture::WIDTH..][..picture::WIDTH];
                let source = source.as_chunks::<8>().0;
                let row = row.as_chunks_mut::<5>().0;
                if scene.mirror {
                    for (pixels, source) in row.iter_mut().zip(source.iter().rev()) {
                        *pixels = COLUMNS.map(|column| source[7 - column]);
                    }
                } else {
                    for (pixels, source) in row.iter_mut().zip(source) {
                        *pixels = COLUMNS.map(|column| source[column]);
                    }
                }
            }
            _ => row.fill(0),
        }
        scene.draw_overlays(y as i32, row);
    }
}
