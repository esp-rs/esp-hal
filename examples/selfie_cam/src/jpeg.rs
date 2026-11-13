//! JPEG files from the 1280x720 RGB565 pictures, and back.

use alloc::{vec, vec::Vec};

use jpeg_encoder::{Encoder, EncodingError, ImageBuffer, JpegColorType, rgb_to_ycbcr};
use zune_core::{bytestream::ZCursor, colorspace::ColorSpace, options::DecoderOptions};
use zune_jpeg::{JpegDecoder, errors::DecodeErrors};

use crate::{
    canvas::rgb565,
    picture::{Frame, HEIGHT, WIDTH},
};

const QUALITY: u8 = 90;

pub fn encode(pixels: &[u16]) -> Result<Vec<u8>, EncodingError> {
    let mut jpeg = Vec::with_capacity(512 * 1024);
    Encoder::new(&mut jpeg, QUALITY).encode_image(Rgb565(pixels))?;
    Ok(jpeg)
}

pub enum DecodeError {
    Jpeg(DecodeErrors),
    /// Only pictures from this camera fit.
    Size(usize, usize),
}

impl core::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Jpeg(error) => write!(f, "{error:?}"),
            Self::Size(width, height) => {
                write!(f, "{width}x{height} pixels instead of {WIDTH}x{HEIGHT}")
            }
        }
    }
}

/// Decodes a 1280x720 JPEG into the frame.
pub fn decode(jpeg: &[u8], frame: &mut Frame) -> Result<(), DecodeError> {
    let options = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::RGB);
    let mut decoder = JpegDecoder::new_with_options(ZCursor::new(jpeg), options);
    decoder.decode_headers().map_err(DecodeError::Jpeg)?;
    let (width, height) = decoder.dimensions().unwrap();
    if (width, height) != (WIDTH, HEIGHT) {
        return Err(DecodeError::Size(width, height));
    }

    let mut rgb = vec![0u8; WIDTH * HEIGHT * 3];
    decoder.decode_into(&mut rgb).map_err(DecodeError::Jpeg)?;
    for (pixel, &[r, g, b]) in frame.pixels_mut().iter_mut().zip(rgb.as_chunks().0) {
        *pixel = rgb565(r, g, b);
    }
    Ok(())
}

struct Rgb565<'a>(&'a [u16]);

impl ImageBuffer for Rgb565<'_> {
    fn get_jpeg_color_type(&self) -> JpegColorType {
        JpegColorType::Ycbcr
    }

    fn width(&self) -> u16 {
        WIDTH as u16
    }

    fn height(&self) -> u16 {
        HEIGHT as u16
    }

    fn fill_buffers(&self, y: u16, buffers: &mut [Vec<u8>; 4]) {
        let row = &self.0[usize::from(y) * WIDTH..][..WIDTH];
        for &pixel in row {
            let pixel = u16::from_be(pixel);
            // Repeat the high bits in the low bits, so that white stays 255.
            let r = (pixel >> 11) as u8;
            let g = (pixel >> 5) as u8 & 0x3f;
            let b = pixel as u8 & 0x1f;
            let (y, cb, cr) = rgb_to_ycbcr(r << 3 | r >> 2, g << 2 | g >> 4, b << 3 | b >> 2);
            buffers[0].push(y);
            buffers[1].push(cb);
            buffers[2].push(cr);
        }
    }
}
