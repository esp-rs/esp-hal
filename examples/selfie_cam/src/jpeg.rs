//! Encodes RGB565 camera frames in hardware and decodes JPEG files in software.

use alloc::vec;

use allocator_api2::vec::Vec;
use esp_alloc::DmaCompatibleExternalMemory;
use esp_hal::jpeg::{Config, Encoder, Error, PixelFormat, Subsampling};
use zune_core::{bytestream::ZCursor, colorspace::ColorSpace, options::DecoderOptions};
use zune_jpeg::{JpegDecoder, errors::DecodeErrors};

use crate::{
    canvas::rgb565,
    picture::{Frame, HEIGHT, WIDTH},
};

const QUALITY: u8 = 90;

pub enum EncodeError {
    OutOfMemory,
    Jpeg(Error),
}

impl core::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::OutOfMemory => f.write_str("not enough PSRAM to encode the photo"),
            Self::Jpeg(error) => write!(f, "{error}"),
        }
    }
}

pub fn encode(
    encoder: &mut Encoder<'_>,
    pixels: &[u8],
) -> Result<Vec<u8, DmaCompatibleExternalMemory>, EncodeError> {
    let config = Config {
        width: WIDTH as u16,
        height: HEIGHT as u16,
        quality: QUALITY,
        input_format: PixelFormat::Rgb565Be,
        subsampling: Subsampling::Yuv444,
    };

    let mut capacity = 512 * 1024;
    loop {
        let mut jpeg = Vec::new_in(DmaCompatibleExternalMemory);
        jpeg.try_reserve_exact(capacity)
            .map_err(|_| EncodeError::OutOfMemory)?;
        jpeg.resize(capacity, 0);
        match encoder.encode(pixels, &mut jpeg, config) {
            Ok(size) => {
                jpeg.truncate(size);
                return Ok(jpeg);
            }
            Err(Error::OutputTooSmall) => capacity *= 2,
            Err(error) => return Err(EncodeError::Jpeg(error)),
        }
    }
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
