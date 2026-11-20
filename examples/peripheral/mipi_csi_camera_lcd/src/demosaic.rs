//! Bilinear demosaic of a RAW8 Bayer frame into little-endian RGB565.
//!
//! Each missing colour is the mean of the nearest pixels of that colour. At
//! the frame edges the row or column outside the frame is replaced by its
//! mirror one pixel further in, which has the same colour.

/// Colours of the top-left 2×2 pixels, top row first.
#[allow(
    dead_code,
    reason = "other sensor modes, or mirroring, give the other layouts"
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BayerOrder {
    Bggr,
    Gbrg,
    Grbg,
    Rggb,
}

impl BayerOrder {
    /// Shifts that move this layout onto BGGR.
    const fn bggr_offset(self) -> (usize, usize) {
        match self {
            Self::Bggr => (0, 0),
            Self::Gbrg => (1, 0),
            Self::Grbg => (0, 1),
            Self::Rggb => (1, 1),
        }
    }
}

/// Channel of the pixel at `(x, y)`: 0 red, 1 green, 2 blue.
pub const fn channel(order: BayerOrder, x: usize, y: usize) -> usize {
    let (ox, oy) = order.bggr_offset();
    match ((x + ox) & 1, (y + oy) & 1) {
        (0, 0) => 2,
        (1, 1) => 0,
        _ => 1,
    }
}

/// The RGB565 field of each 8-bit channel value, with a per-channel gain.
pub struct Rgb565Lut {
    r: [u16; 256],
    g: [u16; 256],
    b: [u16; 256],
}

/// Unit gain in [`Rgb565Lut::new`].
pub const GAIN_ONE: u16 = 256;

impl Rgb565Lut {
    /// Gains are red, green, blue, with [`GAIN_ONE`] as 1. Scaled values
    /// saturate at 255.
    pub const fn new(gains: [u16; 3]) -> Self {
        let mut lut = Self {
            r: [0; 256],
            g: [0; 256],
            b: [0; 256],
        };
        let mut v = 0;
        while v < 256 {
            lut.r[v] = (scale(v, gains[0]) >> 3) << 11;
            lut.g[v] = (scale(v, gains[1]) >> 2) << 5;
            lut.b[v] = scale(v, gains[2]) >> 3;
            v += 1;
        }
        lut
    }

    pub fn pack(&self, r: u8, g: u8, b: u8) -> u16 {
        self.r[r as usize] | self.g[g as usize] | self.b[b as usize]
    }
}

const fn scale(v: usize, gain: u16) -> u16 {
    let scaled = (v as u32 * gain as u32) / GAIN_ONE as u32;
    if scaled > 255 { 255 } else { scaled as u16 }
}

/// Converts the `width`×`height` frame `raw` into `out`, two bytes per pixel.
///
/// # Panics
///
/// Panics if the frame is smaller than 2×2 or a buffer is too short.
pub fn bayer_to_rgb565(
    raw: &[u8],
    width: usize,
    height: usize,
    order: BayerOrder,
    lut: &Rgb565Lut,
    out: &mut [u8],
) {
    assert!(width >= 2 && height >= 2, "frame must be at least 2x2");
    assert!(raw.len() >= width * height, "RAW8 buffer too short");
    assert!(out.len() >= width * height * 2, "RGB565 buffer too short");
    let (ox, oy) = order.bggr_offset();

    for y in 0..height {
        let row = |i: usize| &raw[i * width..][..width];
        let (above, mid, below) = (
            row(mirror(y, height, false)),
            row(y),
            row(mirror(y, height, true)),
        );
        let out_row = &mut out[y * width * 2..][..width * 2];
        // In BGGR the odd rows hold the red pixels.
        let red_row = (y + oy) & 1 == 1;
        for (x, px) in out_row.as_chunks_mut::<2>().0.iter_mut().enumerate() {
            let (l, r) = (mirror(x, width, false), mirror(x, width, true));
            let odd_col = (x + ox) & 1 == 1;
            let c = mid[x];
            let vertical = avg2(above[x], below[x]);
            let horizontal = avg2(mid[l], mid[r]);
            let (red, green, blue) = match (red_row, odd_col) {
                (false, false) => (
                    avg4(above[l], above[r], below[l], below[r]),
                    avg4(above[x], below[x], mid[l], mid[r]),
                    c,
                ),
                (true, true) => (
                    c,
                    avg4(above[x], below[x], mid[l], mid[r]),
                    avg4(above[l], above[r], below[l], below[r]),
                ),
                // A green pixel has blue beside it in a blue row, red in a red row.
                (false, true) => (vertical, c, horizontal),
                (true, false) => (horizontal, c, vertical),
            };
            *px = lut.pack(red, green, blue).to_le_bytes();
        }
    }
}

/// The neighbour of `i` before it, or after it if `next`, mirrored at the
/// edges of `0..len`.
fn mirror(i: usize, len: usize, next: bool) -> usize {
    match (next, i) {
        (false, 0) => 1,
        (false, _) => i - 1,
        (true, _) if i + 1 == len => len - 2,
        (true, _) => i + 1,
    }
}

fn avg2(a: u8, b: u8) -> u8 {
    ((a as u16 + b as u16 + 1) >> 1) as u8
}

fn avg4(a: u8, b: u8, c: u8, d: u8) -> u8 {
    ((a as u16 + b as u16 + c as u16 + d as u16 + 2) >> 2) as u8
}

/// Mean red, green and blue pixel values of the frame.
pub fn channel_means(raw: &[u8], width: usize, height: usize, order: BayerOrder) -> [u32; 3] {
    let mut sums = [0u64; 3];
    let mut counts = [0u64; 3];
    for (y, row) in raw[..width * height].chunks_exact(width).enumerate() {
        for (x, &v) in row.iter().enumerate() {
            let c = channel(order, x, y);
            sums[c] += v as u64;
            counts[c] += 1;
        }
    }
    [0, 1, 2].map(|c| (sums[c] / counts[c].max(1)) as u32)
}

/// Gains that give red and blue the green mean, at most 4.
pub fn gray_world_gains(means: [u32; 3]) -> [u16; 3] {
    let gain = |mean: u32| {
        (means[1] * GAIN_ONE as u32 / mean.max(1)).clamp(GAIN_ONE as u32 / 4, 4 * GAIN_ONE as u32)
            as u16
    };
    [gain(means[0]), GAIN_ONE, gain(means[2])]
}

#[cfg(test)]
mod tests {
    use super::*;

    const ORDERS: [BayerOrder; 4] = [
        BayerOrder::Bggr,
        BayerOrder::Gbrg,
        BayerOrder::Grbg,
        BayerOrder::Rggb,
    ];

    fn mosaic(width: usize, height: usize, order: BayerOrder, rgb: [u8; 3]) -> Vec<u8> {
        (0..height)
            .flat_map(|y| (0..width).map(move |x| rgb[channel(order, x, y)]))
            .collect()
    }

    fn decode(raw: &[u8], width: usize, height: usize, order: BayerOrder) -> Vec<u16> {
        let mut out = vec![0u8; width * height * 2];
        bayer_to_rgb565(
            raw,
            width,
            height,
            order,
            &Rgb565Lut::new([GAIN_ONE; 3]),
            &mut out,
        );
        out.chunks_exact(2)
            .map(|p| u16::from_le_bytes([p[0], p[1]]))
            .collect()
    }

    #[test]
    fn layouts_match_their_names() {
        let top_left = |order| {
            [(0, 0), (1, 0), (0, 1), (1, 1)].map(|(x, y)| ["R", "G", "B"][channel(order, x, y)])
        };
        assert_eq!(top_left(BayerOrder::Bggr), ["B", "G", "G", "R"]);
        assert_eq!(top_left(BayerOrder::Gbrg), ["G", "B", "R", "G"]);
        assert_eq!(top_left(BayerOrder::Grbg), ["G", "R", "B", "G"]);
        assert_eq!(top_left(BayerOrder::Rggb), ["R", "G", "G", "B"]);
    }

    #[test]
    fn uniform_colour_is_exact_everywhere() {
        let rgb = [200, 100, 40];
        let lut = Rgb565Lut::new([GAIN_ONE; 3]);
        for order in ORDERS {
            for (w, h) in [(2, 2), (3, 3), (5, 4), (16, 9)] {
                let out = decode(&mosaic(w, h, order, rgb), w, h, order);
                assert!(
                    out.iter().all(|&p| p == lut.pack(rgb[0], rgb[1], rgb[2])),
                    "{order:?} {w}x{h}"
                );
            }
        }
    }

    #[test]
    fn swapped_layout_swaps_red_and_blue() {
        let (w, h) = (8, 6);
        let raw = mosaic(w, h, BayerOrder::Bggr, [255, 0, 0]);
        let out = decode(&raw, w, h, BayerOrder::Rggb);
        assert!(out.iter().all(|&p| p == 0x001f));
    }

    #[test]
    fn rgb565_fields_and_byte_order() {
        let lut = Rgb565Lut::new([GAIN_ONE; 3]);
        assert_eq!(lut.pack(255, 0, 0), 0xf800);
        assert_eq!(lut.pack(0, 255, 0), 0x07e0);
        assert_eq!(lut.pack(0, 0, 255), 0x001f);
        let mut out = [0u8; 8];
        bayer_to_rgb565(&[255; 4], 2, 2, BayerOrder::Bggr, &lut, &mut out);
        assert_eq!(out, [0xff; 8]);
        let raw = mosaic(2, 2, BayerOrder::Bggr, [255, 0, 0]);
        bayer_to_rgb565(&raw, 2, 2, BayerOrder::Bggr, &lut, &mut out);
        assert_eq!(out[..2], [0x00, 0xf8]);
    }

    #[test]
    fn edge_interpolates_from_mirrored_neighbours() {
        // Green ramps along x, so a red pixel's green is the mean of its
        // left and right neighbours, and the mirror at x = 0 uses x = 1.
        let (w, h) = (4, 2);
        let raw: Vec<u8> = (0..h)
            .flat_map(|y| {
                (0..w).map(move |x| {
                    if channel(BayerOrder::Rggb, x, y) == 1 {
                        40 * x as u8
                    } else {
                        0
                    }
                })
            })
            .collect();
        let out = decode(&raw, w, h, BayerOrder::Rggb);
        let green = |p: u16| ((p >> 5) & 0x3f) << 2;
        // (0, 0) is red: neighbours are (1, 0) = 40 twice and (0, 1) = 0 twice.
        assert_eq!(green(out[0]), 20);
        // (3, 1) is blue: left and mirrored right are (2, 1) = 80, above and
        // mirrored below are (3, 0) = 120.
        assert_eq!(green(out[w + 3]), 100);
    }

    #[test]
    fn gains_scale_and_saturate() {
        let lut = Rgb565Lut::new([2 * GAIN_ONE, GAIN_ONE, GAIN_ONE / 2]);
        assert_eq!(lut.pack(100, 0, 0), (200 >> 3) << 11);
        assert_eq!(lut.pack(200, 0, 0), 0xf800);
        assert_eq!(lut.pack(0, 0, 200), 100 >> 3);
    }

    #[test]
    fn means_and_gray_world() {
        let (w, h) = (6, 4);
        let raw = mosaic(w, h, BayerOrder::Bggr, [50, 100, 200]);
        let means = channel_means(&raw, w, h, BayerOrder::Bggr);
        assert_eq!(means, [50, 100, 200]);
        assert_eq!(gray_world_gains(means), [512, 256, 128]);
        assert_eq!(gray_world_gains([1, 255, 0]), [1024, 256, 1024]);
    }

    #[test]
    #[should_panic(expected = "RGB565 buffer too short")]
    fn short_output_panics() {
        let lut = Rgb565Lut::new([GAIN_ONE; 3]);
        bayer_to_rgb565(&[0; 4], 2, 2, BayerOrder::Bggr, &lut, &mut [0; 7]);
    }

    #[test]
    #[should_panic(expected = "at least 2x2")]
    fn single_column_panics() {
        let lut = Rgb565Lut::new([GAIN_ONE; 3]);
        bayer_to_rgb565(&[0; 4], 1, 4, BayerOrder::Bggr, &lut, &mut [0; 8]);
    }
}
