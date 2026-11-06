mod tables;

use self::tables::{
    CHROMINANCE_AC_COEFFICIENTS,
    CHROMINANCE_AC_VALUES,
    CHROMINANCE_DC_COEFFICIENTS,
    CHROMINANCE_DC_VALUES,
    CHROMINANCE_QUANTIZATION_TABLE,
    LUMINANCE_AC_COEFFICIENTS,
    LUMINANCE_AC_VALUES,
    LUMINANCE_DC_COEFFICIENTS,
    LUMINANCE_DC_VALUES,
    LUMINANCE_QUANTIZATION_TABLE,
    ZIGZAG_ARR,
};

pub(super) const LEN: usize = 640;
const SEGMENT_OVERHEAD: usize = 4;

pub(super) fn quantization_tables(quality: u8) -> ([u8; 64], [u8; 64]) {
    let quality = u32::from(quality);
    let scale = if quality < 50 {
        5000 / quality
    } else {
        200 - 2 * quality
    };
    let scale_table = |base: [u8; 64]| {
        base.map(|value| ((u32::from(value) * scale + 50) / 100).clamp(1, 255) as u8)
    };
    (
        scale_table(LUMINANCE_QUANTIZATION_TABLE),
        scale_table(CHROMINANCE_QUANTIZATION_TABLE),
    )
}

pub(super) fn write(
    output: &mut [u8; LEN],
    width: u16,
    height: u16,
    luminance: &[u8; 64],
    chrominance: &[u8; 64],
) {
    let mut header = Header { output, len: 0 };
    header.bytes(&[0xff, 0xd8]);
    header.segment(0xe0, &[b"JFIF\0\x01\x01\0\0\x01\0\x01\0\0"]);
    for (id, table) in [(0, luminance), (1, chrominance)] {
        let zigzag = ZIGZAG_ARR.map(|index| table[usize::from(index)]);
        header.segment(0xdb, &[&[id], &zigzag]);
    }
    header.segment(
        0xc0,
        &[
            &[8],
            &height.to_be_bytes(),
            &width.to_be_bytes(),
            &[3, 1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1],
        ],
    );
    for (id, lengths, values) in [
        (0x00, &LUMINANCE_DC_COEFFICIENTS, &LUMINANCE_DC_VALUES[..]),
        (0x10, &LUMINANCE_AC_COEFFICIENTS, &LUMINANCE_AC_VALUES[..]),
        (
            0x01,
            &CHROMINANCE_DC_COEFFICIENTS,
            &CHROMINANCE_DC_VALUES[..],
        ),
        (
            0x11,
            &CHROMINANCE_AC_COEFFICIENTS,
            &CHROMINANCE_AC_VALUES[..],
        ),
    ] {
        header.segment(0xc4, &[&[id], lengths, values]);
    }
    let scan = [3, 1, 0, 2, 0x11, 3, 0x11, 0, 63, 0];
    // Keep the DMA payload on a separate cache line from the CPU-written header.
    let padding = LEN - header.len - scan.len() - 2 * SEGMENT_OVERHEAD;
    header.segment(0xfe, &[&[0; 64][..padding]]);
    header.segment(0xda, &[&scan]);
}

struct Header<'a> {
    output: &'a mut [u8; LEN],
    len: usize,
}

impl Header<'_> {
    fn bytes(&mut self, bytes: &[u8]) {
        let end = self.len + bytes.len();
        self.output[self.len..end].copy_from_slice(bytes);
        self.len = end;
    }

    fn segment(&mut self, marker: u8, parts: &[&[u8]]) {
        let length = 2 + parts.iter().map(|part| part.len() as u16).sum::<u16>();
        self.bytes(&[0xff, marker]);
        self.bytes(&length.to_be_bytes());
        for part in parts {
            self.bytes(part);
        }
    }
}
