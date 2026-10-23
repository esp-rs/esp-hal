//! PPA SRM tests. No external wiring is required.

//% CHIP_FILTER: pixel_accelerator_driver_supported
//% FEATURES: unstable psram

#![no_std]
#![no_main]

use allocator_api2::boxed::Box;
use esp_alloc::ExternalMemory;
use esp_hal::{
    Blocking,
    dma::aligned::DmaAlignedMut,
    peripherals::{DMA2D, PPA},
    ppa::srm::{Config, Error, Input, Offset, Output, PixelFormat, Rotation, Size, Srm},
};
use hil_test as _;
use static_cell::ConstStaticCell;

const SOURCE_SIZE: Size = Size {
    width: 64,
    height: 48,
};
const DESTINATION_SIZE: usize = 80;
const SOURCE_BYTES: usize = SOURCE_SIZE.width * SOURCE_SIZE.height * 2;
const DESTINATION_BYTES: usize = DESTINATION_SIZE * DESTINATION_SIZE * 2;
const OFFSET: Offset = Offset { x: 8, y: 8 };
const ROTATIONS: [Rotation; 4] = [
    Rotation::Degrees0,
    Rotation::Degrees90,
    Rotation::Degrees180,
    Rotation::Degrees270,
];

#[repr(C, align(64))]
struct Buffer<const N: usize>([u8; N]);

struct Context {
    ppa: PPA<'static>,
    dma: DMA2D<'static>,
    source: &'static mut [u8; SOURCE_BYTES],
    destination: &'static mut [u8; DESTINATION_BYTES],
}

fn rgb565(x: usize, y: usize) -> u16 {
    (((x & 31) << 11) | ((y & 63) << 5) | ((x + (x >> 5) + y) & 31)) as u16
}

fn uyvy422(x: usize, y: usize) -> ([u8; 4], u16) {
    const COLORS: [([u8; 4], [u16; 2]); 8] = [
        ([128, 16, 128, 235], [0x0000, 0xffff]),
        ([128, 235, 128, 235], [0xffff, 0xffff]),
        ([90, 82, 240, 82], [0xf800, 0xf800]),
        ([54, 145, 34, 145], [0x07e0, 0x07e0]),
        ([240, 41, 110, 41], [0x001f, 0x001f]),
        ([166, 170, 16, 170], [0x07ff, 0x07ff]),
        ([202, 106, 222, 106], [0xf81f, 0xf81f]),
        ([16, 210, 146, 210], [0xffe0, 0xffe0]),
    ];
    let (pair, pixels) = COLORS[(x / 8 + y / 8) % COLORS.len()];
    (pair, pixels[x % 2])
}

fn fill_source(source: &mut [u8], size: Size, format: PixelFormat) {
    for y in 0..size.height {
        for x in 0..size.width {
            let index = (y * size.width + x) * 2;
            match format {
                PixelFormat::Rgb565 => {
                    source[index..index + 2].copy_from_slice(&rgb565(x, y).to_le_bytes());
                }
                PixelFormat::Uyvy422 => {
                    let (pair, _) = uyvy422(x, y);
                    let offset = (x % 2) * 2;
                    source[index..index + 2].copy_from_slice(&pair[offset..offset + 2]);
                }
                _ => unreachable!(),
            }
        }
    }
}

fn check_unscaled(
    srm: &mut Srm<'_, Blocking>,
    input: Input<'_>,
    destination: &mut [u8],
    offset: Offset,
    config: Config,
) {
    let size = input.size;
    let format = input.format;
    let rotation = config.rotation();
    destination.fill(0xa5);
    srm.run(
        input,
        Output {
            data: destination,
            size: Size {
                width: DESTINATION_SIZE,
                height: DESTINATION_SIZE,
            },
            format: PixelFormat::Rgb565,
            offset,
        },
        config,
    )
    .unwrap();

    let (width, height) = match rotation {
        Rotation::Degrees0 | Rotation::Degrees180 => (size.width, size.height),
        Rotation::Degrees90 | Rotation::Degrees270 => (size.height, size.width),
        _ => unreachable!(),
    };
    for y in 0..DESTINATION_SIZE {
        for x in 0..DESTINATION_SIZE {
            let expected = if (offset.x..offset.x + width).contains(&x)
                && (offset.y..offset.y + height).contains(&y)
            {
                let (mut x, mut y) = (x - offset.x, y - offset.y);
                if config.mirror_x() {
                    x = width - 1 - x;
                }
                if config.mirror_y() {
                    y = height - 1 - y;
                }
                let (source_x, source_y) = match rotation {
                    Rotation::Degrees0 => (x, y),
                    Rotation::Degrees90 => (size.width - 1 - y, x),
                    Rotation::Degrees180 => (size.width - 1 - x, size.height - 1 - y),
                    Rotation::Degrees270 => (y, size.height - 1 - x),
                    _ => unreachable!(),
                };
                match format {
                    PixelFormat::Rgb565 => rgb565(source_x, source_y),
                    PixelFormat::Uyvy422 => uyvy422(source_x, source_y).1,
                    _ => unreachable!(),
                }
            } else {
                0xa5a5
            };
            let index = (y * DESTINATION_SIZE + x) * 2;
            let actual = u16::from_le_bytes([destination[index], destination[index + 1]]);
            assert_eq!(actual, expected, "pixel ({x}, {y}), rotation {rotation:?}");
        }
    }
}

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
        static SOURCE: ConstStaticCell<Buffer<SOURCE_BYTES>> =
            ConstStaticCell::new(Buffer([0; SOURCE_BYTES]));
        static DESTINATION: ConstStaticCell<Buffer<DESTINATION_BYTES>> =
            ConstStaticCell::new(Buffer([0; DESTINATION_BYTES]));
        Context {
            ppa: peripherals.PPA,
            dma: peripherals.DMA2D,
            source: &mut SOURCE.take().0,
            destination: &mut DESTINATION.take().0,
        }
    }

    #[test]
    fn test_rgb565_rotation(mut ctx: Context) {
        fill_source(ctx.source, SOURCE_SIZE, PixelFormat::Rgb565);
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for rotation in ROTATIONS {
            check_unscaled(
                &mut srm,
                Input {
                    data: ctx.source,
                    size: SOURCE_SIZE,
                    format: PixelFormat::Rgb565,
                },
                ctx.destination,
                OFFSET,
                Config::default().with_rotation(rotation),
            );
        }
    }

    #[test]
    fn test_uyvy422_conversion(mut ctx: Context) {
        fill_source(ctx.source, SOURCE_SIZE, PixelFormat::Uyvy422);
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for rotation in ROTATIONS {
            check_unscaled(
                &mut srm,
                Input {
                    data: ctx.source,
                    size: SOURCE_SIZE,
                    format: PixelFormat::Uyvy422,
                },
                ctx.destination,
                OFFSET,
                Config::default().with_rotation(rotation),
            );
        }
    }

    #[test]
    fn test_mirroring(mut ctx: Context) {
        fill_source(ctx.source, SOURCE_SIZE, PixelFormat::Rgb565);
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for rotation in ROTATIONS {
            for (mirror_x, mirror_y) in [(true, false), (false, true), (true, true)] {
                check_unscaled(
                    &mut srm,
                    Input {
                        data: ctx.source,
                        size: SOURCE_SIZE,
                        format: PixelFormat::Rgb565,
                    },
                    ctx.destination,
                    OFFSET,
                    Config::default()
                        .with_rotation(rotation)
                        .with_mirror_x(mirror_x)
                        .with_mirror_y(mirror_y),
                );
            }
        }
    }

    #[test]
    fn test_scaling(mut ctx: Context) {
        const SIDE: usize = 80;
        let size = Size {
            width: 40,
            height: 36,
        };
        let offset = Offset { x: 3, y: 5 };
        let source = &mut ctx.source[..size.width * size.height];
        for y in 0..size.height {
            for x in 0..size.width {
                source[y * size.width + x] = (2 * x + 4 * y) as u8;
            }
        }
        let destination = &mut ctx.destination[..SIDE * SIDE];
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for (config, (scale_x, scale_y), output_size) in [
            (
                Config::default()
                    .with_scale_x(0.5)
                    .with_scale_y(0.25)
                    .with_rotation(Rotation::Degrees90)
                    .with_mirror_x(true),
                (0.5, 0.25),
                Size {
                    width: 9,
                    height: 20,
                },
            ),
            (
                Config::default().with_scale_x(1.5).with_scale_y(2.0),
                (1.5, 2.0),
                Size {
                    width: 60,
                    height: 72,
                },
            ),
            (
                Config::default()
                    .with_scale_x(1.24)
                    .with_scale_y(1.0 / 16.0),
                (19.0 / 16.0, 1.0 / 16.0),
                Size {
                    width: 47,
                    height: 2,
                },
            ),
            (
                Config::default()
                    .with_scale_x(1.0 / 16.0)
                    .with_scale_y(1.24),
                (1.0 / 16.0, 19.0 / 16.0),
                Size {
                    width: 2,
                    height: 42,
                },
            ),
        ] {
            assert_eq!(config.output_size(size), Ok(output_size));
            destination.fill(0xa5);
            srm.run(
                Input {
                    data: source,
                    size,
                    format: PixelFormat::Gray8,
                },
                Output {
                    data: destination,
                    size: Size {
                        width: SIDE,
                        height: SIDE,
                    },
                    format: PixelFormat::Gray8,
                    offset,
                },
                config,
            )
            .unwrap();
            for y in 0..SIDE {
                for x in 0..SIDE {
                    let (expected, tolerance) = if (offset.x..offset.x + output_size.width)
                        .contains(&x)
                        && (offset.y..offset.y + output_size.height).contains(&y)
                    {
                        let (mut x, y) = (x - offset.x, y - offset.y);
                        if config.mirror_x() {
                            x = output_size.width - 1 - x;
                        }
                        let (x, y) = match config.rotation() {
                            Rotation::Degrees0 => (x, y),
                            Rotation::Degrees90 => (output_size.height - 1 - y, x),
                            _ => unreachable!(),
                        };
                        // Pixel-center sampling; upscaling allows one level of rounding error.
                        let x =
                            ((x as f32 + 0.5) / scale_x - 0.5).clamp(0.0, (size.width - 1) as f32);
                        let y =
                            ((y as f32 + 0.5) / scale_y - 0.5).clamp(0.0, (size.height - 1) as f32);
                        (
                            (2.0 * x + 4.0 * y + 0.5) as u8,
                            u8::from(scale_x > 1.0 || scale_y > 1.0),
                        )
                    } else {
                        (0xa5, 0)
                    };
                    let actual = destination[y * SIDE + x];
                    assert!(
                        actual.abs_diff(expected) <= tolerance,
                        "pixel ({x}, {y}), {config:?}: actual={actual}, expected={expected}",
                    );
                }
            }
        }
    }

    #[test]
    fn test_packed_formats(mut ctx: Context) {
        const SIDE: usize = 16;
        const PIXELS: usize = SIDE * SIDE;
        const RGB: [[u8; 3]; 8] = [
            [0, 0, 0],
            [255, 255, 255],
            [0, 0, 255],
            [0, 0, 255],
            [0, 255, 0],
            [0, 255, 0],
            [255, 0, 0],
            [255, 0, 0],
        ];
        const UYVY: [[u8; 2]; 8] = [
            [128, 16],
            [128, 235],
            [90, 82],
            [240, 82],
            [54, 145],
            [34, 145],
            [240, 41],
            [110, 41],
        ];
        let size = Size {
            width: SIDE,
            height: SIDE,
        };
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for (input_format, output_format) in [
            (PixelFormat::Argb8888, PixelFormat::Argb8888),
            (PixelFormat::Argb8888, PixelFormat::Rgb888),
            (PixelFormat::Argb8888, PixelFormat::Gray8),
            (PixelFormat::Rgb888, PixelFormat::Argb8888),
            (PixelFormat::Rgb888, PixelFormat::Uyvy422),
        ] {
            let source = &mut ctx.source[..PIXELS * input_format.bytes_per_pixel()];
            for (i, pixel) in source
                .chunks_exact_mut(input_format.bytes_per_pixel())
                .enumerate()
            {
                match input_format {
                    PixelFormat::Argb8888 => pixel.copy_from_slice(&[
                        i as u8,
                        (i * 3) as u8,
                        (i * 5) as u8,
                        (i * 7) as u8,
                    ]),
                    PixelFormat::Rgb888 => {
                        pixel.copy_from_slice(&RGB[(i + 2 * (i / SIDE)) % RGB.len()])
                    }
                    _ => unreachable!(),
                }
            }
            let destination = &mut ctx.destination[..PIXELS * output_format.bytes_per_pixel()];
            destination.fill(0xa5);
            srm.run(
                Input {
                    data: source,
                    size,
                    format: input_format,
                },
                Output {
                    data: destination,
                    size,
                    format: output_format,
                    offset: Offset::default(),
                },
                Config::default(),
            )
            .unwrap();
            for (i, (input, output)) in source
                .chunks_exact(input_format.bytes_per_pixel())
                .zip(destination.chunks_exact(output_format.bytes_per_pixel()))
                .enumerate()
            {
                match output_format {
                    PixelFormat::Argb8888 => {
                        let alpha = if input_format == PixelFormat::Argb8888 {
                            input[3]
                        } else {
                            255
                        };
                        assert_eq!(
                            output,
                            &[input[0], input[1], input[2], alpha],
                            "pixel {i}, {input_format:?} -> {output_format:?}",
                        );
                    }
                    PixelFormat::Rgb888 => {
                        assert_eq!(
                            output,
                            &input[..3],
                            "pixel {i}, {input_format:?} -> {output_format:?}",
                        );
                    }
                    PixelFormat::Gray8 => assert_eq!(
                        output[0],
                        ((85 * input[2] as u32 + 86 * input[1] as u32 + 85 * input[0] as u32) >> 8)
                            as u8,
                        "pixel {i}, {input_format:?} -> {output_format:?}",
                    ),
                    PixelFormat::Uyvy422 => {
                        let expected = UYVY[(i + 2 * (i / SIDE)) % UYVY.len()];
                        // Allow rounding differences in BT.601 conversion.
                        for (byte, (&actual, expected)) in output.iter().zip(expected).enumerate() {
                            assert!(
                                actual.abs_diff(expected) <= 2,
                                "pixel {i}, byte {byte}, {input_format:?} -> {output_format:?}: actual={actual}, expected={expected}",
                            );
                        }
                    }
                    _ => unreachable!(),
                }
            }
        }
    }

    #[test]
    fn test_rgb565_odd_size_and_offset(mut ctx: Context) {
        let size = Size {
            width: 17,
            height: 32,
        };
        let source = &mut ctx.source[..size.width * size.height * 2];
        fill_source(source, size, PixelFormat::Rgb565);
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        check_unscaled(
            &mut srm,
            Input {
                data: source,
                size,
                format: PixelFormat::Rgb565,
            },
            ctx.destination,
            Offset { x: 3, y: 5 },
            Config::default().with_rotation(Rotation::Degrees90),
        );
    }

    #[test]
    fn test_buffer_layouts(mut ctx: Context) {
        let size = Size {
            width: 17,
            height: 3,
        };
        let bytes = size.width * size.height * 2;
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for (source_start, destination_start, padding) in
            [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (1, 1, 17)]
        {
            ctx.source.fill(0x5a);
            ctx.destination.fill(0xa5);
            let source = &mut ctx.source[source_start..source_start + bytes + padding];
            fill_source(source, size, PixelFormat::Rgb565);
            srm.run(
                Input {
                    data: source,
                    size,
                    format: PixelFormat::Rgb565,
                },
                Output {
                    data: &mut ctx.destination
                        [destination_start..destination_start + bytes + padding],
                    size,
                    format: PixelFormat::Rgb565,
                    offset: Offset::default(),
                },
                Config::default(),
            )
            .unwrap();
            assert_eq!(
                &ctx.destination[destination_start..destination_start + bytes],
                &source[..bytes],
                "source {source_start}, destination {destination_start}, padding {padding}",
            );
            assert!(
                ctx.destination[..destination_start]
                    .iter()
                    .all(|&byte| byte == 0xa5)
            );
            assert!(
                ctx.destination[destination_start + bytes..]
                    .iter()
                    .all(|&byte| byte == 0xa5)
            );
            assert!(ctx.source[..source_start].iter().all(|&byte| byte == 0x5a));
            assert!(
                ctx.source[source_start + bytes..]
                    .iter()
                    .all(|&byte| byte == 0x5a)
            );
        }
    }

    #[test]
    fn test_dimension_limits(mut ctx: Context) {
        use allocator_api2::vec::Vec;

        const DMA_MAX: usize = 0x3fff;
        const SRM_MAX: usize = 0x1fff;
        const CAPACITY: usize = (DMA_MAX * 3).next_multiple_of(64);
        let dimensions = |long, vertical| {
            if vertical {
                Size {
                    width: 3,
                    height: long,
                }
            } else {
                Size {
                    width: long,
                    height: 3,
                }
            }
        };
        let mut source = Vec::new_in(ExternalMemory);
        source.resize(CAPACITY, 0u8);
        for (i, byte) in source.iter_mut().enumerate() {
            *byte = ((17 * i) ^ (i >> 8)) as u8;
        }
        let mut storage = Vec::new_in(ExternalMemory);
        storage.resize(CAPACITY + 3 * 64 - 1, 0xa5);
        let start = storage.as_ptr().align_offset(64) + 64;
        for range in [start - 64..start, start + CAPACITY..start + CAPACITY + 64] {
            DmaAlignedMut::new(&mut storage[range]).unwrap().writeback();
        }
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for (dimension, step) in [(SRM_MAX, 1), (DMA_MAX, 2)] {
            for vertical in [false, true] {
                let size = dimensions(dimension, vertical);
                let scaled = dimensions(SRM_MAX, vertical);
                for rotation in [Rotation::Degrees0, Rotation::Degrees90] {
                    let config = if vertical {
                        Config::default().with_scale_y(1.0 / step as f32)
                    } else {
                        Config::default().with_scale_x(1.0 / step as f32)
                    }
                    .with_rotation(rotation);
                    let canvas_vertical = vertical != (rotation == Rotation::Degrees90);
                    let canvas = dimensions(DMA_MAX, canvas_vertical);
                    let transformed = dimensions(SRM_MAX, canvas_vertical);
                    let offset = Offset {
                        x: canvas.width - transformed.width,
                        y: canvas.height - transformed.height,
                    };
                    assert_eq!(config.output_size(size), Ok(transformed));
                    let destination = &mut storage[start..start + CAPACITY];
                    destination.fill(0xa5);
                    assert_eq!(
                        srm.run(
                            Input {
                                data: &source[..size.width * size.height],
                                size,
                                format: PixelFormat::Gray8,
                            },
                            Output {
                                data: destination,
                                size: canvas,
                                format: PixelFormat::Gray8,
                                offset,
                            },
                            config,
                        ),
                        Ok(()),
                        "{size:?}, {config:?}",
                    );
                    for y in 0..canvas.height {
                        for x in 0..canvas.width {
                            let (expected, tolerance) = if x >= offset.x && y >= offset.y {
                                let (x, y) = (x - offset.x, y - offset.y);
                                let (x, y) = match rotation {
                                    Rotation::Degrees0 => (x, y),
                                    Rotation::Degrees90 => (scaled.width - 1 - y, x),
                                    _ => unreachable!(),
                                };
                                let (first, second) = if vertical {
                                    let first = y * step * size.width + x;
                                    (first, first + (step - 1) * size.width)
                                } else {
                                    let first = y * size.width + x * step;
                                    (first, first + step - 1)
                                };
                                (
                                    (source[first] as u16 + source[second] as u16).div_ceil(2)
                                        as u8,
                                    (step - 1) as u8,
                                )
                            } else {
                                (0xa5, 0)
                            };
                            let actual = destination[y * canvas.width + x];
                            assert!(
                                actual.abs_diff(expected) <= tolerance,
                                "pixel ({x}, {y}), {size:?}, {config:?}: actual={actual}, expected={expected}",
                            );
                        }
                    }
                    assert!(
                        destination[canvas.width * canvas.height..]
                            .iter()
                            .all(|&b| b == 0xa5)
                    );
                    for range in [start - 64..start, start + CAPACITY..start + CAPACITY + 64] {
                        let guard = DmaAlignedMut::new(&mut storage[range]).unwrap();
                        guard.invalidate();
                        assert!(guard.iter().all(|&byte| byte == 0xa5));
                    }
                }
            }
        }
        for dimension in [SRM_MAX + 1, DMA_MAX] {
            for vertical in [false, true] {
                let size = dimensions(dimension, vertical);
                for rotation in [Rotation::Degrees0, Rotation::Degrees90] {
                    let config = Config::default().with_rotation(rotation);
                    let canvas = dimensions(DMA_MAX, vertical != (rotation == Rotation::Degrees90));
                    assert_eq!(config.output_size(size), Err(Error::InvalidSize));
                    let destination = &mut storage[start..start + CAPACITY];
                    destination.fill(0xa5);
                    assert_eq!(
                        srm.run(
                            Input {
                                data: &source[..size.width * size.height],
                                size,
                                format: PixelFormat::Gray8,
                            },
                            Output {
                                data: destination,
                                size: canvas,
                                format: PixelFormat::Gray8,
                                offset: Offset::default(),
                            },
                            config,
                        ),
                        Err(Error::InvalidSize),
                    );
                    assert!(destination.iter().all(|&byte| byte == 0xa5));
                }
            }
        }
    }

    #[test]
    fn test_invalid_sizes(mut ctx: Context) {
        let size = Size {
            width: 17,
            height: 3,
        };
        let bytes = size.width * size.height * 2;
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for (source_len, destination_len, offset) in [
            (bytes - 1, bytes, Offset::default()),
            (bytes, bytes - 1, Offset::default()),
            (bytes, bytes, Offset { x: 1, y: 0 }),
            (bytes, bytes, Offset { x: 0, y: 1 }),
            (
                bytes,
                bytes,
                Offset {
                    x: usize::MAX,
                    y: 0,
                },
            ),
            (
                bytes,
                bytes,
                Offset {
                    x: 0,
                    y: usize::MAX,
                },
            ),
        ] {
            ctx.destination.fill(0xa5);
            assert_eq!(
                srm.run(
                    Input {
                        data: &ctx.source[..source_len],
                        size,
                        format: PixelFormat::Rgb565,
                    },
                    Output {
                        data: &mut ctx.destination[..destination_len],
                        size,
                        format: PixelFormat::Rgb565,
                        offset,
                    },
                    Config::default(),
                ),
                Err(Error::InvalidSize),
            );
            assert!(ctx.destination.iter().all(|&byte| byte == 0xa5));
        }
    }

    #[test]
    fn test_invalid_dimensions(mut ctx: Context) {
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for dimension in [0, 0x4000, usize::MAX] {
            for invalid in [
                Size {
                    width: dimension,
                    height: 1,
                },
                Size {
                    width: 1,
                    height: dimension,
                },
            ] {
                for (input_size, output_size) in [(invalid, SOURCE_SIZE), (SOURCE_SIZE, invalid)] {
                    ctx.destination.fill(0xa5);
                    assert_eq!(
                        srm.run(
                            Input {
                                data: ctx.source,
                                size: input_size,
                                format: PixelFormat::Rgb565,
                            },
                            Output {
                                data: ctx.destination,
                                size: output_size,
                                format: PixelFormat::Rgb565,
                                offset: Offset::default(),
                            },
                            Config::default(),
                        ),
                        Err(Error::InvalidSize),
                        "{input_size:?} -> {output_size:?}",
                    );
                    assert!(ctx.destination.iter().all(|&byte| byte == 0xa5));
                }
            }
        }
    }

    #[test]
    fn test_invalid_scaling(mut ctx: Context) {
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for (scale, expected) in [
            (-1.0, Error::InvalidScale),
            (0.0, Error::InvalidScale),
            (1.0 / 32.0, Error::InvalidScale),
            (1.0 / 16.0, Error::InvalidSize),
            (256.0, Error::InvalidScale),
            (f32::NAN, Error::InvalidScale),
            (f32::INFINITY, Error::InvalidScale),
            (f32::NEG_INFINITY, Error::InvalidScale),
        ] {
            for config in [
                Config::default().with_scale_x(scale),
                Config::default().with_scale_y(scale),
            ] {
                ctx.destination.fill(0xa5);
                assert_eq!(
                    srm.run(
                        Input {
                            data: ctx.source,
                            size: Size {
                                width: 1,
                                height: 1,
                            },
                            format: PixelFormat::Rgb565,
                        },
                        Output {
                            data: ctx.destination,
                            size: SOURCE_SIZE,
                            format: PixelFormat::Rgb565,
                            offset: Offset::default(),
                        },
                        config,
                    ),
                    Err(expected),
                    "{config:?}",
                );
                assert!(ctx.destination.iter().all(|&byte| byte == 0xa5));
            }
        }
    }

    #[test]
    fn test_invalid_uyvy422_sizes(mut ctx: Context) {
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for (input_format, output_format, input_width, output_width, offset_x) in [
            (PixelFormat::Uyvy422, PixelFormat::Rgb565, 17, 20, 0),
            (PixelFormat::Rgb565, PixelFormat::Uyvy422, 16, 19, 0),
            (PixelFormat::Rgb565, PixelFormat::Uyvy422, 16, 20, 1),
            (PixelFormat::Rgb565, PixelFormat::Uyvy422, 17, 20, 0),
        ] {
            ctx.destination.fill(0xa5);
            assert_eq!(
                srm.run(
                    Input {
                        data: ctx.source,
                        size: Size {
                            width: input_width,
                            height: 2,
                        },
                        format: input_format,
                    },
                    Output {
                        data: ctx.destination,
                        size: Size {
                            width: output_width,
                            height: 2,
                        },
                        format: output_format,
                        offset: Offset { x: offset_x, y: 0 },
                    },
                    Config::default(),
                ),
                Err(Error::InvalidSize),
                "{input_format:?} -> {output_format:?}, widths {input_width} -> {output_width}, offset {offset_x}",
            );
            assert!(ctx.destination.iter().all(|&byte| byte == 0xa5));
        }
    }

    #[test]
    fn test_psram(mut ctx: Context) {
        let mut source = Box::new_in(Buffer([0x5a; SOURCE_BYTES + 128]), ExternalMemory);
        let mut destination = Box::new_in(Buffer([0xa5; DESTINATION_BYTES + 128]), ExternalMemory);
        // Seed guard bytes in RAM before dirtying the source image.
        DmaAlignedMut::new(&mut source.0).unwrap().writeback();
        DmaAlignedMut::new(&mut destination.0[..64])
            .unwrap()
            .writeback();
        fill_source(
            &mut source.0[1..SOURCE_BYTES + 1],
            SOURCE_SIZE,
            PixelFormat::Rgb565,
        );
        let input = Input {
            data: &source.0[1..SOURCE_BYTES + 18],
            size: SOURCE_SIZE,
            format: PixelFormat::Rgb565,
        };
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for (start, len) in [(1, DESTINATION_BYTES), (0, DESTINATION_BYTES + 1)] {
            assert_eq!(
                srm.run(
                    Input { ..input },
                    Output {
                        data: &mut destination.0[start..start + len],
                        size: Size {
                            width: DESTINATION_SIZE,
                            height: DESTINATION_SIZE,
                        },
                        format: PixelFormat::Rgb565,
                        offset: Offset::default(),
                    },
                    Config::default(),
                ),
                Err(Error::UnalignedBuffer),
            );
            assert!(destination.0.iter().all(|&byte| byte == 0xa5));
        }
        fill_source(ctx.source, SOURCE_SIZE, PixelFormat::Rgb565);
        for rotation in [Rotation::Degrees0, Rotation::Degrees90] {
            check_unscaled(
                &mut srm,
                Input { ..input },
                ctx.destination,
                OFFSET,
                Config::default().with_rotation(rotation),
            );
            for data in [ctx.source.as_slice(), input.data] {
                check_unscaled(
                    &mut srm,
                    Input { data, ..input },
                    &mut destination.0[64..],
                    OFFSET,
                    Config::default().with_rotation(rotation),
                );
                DmaAlignedMut::new(&mut destination.0[..64])
                    .unwrap()
                    .invalidate();
                assert!(destination.0[..64].iter().all(|&byte| byte == 0xa5));
                assert!(
                    destination.0[64 + DESTINATION_BYTES..]
                        .iter()
                        .all(|&byte| byte == 0xa5)
                );
            }
        }
        DmaAlignedMut::new(&mut source.0).unwrap().invalidate();
        assert_eq!(source.0[0], 0x5a);
        assert!(
            source.0[SOURCE_BYTES + 1..]
                .iter()
                .all(|&byte| byte == 0x5a)
        );
    }

    #[test]
    fn test_hardware_error_recovery(mut ctx: Context) {
        use core::sync::atomic::{AtomicBool, Ordering};

        use allocator_api2::vec::Vec;
        use esp_hal::{handler, interrupt, peripherals::Interrupt, system::Cpu};

        static INJECTED: AtomicBool = AtomicBool::new(false);
        #[unsafe(link_section = ".data")]
        static INVALID: Buffer<24> = Buffer([0; 24]);

        #[handler]
        fn inject_error() {
            let dma = DMA2D::regs();
            dma.out_int_ena_ch(0)
                .write(|w| w.out_done_ch_int_ena().clear_bit());
            dma.out_link_conf_ch(0)
                .modify(|_, w| w.outlink_stop_ch().set_bit());
            dma.out_conf0_ch(0)
                .modify(|_, w| w.out_cmd_disable_ch().set_bit());
            while !dma.out_state_ch(0).read().out_reset_avail_ch().bit() {
                core::hint::spin_loop();
            }
            dma.out_conf0_ch(0).modify(|_, w| w.out_rst_ch().set_bit());
            dma.out_conf0_ch(0).modify(|_, w| {
                w.out_rst_ch().clear_bit();
                w.out_cmd_disable_ch().clear_bit();
                w.out_check_owner_ch().set_bit()
            });
            dma.out_link_addr_ch(0)
                .write(|w| unsafe { w.outlink_addr_ch().bits((&raw const INVALID) as u32) });
            dma.out_link_conf_ch(0)
                .modify(|_, w| w.outlink_start_ch().set_bit());
            while !dma.out_int_raw_ch(0).read().out_dscr_err_ch_int_raw().bit() {
                core::hint::spin_loop();
            }
            INJECTED.store(true, Ordering::Relaxed);
        }

        let size = Size {
            width: SOURCE_SIZE.width * 8,
            height: SOURCE_SIZE.height * 8,
        };
        let bytes = size.width * size.height * 2;
        let mut storage = Vec::new_in(ExternalMemory);
        // Own complete cache lines for both guards.
        storage.resize(bytes + 3 * 64 - 1, 0xa5);
        let start = storage.as_ptr().align_offset(64) + 64;
        for range in [start - 64..start, start + bytes..start + bytes + 64] {
            DmaAlignedMut::new(&mut storage[range]).unwrap().writeback();
        }
        fill_source(ctx.source, SOURCE_SIZE, PixelFormat::Rgb565);
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        interrupt::bind_handler(Interrupt::DMA2D_OUT_CH0, inject_error);
        for rotation in [Rotation::Degrees0, Rotation::Degrees90] {
            INJECTED.store(false, Ordering::Relaxed);
            let dma = DMA2D::regs();
            dma.out_int_clr_ch(0)
                .write(|w| w.out_done_ch_int_clr().set_bit());
            // Upscaling keeps RX active while TX completion triggers the descriptor fault.
            dma.out_int_ena_ch(0)
                .write(|w| w.out_done_ch_int_ena().set_bit());
            let result = srm.run(
                Input {
                    data: ctx.source,
                    size: SOURCE_SIZE,
                    format: PixelFormat::Rgb565,
                },
                Output {
                    data: &mut storage[start..start + bytes],
                    size,
                    format: PixelFormat::Rgb565,
                    offset: Offset::default(),
                },
                Config::default().with_scale_x(8.0).with_scale_y(8.0),
            );
            assert!(INJECTED.load(Ordering::Relaxed));
            assert_eq!(result, Err(Error::Hardware));
            for range in [start - 64..start, start + bytes..start + bytes + 64] {
                let guard = DmaAlignedMut::new(&mut storage[range]).unwrap();
                guard.invalidate();
                assert!(guard.iter().all(|&byte| byte == 0xa5));
            }
            check_unscaled(
                &mut srm,
                Input {
                    data: ctx.source,
                    size: SOURCE_SIZE,
                    format: PixelFormat::Rgb565,
                },
                ctx.destination,
                OFFSET,
                Config::default().with_rotation(rotation),
            );
        }
        interrupt::disable(Cpu::current(), Interrupt::DMA2D_OUT_CH0);
    }

    #[test]
    fn test_reinitialization(mut ctx: Context) {
        fill_source(ctx.source, SOURCE_SIZE, PixelFormat::Rgb565);
        for _ in 0..2 {
            let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
            check_unscaled(
                &mut srm,
                Input {
                    data: ctx.source,
                    size: SOURCE_SIZE,
                    format: PixelFormat::Rgb565,
                },
                ctx.destination,
                OFFSET,
                Config::default(),
            );
        }
    }

    #[test]
    fn test_unsupported_memory(mut ctx: Context) {
        #[unsafe(link_section = ".rodata")]
        static SOURCE: Buffer<SOURCE_BYTES> = Buffer([0; SOURCE_BYTES]);

        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        ctx.destination.fill(0xa5);
        assert_eq!(
            srm.run(
                Input {
                    data: &SOURCE.0,
                    size: SOURCE_SIZE,
                    format: PixelFormat::Rgb565,
                },
                Output {
                    data: ctx.destination,
                    size: Size {
                        width: DESTINATION_SIZE,
                        height: DESTINATION_SIZE,
                    },
                    format: PixelFormat::Rgb565,
                    offset: Offset::default(),
                },
                Config::default(),
            ),
            Err(Error::UnsupportedMemoryRegion),
        );
        assert!(ctx.destination.iter().all(|&byte| byte == 0xa5));

        fill_source(ctx.source, SOURCE_SIZE, PixelFormat::Rgb565);
        check_unscaled(
            &mut srm,
            Input {
                data: ctx.source,
                size: SOURCE_SIZE,
                format: PixelFormat::Rgb565,
            },
            ctx.destination,
            OFFSET,
            Config::default(),
        );
    }
}
