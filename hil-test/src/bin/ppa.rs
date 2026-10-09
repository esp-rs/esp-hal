//! PPA SRM tests. No external wiring is required.

//% CHIP_FILTER: pixel_accelerator_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    Blocking,
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
        const SIDE: usize = 32;
        let size = Size {
            width: 16,
            height: 8,
        };
        let offset = Offset { x: 3, y: 5 };
        let source = &mut ctx.source[..size.width * size.height];
        for y in 0..size.height {
            for x in 0..size.width {
                source[y * size.width + x] = (4 * x + 12 * y) as u8;
            }
        }
        let destination = &mut ctx.destination[..SIDE * SIDE];
        let mut srm = Srm::new(ctx.ppa.reborrow(), ctx.dma.reborrow());
        for (config, output_size) in [
            (
                Config::default()
                    .with_scale_x(0.5)
                    .with_scale_y(0.25)
                    .with_rotation(Rotation::Degrees90)
                    .with_mirror_x(true),
                Size {
                    width: 2,
                    height: 8,
                },
            ),
            (
                Config::default().with_scale_x(1.5).with_scale_y(2.0),
                Size {
                    width: 24,
                    height: 16,
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
                        let x = ((x as f32 + 0.5) / config.scale_x() - 0.5)
                            .clamp(0.0, (size.width - 1) as f32);
                        let y = ((y as f32 + 0.5) / config.scale_y() - 0.5)
                            .clamp(0.0, (size.height - 1) as f32);
                        (
                            (4.0 * x + 12.0 * y + 0.5) as u8,
                            u8::from(config.scale_x() > 1.0 || config.scale_y() > 1.0),
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
