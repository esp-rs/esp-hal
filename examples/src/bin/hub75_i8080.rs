//! Drives a 64x64 LED matrix using the Hub75e protocol
//!
//! This example draws a color gradient on the top 24 rors of the matrx
//! and displays the refresh and render rates on the bottom.
//!
//! Pins used: (Level convcerters are usuallu required 3.3v->5v)
//!
//! R1     GPIO38
//! G1     GPIO42
//! B1     GPIO48
//! R2     GPIO47
//! G2     GPIO2
//! B2     GPIO21
//! A      GPIO14
//! B      GPIO46
//! C      GPIO13
//! D      GPIO9
//! E      GPIO3
//! OE     GPIO11
//! CLK    GPIO12
//! LAT    GPIO10

//% CHIPS: esp32s3
//% FEATURES: async embassy embassy-time-timg0 embassy-generic-timers bitfield

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use alloc::fmt;
use core::{
    mem::MaybeUninit,
    ops::DerefMut,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use bitfield::bitfield;
use embassy_executor::{task, Spawner};
use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::{
    framebuffer::{buffer_size, Framebuffer},
    geometry::Point,
    image::GetPixel,
    iterator::raw::RawDataSlice,
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
    pixelcolor::{
        raw::{ByteOrder, LittleEndian},
        Rgb888,
        RgbColor,
    },
    text::{Alignment, Text},
    Drawable,
};
use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, Clocks},
    cpu_control::{CpuControl, Stack},
    dma::{Dma, DmaPriority},
    gpio::{any_pin::AnyPin, dummy_pin::DummyPin, GpioPin, Io},
    lcd_cam::{
        lcd::{
            i8080,
            i8080::{Command, TxSixteenBits, I8080},
        },
        LcdCam,
    },
    peripherals::{Peripherals, LCD_CAM},
    prelude::*,
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_println::println;
use heapless::String;
use static_cell::StaticCell;

extern crate alloc;
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 1 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq)]
    #[repr(transparent)]
    struct Entry(u16);
    impl Debug;
    blu2, set_blu2: 15;
    grn2, set_grn2: 14;
    red2, set_red2: 13;
    blu1, set_blu1: 12;
    grn1, set_grn1: 11;
    red1, set_red1: 10;
    dummy2, set_dummy2: 9;
    dummy1, set_dummy1: 8;
    output_enable, set_output_enable: 7;
    clock, set_clock: 6;
    latch, set_latch: 5;
    addr, set_addr: 4, 0;
}

impl Entry {
    const fn new() -> Self {
        Self(0)
    }

    fn set_colors<C: RgbColor>(&mut self, color0: C, color1: C, brightness: u8) {
        self.set_red1(color0.r() >= brightness);
        self.set_grn1(color0.g() >= brightness);
        self.set_blu1(color0.b() >= brightness);
        self.set_red2(color1.r() >= brightness);
        self.set_grn2(color1.g() >= brightness);
        self.set_blu2(color1.b() >= brightness);
    }
}

type Color = Rgb888;
const BRIGHTNESS_BITS: u8 = 3; // must be < the number of bits per pixel!!!!
const BRIGHTNESS_COUNT: u8 = (1 << BRIGHTNESS_BITS) - 1;
const BRIGHTNESS_STEP: u8 = 1 << (8 - BRIGHTNESS_BITS);

// The matrix size
const MATRIX_COLS: usize = 64;
const MATRIX_ROWS: usize = 64;
const FRAMEBUFFER_SIZE: usize = buffer_size::<Color>(MATRIX_COLS, MATRIX_ROWS);

const BLANKING_DELAY: usize = 7;
const LATCH_DELAY: usize = 4;
// sizing for DMA buffers
const DMA_ROW_SIZE: usize = 64 * 2 + BLANKING_DELAY + LATCH_DELAY + 3; // 64 pixels blank and latch delay + + 1 for blank, 1 for unlatch, 1 for latch
const DMA_FRAME_SIZE: usize = DMA_ROW_SIZE * MATRIX_ROWS / 2; // Each "row" in the DMA buffer is 2 rows of the matrix
const DMA_FRAMES_PER_BUFFER: usize = 1 << BRIGHTNESS_BITS; // Multiple frames for BCM ()
const DMA_BUFFER_SIZE: usize = DMA_FRAME_SIZE * DMA_FRAMES_PER_BUFFER;

type DmaBufferType = Mutex<CriticalSectionRawMutex, [Entry; DMA_BUFFER_SIZE]>;

// we double buffer, while one us displayed on the panel the other is used to render the next display
static BUFFER0: DmaBufferType = Mutex::new([Entry::new(); DMA_BUFFER_SIZE]);
static BUFFER1: DmaBufferType = Mutex::new([Entry::new(); DMA_BUFFER_SIZE]);
static DISPLAY_BUFFER: AtomicBool = AtomicBool::new(false);

pub fn swap_buffers() {
    DISPLAY_BUFFER.store(!DISPLAY_BUFFER.load(Ordering::Relaxed), Ordering::Relaxed);
}

macro_rules! get_display_buffer {
    () => {
        if DISPLAY_BUFFER.load(Ordering::Relaxed) {
            BUFFER0.lock().await
        } else {
            BUFFER1.lock().await
        }
    };
}
macro_rules! get_render_buffer {
    () => {
        if DISPLAY_BUFFER.load(Ordering::Relaxed) {
            BUFFER1.lock().await
        } else {
            BUFFER0.lock().await
        }
    };
}

// render a single frame (BCM) into a DMA buffer
fn render_frame<C, BO, const WIDTH: usize, const HEIGHT: usize, const N: usize>(
    buffer: &mut [Entry],
    fb: &Framebuffer<C, C::Raw, BO, WIDTH, HEIGHT, N>,
    brightness: u8,
) where
    C: RgbColor + From<C::Raw>,
    BO: ByteOrder,
    for<'a> RawDataSlice<'a, C::Raw, BO>: IntoIterator<Item = C::Raw>,
{
    let mut prev_addr = 0u8;
    for y in 0..(HEIGHT / 2) as u8 {
        // we render in reverse order because when the lcd_cam device finishes renering
        // it sets the address lines back to 0 causing ghosting
        let addr = HEIGHT as u8 / 2 - 1 - y;
        let start = y as usize * DMA_ROW_SIZE;
        let mut entry = Entry::new();
        // render pixels first
        // each pixel pair (2 rows) takes 2 Entries, one with the clock low and one with the clock high
        for x in 0..WIDTH {
            let color0 = fb.pixel(Point::new(x as i32, addr as i32)).unwrap();
            let color1 = fb
                .pixel(Point::new(x as i32, (addr + HEIGHT as u8 / 2) as i32))
                .unwrap();
            entry.set_addr(prev_addr as u16);
            entry.set_output_enable(true);
            entry.set_latch(false);
            entry.set_colors(color0, color1, brightness);

            let i2 = start + x * 2;

            entry.set_clock(false);
            buffer[i2] = entry;

            entry.set_clock(true);
            buffer[i2 + 1] = entry;
        }

        let mut i = start + WIDTH * 2;

        // one entry to set the clock low and disable output
        // output is disabled to prevent ghosting when the address lines are changed
        entry.set_clock(false);
        entry.set_output_enable(false);
        buffer[i] = entry;
        i += 1;

        // delay for blanking, without some delay there would be ghosting
        for _ in 0..BLANKING_DELAY {
            buffer[i] = entry;
            i += 1;
        }

        // open latch to grab the new values shifted in above and set the new address
        entry.set_latch(true);
        entry.set_addr(addr as u16);
        buffer[i] = entry; // unlatch
        i += 1;

        // close latch to lock in the values
        entry.set_latch(false);
        buffer[i] = entry;
        i += 1;

        // another delay to prevent ghosting
        for _ in 0..LATCH_DELAY {
            buffer[i] = entry;
            i += 1;
        }

        // next address
        prev_addr = addr;
    }
}

// render the whole frame buffer into a DMA buffer
fn render_buffer<C, BO, const WIDTH: usize, const HEIGHT: usize, const N: usize>(
    buffer: &mut [Entry],
    fb: &Framebuffer<C, C::Raw, BO, WIDTH, HEIGHT, N>,
) where
    C: RgbColor + From<C::Raw>,
    BO: ByteOrder,
    for<'a> RawDataSlice<'a, C::Raw, BO>: IntoIterator<Item = C::Raw>,
{
    // Binary Code Modulation a.k.a. Bit Angle Modulation used to control color intensity per pixel
    for brightness in 0..BRIGHTNESS_COUNT {
        let start = brightness as usize * DMA_FRAME_SIZE;
        let end = start + DMA_FRAME_SIZE;
        let buffer = &mut buffer[start..end];
        let brightness = (brightness + 1).saturating_mul(BRIGHTNESS_STEP);
        render_frame(buffer, fb, brightness);
    }
}

pub static REFRESH_RATE: AtomicU32 = AtomicU32::new(0);
pub static RENDER_RATE: AtomicU32 = AtomicU32::new(0);

#[task]
async fn display_task() {
    println!("display task started");
    // TODO: implement the embedded-graphics traits for the DMA buffer
    //       that should improve the performance somewhat
    let mut fb: Framebuffer<Color, _, LittleEndian, MATRIX_COLS, MATRIX_ROWS, FRAMEBUFFER_SIZE> =
        Framebuffer::new();

    const STEP: u8 = (256 / MATRIX_COLS) as u8;
    for x in 0..MATRIX_COLS {
        let brightness = (x as u8) * STEP;
        for y in 0..8 {
            fb.set_pixel(Point::new(x as i32, y), Color::new(brightness, 0, 0));
        }
        for y in 8..16 {
            fb.set_pixel(Point::new(x as i32, y), Color::new(0, brightness, 0));
        }
        for y in 16..24 {
            fb.set_pixel(Point::new(x as i32, y), Color::new(0, 0, brightness));
        }
    }

    let fps_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();
    let mut count = 0u32;
    let mut start = Instant::now();
    loop {
        let mut buffer: String<64> = String::new();

        fmt::write(
            &mut buffer,
            format_args!("Refresh {:4}", REFRESH_RATE.load(Ordering::Relaxed)),
        )
        .unwrap();

        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, 63),
            fps_style,
            Alignment::Left,
        )
        .draw(&mut fb)
        .unwrap();

        buffer.clear();
        fmt::write(
            &mut buffer,
            format_args!("Render  {:4}", RENDER_RATE.load(Ordering::Relaxed)),
        )
        .unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, 63 - 8),
            fps_style,
            Alignment::Left,
        )
        .draw(&mut fb)
        .unwrap();

        let mut buffer = get_render_buffer!();
        render_buffer(buffer.deref_mut(), &fb);
        swap_buffers();
        count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            RENDER_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
        }
        yield_now().await;
    }
}

struct DisplayPeripherals {
    pub lcd_cam: LCD_CAM,
    pub dma_channel: esp_hal::dma::ChannelCreator<0>,
    pub red1: GpioPin<38>,
    pub grn1: GpioPin<42>,
    pub blu1: GpioPin<48>,
    pub red2: GpioPin<47>,
    pub grn2: GpioPin<2>,
    pub blu2: GpioPin<21>,
    pub addr0: GpioPin<14>,
    pub addr1: GpioPin<46>,
    pub addr2: GpioPin<13>,
    pub addr3: GpioPin<9>,
    pub addr4: GpioPin<3>,
    pub blank: GpioPin<11>,
    pub clock: GpioPin<12>,
    pub latch: GpioPin<10>,
}

#[task]
async fn hub75_task(peripherals: DisplayPeripherals, clocks: Clocks<'static>) {
    let channel: esp_hal::dma::ChannelCreator<0> = peripherals.dma_channel;
    let (tx_descriptors, _) = esp_hal::dma_descriptors!(DMA_BUFFER_SIZE * 2, 0);

    let channel = channel.configure(false, DmaPriority::Priority0);
    let pins = TxSixteenBits::new(
        AnyPin::new(peripherals.addr0),
        AnyPin::new(peripherals.addr1),
        AnyPin::new(peripherals.addr2),
        AnyPin::new(peripherals.addr3),
        AnyPin::new(peripherals.addr4),
        AnyPin::new(peripherals.latch),
        AnyPin::new(peripherals.clock),
        AnyPin::new_inverted(peripherals.blank),
        DummyPin::new(),
        DummyPin::new(),
        AnyPin::new(peripherals.red1),
        AnyPin::new(peripherals.grn1),
        AnyPin::new(peripherals.blu1),
        AnyPin::new(peripherals.red2),
        AnyPin::new(peripherals.grn2),
        AnyPin::new(peripherals.blu2),
    );
    let lcd_cam = LcdCam::new(peripherals.lcd_cam);

    let mut i8080 = I8080::new(
        lcd_cam.lcd,
        channel.tx,
        tx_descriptors,
        pins,
        40.MHz(),
        i8080::Config::default(),
        &clocks,
    );

    let mut count = 0u32;
    let mut start = Instant::now();

    loop {
        let x = unsafe { core::mem::transmute::<&[Entry], &[u16]>(&*get_display_buffer!()) };
        let xfer = i8080.send_dma(Command::<u16>::None, 0, &x).unwrap();
        // TODO: Starting the next transfer in an ISR woulb be much more efficient, we
        // would not need to waist a whole core "waiting"
        xfer.wait().unwrap();
        count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            REFRESH_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
        }
    }
}

extern "C" {
    static _stack_end_cpu0: u32;
    static _stack_start_cpu0: u32;
}

#[main]
async fn main(spawner: Spawner) {
    println!("Init!");
    println!("stack size:  {}", unsafe {
        core::ptr::addr_of!(_stack_start_cpu0).offset_from(core::ptr::addr_of!(_stack_end_cpu0))
    });
    println!("FRAMEBUFFER_SIZE: {}", FRAMEBUFFER_SIZE);
    println!("DMA_ROW_SIZE: {}", DMA_ROW_SIZE);
    println!("DMA_FRAME_SIZE: {}", DMA_FRAME_SIZE);
    println!("DMA_BUFFER_SIZE: {}", DMA_BUFFER_SIZE);

    init_heap();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let dma = Dma::new(peripherals.DMA);
    esp_hal_embassy::init(&clocks, timer_group0);

    spawner.spawn(display_task()).ok();

    let display_peripherals = DisplayPeripherals {
        lcd_cam: peripherals.LCD_CAM,
        dma_channel: dma.channel0,
        red1: io.pins.gpio38,
        grn1: io.pins.gpio42,
        blu1: io.pins.gpio48,
        red2: io.pins.gpio47,
        grn2: io.pins.gpio2,
        blu2: io.pins.gpio21,
        addr0: io.pins.gpio14,
        addr1: io.pins.gpio46,
        addr2: io.pins.gpio13,
        addr3: io.pins.gpio9,
        addr4: io.pins.gpio3,
        blank: io.pins.gpio11,
        clock: io.pins.gpio12,
        latch: io.pins.gpio10,
    };
    // run hub75 on second core
    let cpu1_fnctn = {
        move || {
            let executor = static_cell::make_static!(esp_hal_embassy::Executor::new());
            executor.run(|spawner| {
                spawner.spawn(hub75_task(display_peripherals, clocks)).ok();
            });
        }
    };

    const DISPLAY_STACK_SIZE: usize = 4096;
    static APP_CORE_STACK: StaticCell<Stack<DISPLAY_STACK_SIZE>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());
    let mut _cpu_control = cpu_control;

    println!("starting Hub75 task");
    #[allow(static_mut_refs)]
    let _guard = _cpu_control
        .start_app_core(app_core_stack, cpu1_fnctn)
        .unwrap();

    loop {
        Timer::after(Duration::from_millis(100)).await;
    }
}
