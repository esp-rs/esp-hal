//! The 800x480 RGB LCD of the Korvo-1, fed from internal RAM.
//!
//! PSRAM cannot feed the LCD while the camera and the CPU use it too: the DMA falls
//! behind, and the picture shifts. So the LCD's DMA reads two small bounce buffers in
//! internal RAM in a loop instead. When it finishes one, an interrupt draws the next
//! rows into it, see [`scene::draw_rows`].

use core::{
    ptr::addr_of_mut,
    sync::atomic::{AtomicU32, AtomicUsize, Ordering},
};

use embassy_time::Instant;
use esp_hal::{
    Blocking,
    dma::{BurstConfig, DmaDescriptor, DmaTxBuffer, Owner, Preparation},
    gpio::Level,
    handler,
    interrupt::{self, Priority},
    lcd_cam::{
        ByteOrder,
        lcd::{
            ClockMode,
            DelayMode,
            Lcd,
            Phase,
            Polarity,
            dpi::{Config, Dpi, DpiTransfer, Format, FrameTiming},
        },
    },
    peripherals::{AXI_GDMA, Interrupt, LCD_CAM},
    ram,
    time::Rate,
};
use static_cell::ConstStaticCell;

use crate::{LcdResources, scene};

pub const WIDTH: usize = 800;
pub const HEIGHT: usize = 480;

/// Rows per bounce buffer. The interrupt has the time of this many rows to draw them,
/// and 8 rows are too short while the other core loads a photo.
const BOUNCE_ROWS: usize = 16;
const BOUNCE_PIXELS: usize = WIDTH * BOUNCE_ROWS;
const CHUNKS_PER_FRAME: usize = HEIGHT / BOUNCE_ROWS;
/// Two rows per descriptor, because a descriptor holds at most 4095 bytes.
const DESCRIPTOR_BYTES: usize = WIDTH * 2 * 2;
const DESCRIPTORS_PER_BUFFER: usize = BOUNCE_PIXELS * 2 / DESCRIPTOR_BYTES;
/// The channel of `DMA_AXI_CH0`.
const LCD_CHANNEL: usize = 0;

#[repr(C, align(64))]
struct BounceBuffers([[u16; BOUNCE_PIXELS]; 2]);

static mut BUFFERS: BounceBuffers = BounceBuffers([[0; BOUNCE_PIXELS]; 2]);
static DESCRIPTORS: ConstStaticCell<[DmaDescriptor; 2 * DESCRIPTORS_PER_BUFFER]> =
    ConstStaticCell::new([DmaDescriptor::EMPTY; 2 * DESCRIPTORS_PER_BUFFER]);

/// The address of the last descriptor of the first bounce buffer.
static FIRST_BUFFER_END: AtomicU32 = AtomicU32::new(0);
/// The bounce buffer that the DMA finished last.
static LAST_FINISHED: AtomicUsize = AtomicUsize::new(1);
/// The chunk of rows that goes into the next finished bounce buffer.
static NEXT_CHUNK: AtomicUsize = AtomicUsize::new(2);
static UNDERRUNS: AtomicU32 = AtomicU32::new(0);
static LATE_REFILLS: AtomicU32 = AtomicU32::new(0);
static RESYNCS: AtomicU32 = AtomicU32::new(0);
static DRAW_MICROS: AtomicU32 = AtomicU32::new(0);

/// The running LCD.
pub struct Display {
    _transfer: DpiTransfer<'static, BounceChain, Blocking>,
}

impl Display {
    /// Starts showing what [`scene::draw_rows`] draws. The interrupt runs on the core
    /// that calls this.
    pub fn new(lcd: Lcd<'static, Blocking>, r: LcdResources<'static>) -> Self {
        let dpi = connect(lcd, r);

        let chain = BounceChain::new();
        // SAFETY: the DMA does not run yet, and the interrupt is not bound yet.
        unsafe {
            let buffers = &mut *addr_of_mut!(BUFFERS);
            scene::draw_rows(0, &mut buffers.0[0]);
            scene::draw_rows(BOUNCE_ROWS, &mut buffers.0[1]);
        }
        interrupt::bind_handler(Interrupt::AXI_PDMA_OUT_CH0, refill);

        let transfer = dpi.send(true, chain).map_err(|error| error.0).unwrap();

        let channel = AXI_GDMA::regs().out_ch(LCD_CHANNEL);
        channel.out_pri().write(|w| unsafe { w.tx_pri().bits(6) });
        channel.out_conf0().modify(|_, w| {
            // EOF when the DMA has read a buffer. The reset value raises EOF when the
            // data leaves the FIFO, and then the channel raises it only for every third
            // bounce buffer.
            // See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_lcd/rgb/esp_lcd_panel_rgb.c#L1250-L1253>.
            w.out_eof_mode().clear_bit();
            unsafe { w.out_burst_size_sel().bits(1) }
        });
        channel
            .out_int()
            .clr()
            .write(|w| w.out_eof().clear_bit_by_one());
        channel.out_int().ena().modify(|_, w| w.out_eof().set_bit());

        Self {
            _transfer: transfer,
        }
    }
}

/// The frames with an underrun since the last call.
pub fn take_underruns() -> u32 {
    UNDERRUNS.swap(0, Ordering::Relaxed)
}

/// The bounce buffers that the DMA read before the interrupt refilled them, since the
/// last call.
pub fn take_late_refills() -> u32 {
    LATE_REFILLS.swap(0, Ordering::Relaxed)
}

/// The frames that started out of step with the panel, since the last call.
pub fn take_resyncs() -> u32 {
    RESYNCS.swap(0, Ordering::Relaxed)
}

/// The time that the interrupt spent drawing since the last call.
pub fn take_draw_micros() -> u32 {
    DRAW_MICROS.swap(0, Ordering::Relaxed)
}

/// Draws the next rows into the bounce buffer that the DMA has just finished.
#[handler(priority = Priority::max())]
#[ram]
fn refill() {
    let started = Instant::now();
    let channel = AXI_GDMA::regs().out_ch(LCD_CHANNEL);
    channel
        .out_int()
        .clr()
        .write(|w| w.out_eof().clear_bit_by_one());

    let finished_end = channel.out_eof_des_addr().read().bits();
    let finished = if finished_end == FIRST_BUFFER_END.load(Ordering::Relaxed) {
        0
    } else {
        1
    };
    let mut chunk = NEXT_CHUNK.load(Ordering::Relaxed);
    if finished == LAST_FINISHED.swap(finished, Ordering::Relaxed) {
        // The interrupt was too late for the other buffer, and the DMA has read it
        // with old rows. Skip its rows to stay in step with the panel.
        chunk = (chunk + 1) % CHUNKS_PER_FRAME;
        LATE_REFILLS.fetch_add(1, Ordering::Relaxed);
    }

    let lcd_cam = LCD_CAM::regs();
    let events = lcd_cam.lc_dma_int_raw().read();
    if events.lcd_vsync_int_raw().bit_is_set() {
        // The DMA stops in the vertical blank, inside the buffer with the first rows.
        // So it has just finished those rows, and the next buffer gets the third
        // chunk. This puts the rows back in step after a long interrupt delay.
        if chunk != 2 {
            RESYNCS.fetch_add(1, Ordering::Relaxed);
            chunk = 2;
        }
        if events.lcd_underrun_int_raw().bit_is_set() {
            UNDERRUNS.fetch_add(1, Ordering::Relaxed);
        }
        lcd_cam.lc_dma_int_clr().write(|w| {
            w.lcd_vsync_int_clr().set_bit();
            w.lcd_underrun_int_clr().set_bit()
        });
    }
    NEXT_CHUNK.store((chunk + 1) % CHUNKS_PER_FRAME, Ordering::Relaxed);

    // SAFETY: the DMA reads the other buffer now, and only this interrupt writes here.
    let buffer = unsafe { &mut (*addr_of_mut!(BUFFERS)).0[finished] };
    scene::draw_rows(chunk * BOUNCE_ROWS, buffer);

    DRAW_MICROS.fetch_add(started.elapsed().as_micros() as u32, Ordering::Relaxed);
}

/// Sets up the LCD for the board's panel: 14 MHz, 880x530 in total, 30 frames per
/// second.
fn connect(lcd: Lcd<'static, Blocking>, r: LcdResources<'static>) -> Dpi<'static, Blocking> {
    let config = Config::default()
        .with_format(Format {
            enable_2byte_mode: true,
            // The pixels have the high byte first.
            byte_order: ByteOrder::Inverted,
            ..Default::default()
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false)
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftLow,
        })
        .with_frequency(Rate::from_mhz(14))
        .with_timing(FrameTiming {
            horizontal_active_width: WIDTH,
            horizontal_total_width: 880,
            horizontal_blank_front_porch: 32,
            vertical_active_height: HEIGHT,
            vertical_total_height: 530,
            vertical_blank_front_porch: 37,
            hsync_width: 40,
            vsync_width: 23,
            hsync_position: 0,
        })
        // Half a clock of delay on the data keeps vertical lines in the right color.
        .with_output_bit_mode(DelayMode::FallingEdge);

    Dpi::new(lcd, r.dma, config)
        .unwrap()
        .with_vsync(r.vsync)
        .with_hsync(r.hsync)
        .with_de(r.de)
        .with_pclk(r.pclk)
        .with_data0(r.data0)
        .with_data1(r.data1)
        .with_data2(r.data2)
        .with_data3(r.data3)
        .with_data4(r.data4)
        .with_data5(r.data5)
        .with_data6(r.data6)
        .with_data7(r.data7)
        .with_data8(r.data8)
        .with_data9(r.data9)
        .with_data10(r.data10)
        .with_data11(r.data11)
        .with_data12(r.data12)
        .with_data13(r.data13)
        .with_data14(r.data14)
        .with_data15(r.data15)
}

/// The two bounce buffers in a circular descriptor chain. The last descriptor of each
/// buffer has EOF set, so that the DMA interrupts when it finishes the buffer.
pub struct BounceChain {
    descriptors: &'static mut [DmaDescriptor; 2 * DESCRIPTORS_PER_BUFFER],
}

impl BounceChain {
    fn new() -> Self {
        let descriptors = DESCRIPTORS.take();
        let first = descriptors.as_mut_ptr();
        let buffers = addr_of_mut!(BUFFERS).cast::<u8>();
        for (i, descriptor) in descriptors.iter_mut().enumerate() {
            descriptor.set_size(DESCRIPTOR_BYTES);
            descriptor.set_length(DESCRIPTOR_BYTES);
            descriptor.set_owner(Owner::Dma);
            descriptor.set_suc_eof(i % DESCRIPTORS_PER_BUFFER == DESCRIPTORS_PER_BUFFER - 1);
            descriptor.buffer = buffers.wrapping_add(i * DESCRIPTOR_BYTES);
            descriptor.next = first.wrapping_add((i + 1) % (2 * DESCRIPTORS_PER_BUFFER));
        }
        FIRST_BUFFER_END.store(
            first.wrapping_add(DESCRIPTORS_PER_BUFFER - 1) as u32,
            Ordering::Relaxed,
        );
        Self { descriptors }
    }
}

// SAFETY: the descriptors and the buffers are static.
unsafe impl DmaTxBuffer for BounceChain {
    type View = Self;
    type Final = Self;

    fn prepare(&mut self) -> Preparation {
        Preparation {
            start: self.descriptors.as_mut_ptr(),
            accesses_psram: false,
            burst_transfer: BurstConfig::default(),
            check_owner: Some(false),
            auto_write_back: false,
        }
    }

    fn into_view(self) -> Self::View {
        self
    }

    fn from_view(view: Self::View) -> Self::Final {
        view
    }
}
