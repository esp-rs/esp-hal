//! The 800x480 RGB LCD. Its DMA reads the PSRAM framebuffer through a circular
//! descriptor chain, so the panel shows every change without a new transfer.

use core::{
    alloc::Layout,
    ops::{Deref, DerefMut, Range},
};

use allocator_api2::alloc::Allocator;
use esp_alloc::DmaCompatibleExternalMemory;
use esp_hal::{
    Blocking,
    dma::{BurstConfig, DmaDescriptor, DmaTxBuffer, Owner, Preparation, aligned::DmaAlignedMut},
    gpio::Level,
    lcd_cam::{
        LcdCam,
        lcd::{
            ClockMode,
            DelayMode,
            Phase,
            Polarity,
            dpi::{Config, Dpi, DpiTransfer, Format, FrameTiming},
        },
    },
    peripherals::AXI_GDMA,
    time::Rate,
};

use crate::DisplayResources;

pub const WIDTH: usize = 800;
pub const HEIGHT: usize = 480;
const FB_BYTES: usize = WIDTH * HEIGHT * 2;
/// The largest multiple of 64 bytes that fits in a descriptor. A descriptor boundary
/// must not split a PSRAM burst.
const CHUNK: usize = 4032;
const CHUNKS: usize = FB_BYTES.div_ceil(CHUNK);
/// The AXI-GDMA channel of `DMA_AXI_CH0`.
const LCD_CHANNEL: usize = 0;

/// The framebuffer that the panel shows.
pub struct Display {
    transfer: DpiTransfer<'static, CircularFrameBuf, Blocking>,
}

impl Display {
    /// Starts showing a black framebuffer.
    pub fn new(r: DisplayResources<'static>) -> Self {
        let config = Config::default()
            .with_clock_mode(ClockMode {
                polarity: Polarity::IdleLow,
                phase: Phase::ShiftLow,
            })
            // 14 MHz, 880x530 in total: 30 frames per second. Faster clocks underrun
            // when the CPU uses PSRAM.
            .with_frequency(Rate::from_mhz(14))
            .with_format(Format {
                enable_2byte_mode: true,
                ..Default::default()
            })
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
            .with_vsync_idle_level(Level::High)
            .with_hsync_idle_level(Level::High)
            .with_de_idle_level(Level::Low)
            .with_disable_black_region(false)
            // Half a clock of delay on the data. Without it, vertical lines have the
            // wrong color.
            .with_output_bit_mode(DelayMode::FallingEdge);

        let dpi = Dpi::new(LcdCam::new(r.lcd_cam).lcd, r.dma, config)
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
            .with_data15(r.data15);

        let descriptors = esp_hal::dma_descriptors_impl!(FB_BYTES, CHUNK);
        let transfer = dpi
            .send(
                true,
                CircularFrameBuf::new(descriptors, allocate_framebuffer()),
            )
            .map_err(|error| error.0)
            .unwrap();

        // The LCD's reads come before other PSRAM traffic, in 16-byte bursts.
        let channel = AXI_GDMA::regs().out_ch(LCD_CHANNEL);
        channel.out_pri().write(|w| unsafe { w.tx_pri().bits(6) });
        channel
            .out_conf0()
            .modify(|_, w| unsafe { w.out_burst_size_sel().bits(1) });

        Self { transfer }
    }

    /// Makes the given framebuffer rows visible to the DMA.
    pub fn writeback(&mut self, rows: Range<usize>) {
        if rows.is_empty() {
            return;
        }
        let bytes = &mut self[rows.start * WIDTH * 2..rows.end * WIDTH * 2];
        DmaAlignedMut::new(bytes).unwrap().writeback();
    }
}

impl Deref for Display {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.transfer.buffer
    }
}

impl DerefMut for Display {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.transfer.buffer
    }
}

fn allocate_framebuffer() -> DmaAlignedMut<'static, [u8]> {
    let layout = Layout::from_size_align(FB_BYTES, 64).unwrap();
    let mut allocation = DmaCompatibleExternalMemory
        .allocate_zeroed(layout)
        .expect("PSRAM framebuffer allocation failed");
    DmaAlignedMut::new(unsafe { allocation.as_mut() }).unwrap()
}

struct CircularFrameBuf {
    descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
    buffer: DmaAlignedMut<'static, [u8]>,
}

impl CircularFrameBuf {
    fn new(
        mut descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        mut buffer: DmaAlignedMut<'static, [u8]>,
    ) -> Self {
        let base = buffer.as_mut_ptr();
        let first = descriptors.as_mut_ptr();
        for (i, descriptor) in descriptors.iter_mut().enumerate() {
            let length = (FB_BYTES - i * CHUNK).min(CHUNK);
            descriptor.set_size(length);
            descriptor.set_length(length);
            descriptor.set_owner(Owner::Dma);
            descriptor.set_suc_eof(false);
            descriptor.buffer = base.wrapping_add(i * CHUNK);
            descriptor.next = first.wrapping_add((i + 1) % CHUNKS);
        }
        Self {
            descriptors,
            buffer,
        }
    }
}

unsafe impl DmaTxBuffer for CircularFrameBuf {
    type View = Self;
    type Final = Self;

    fn prepare(&mut self) -> Preparation {
        self.descriptors.writeback();
        self.buffer.writeback();
        Preparation {
            start: self.descriptors.as_mut_ptr(),
            accesses_psram: true,
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
