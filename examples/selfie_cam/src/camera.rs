//! The SmartSens SC101IOT camera on the DVP port: 1280x720 YUV422 at about 15 frames
//! per second from a 20 MHz master clock.
//!
//! The camera unit converts the YUV422 stream to RGB565, and its DMA writes each frame
//! into a frame buffer in PSRAM. The sensor has no reset or power-down pin on the
//! board, so it streams from the moment the master clock starts.
//!
//! Between frames, the sensor pauses for about 2.5 ms, too short for a task to restart
//! the DMA in time. So an interrupt does it at each end of frame, into a frame buffer
//! that the [`capture`] task has made ready. Without one, the interrupt drops the frame
//! and captures into the same buffer again. While paused, the interrupt leaves the
//! camera stopped, and [`resume`] starts it again.

use core::{
    cell::RefCell,
    ptr::{addr_of_mut, null_mut},
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use critical_section::Mutex;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_hal::{
    Async,
    Blocking,
    dma::{BurstConfig, DmaDescriptor, DmaRxBuffer, Owner, Preparation, aligned::DmaAlignedMut},
    handler,
    i2c::master::{Error, I2c},
    interrupt::{self, Priority},
    lcd_cam::cam::{Cam, Camera, Config},
    peripherals::{AXI_GDMA, GPIO, Interrupt, LCD_CAM},
    ram,
    time::Rate,
};
use esp_println::println;

use crate::{
    CameraResources,
    picture::{self, Frame, Picture},
};

const ADDRESS: u8 = 0x68;
/// Registers 0xf7 and 0xf8.
const ID: [u8; 2] = [0xda, 0x4a];

/// One and a half rows of the picture. Frames end on a descriptor boundary.
const DESCRIPTOR_BYTES: usize = 3840;
/// The descriptors for a frame buffer, with room for longer frames at the end.
const DESCRIPTORS: usize = picture::BUFFER_BYTES / DESCRIPTOR_BYTES;
/// The channel of `DMA_AXI_CH1`.
const CAMERA_CHANNEL: usize = 1;

/// A chain of descriptors over a frame buffer.
type Chain = [DmaDescriptor; DESCRIPTORS];
/// One chain for the frame that the DMA writes, one for the next frame buffer.
static mut CHAINS: [Chain; 2] = [[DmaDescriptor::EMPTY; DESCRIPTORS]; 2];

/// A frame buffer and the chain over it.
struct Capture {
    frame: Frame,
    chain: usize,
}

/// The frame buffer that the DMA writes. Only the interrupt uses it after the start.
static CURRENT: Mutex<RefCell<Option<Capture>>> = Mutex::new(RefCell::new(None));
/// The frame buffer for the next frame, made ready by the task.
static NEXT: Mutex<RefCell<Option<Capture>>> = Mutex::new(RefCell::new(None));
/// A finished frame buffer and the bytes in it, for the task.
static FINISHED: Mutex<RefCell<Option<(Capture, usize)>>> = Mutex::new(RefCell::new(None));
static FINISHED_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The newest complete frame.
static LATEST: Signal<CriticalSectionRawMutex, alloc::sync::Arc<Picture>> = Signal::new();
static RUNNING: AtomicBool = AtomicBool::new(true);
/// The interrupt left the camera stopped, because it was paused.
static PARKED: AtomicBool = AtomicBool::new(false);
static RESUME: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static FRAMES: AtomicU32 = AtomicU32::new(0);
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// Starts the master clock and sets up the sensor. Returns the camera unit, ready to
/// capture.
pub async fn start(
    cam: Cam<'static>,
    r: CameraResources<'static>,
    i2c: &mut I2c<'static, Async>,
) -> Result<Camera<'static, Blocking>, Error> {
    let camera = Camera::new(
        cam,
        r.dma,
        Config::default()
            .with_frequency(Rate::from_mhz(20))
            // The sensor's VSYNC is high while it sends a frame, and low for the 2.5 ms
            // between frames. Inverted, the frame ends where that gap starts, and the
            // interrupt has the whole gap to restart the DMA.
            .with_invert_vsync(true),
    )
    .unwrap()
    .with_master_clock(r.xclk)
    .with_pixel_clock(r.pclk)
    .with_vsync(r.vsync)
    .with_h_enable(r.href)
    .with_data0(r.data0)
    .with_data1(r.data1)
    .with_data2(r.data2)
    .with_data3(r.data3)
    .with_data4(r.data4)
    .with_data5(r.data5)
    .with_data6(r.data6)
    .with_data7(r.data7);
    convert_to_rgb565();

    // The sensor needs the master clock before it answers.
    Timer::after(Duration::from_millis(20)).await;
    let mut id = [0u8; 2];
    i2c.write_read_async(ADDRESS, &[0xf7], &mut id[..1]).await?;
    i2c.write_read_async(ADDRESS, &[0xf8], &mut id[1..]).await?;
    if id != ID {
        println!("Unexpected camera ID {:02x}{:02x}", id[0], id[1]);
    }

    for &[register, value] in INIT {
        i2c.write_async(ADDRESS, &[register, value]).await?;
    }
    Timer::after(Duration::from_millis(50)).await;
    Ok(camera)
}

/// Turns on the camera unit's YUV422 to RGB565 converter. It writes the high byte of each
/// pixel first.
fn convert_to_rgb565() {
    LCD_CAM::regs().cam_rgb_yuv().write(|w| unsafe {
        // YUV to RGB, from YUV422, without YUV to YUV conversion.
        w.cam_conv_trans_mode().clear_bit();
        w.cam_conv_yuv_mode().bits(0);
        w.cam_conv_yuv2yuv_mode().bits(3);
        w.cam_conv_protocol_mode().clear_bit();
        w.cam_conv_data_in_mode().set_bit();
        w.cam_conv_data_out_mode().set_bit();
        w.cam_conv_mode_8bits_on().set_bit();
        w.cam_conv_enable().set_bit()
    });
}

/// Waits for a frame that is newer than the last one.
pub async fn next_picture() -> alloc::sync::Arc<Picture> {
    LATEST.wait().await
}

/// Stops publishing frames, and drops the waiting one, to free frame buffers.
pub fn pause() {
    RUNNING.store(false, Ordering::Relaxed);
    LATEST.reset();
}

pub async fn resume() {
    RUNNING.store(true, Ordering::Relaxed);
    RESUME.signal(());
    if !PARKED.load(Ordering::Relaxed) {
        return;
    }
    frame_gap().await;
    critical_section::with(|cs| {
        if PARKED.swap(false, Ordering::Relaxed)
            && let Some(capture) = CURRENT.borrow_ref(cs).as_ref()
        {
            let chain = chain(capture.chain);
            link(chain, chain[0].buffer);
            restart(chain.as_mut_ptr());
        }
    });
}

/// The complete and the incomplete or dropped frames since the last call.
pub fn take_counts() -> (u32, u32) {
    (
        FRAMES.swap(0, Ordering::Relaxed),
        DROPPED.swap(0, Ordering::Relaxed),
    )
}

/// Starts capturing, publishes the complete frames, and keeps a free frame buffer ready
/// for the next frame.
#[embassy_executor::task]
pub async fn capture(camera: Camera<'static, Blocking>) -> ! {
    let first = prepare(picture::take().await, 0);
    let start = chain(0).as_mut_ptr();
    critical_section::with(|cs| CURRENT.borrow_ref_mut(cs).replace(first));
    interrupt::bind_handler(Interrupt::AXI_PDMA_IN_CH1, frame_done);
    frame_gap().await;
    let _transfer = match camera.receive(FirstChain(start)) {
        Ok(transfer) => transfer,
        Err((error, _, _)) => panic!("cannot start the camera: {error:?}"),
    };
    // When the DMA falls behind, lose the bytes instead of stopping the camera unit.
    // Stopped, it would never end a frame, and nothing would start it again.
    LCD_CAM::regs().cam_ctrl().modify(|_, w| {
        w.cam_stop_en().clear_bit();
        w.cam_update().set_bit()
    });
    let channel = AXI_GDMA::regs().in_ch(CAMERA_CHANNEL);
    channel.in_int().clr().write(|w| {
        w.in_suc_eof().clear_bit_by_one();
        w.in_dscr_empty().clear_bit_by_one()
    });
    channel.in_int().ena().modify(|_, w| {
        w.in_suc_eof().set_bit();
        w.in_dscr_empty().set_bit()
    });

    // The chain that no frame buffer uses.
    let mut free_chain = Some(1);
    loop {
        if let Some((capture, bytes)) = critical_section::with(|cs| FINISHED.take(cs)) {
            free_chain = Some(capture.chain);
            let frame = capture.frame;
            // Drop what the CPU may have cached while the DMA wrote.
            invalidate(&frame);
            if RUNNING.load(Ordering::Relaxed) && bytes == picture::BYTES {
                FRAMES.fetch_add(1, Ordering::Relaxed);
                LATEST.signal(Picture::new(frame));
            } else {
                if DROPPED.fetch_add(1, Ordering::Relaxed).is_multiple_of(16) {
                    println!("incomplete frame: {bytes} bytes");
                }
                picture::give_back(frame);
            }
        }

        let need_next = RUNNING.load(Ordering::Relaxed) && free_chain.is_some();
        if !need_next {
            match select(FINISHED_SIGNAL.wait(), RESUME.wait()).await {
                Either::First(()) | Either::Second(()) => continue,
            }
        }
        match select(FINISHED_SIGNAL.wait(), picture::take()).await {
            Either::First(()) => {}
            Either::Second(frame) => {
                let next = prepare(frame, free_chain.take().unwrap());
                critical_section::with(|cs| NEXT.borrow_ref_mut(cs).replace(next));
            }
        }
    }
}

/// Links a chain over the frame buffer, and drops its cached lines so that they cannot
/// overwrite what the DMA writes.
fn prepare(frame: Frame, chain_index: usize) -> Capture {
    invalidate(&frame);
    link(
        chain(chain_index),
        frame.pixels().as_ptr().cast_mut().cast(),
    );
    Capture {
        frame,
        chain: chain_index,
    }
}

fn invalidate(frame: &Frame) {
    let bytes = frame.pixels().as_ptr().cast::<u8>();
    // SAFETY: only the cache sees this view, and the frame buffer is DMA aligned.
    let view = unsafe { core::slice::from_raw_parts_mut(bytes.cast_mut(), picture::BUFFER_BYTES) };
    DmaAlignedMut::new(view).unwrap().invalidate();
}

fn chain(index: usize) -> &'static mut Chain {
    // SAFETY: each chain belongs to one frame buffer at a time, and only its owner
    // touches it: the task while it prepares the buffer, the interrupt after that.
    unsafe { &mut (*addr_of_mut!(CHAINS))[index] }
}

#[ram]
fn link(chain: &mut Chain, buffer: *mut u8) {
    let first = chain.as_mut_ptr();
    for (i, descriptor) in chain.iter_mut().enumerate() {
        descriptor.set_size(DESCRIPTOR_BYTES);
        descriptor.set_length(0);
        descriptor.set_suc_eof(false);
        descriptor.set_owner(Owner::Dma);
        descriptor.buffer = buffer.wrapping_add(i * DESCRIPTOR_BYTES);
        descriptor.next = if i + 1 < DESCRIPTORS {
            first.wrapping_add(i + 1)
        } else {
            null_mut()
        };
    }
}

/// Moves the DMA to the next frame buffer at the end of each frame.
#[handler(priority = Priority::Priority3)]
#[ram]
fn frame_done() {
    let channel = AXI_GDMA::regs().in_ch(CAMERA_CHANNEL);
    let events = channel.in_int().raw().read();
    channel.in_int().clr().write(|w| {
        w.in_suc_eof().clear_bit_by_one();
        w.in_dscr_empty().clear_bit_by_one()
    });
    let eof_descriptor = channel.in_suc_eof_des_addr().read().bits() as usize;
    channel.in_link1().modify(|_, w| w.inlink_stop().set_bit());

    critical_section::with(|cs| {
        let mut current = CURRENT.borrow_ref_mut(cs);
        let Some(capture) = current.take() else {
            return;
        };
        let first = chain(capture.chain).as_ptr() as usize;
        let index = eof_descriptor.wrapping_sub(first) / size_of::<DmaDescriptor>();
        let bytes = if events.in_suc_eof().bit_is_set() && index < DESCRIPTORS {
            index * DESCRIPTOR_BYTES + chain(capture.chain)[index].len()
        } else {
            0
        };

        if !RUNNING.load(Ordering::Relaxed) {
            // Stop writing PSRAM while the gallery uses it.
            LCD_CAM::regs()
                .cam_ctrl1()
                .modify(|_, w| w.cam_start().clear_bit());
            PARKED.store(true, Ordering::Relaxed);
            current.replace(capture);
            return;
        }

        let mut finished = FINISHED.borrow_ref_mut(cs);
        let next = if finished.is_none() {
            NEXT.borrow_ref_mut(cs).take()
        } else {
            None
        };
        let capture = match next {
            Some(next) => {
                finished.replace((capture, bytes));
                FINISHED_SIGNAL.signal(());
                next
            }
            None => {
                DROPPED.fetch_add(1, Ordering::Relaxed);
                let buffer = chain(capture.chain)[0].buffer;
                link(chain(capture.chain), buffer);
                capture
            }
        };
        restart(chain(capture.chain).as_mut_ptr());
        current.replace(capture);
    });
}

/// Resets the stopped DMA and starts it on a new chain, at each frame.
/// See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_driver_cam/dvp/src/esp_cam_ctlr_dvp_cam.c#L184-L185>.
#[ram]
fn restart(start: *mut DmaDescriptor) {
    let channel = AXI_GDMA::regs().in_ch(CAMERA_CHANNEL);
    channel.in_conf0().modify(|_, w| w.in_rst().set_bit());
    channel.in_conf0().modify(|_, w| w.in_rst().clear_bit());
    channel
        .in_link2()
        .write(|w| unsafe { w.inlink_addr().bits(start as u32) });
    channel.in_link1().modify(|_, w| w.inlink_start().set_bit());

    // The converter pairs the bytes from where the camera unit starts, so start it
    // again between frames.
    let lcd_cam = LCD_CAM::regs();
    lcd_cam.cam_ctrl1().modify(|_, w| w.cam_reset().set_bit());
    lcd_cam.cam_ctrl1().modify(|_, w| w.cam_reset().clear_bit());
    lcd_cam
        .cam_ctrl1()
        .modify(|_, w| w.cam_afifo_reset().set_bit());
    lcd_cam
        .cam_ctrl1()
        .modify(|_, w| w.cam_afifo_reset().clear_bit());
    lcd_cam.cam_ctrl().modify(|_, w| w.cam_update().set_bit());
    lcd_cam.cam_ctrl1().modify(|_, w| w.cam_start().set_bit());
}

/// Waits for the start of the gap between two frames. Started in the middle of a frame,
/// the camera unit ends a frame again and again until that frame is over.
async fn frame_gap() {
    // VSYNC, GPIO56, is high while the sensor sends a frame.
    let vsync = || GPIO::regs().in1().read().bits() & 1 << (56 - 32) != 0;
    while !vsync() {
        Timer::after(Duration::from_millis(1)).await;
    }
    while vsync() {
        Timer::after(Duration::from_micros(200)).await;
    }
}

/// The first chain, for starting the camera through esp-hal.
struct FirstChain(*mut DmaDescriptor);

// SAFETY: the chain is static, and its frame buffer is in DMA-capable PSRAM.
unsafe impl DmaRxBuffer for FirstChain {
    type View = Self;
    type Final = Self;

    fn prepare(&mut self) -> Preparation {
        Preparation {
            start: self.0,
            accesses_psram: true,
            burst_transfer: BurstConfig::default(),
            check_owner: Some(false),
            auto_write_back: true,
        }
    }

    fn into_view(self) -> Self::View {
        self
    }

    fn from_view(view: Self::View) -> Self::Final {
        view
    }
}

/// The sensor setup for 1280x720 YUV422 (YCbYCr) at 15 frames per second. Writes to
/// register 0xf0 select the page.
/// See <https://github.com/espressif/esp32-camera/blob/281d951f75b/sensors/private_include/sc101iot_settings.h#L50-L256>.
#[rustfmt::skip]
const INIT: &[[u8; 2]] = &[
    [0xf0, 0x30], [0x01, 0xff], [0x02, 0xe0], [0x30, 0x10], [0x3f, 0x81],
    [0xf0, 0x00], [0x70, 0x6b], [0x72, 0x30], [0x84, 0xb4], [0x8b, 0x00],
    [0x8c, 0x20], [0x8d, 0x02], [0x8e, 0xec], [0x9e, 0x10], [0xb0, 0xc1],
    [0xc8, 0x10], [0xc9, 0x10], [0xc6, 0x00], [0xe0, 0x0f], [0xb5, 0xf0],
    [0xde, 0x80], [0xb5, 0xf0], [0xde, 0x80], [0xb2, 0x50], [0xb3, 0xfc],
    [0xb4, 0x40], [0xb5, 0xc0], [0xb6, 0x50], [0xb7, 0xfc], [0xb8, 0x40],
    [0xb9, 0xc0], [0xba, 0xff], [0xbb, 0xcc], [0xbc, 0xa9], [0xbd, 0x7d],
    [0xc1, 0x77], [0xf0, 0x01], [0x70, 0x02], [0x71, 0x02], [0x72, 0x50],
    [0x73, 0x02], [0x74, 0xd2], [0x75, 0x20], [0x76, 0x81], [0x77, 0x8c],
    [0x78, 0x81], [0xf4, 0x01], [0xf5, 0x00], [0xf6, 0x00], [0xf0, 0x36],
    [0x40, 0x03], [0x41, 0x01], [0xf0, 0x39], [0x02, 0x70], [0xf0, 0x32],
    [0x41, 0x00], [0x43, 0x01], [0x48, 0x02], [0xf0, 0x45], [0x09, 0x20],
    [0xf0, 0x33], [0x33, 0x10], [0xf0, 0x30], [0x38, 0x44], [0xf0, 0x39],
    [0x07, 0x00], [0x08, 0x19], [0x47, 0x00], [0x48, 0x00], [0xf0, 0x37],
    [0x24, 0x31], [0xf0, 0x34], [0x9f, 0x02], [0xa6, 0x51], [0xa7, 0x57],
    [0xe8, 0x5f], [0xa8, 0x50], [0xa9, 0x50], [0xe9, 0x50], [0xf0, 0x33],
    [0xb3, 0x58], [0xb2, 0x78], [0xf0, 0x34], [0x9f, 0x03], [0xa6, 0x51],
    [0xa7, 0x57], [0xaa, 0x01], [0xab, 0x28], [0xac, 0x01], [0xad, 0x38],
    [0xf0, 0x33], [0x0a, 0x01], [0x0b, 0x28], [0xf0, 0x33], [0x64, 0x0f],
    [0xec, 0x51], [0xed, 0x57], [0x06, 0x58], [0xe9, 0x58], [0xeb, 0x68],
    [0xf0, 0x33], [0x64, 0x0f], [0xf0, 0x36], [0x70, 0xdf], [0xb6, 0x40],
    [0xb7, 0x51], [0xb8, 0x53], [0xb9, 0x57], [0xba, 0x5f], [0xb0, 0x84],
    [0xb1, 0x82], [0xb2, 0x84], [0xb3, 0x88], [0xb4, 0x90], [0xb5, 0x90],
    [0xf0, 0x36], [0x7e, 0x50], [0x7f, 0x51], [0x77, 0x81], [0x78, 0x86],
    [0x79, 0x89], [0xf0, 0x36], [0x70, 0xdf], [0x9c, 0x51], [0x9d, 0x57],
    [0x90, 0x54], [0x91, 0x54], [0x92, 0x56], [0xf0, 0x36], [0xa0, 0x51],
    [0xa1, 0x57], [0x96, 0x33], [0x97, 0x43], [0x98, 0x43], [0xf0, 0x36],
    [0x70, 0xdf], [0x7c, 0x40], [0x7d, 0x53], [0x74, 0xd0], [0x75, 0xf0],
    [0x76, 0xf0], [0xf0, 0x37], [0x0f, 0xd5], [0x7a, 0x40], [0x7b, 0x57],
    [0x71, 0x09], [0x72, 0x09], [0x73, 0x05], [0xf0, 0x33], [0x01, 0x44],
    [0xf0, 0x36], [0x37, 0xfb], [0xf0, 0x36], [0x3c, 0x0d], [0xf0, 0x33],
    [0x14, 0x95], [0xf0, 0x33], [0x8f, 0x80], [0xf0, 0x37], [0x27, 0x14],
    [0x28, 0x03], [0xf0, 0x36], [0x37, 0xf4], [0xf0, 0x33], [0x01, 0x44],
    [0xf0, 0x36], [0x79, 0x89], [0xf0, 0x34], [0xac, 0x01], [0xad, 0x40],
    [0xf0, 0x33], [0xeb, 0x70], [0xf0, 0x34], [0xa8, 0x50], [0xa9, 0x50],
    [0xf0, 0x33], [0xb3, 0x58], [0xf0, 0x36], [0x11, 0x80], [0xf0, 0x36],
    [0x41, 0x51], [0xf0, 0x3f], [0x03, 0x09], [0xf0, 0x32], [0x0c, 0x06],
    [0x0d, 0x82], [0x0e, 0x02], [0x0f, 0xee], [0xf0, 0x36], [0xea, 0x09],
    [0xeb, 0xf5], [0xec, 0x11], [0xed, 0x27], [0xe9, 0x20],
];
