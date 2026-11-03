//! A selfie camera on the ESP32-S31-Korvo-1, in pure Rust.
//!
//! The screen shows the camera like a mirror. The SET key (or BOOT) takes a 1280x720
//! photo and saves it as a JPEG file on the microSD card. The GALLERY button on the
//! screen shows the saved photos.

//% CHIP_FILTER: esp32s31
//% TAG: complex

#![no_std]
#![no_main]

extern crate alloc;

use alloc::{format, string::String, sync::Arc, vec, vec::Vec};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    assign_resources,
    clock::{ClockConfig, CpuClock},
    i2c::master::{Config as I2cConfig, I2c},
    lcd_cam::LcdCam,
    ram,
    system::Stack,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_rtos::CurrentThreadHandle;
use static_cell::ConstStaticCell;

use crate::{
    buttons::Buttons,
    canvas::Label,
    display::{Display, HEIGHT, WIDTH},
    picture::Picture,
    scene::Scene,
    storage::Storage,
    touch::Touch,
    ui::{Action, Screen},
};

mod buttons;
mod camera;
mod canvas;
mod display;
mod jpeg;
mod picture;
mod scene;
mod storage;
mod touch;
mod ui;

esp_bootloader_esp_idf::esp_app_desc!();

/// How often the main loop reads the touch panel and the keys.
const POLL: Duration = Duration::from_millis(20);
/// How long a message stays on the screen.
const MESSAGE_TIME: Duration = Duration::from_secs(2);
const STATUS_Y: i32 = 44;

static DISPLAY_STACK: ConstStaticCell<Stack<{ 8 * 1024 }>> = ConstStaticCell::new(Stack::new());

// ESP32-S31-Korvo-1 with its LCD subboard.
assign_resources! {
    pub Resources<'d> {
        lcd: LcdResources<'d> {
            dma: DMA_AXI_CH0,
            vsync: GPIO45,
            hsync: GPIO44,
            de: GPIO43,
            pclk: GPIO40,
            data0: GPIO8,
            data1: GPIO9,
            data2: GPIO10,
            data3: GPIO11,
            data4: GPIO12,
            data5: GPIO13,
            data6: GPIO14,
            data7: GPIO15,
            data8: GPIO16,
            data9: GPIO17,
            data10: GPIO18,
            data11: GPIO19,
            data12: GPIO33,
            data13: GPIO34,
            data14: GPIO35,
            data15: GPIO36,
        },
        camera: CameraResources<'d> {
            dma: DMA_AXI_CH1,
            xclk: GPIO55,
            pclk: GPIO54,
            vsync: GPIO56,
            href: GPIO57,
            data0: GPIO46,
            data1: GPIO47,
            data2: GPIO48,
            data3: GPIO49,
            data4: GPIO50,
            data5: GPIO51,
            data6: GPIO52,
            data7: GPIO53,
        },
        // The touch panel, the camera and the audio codec share this bus.
        i2c: I2cResources<'d> {
            i2c: I2C0,
            sda: GPIO0,
            scl: GPIO1,
        },
        buttons: ButtonResources<'d> {
            adc: ADC1,
            ladder: GPIO42,
            boot: GPIO61,
        },
        sd: SdResources<'d> {
            sdhost: SDHOST,
            power: GPIO39,
            clk: GPIO24,
            cmd: GPIO25,
            data0: GPIO20,
            data1: GPIO21,
            data2: GPIO22,
            data3: GPIO23,
        },
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = esp_hal::init(
        esp_hal::Config::default().with_cpu_clock(ClockConfig::from(CpuClock::max())),
    );
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 128 * 1024);
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let timer = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer.timer0);

    let resources = split_resources!(peripherals);
    picture::init();

    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
    scene::post(Scene {
        labels: vec![message("Starting the camera...")],
        ..Default::default()
    });
    // The panel's interrupt draws most of the time, so it gets the second core.
    let (lcd, lcd_resources) = (lcd_cam.lcd, resources.lcd);
    esp_rtos::start_second_core(peripherals.CPU_CTRL, DISPLAY_STACK.take(), move || {
        let _display = Display::new(lcd, lcd_resources);
        loop {
            CurrentThreadHandle::get().delay(esp_hal::time::Duration::from_secs(3600));
        }
    });

    let mut i2c = I2c::new(
        resources.i2c.i2c,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(resources.i2c.sda)
    .with_scl(resources.i2c.scl)
    .into_async();
    let camera = camera::start(lcd_cam.cam, resources.camera, &mut i2c)
        .await
        .expect("the camera does not answer");
    spawner.spawn(camera::capture(camera).unwrap());

    let touch = Touch::new(i2c).await;
    println!("Goodix touch controller at 0x{:02x}", touch.address());

    let mut app = App {
        touch,
        buttons: Buttons::new(resources.buttons),
        storage: Storage::new(resources.sd),
        photos: None,
        camera: Screen::camera(),
        gallery: Screen::gallery(),
        status: None,
        message: None,
    };
    app.mount().await;

    loop {
        app.run_camera().await;
        app.run_gallery().await;
    }
}

struct App {
    touch: Touch,
    buttons: Buttons,
    storage: Storage,
    /// The photo numbers on the card, or `None` without a card.
    photos: Option<Vec<u32>>,
    camera: Screen,
    gallery: Screen,
    status: Option<Arc<Label>>,
    /// A message, and when it goes away.
    message: Option<(Arc<Label>, Instant)>,
}

impl App {
    /// Opens the SD card if needed, and lists the photos on it.
    async fn mount(&mut self) {
        let photos = match self.storage.mount().await {
            Ok(()) => self.storage.photos().await.ok(),
            Err(_) => None,
        };
        if let Some(photos) = &photos {
            println!("{} photos on the SD card", photos.len());
        }
        self.photos = photos;
    }

    fn show_message(&mut self, text: &str) {
        self.message = Some((message(text), Instant::now() + MESSAGE_TIME));
    }

    /// Removes the message when its time is up. Returns whether it did.
    fn expire_message(&mut self) -> bool {
        let expired = self
            .message
            .as_ref()
            .is_some_and(|(_, until)| Instant::now() >= *until);
        if expired {
            self.message = None;
        }
        expired
    }

    /// The status line and the message.
    fn labels(&self) -> Vec<Arc<Label>> {
        self.status
            .iter()
            .chain(self.message.iter().map(|(label, _)| label))
            .cloned()
            .collect()
    }

    fn set_status(&mut self, text: &str) {
        self.status = Some(Arc::new(Label::new(text, WIDTH as i32 / 2, STATUS_Y)));
    }

    fn camera_status(&mut self) {
        let text = match &self.photos {
            None => String::from("No SD card"),
            Some(photos) => format!("{} photos  -  press SET to take one", photos.len()),
        };
        self.set_status(&text);
    }

    fn post(&mut self, picture: Option<Arc<Picture>>, mirror: bool, screen: Mode) {
        let layer = match screen {
            Mode::Camera => self.camera.layer,
            Mode::Gallery => self.gallery.layer,
        };
        scene::post(Scene {
            picture,
            mirror,
            layer: Some(layer),
            labels: self.labels(),
        });
    }

    /// Shows the camera until the GALLERY button.
    async fn run_camera(&mut self) {
        camera::resume().await;
        self.camera_status();
        let mut shown: Option<Arc<Picture>> = None;
        let mut report = Instant::now();

        loop {
            scene::collect();
            match select(camera::next_picture(), Timer::after(POLL)).await {
                Either::First(picture) => {
                    self.expire_message();
                    shown = Some(picture);
                    self.post(shown.clone(), true, Mode::Camera);
                }
                Either::Second(()) => {
                    if self.expire_message() {
                        self.post(shown.clone(), true, Mode::Camera);
                    }
                }
            }

            if self.buttons.shutter_pressed()
                && let Some(picture) = &shown
            {
                self.take_photo(picture.clone()).await;
            }
            if let Some((x, y)) = self.touch.tap().await
                && self.camera.action_at(x, y) == Some(Action::OpenGallery)
            {
                return;
            }

            let elapsed = report.elapsed();
            if elapsed >= Duration::from_secs(5) {
                let (frames, dropped) = camera::take_counts();
                println!(
                    "camera {} fps, {} incomplete frames; drawing the panel takes {}% of this core; {} late refills, {} resyncs, {} frames with LCD underruns",
                    frames as u64 * 1000 / elapsed.as_millis(),
                    dropped,
                    display::take_draw_micros() as u64 * 100 / elapsed.as_micros(),
                    display::take_late_refills(),
                    display::take_resyncs(),
                    display::take_underruns(),
                );
                report = Instant::now();
            }
        }
    }

    /// Saves the picture as the next photo on the SD card.
    async fn take_photo(&mut self, picture: Arc<Picture>) {
        if self.photos.is_none() {
            self.mount().await;
        }
        let Some(number) = self
            .photos
            .as_ref()
            .map(|photos| photos.last().map_or(1, |last| last + 1))
        else {
            self.camera_status();
            self.show_message("No SD card");
            return;
        };

        // Hold the picture on the screen while saving.
        self.show_message("Saving...");
        self.post(Some(picture.clone()), true, Mode::Camera);

        let started = Instant::now();
        let jpeg = jpeg::encode(picture.pixels());
        let encoded = started.elapsed();
        let saved = match jpeg {
            Ok(jpeg) => self
                .storage
                .save(number, &jpeg)
                .await
                .map(|()| jpeg.len())
                .map_err(|error| format!("{error:?}")),
            Err(error) => Err(format!("{error:?}")),
        };
        match saved {
            Ok(bytes) => {
                println!(
                    "Saved {} ({} KB): encoding took {} ms, writing {} ms",
                    storage::file_name(number),
                    bytes / 1024,
                    encoded.as_millis(),
                    (started.elapsed() - encoded).as_millis()
                );
                if let Some(photos) = &mut self.photos {
                    photos.push(number);
                }
                self.show_message(&format!("Saved {}", storage::file_name(number)));
            }
            Err(error) => {
                println!("Cannot save the photo: {error}");
                self.show_message("Cannot save the photo");
            }
        }
        self.camera_status();
    }

    /// Shows the photos until the CAMERA button or the shutter key.
    async fn run_gallery(&mut self) {
        camera::pause();
        self.message = None;
        self.set_status("Loading...");
        self.post(None, false, Mode::Gallery);
        self.mount().await;

        let mut photo: Option<Arc<Picture>> = None;
        let mut index = self
            .photos
            .as_ref()
            .map_or(0, |photos| photos.len().saturating_sub(1));
        let mut load = true;
        // DELETE needs a second tap while its question is on the screen.
        let mut confirm_until: Option<Instant> = None;
        loop {
            if load {
                load = false;
                photo = self.load_photo(index, photo.take()).await;
            }

            scene::collect();
            Timer::after(POLL).await;
            if self.expire_message() {
                self.post(photo.clone(), false, Mode::Gallery);
            }

            if self.buttons.shutter_pressed() {
                return;
            }
            let Some((x, y)) = self.touch.tap().await else {
                continue;
            };
            let count = self.photos.as_ref().map_or(0, Vec::len);
            match self.gallery.action_at(x, y) {
                Some(Action::OpenCamera) => return,
                Some(Action::Previous) if count > 1 => {
                    index = (index + count - 1) % count;
                    load = true;
                }
                Some(Action::Next) if count > 1 => {
                    index = (index + 1) % count;
                    load = true;
                }
                Some(Action::Delete) if count > 0 => {
                    if confirm_until
                        .take()
                        .is_some_and(|until| Instant::now() < until)
                    {
                        if self.delete_photo(index).await {
                            index = index.min(count.saturating_sub(2));
                            load = true;
                        }
                    } else {
                        confirm_until = Some(Instant::now() + MESSAGE_TIME);
                        self.show_message("Tap DELETE again to delete");
                    }
                    self.post(photo.clone(), false, Mode::Gallery);
                }
                _ => {}
            }
        }
    }

    /// Deletes the photo at `index` from the card.
    async fn delete_photo(&mut self, index: usize) -> bool {
        let Some(&number) = self.photos.as_ref().and_then(|photos| photos.get(index)) else {
            return false;
        };
        let name = storage::file_name(number);
        match self.storage.delete(number).await {
            Ok(()) => {
                println!("Deleted {name}");
                if let Some(photos) = &mut self.photos {
                    photos.remove(index);
                }
                true
            }
            Err(error) => {
                println!("Cannot delete {name}: {error:?}");
                self.show_message("Cannot delete the photo");
                false
            }
        }
    }

    /// Shows photo `index`, with `old` on the screen while it loads.
    async fn load_photo(
        &mut self,
        index: usize,
        old: Option<Arc<Picture>>,
    ) -> Option<Arc<Picture>> {
        let Some(&number) = self.photos.as_ref().and_then(|photos| photos.get(index)) else {
            let text = if self.photos.is_some() {
                "No photos yet"
            } else {
                "No SD card"
            };
            self.set_status(text);
            self.post(None, false, Mode::Gallery);
            return None;
        };
        let count = self.photos.as_ref().map_or(0, Vec::len);
        let name = storage::file_name(number);
        self.set_status(&format!("{} / {}   {}", index + 1, count, name));
        self.show_message("Loading...");
        self.post(old.clone(), false, Mode::Gallery);
        // Free the old photo's frame for the new one, as soon as the panel lets go of it.
        drop(old);

        let started = Instant::now();
        let mut frame = picture::take().await;
        let decoded = match self.storage.load(number).await {
            Ok(file) => jpeg::decode(&file, &mut frame).map_err(|error| format!("{error}")),
            Err(error) => Err(format!("{error:?}")),
        };
        self.message = None;
        match decoded {
            Ok(()) => {
                println!("Loaded {name} in {} ms", started.elapsed().as_millis());
                let photo = Picture::new(frame);
                self.post(Some(photo.clone()), false, Mode::Gallery);
                Some(photo)
            }
            Err(error) => {
                println!("Cannot load {name}: {error}");
                picture::give_back(frame);
                self.show_message("Cannot load this photo");
                self.post(None, false, Mode::Gallery);
                None
            }
        }
    }
}

#[derive(Clone, Copy)]
enum Mode {
    Camera,
    Gallery,
}

fn message(text: &str) -> Arc<Label> {
    Arc::new(Label::new(text, WIDTH as i32 / 2, HEIGHT as i32 / 2))
}
