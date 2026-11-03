//! The photos on the microSD card, as `DCIM/IMG_NNNN.JPG` on its FAT volume.
//!
//! The card runs on the SDMMC host with a 4-bit bus. Until a card answers, each
//! [`Storage::mount`] tries again, so a card that goes in after boot still works.

use alloc::{boxed::Box, format, string::String, vec, vec::Vec};

use aligned::{A4, Aligned};
use block_device_adapters::{BufStream, StreamSlice};
use block_device_driver::BlockDevice as _;
use embassy_time::Delay;
use embedded_fatfs::{DefaultTimeProvider, Dir, FileSystem, FsOptions, LossyOemCpConverter};
use embedded_io_async::{Read, Write};
use embedded_partitions::mbr::Scheme;
use esp_hal::{
    Async,
    gpio::{Level, Output, OutputConfig},
    sdmmc::{Config, SdHostController, Slot, SlotConfig},
};
use esp_println::println;
use sdio::{BlockDevice, sd::Card as SdCard};
use static_cell::StaticCell;

use crate::SdResources;

const CARD_HZ: u32 = 40_000_000;
const FOLDER: &str = "DCIM";

type Blocks = BlockDevice<SdCard, Slot<'static, 0, Async>, Delay, 512>;
type Block = Aligned<A4, [u8; 512]>;
type Volume = StreamSlice<BufStream<Card, 512>>;
type Fs = FileSystem<Volume, DefaultTimeProvider, LossyOemCpConverter>;

/// The slots borrow the controller.
static CONTROLLER: StaticCell<SdHostController<'static>> = StaticCell::new();

const BOUNCE_BLOCKS: usize = 32;
static BOUNCE: StaticCell<[Block; BOUNCE_BLOCKS]> = StaticCell::new();

/// The card, with every transfer through a buffer in internal RAM. The card's DMA
/// reaches neither flash, where the FAT library keeps its block of zeros, nor the
/// photos in PSRAM.
struct Card {
    blocks: Blocks,
    bounce: &'static mut [Block; BOUNCE_BLOCKS],
}

impl block_device_driver::BlockDevice<512> for Card {
    type Error = <Blocks as block_device_driver::BlockDevice<512>>::Error;
    type Align = A4;

    async fn read(&mut self, block_address: u32, data: &mut [Block]) -> Result<(), Self::Error> {
        let mut address = block_address;
        for part in data.chunks_mut(BOUNCE_BLOCKS) {
            let bounce = &mut self.bounce[..part.len()];
            self.blocks.read(address, bounce).await?;
            part.copy_from_slice(bounce);
            address += part.len() as u32;
        }
        Ok(())
    }

    async fn write(&mut self, block_address: u32, data: &[Block]) -> Result<(), Self::Error> {
        let mut address = block_address;
        for part in data.chunks(BOUNCE_BLOCKS) {
            let bounce = &mut self.bounce[..part.len()];
            bounce.copy_from_slice(part);
            self.blocks.write(address, bounce).await?;
            address += part.len() as u32;
        }
        Ok(())
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        self.blocks.size().await
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// No card answers.
    NoCard,
    /// The card has no FAT volume. It needs a reboot after formatting.
    NoVolume,
    /// Reading or writing failed.
    Io,
}

enum State {
    Unmounted(Card),
    Mounted(Box<Fs>),
    Failed,
}

pub struct Storage {
    state: State,
    /// Low powers the card.
    _power: Output<'static>,
}

impl Storage {
    pub fn new(r: SdResources<'static>) -> Self {
        let power = Output::new(r.power, Level::Low, OutputConfig::default());
        let controller =
            CONTROLLER.init(SdHostController::new(r.sdhost, Config::default()).unwrap());
        let slot = controller
            .slot::<0>(SlotConfig::default())
            .unwrap()
            .with_clk(r.clk)
            .with_cmd(r.cmd)
            .with_data0(r.data0)
            .with_data1(r.data1)
            .with_data2(r.data2)
            .with_data3(r.data3)
            .into_async();
        Self {
            state: State::Unmounted(Card {
                blocks: BlockDevice::new_uninit(slot, Delay),
                bounce: BOUNCE.init([Aligned([0; 512]); BOUNCE_BLOCKS]),
            }),
            _power: power,
        }
    }

    /// Opens the card's FAT volume, if it is not open yet.
    pub async fn mount(&mut self) -> Result<(), Error> {
        let card = match &mut self.state {
            State::Mounted(_) => return Ok(()),
            State::Failed => return Err(Error::NoVolume),
            State::Unmounted(card) => card,
        };
        if let Err(error) = card.blocks.reacquire(CARD_HZ).await {
            println!("No SD card: {error:?}");
            return Err(Error::NoCard);
        }
        println!(
            "SD card at {} MHz: {:?}",
            card.blocks.freq() / 1_000_000,
            card.blocks.card()
        );

        let State::Unmounted(card) = core::mem::replace(&mut self.state, State::Failed) else {
            unreachable!()
        };
        let fs = open_volume(card).await.ok_or(Error::NoVolume)?;
        self.state = State::Mounted(Box::new(fs));
        Ok(())
    }

    fn fs(&self) -> Result<&Fs, Error> {
        match &self.state {
            State::Mounted(fs) => Ok(fs),
            _ => Err(Error::NoCard),
        }
    }

    /// The numbers of the photos on the card, in order.
    pub async fn photos(&mut self) -> Result<Vec<u32>, Error> {
        let fs = self.fs()?;
        let mut numbers = Vec::new();
        let Ok(folder) = fs.root_dir().open_dir(FOLDER).await else {
            return Ok(numbers);
        };
        let mut entries = folder.iter();
        while let Some(entry) = entries.next().await {
            let entry = entry.map_err(io_error)?;
            if entry.is_file()
                && let Some(number) = photo_number(entry.short_file_name_as_bytes())
            {
                numbers.push(number);
            }
        }
        numbers.sort_unstable();
        Ok(numbers)
    }

    pub async fn save(&mut self, number: u32, jpeg: &[u8]) -> Result<(), Error> {
        let fs = self.fs()?;
        let folder = folder(fs).await?;
        let mut file = folder
            .create_file(&file_name(number))
            .await
            .map_err(io_error)?;
        file.truncate().await.map_err(io_error)?;
        file.write_all(jpeg).await.map_err(io_error)?;
        file.flush().await.map_err(io_error)?;
        drop(file);
        fs.flush().await.map_err(io_error)
    }

    pub async fn delete(&mut self, number: u32) -> Result<(), Error> {
        let fs = self.fs()?;
        let folder = fs.root_dir().open_dir(FOLDER).await.map_err(io_error)?;
        folder.remove(&file_name(number)).await.map_err(io_error)?;
        fs.flush().await.map_err(io_error)
    }

    pub async fn load(&mut self, number: u32) -> Result<Vec<u8>, Error> {
        let fs = self.fs()?;
        let folder = fs.root_dir().open_dir(FOLDER).await.map_err(io_error)?;
        let name = file_name(number);
        let size = folder.open_meta(&name).await.map_err(io_error)?.len() as usize;
        let mut file = folder.open_file(&name).await.map_err(io_error)?;
        let mut data = vec![0u8; size];
        file.read_exact(&mut data).await.map_err(io_error)?;
        Ok(data)
    }
}

/// Opens the first FAT partition, or a FAT volume without a partition table.
async fn open_volume(mut card: Card) -> Option<Fs> {
    let size = card.size().await.ok()?;
    let volume = match Scheme::open(BufStream::new(card)).await {
        Ok(Scheme::Mbr(mbr)) => {
            let index = mbr.iter_used().find(|(_, entry)| entry.is_fat())?.0;
            mbr.into_partition(index).await.ok()?
        }
        Ok(Scheme::Superfloppy(io)) => StreamSlice::new(io, 0, size).await.ok()?,
        Ok(Scheme::Unknown(_)) => {
            println!("The SD card has no FAT volume");
            return None;
        }
        Err(error) => {
            println!("Cannot read the SD card: {error:?}");
            return None;
        }
    };
    match FileSystem::new(volume, FsOptions::new()).await {
        Ok(fs) => Some(fs),
        Err(error) => {
            println!("Cannot open the FAT volume: {error:?}");
            None
        }
    }
}

/// Opens the photo folder, and creates it if needed.
async fn folder(
    fs: &Fs,
) -> Result<Dir<'_, Volume, DefaultTimeProvider, LossyOemCpConverter>, Error> {
    let root = fs.root_dir();
    match root.open_dir(FOLDER).await {
        Ok(folder) => Ok(folder),
        Err(_) => root.create_dir(FOLDER).await.map_err(io_error),
    }
}

pub fn file_name(number: u32) -> String {
    format!("IMG_{number:04}.JPG")
}

/// The number in a short name like `IMG_0042.JPG`.
fn photo_number(name: &[u8]) -> Option<u32> {
    let digits = name.strip_prefix(b"IMG_")?.strip_suffix(b".JPG")?;
    if digits.is_empty() || !digits.iter().all(u8::is_ascii_digit) {
        return None;
    }
    core::str::from_utf8(digits).ok()?.parse().ok()
}

fn io_error(error: impl core::fmt::Debug) -> Error {
    println!("SD card error: {error:?}");
    Error::Io
}
