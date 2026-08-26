//! Internal SPI flash driver tests.
//!
//! Assumes a certain (i.e. default) partition table layout.
//% CHIP_FILTER: flash_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

use core::ptr::addr_of;

use esp_hal::{
    Blocking,
    flash::{Config, Error, Flash},
};
use hil_test as _;

/// Default factory-app image: `0x1_0000` plus 32 bytes of headers.
const APP_DESC_OFFSET: u32 = 0x10_020;
/// Default NVS data partition. First sector is used as scratch.
const NVS: u32 = 0x9000;

/// Source that lives in flash (`.rodata`) so writes take the bounced path.
static FLASH_PATTERN: [u8; 16] = [0x3C; 16];

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    fn flash_from_peripherals(
        peripherals: esp_hal::peripherals::Peripherals,
    ) -> Flash<'static, Blocking> {
        Flash::new(peripherals.FLASH, Config::default()).unwrap()
    }

    fn sector_size(flash: &Flash<'static, Blocking>) -> u32 {
        flash.chip_info().sector_size
    }

    fn assert_range_erased(flash: &mut Flash<'static, Blocking>, offset: u32, len: usize) {
        let mut buf = [0u8; 256];
        let mut remaining = len;
        let mut addr = offset;
        while remaining > 0 {
            let n = remaining.min(buf.len());
            flash.read(addr, &mut buf[..n]).unwrap();
            assert!(buf[..n].iter().all(|&b| b == 0xFF));
            addr += n as u32;
            remaining -= n;
        }
    }

    fn expected_app_desc() -> &'static [u8] {
        unsafe { core::slice::from_raw_parts(addr_of!(hil_test::ESP_APP_DESC) as *const u8, 256) }
    }

    #[test]
    fn test_can_read_app_desc() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let mut bytes = [0u8; 256];
        flash.read(APP_DESC_OFFSET, &mut bytes).unwrap();
        assert_eq!(&bytes, expected_app_desc());
    }

    #[test]
    fn test_chip_info() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let flash = flash_from_peripherals(peripherals);
        let info = flash.chip_info();

        // TODO: Assert decoded `capacity` against this board's known flash size.
        // Checking only that decode succeeded can pass with the JEDEC byte order reversed.
        assert_eq!(info.capacity as usize, flash.capacity());
        assert_eq!(info.sector_size, 4096);
        assert_eq!(info.block_size, 65536);
        assert_eq!(info.page_size, 256);
    }

    #[test]
    fn test_erase_write_read_nvs() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let sector = sector_size(&flash);
        let pattern = [0xA5u8; 256];
        let mut read_back = [0u8; 256];

        flash.erase(NVS, NVS + sector).unwrap();
        assert_range_erased(&mut flash, NVS, pattern.len());

        flash.write(NVS, &pattern).unwrap();
        flash.read(NVS, &mut read_back).unwrap();
        assert_eq!(pattern, read_back);

        // NOR: programming cannot turn 0-bits back to 1 without an erase.
        flash.write(NVS, &[0x00u8; 4]).unwrap();
        flash.write(NVS, &[0xFFu8; 4]).unwrap();
        let mut word = [0u8; 4];
        flash.read(NVS, &mut word).unwrap();
        assert_eq!(word, [0x00; 4]);
    }

    #[test]
    fn test_buffer_paths() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let sector = sector_size(&flash);
        let mut pattern = [0u8; 512];
        for (i, b) in pattern.iter_mut().enumerate() {
            *b = i as u8;
        }

        flash.erase(NVS, NVS + sector).unwrap();
        assert_range_erased(&mut flash, NVS, pattern.len());
        flash.write(NVS, &pattern).unwrap();

        for len in 1..=3 {
            let mut buf = [0u8; 4];
            flash.read(NVS, &mut buf[..len]).unwrap();
            assert_eq!(&buf[..len], &pattern[..len]);

            flash.read(NVS + 1, &mut buf[..len]).unwrap();
            assert_eq!(&buf[..len], &pattern[1..1 + len]);
        }

        let mut five = [0u8; 5];
        flash.read(NVS + 1, &mut five).unwrap();
        assert_eq!(&five[..], &pattern[1..6]);

        // Unaligned destination pointer; length that is 1/2/3 bytes short of a word.
        let mut pad = [0u8; 8];
        let dest = &mut pad[1..6];
        flash.read(NVS + 2, dest).unwrap();
        assert_eq!(dest, &pattern[2..7]);

        // Aligned offset, unaligned destination, word-sized length.
        let dest = &mut pad[1..5];
        flash.read(NVS, dest).unwrap();
        assert_eq!(dest, &pattern[..4]);

        // Window whose aligned superset crosses a 256-byte bounce-buffer chunk.
        let mut cross = [0u8; 8];
        flash.read(NVS + 254, &mut cross).unwrap();
        assert_eq!(&cross[..], &pattern[254..262]);

        // Multi-chunk bounced read (length > bounce payload at an unaligned offset).
        let mut large = [0u8; 300];
        flash.read(NVS + 1, &mut large).unwrap();
        assert_eq!(&large[..], &pattern[1..301]);

        // Remaining writes need 1-bits; erase first (NOR cannot program 0→1).
        flash.erase(NVS, NVS + sector).unwrap();

        let page_cross = [0xAAu8; 8];
        flash.write(NVS + 252, &page_cross).unwrap();
        let mut page_cross_read = [0u8; 8];
        flash.read(NVS + 252, &mut page_cross_read).unwrap();
        assert_eq!(page_cross, page_cross_read);

        // Flash-resident source (not in DRAM) takes the bounced write path.
        flash.write(NVS, &FLASH_PATTERN).unwrap();
        let mut from_flash = [0u8; 16];
        flash.read(NVS, &mut from_flash).unwrap();
        assert_eq!(from_flash, FLASH_PATTERN);

        // Unaligned DRAM source, aligned flash offset.
        let mut unaligned_src = [0u8; 8];
        unaligned_src[1..5].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        flash.write(NVS + 16, &unaligned_src[1..5]).unwrap();
        let mut unaligned_read = [0u8; 4];
        flash.read(NVS + 16, &mut unaligned_read).unwrap();
        assert_eq!(unaligned_read, [0x11, 0x22, 0x33, 0x44]);
    }

    #[test]
    fn test_multi_sector_erase() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let sector = sector_size(&flash);
        flash.erase(NVS, NVS + 2 * sector).unwrap();
        assert_range_erased(&mut flash, NVS, sector as usize);
        assert_range_erased(&mut flash, NVS + sector, 256);
    }

    #[test]
    fn test_out_of_bounds_and_unaligned() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let cap = flash.capacity() as u32;
        let sector = sector_size(&flash);
        let mut buf = [0u8; 4];

        assert_eq!(flash.read(cap, &mut buf), Err(Error::OutOfBounds));
        assert_eq!(flash.write(cap, &[0u8; 4]), Err(Error::OutOfBounds));
        assert_eq!(flash.erase(cap, cap + sector), Err(Error::OutOfBounds));
        assert_eq!(flash.erase(NVS + sector, NVS), Err(Error::OutOfBounds));

        flash.read(0, &mut []).unwrap();
        flash.read(cap, &mut []).unwrap();
        assert_eq!(flash.read(cap + 1, &mut []), Err(Error::OutOfBounds));
        flash.write(NVS, &[]).unwrap();
        flash.erase(NVS, NVS).unwrap();

        assert_eq!(flash.write(1, &buf), Err(Error::NotAligned));
        assert_eq!(flash.write(0, &[0u8; 1]), Err(Error::NotAligned));
        assert_eq!(flash.erase(1, 1 + sector), Err(Error::NotAligned));
    }

    #[test]
    fn test_mapped_read_after_nvs_program() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let mut before = [0u8; 256];
        flash.read(APP_DESC_OFFSET, &mut before).unwrap();
        assert_eq!(&before, expected_app_desc());

        let sector = sector_size(&flash);
        flash.erase(NVS, NVS + sector).unwrap();
        flash.write(NVS, &[0x5Au8; 16]).unwrap();

        let mut after = [0u8; 256];
        flash.read(APP_DESC_OFFSET, &mut after).unwrap();
        assert_eq!(before, after);
        assert_eq!(&after, expected_app_desc());
    }

    #[cfg(multi_core)]
    #[test]
    fn test_auto_park_other_core() {
        use core::sync::atomic::{AtomicU32, Ordering};

        use esp_hal::system::{CpuControl, Stack};
        use hil_test::mk_static;

        static TICKS: AtomicU32 = AtomicU32::new(0);

        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
        let stack = mk_static!(Stack<8192>, Stack::new());
        let _guard = cpu_control
            .start_app_core(stack, || {
                loop {
                    TICKS.fetch_add(1, Ordering::Relaxed);
                }
            })
            .unwrap();

        while TICKS.load(Ordering::Relaxed) == 0 {}

        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();
        let sector = sector_size(&flash);
        let pattern = [0x5Au8; 256];
        let mut read_back = [0u8; 256];

        flash.erase(NVS, NVS + sector).unwrap();
        flash.write(NVS, &pattern).unwrap();
        flash.read(NVS, &mut read_back).unwrap();
        assert_eq!(pattern, read_back);

        let before = TICKS.load(Ordering::Relaxed);
        while TICKS.load(Ordering::Relaxed) == before {}
    }

    #[cfg(multi_core)]
    #[test]
    fn test_error_if_other_core_running() {
        use core::sync::atomic::{AtomicBool, Ordering};

        use esp_hal::{
            flash::MultiCoreStrategy,
            system::{CpuControl, Stack},
        };
        use hil_test::mk_static;

        static STARTED: AtomicBool = AtomicBool::new(false);

        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
        let stack = mk_static!(Stack<8192>, Stack::new());
        let _guard = cpu_control
            .start_app_core(stack, || {
                STARTED.store(true, Ordering::Relaxed);
                loop {}
            })
            .unwrap();

        while !STARTED.load(Ordering::Relaxed) {}

        let mut flash = Flash::new(
            peripherals.FLASH,
            Config::default().with_multi_core_strategy(MultiCoreStrategy::Error),
        )
        .unwrap();

        let mut buf = [0u8; 4];
        assert_eq!(
            flash.read(APP_DESC_OFFSET, &mut buf),
            Err(Error::OtherCoreRunning)
        );
    }
}
