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

/// Source that lives in flash (`.rodata`); writes of this must be rejected.
static FLASH_PATTERN: [u8; 16] = [0x3C; 16];

#[repr(align(4))]
struct Aligned<const N: usize>([u8; N]);

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    fn flash_from_peripherals(
        peripherals: esp_hal::peripherals::Peripherals,
    ) -> Flash<'static, Blocking> {
        Flash::new(peripherals.FLASH, Config::default()).unwrap()
    }

    fn assert_range_erased(flash: &mut Flash<'static, Blocking>, offset: u32, len: usize) {
        let mut buf = Aligned([0u8; 256]);
        let mut remaining = len;
        let mut addr = offset;
        while remaining > 0 {
            let n = remaining.min(buf.0.len());
            flash.read(addr, &mut buf.0[..n]).unwrap();
            assert!(buf.0[..n].iter().all(|&b| b == 0xFF));
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

        let mut bytes = Aligned([0u8; 256]);
        flash.read(APP_DESC_OFFSET, &mut bytes.0).unwrap();
        assert_eq!(&bytes.0, expected_app_desc());
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

        let sector = flash.chip_info().sector_size;
        let pattern = Aligned([0xA5u8; 256]);
        let mut read_back = Aligned([0u8; 256]);

        // SAFETY: NVS is not mapped firmware.
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };
        assert_range_erased(&mut flash, NVS, pattern.0.len());

        unsafe { flash.write(NVS, &pattern.0).unwrap() };
        flash.read(NVS, &mut read_back.0).unwrap();
        assert_eq!(pattern.0, read_back.0);

        // NOR: programming cannot turn 0-bits back to 1 without an erase.
        let zeros = Aligned([0x00u8; 4]);
        let ones = Aligned([0xFFu8; 4]);
        unsafe { flash.write(NVS, &zeros.0).unwrap() };
        unsafe { flash.write(NVS, &ones.0).unwrap() };
        let mut word = Aligned([0u8; 4]);
        flash.read(NVS, &mut word.0).unwrap();
        assert_eq!(word.0, [0x00; 4]);
    }

    #[test]
    fn test_aligned_buffers() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let sector = flash.chip_info().sector_size;
        let mut pattern = Aligned([0u8; 512]);
        for (i, b) in pattern.0.iter_mut().enumerate() {
            *b = i as u8;
        }

        // SAFETY: NVS is not mapped firmware.
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };
        assert_range_erased(&mut flash, NVS, pattern.0.len());
        unsafe { flash.write(NVS, &pattern.0).unwrap() };

        let mut read_back = Aligned([0u8; 512]);
        flash.read(NVS, &mut read_back.0).unwrap();
        assert_eq!(pattern.0, read_back.0);

        // Remaining writes need 1-bits; erase first (NOR cannot program 0→1).
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };

        let page_cross = Aligned([0xAAu8; 8]);
        unsafe { flash.write(NVS + 252, &page_cross.0).unwrap() };
        let mut page_cross_read = Aligned([0u8; 8]);
        flash.read(NVS + 252, &mut page_cross_read.0).unwrap();
        assert_eq!(page_cross.0, page_cross_read.0);

        // Flash-resident source is rejected; the caller must stage through RAM.
        assert_eq!(
            unsafe { flash.write(NVS, &FLASH_PATTERN) },
            Err(Error::NotSupported)
        );
    }

    #[test]
    fn test_multi_sector_erase() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let sector = flash.chip_info().sector_size;
        // SAFETY: NVS is not mapped firmware.
        unsafe { flash.erase(NVS, NVS + 2 * sector).unwrap() };
        assert_range_erased(&mut flash, NVS, sector as usize);
        assert_range_erased(&mut flash, NVS + sector, 256);
    }

    #[test]
    fn test_out_of_bounds_and_unaligned() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let cap = flash.capacity() as u32;
        let sector = flash.chip_info().sector_size;
        let mut buf = Aligned([0u8; 4]);

        assert_eq!(flash.read(cap, &mut buf.0), Err(Error::OutOfBounds));
        assert_eq!(
            unsafe { flash.write(cap, &[0u8; 4]) },
            Err(Error::OutOfBounds)
        );
        assert_eq!(
            unsafe { flash.erase(cap, cap + sector) },
            Err(Error::OutOfBounds)
        );
        assert_eq!(
            unsafe { flash.erase(NVS + sector, NVS) },
            Err(Error::OutOfBounds)
        );

        flash.read(0, &mut []).unwrap();
        flash.read(cap, &mut []).unwrap();
        assert_eq!(flash.read(cap + 1, &mut []), Err(Error::OutOfBounds));
        unsafe { flash.write(NVS, &[]).unwrap() };
        unsafe { flash.erase(NVS, NVS).unwrap() };

        assert_eq!(flash.read(1, &mut buf.0), Err(Error::NotAligned));
        assert_eq!(flash.read(0, &mut [0u8; 1]), Err(Error::NotAligned));
        assert_eq!(unsafe { flash.write(1, &buf.0) }, Err(Error::NotAligned));
        assert_eq!(unsafe { flash.write(0, &[0u8; 1]) }, Err(Error::NotAligned));
        assert_eq!(
            unsafe { flash.erase(1, 1 + sector) },
            Err(Error::NotAligned)
        );

        let mut pad = [0u8; 8];
        assert_eq!(flash.read(NVS, &mut pad[1..5]), Err(Error::NotAligned));
        assert_eq!(
            unsafe { flash.write(NVS, &pad[1..5]) },
            Err(Error::NotAligned)
        );
    }

    #[test]
    fn test_mapped_read_after_nvs_program() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let mut before = Aligned([0u8; 256]);
        flash.read(APP_DESC_OFFSET, &mut before.0).unwrap();
        assert_eq!(&before.0, expected_app_desc());

        let sector = flash.chip_info().sector_size;
        // SAFETY: NVS is not mapped firmware.
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };
        let marker = Aligned([0x5Au8; 16]);
        unsafe { flash.write(NVS, &marker.0).unwrap() };

        let mut after = Aligned([0u8; 256]);
        flash.read(APP_DESC_OFFSET, &mut after.0).unwrap();
        assert_eq!(before.0, after.0);
        assert_eq!(&after.0, expected_app_desc());
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
        let sector = flash.chip_info().sector_size;
        let pattern = Aligned([0x5Au8; 256]);
        let mut read_back = Aligned([0u8; 256]);

        // SAFETY: NVS is not mapped firmware.
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };
        unsafe { flash.write(NVS, &pattern.0).unwrap() };
        flash.read(NVS, &mut read_back.0).unwrap();
        assert_eq!(pattern.0, read_back.0);

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

        let mut buf = Aligned([0u8; 4]);
        assert_eq!(
            flash.read(APP_DESC_OFFSET, &mut buf.0),
            Err(Error::OtherCoreRunning)
        );
    }
}
