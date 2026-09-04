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
static FLASH_PATTERN: [u32; 4] = [0x3C3C_3C3C; 4];

fn as_bytes(words: &[u32]) -> &[u8] {
    // SAFETY: inspecting the in-memory bytes of `u32` words.
    unsafe { core::slice::from_raw_parts(words.as_ptr().cast(), size_of_val(words)) }
}

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    fn flash_from_peripherals(
        peripherals: esp_hal::peripherals::Peripherals,
    ) -> Flash<'static, Blocking> {
        Flash::new(peripherals.FLASH, Config::default()).unwrap()
    }

    fn assert_range_erased(flash: &mut Flash<'static, Blocking>, offset: u32, len: usize) {
        let mut buf = [0u32; 64];
        let mut remaining = len;
        let mut addr = offset;
        while remaining > 0 {
            let n = remaining.min(size_of_val(&buf));
            flash.read(addr, &mut buf[..n / 4]).unwrap();
            assert!(as_bytes(&buf)[..n].iter().all(|&b| b == 0xFF));
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

        let mut words = [0u32; 64];
        flash.read(APP_DESC_OFFSET, &mut words).unwrap();
        assert_eq!(as_bytes(&words), expected_app_desc());
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
        let pattern = [0xA5A5_A5A5u32; 64];
        let mut read_back = [0u32; 64];

        // SAFETY: NVS is not mapped firmware.
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };
        assert_range_erased(&mut flash, NVS, size_of_val(&pattern));

        unsafe { flash.write(NVS, &pattern).unwrap() };
        flash.read(NVS, &mut read_back).unwrap();
        assert_eq!(pattern, read_back);

        // NOR: programming cannot turn 0-bits back to 1 without an erase.
        let zeros = [0x0000_0000u32; 1];
        let ones = [0xFFFF_FFFFu32; 1];
        unsafe { flash.write(NVS, &zeros).unwrap() };
        unsafe { flash.write(NVS, &ones).unwrap() };
        let mut word = [0u32; 1];
        flash.read(NVS, &mut word).unwrap();
        assert_eq!(word, [0x0000_0000; 1]);
    }

    #[test]
    fn test_aligned_buffers() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let sector = flash.chip_info().sector_size;
        let mut pattern = [0u32; 128];
        for (i, word) in pattern.iter_mut().enumerate() {
            let b0 = (4 * i) as u8;
            *word = u32::from_le_bytes([
                b0,
                b0.wrapping_add(1),
                b0.wrapping_add(2),
                b0.wrapping_add(3),
            ]);
        }

        // SAFETY: NVS is not mapped firmware.
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };
        assert_range_erased(&mut flash, NVS, size_of_val(&pattern));
        unsafe { flash.write(NVS, &pattern).unwrap() };

        let mut read_back = [0u32; 128];
        flash.read(NVS, &mut read_back).unwrap();
        assert_eq!(pattern, read_back);

        // Remaining writes need 1-bits; erase first (NOR cannot program 0→1).
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };

        let page_cross = [0xAAAA_AAAAu32; 2];
        unsafe { flash.write(NVS + 252, &page_cross).unwrap() };
        let mut page_cross_read = [0u32; 2];
        flash.read(NVS + 252, &mut page_cross_read).unwrap();
        assert_eq!(page_cross, page_cross_read);

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
        let mut buf = [0u32; 1];

        assert_eq!(flash.read(cap, &mut buf), Err(Error::OutOfBounds));
        assert_eq!(
            unsafe { flash.write(cap, &[0u32; 1]) },
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

        assert_eq!(flash.read(1, &mut buf), Err(Error::NotAligned));
        assert_eq!(unsafe { flash.write(1, &buf) }, Err(Error::NotAligned));
        assert_eq!(
            unsafe { flash.erase(1, 1 + sector) },
            Err(Error::NotAligned)
        );
    }

    #[test]
    fn test_mapped_read_after_nvs_program() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let mut before = [0u32; 64];
        flash.read(APP_DESC_OFFSET, &mut before).unwrap();
        assert_eq!(as_bytes(&before), expected_app_desc());

        let sector = flash.chip_info().sector_size;
        // SAFETY: NVS is not mapped firmware.
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };
        let marker = [0x5A5A_5A5Au32; 4];
        unsafe { flash.write(NVS, &marker).unwrap() };

        let mut after = [0u32; 64];
        flash.read(APP_DESC_OFFSET, &mut after).unwrap();
        assert_eq!(before, after);
        assert_eq!(as_bytes(&after), expected_app_desc());
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
        let pattern = [0x5A5A_5A5Au32; 64];
        let mut read_back = [0u32; 64];

        // SAFETY: NVS is not mapped firmware.
        unsafe { flash.erase(NVS, NVS + sector).unwrap() };
        unsafe { flash.write(NVS, &pattern).unwrap() };
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

        let mut buf = [0u32; 1];
        assert_eq!(
            flash.read(APP_DESC_OFFSET, &mut buf),
            Err(Error::OtherCoreRunning)
        );
    }
}
