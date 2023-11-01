#![no_std]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]
#![feature(c_variadic)]
#![feature(linkage)]
#![cfg_attr(feature = "async", feature(async_fn_in_trait))]
#![cfg_attr(feature = "async", allow(incomplete_features))]
#![doc = include_str!("../../README.md")]

// MUST be the first module
mod fmt;

use core::cell::RefCell;
use core::mem::MaybeUninit;

use common_adapter::RADIO_CLOCKS;
use critical_section::Mutex;

#[cfg(esp32)]
use esp32_hal as hal;
#[cfg(esp32c2)]
use esp32c2_hal as hal;
#[cfg(esp32c3)]
use esp32c3_hal as hal;
#[cfg(esp32c6)]
use esp32c6_hal as hal;
#[cfg(esp32s2)]
use esp32s2_hal as hal;
#[cfg(esp32s3)]
use esp32s3_hal as hal;

#[cfg(any(esp32c2, esp32c3, esp32c6))]
use hal::systimer::{Alarm, Target};

use common_adapter::init_radio_clock_control;
use hal::system::RadioClockController;

use fugit::MegahertzU32;
use hal::clock::Clocks;
use linked_list_allocator::Heap;
#[cfg(feature = "wifi")]
use wifi::WifiError;

use crate::common_adapter::init_rng;
use crate::tasks::init_tasks;
use crate::timer::setup_timer_isr;
use common_adapter::chip_specific::phy_mem_init;

mod binary {
    pub use esp_wifi_sys::*;
}
mod compat;
mod preempt;

mod timer;

#[cfg(feature = "wifi")]
pub mod wifi;

#[cfg(feature = "ble")]
pub mod ble;

#[cfg(feature = "esp-now")]
pub mod esp_now;

pub(crate) mod common_adapter;

#[doc(hidden)]
pub mod tasks;

pub(crate) mod memory_fence;

use critical_section;
use timer::{get_systimer_count, ticks_to_millis};

#[cfg(all(feature = "embedded-svc", feature = "wifi"))]
pub mod wifi_interface;

/// Return the current systimer time in milliseconds
pub fn current_millis() -> u64 {
    ticks_to_millis(get_systimer_count())
}

#[allow(unused)]
#[cfg(all(not(feature = "big-heap")))]
const DEFAULT_HEAP_SIZE: usize = 64 * 1024;

#[allow(unused)]
#[cfg(all(not(esp32s2), feature = "big-heap"))]
const DEFAULT_HEAP_SIZE: usize = 110 * 1024;

#[allow(unused)]
#[cfg(all(esp32s2, feature = "big-heap"))]
const DEFAULT_HEAP_SIZE: usize = 72 * 1024;

const HEAP_SIZE: usize = crate::CONFIG.heap_size;

#[allow(unused)]
#[cfg(debug_assertions)]
const DEFAULT_TICK_RATE_HZ: u32 = 50;

#[allow(unused)]
#[cfg(not(debug_assertions))]
const DEFAULT_TICK_RATE_HZ: u32 = 100;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[toml_cfg::toml_config]
/// Tunable parameters for the WiFi driver
struct Config {
    #[default(5)]
    rx_queue_size: usize,
    #[default(3)]
    tx_queue_size: usize,
    #[default(10)]
    static_rx_buf_num: usize,
    #[default(32)]
    dynamic_rx_buf_num: usize,
    #[default(0)]
    static_tx_buf_num: usize,
    #[default(32)]
    dynamic_tx_buf_num: usize,
    #[default(0)]
    ampdu_rx_enable: usize,
    #[default(0)]
    ampdu_tx_enable: usize,
    #[default(0)]
    amsdu_tx_enable: usize,
    #[default(6)]
    rx_ba_win: usize,
    #[default(1)]
    max_burst_size: usize,
    #[default("CN")]
    country_code: &'static str,
    #[default(0)]
    country_code_operating_class: u8,
    #[default(1492)]
    mtu: usize,
    #[default(DEFAULT_HEAP_SIZE)]
    heap_size: usize,
    #[default(DEFAULT_TICK_RATE_HZ)]
    tick_rate_hz: u32,
    #[default(3)]
    listen_interval: u16,
    #[default(6)]
    beacon_timeout: u16,
    #[default(300)]
    ap_beacon_timeout: u16,
    #[default(1)]
    failure_retry_cnt: u8,
    #[default(0)]
    scan_method: u32,
}

#[cfg_attr(esp32, link_section = ".dram2_uninit")]
static mut HEAP_DATA: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

pub(crate) static HEAP: Mutex<RefCell<Heap>> = Mutex::new(RefCell::new(Heap::empty()));

fn init_heap() {
    critical_section::with(|cs| {
        HEAP.borrow_ref_mut(cs)
            .init_from_slice(unsafe { &mut HEAP_DATA })
    });
}

#[cfg(any(esp32c3, esp32c2, esp32c6))]
pub type EspWifiTimer = Alarm<Target, 0>;

#[cfg(any(esp32, esp32s3, esp32s2))]
pub type EspWifiTimer = hal::timer::Timer<hal::timer::Timer0<hal::peripherals::TIMG1>>;

#[derive(Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
/// An internal struct designed to make [`EspWifiInitialization`] uncreatable outside of this crate.
pub struct EspWifiInitializationInternal;

#[derive(Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Initialized the driver for WiFi, Bluetooth or both.
pub enum EspWifiInitialization {
    #[cfg(feature = "wifi")]
    Wifi(EspWifiInitializationInternal),
    #[cfg(feature = "ble")]
    Ble(EspWifiInitializationInternal),
    #[cfg(coex)]
    WifiBle(EspWifiInitializationInternal),
}

impl EspWifiInitialization {
    #[allow(unused)]
    fn is_wifi(&self) -> bool {
        match self {
            #[cfg(feature = "ble")]
            EspWifiInitialization::Ble(_) => false,
            _ => true,
        }
    }

    #[allow(unused)]
    fn is_ble(&self) -> bool {
        match self {
            #[cfg(feature = "wifi")]
            EspWifiInitialization::Wifi(_) => false,
            _ => true,
        }
    }
}

#[derive(Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Initialize the driver for WiFi, Bluetooth or both.
pub enum EspWifiInitFor {
    #[cfg(feature = "wifi")]
    Wifi,
    #[cfg(feature = "ble")]
    Ble,
    #[cfg(coex)]
    WifiBle,
}

impl EspWifiInitFor {
    #[allow(unused)]
    fn is_wifi(&self) -> bool {
        match self {
            #[cfg(feature = "ble")]
            EspWifiInitFor::Ble => false,
            _ => true,
        }
    }

    #[allow(unused)]
    fn is_ble(&self) -> bool {
        match self {
            #[cfg(feature = "wifi")]
            EspWifiInitFor::Wifi => false,
            _ => true,
        }
    }
}

/// Initialize for using WiFi and or BLE
pub fn initialize(
    init_for: EspWifiInitFor,
    timer: EspWifiTimer,
    rng: hal::Rng,
    radio_clocks: hal::system::RadioClockControl,
    clocks: &Clocks,
) -> Result<EspWifiInitialization, InitializationError> {
    #[cfg(any(esp32, esp32s3, esp32s2))]
    const MAX_CLOCK: u32 = 240;

    #[cfg(any(esp32c3, esp32c6))]
    const MAX_CLOCK: u32 = 160;

    #[cfg(esp32c2)]
    const MAX_CLOCK: u32 = 120;

    if clocks.cpu_clock != MegahertzU32::MHz(MAX_CLOCK) {
        return Err(InitializationError::WrongClockConfig);
    }

    #[cfg(esp32s3)]
    unsafe {
        // should be done by the HAL in `ClockControl::configure`
        const ETS_UPDATE_CPU_FREQUENCY: u32 = 0x40001a4c;

        // cast to usize is just needed because of the way we run clippy in CI
        let rom_ets_update_cpu_frequency: fn(ticks_per_us: u32) =
            core::mem::transmute(ETS_UPDATE_CPU_FREQUENCY as usize);

        rom_ets_update_cpu_frequency(240); // we know it's 240MHz because of the check above
    }

    info!("esp-wifi configuration {:?}", crate::CONFIG);

    crate::common_adapter::chip_specific::enable_wifi_power_domain();

    init_heap();
    phy_mem_init();
    init_radio_clock_control(radio_clocks);
    init_rng(rng);
    init_tasks();
    setup_timer_isr(timer);
    wifi_set_log_verbose();
    init_clocks();

    #[cfg(coex)]
    match crate::wifi::coex_initialize() {
        0 => {}
        error => return Err(InitializationError::General(error)),
    }

    #[cfg(feature = "wifi")]
    if init_for.is_wifi() {
        debug!("wifi init");
        // wifi init
        crate::wifi::wifi_init()?;
    }

    #[cfg(feature = "ble")]
    if init_for.is_ble() {
        // ble init
        // for some reason things don't work when initializing things the other way around
        // while the original implementation in NuttX does it like that
        debug!("ble init");
        crate::ble::ble_init();
    }

    match init_for {
        #[cfg(feature = "wifi")]
        EspWifiInitFor::Wifi => Ok(EspWifiInitialization::Wifi(EspWifiInitializationInternal)),
        #[cfg(feature = "ble")]
        EspWifiInitFor::Ble => Ok(EspWifiInitialization::Ble(EspWifiInitializationInternal)),
        #[cfg(coex)]
        EspWifiInitFor::WifiBle => Ok(EspWifiInitialization::WifiBle(
            EspWifiInitializationInternal,
        )),
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error which can be returned during [`initialize`].
pub enum InitializationError {
    General(i32),
    #[cfg(feature = "wifi")]
    WifiError(WifiError),
    WrongClockConfig,
}

#[cfg(feature = "wifi")]
impl From<WifiError> for InitializationError {
    fn from(value: WifiError) -> Self {
        InitializationError::WifiError(value)
    }
}

/// Enable verbose logging within the WiFi driver
/// Does nothing unless the `wifi-logs` feature is enabled.
pub fn wifi_set_log_verbose() {
    #[cfg(feature = "wifi-logs")]
    unsafe {
        use crate::binary::include::{
            esp_wifi_internal_set_log_level, wifi_log_level_t_WIFI_LOG_VERBOSE,
        };

        esp_wifi_internal_set_log_level(wifi_log_level_t_WIFI_LOG_VERBOSE);
    }
}

fn init_clocks() {
    unsafe {
        unwrap!(RADIO_CLOCKS.as_mut()).init_clocks();
    }
}
