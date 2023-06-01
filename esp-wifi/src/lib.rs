#![no_std]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]
#![feature(c_variadic)]
#![cfg_attr(feature = "async", feature(async_fn_in_trait))]
#![cfg_attr(feature = "async", allow(incomplete_features))]

use core::cell::RefCell;
use core::mem::MaybeUninit;

use common_adapter::RADIO_CLOCKS;
use critical_section::Mutex;
#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "esp32c2")]
use esp32c2_hal as hal;
#[cfg(feature = "esp32c2")]
use esp32c2_hal::systimer::{Alarm, Target};
#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c3")]
use esp32c3_hal::systimer::{Alarm, Target};
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal::systimer::{Alarm, Target};
#[cfg(feature = "esp32s2")]
use esp32s2_hal as hal;
#[cfg(feature = "esp32s3")]
use esp32s3_hal as hal;

use crate::hal::system::RadioClockController;
use common_adapter::init_radio_clock_control;

use fugit::MegahertzU32;
use hal::clock::Clocks;
use linked_list_allocator::Heap;
#[cfg(feature = "wifi")]
use wifi::WifiError;

use crate::common_adapter::init_rng;
use crate::tasks::init_tasks;
use crate::timer::setup_timer_isr;
use common_adapter::chip_specific::phy_mem_init;

#[doc(hidden)]
pub mod binary {
    pub use esp_wifi_sys::*;
}

#[doc(hidden)]
pub mod compat;

#[doc(hidden)]
pub mod preempt;

#[doc(hidden)]
#[cfg_attr(feature = "esp32", path = "timer_esp32.rs")]
#[cfg_attr(feature = "esp32c3", path = "timer_esp32c3.rs")]
#[cfg_attr(feature = "esp32c2", path = "timer_esp32c2.rs")]
#[cfg_attr(feature = "esp32c6", path = "timer_esp32c6.rs")]
#[cfg_attr(feature = "esp32s3", path = "timer_esp32s3.rs")]
#[cfg_attr(feature = "esp32s2", path = "timer_esp32s2.rs")]
pub mod timer;

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

pub use critical_section;
use timer::{get_systimer_count, TICKS_PER_SECOND};

#[cfg(all(feature = "embedded-svc", feature = "wifi"))]
pub mod wifi_interface;

pub fn current_millis() -> u64 {
    get_systimer_count() / (TICKS_PER_SECOND / 1000)
}

#[cfg(all(not(feature = "coex"), not(feature = "big-heap")))]
const HEAP_SIZE: usize = 64 * 1024;

#[cfg(all(feature = "coex", not(feature = "big-heap")))]
const HEAP_SIZE: usize = 64 * 1024;

#[cfg(all(not(feature = "coex"), not(feature = "esp32s2"), feature = "big-heap"))]
const HEAP_SIZE: usize = 110 * 1024;

#[cfg(all(not(feature = "coex"), feature = "esp32s2", feature = "big-heap"))]
const HEAP_SIZE: usize = 72 * 1024;

#[cfg(all(feature = "coex", feature = "big-heap"))]
const HEAP_SIZE: usize = 110 * 1024;

#[cfg_attr(feature = "esp32", link_section = ".dram2_uninit")]
static mut HEAP_DATA: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

pub(crate) static HEAP: Mutex<RefCell<Heap>> = Mutex::new(RefCell::new(Heap::empty()));

fn init_heap() {
    critical_section::with(|cs| {
        HEAP.borrow(cs)
            .borrow_mut()
            .init_from_slice(unsafe { &mut HEAP_DATA })
    });
}

#[cfg(any(feature = "esp32c3", feature = "esp32c2", feature = "esp32c6"))]
type Timer = Alarm<Target, 0>;

#[cfg(any(feature = "esp32", feature = "esp32s3", feature = "esp32s2"))]
type Timer = hal::timer::Timer<hal::timer::Timer0<hal::peripherals::TIMG1>>;

#[derive(Debug, PartialEq, PartialOrd)]
pub struct EspWifiInitializationInternal {
    _private: (),
}

impl EspWifiInitializationInternal {
    fn new() -> Self {
        Self { _private: () }
    }
}

#[derive(Debug, PartialEq, PartialOrd)]
pub enum EspWifiInitialization {
    #[cfg(feature = "wifi")]
    Wifi(EspWifiInitializationInternal),
    #[cfg(feature = "ble")]
    Ble(EspWifiInitializationInternal),
    #[cfg(all(feature = "wifi", feature = "ble"))]
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
pub enum EspWifiInitFor {
    #[cfg(feature = "wifi")]
    Wifi,
    #[cfg(feature = "ble")]
    Ble,
    #[cfg(all(feature = "wifi", feature = "ble"))]
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

/// Initialize for using WiFi / BLE
/// This will initialize internals and also initialize WiFi and BLE
pub fn initialize(
    init_for: EspWifiInitFor,
    timer: Timer,
    rng: hal::Rng,
    radio_clocks: hal::system::RadioClockControl,
    clocks: &Clocks,
) -> Result<EspWifiInitialization, InitializationError> {
    #[cfg(all(not(feature = "coex"), feature = "wifi", feature = "bluetooth"))]
    if init_for == EspWifiInitFor::WifiBle {
        panic!("Trying to use Wifi and BLE without COEX feature");
    }

    #[cfg(any(feature = "esp32", feature = "esp32s3", feature = "esp32s2"))]
    if clocks.cpu_clock != MegahertzU32::MHz(240) {
        return Err(InitializationError::WrongClockConfig);
    }

    #[cfg(feature = "esp32c6")]
    if clocks.cpu_clock != MegahertzU32::MHz(160) {
        return Err(InitializationError::WrongClockConfig);
    }

    #[cfg(feature = "esp32c3")]
    if clocks.cpu_clock != MegahertzU32::MHz(160) {
        return Err(InitializationError::WrongClockConfig);
    }

    #[cfg(feature = "esp32c2")]
    if clocks.cpu_clock != MegahertzU32::MHz(120) {
        return Err(InitializationError::WrongClockConfig);
    }

    #[cfg(feature = "esp32s3")]
    unsafe {
        // should be done by the HAL in `ClockControl::configure`
        const ETS_UPDATE_CPU_FREQUENCY: u32 = 0x40001a4c;

        // cast to usize is just needed because of the way we run clippy in CI
        let rom_ets_update_cpu_frequency: fn(ticks_per_us: u32) =
            core::mem::transmute(ETS_UPDATE_CPU_FREQUENCY as usize);

        rom_ets_update_cpu_frequency(240); // we know it's 240MHz because of the check above
    }

    init_heap();
    phy_mem_init();
    init_radio_clock_control(radio_clocks);
    init_rng(rng);
    init_tasks();
    setup_timer_isr(timer);
    wifi_set_log_verbose();
    init_clocks();
    init_buffer();

    #[cfg(feature = "coex")]
    {
        log::debug!("coex init");
        let res = crate::wifi::coex_initialize();
        if res != 0 {
            return Err(InitializationError::General(res));
        }
    }

    #[cfg(feature = "wifi")]
    {
        if init_for.is_wifi() {
            log::debug!("wifi init");
            // wifi init
            crate::wifi::wifi_init()?;
        }
    }

    #[cfg(feature = "ble")]
    {
        if init_for.is_ble() {
            // ble init
            // for some reason things don't work when initializing things the other way around
            // while the original implementation in NuttX does it like that
            log::debug!("ble init");
            crate::ble::ble_init();
        }
    }

    match init_for {
        #[cfg(feature = "wifi")]
        EspWifiInitFor::Wifi => Ok(EspWifiInitialization::Wifi(
            EspWifiInitializationInternal::new(),
        )),
        #[cfg(feature = "ble")]
        EspWifiInitFor::Ble => Ok(EspWifiInitialization::Ble(
            EspWifiInitializationInternal::new(),
        )),
        #[cfg(all(feature = "wifi", feature = "ble"))]
        EspWifiInitFor::WifiBle => Ok(EspWifiInitialization::WifiBle(
            EspWifiInitializationInternal::new(),
        )),
    }
}

#[derive(Debug, Clone, Copy)]
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

pub fn wifi_set_log_verbose() {
    #[cfg(feature = "wifi-logs")]
    unsafe {
        use crate::binary::include::{esp_wifi_internal_set_log_level, wifi_log_level_t};

        let level: wifi_log_level_t = crate::binary::include::wifi_log_level_t_WIFI_LOG_VERBOSE;
        esp_wifi_internal_set_log_level(level);
    }
}

pub fn init_buffer() {
    // nothing anymore for now
}

pub fn init_clocks() {
    unsafe {
        RADIO_CLOCKS.as_mut().unwrap().init_clocks();
    }
}
