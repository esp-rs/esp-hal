//! Retention with the REGDMA.
//!
//! The PMU powers the modem domain down in light sleep, while every radio that is up can retain its
//! MAC and baseband. On the way into the sleep and out of it, the REGDMA saves and restores the
//! modules of the radios, and the modules of this file: the modem clocks, and the PHY front end
//! that the radios share.

use core::ptr::NonNull;

use allocator_api2::boxed::Box;
use esp_hal::rtc_cntl::regdma::{self, LinkConfig, LinkMemory, Module, SLEEP_ENTRY};

use crate::compat::malloc::InternalMemory;

const fn registers(id: u16, address: u32, length: u16) -> LinkConfig {
    LinkConfig::continuous(id, address, address, length, SLEEP_ENTRY)
}

/// The clock and reset registers of the modem, `MODEM_SYSCON` and `MODEM_LPCON`.
// See `sleep_clock.c` of each chip in
// <https://github.com/espressif/esp-idf/tree/4d80c1b29ae/components/esp_hw_support/lowpower/port>.
const CLOCK_MODEM: &[LinkConfig] = cfg_select! {
    esp32c6 => &[registers(0x0200, 0x600a_9800, 9)],
    any(esp32c5, esp32c61) => &[registers(0x0200, 0x600a_9c00, 10)],
    esp32h2 => &[
        registers(0x0200, 0x600a_5400, 7),
        registers(0x0300, 0x600a_d000, 7),
    ],
    esp32h4 => &[
        registers(0x0200, 0x600c_9c00, 10),
        registers(0x0300, 0x600c_f000, 12),
    ],
    esp32s31 => &[
        // The PMU may not ungate the PLL in time for the restore. A dummy write to the date
        // register first keeps the next nodes from a bus timeout.
        LinkConfig::write(0x0200, 0x2010_9c30, 0x0fff_ffff, 0x0fff_ffff, SLEEP_ENTRY),
        registers(0x0201, 0x2010_9c00, 10),
        registers(0x0300, 0x2010_f000, 21),
    ],
};

/// The PHY front end, which Wi-Fi and Bluetooth share.
// See `phy_init_data.c` of each chip in
// <https://github.com/espressif/esp-idf/tree/4d80c1b29ae/components/esp_phy>.
const PHY_FE: &[LinkConfig] = cfg_select! {
    esp32c6 => &[registers(0xff00, 0x600a_0000, 58)],
    esp32c61 => &[
        registers(0xff00, 0x600a_0000, 21),
        registers(0xff01, 0x600a_0400, 34),
        registers(0xff02, 0x600a_0800, 56),
        registers(0xff03, 0x600a_0c00, 21),
    ],
    esp32c5 => &[
        registers(0xff00, 0x600a_0000, 21),
        registers(0xff01, 0x600a_0400, 35),
        registers(0xff02, 0x600a_0800, 56),
        registers(0xff03, 0x600a_0c00, 22),
    ],
    esp32s31 => &[
        registers(0xff00, 0x2010_0000, 22),
        registers(0xff01, 0x2010_0400, 35),
        registers(0xff02, 0x2010_0800, 57),
        registers(0xff03, 0x2010_0c00, 22),
    ],
    // ESP-IDF retains no front end on the ESP32-H2 and the ESP32-H4.
    _ => &[],
};

const MEMORY_WORDS: usize = (regdma::links_size(CLOCK_MODEM) + regdma::links_size(PHY_FE)) / 4;

pub(super) struct State {
    /// The nodes of the modem clocks and the PHY front end, present while a radio can power the
    /// modem domain down.
    memory: Option<Box<[u32; MEMORY_WORDS], InternalMemory>>,
}

impl State {
    pub(super) const fn new() -> Self {
        Self { memory: None }
    }

    /// Builds the nodes of the modem clocks and the PHY front end. Returns whether they are ready.
    pub(super) fn enable(&mut self) -> bool {
        if self.memory.is_some() {
            return true;
        }
        let Ok(mut memory) = Box::try_new_in([0; MEMORY_WORDS], InternalMemory) else {
            return false;
        };
        let mut links = unsafe {
            LinkMemory::new(
                NonNull::from(memory.as_mut()).cast(),
                size_of::<[u32; MEMORY_WORDS]>(),
            )
        };
        // SAFETY: The tables describe registers of the modem domain.
        let created = unsafe {
            regdma::create(Module::ClockModem, &[CLOCK_MODEM], &mut links).and_then(|()| {
                regdma::create(Module::PhyFe, &[PHY_FE], &mut links).inspect_err(|_| {
                    regdma::destroy(Module::ClockModem);
                })
            })
        };
        if created.is_err() {
            return false;
        }
        regdma::attach(Module::ClockModem);
        regdma::attach(Module::PhyFe);
        self.memory = Some(memory);
        true
    }

    /// Lets light sleep power the modem domain down, if [`Self::enable`] built the nodes.
    pub(super) fn allow_power_down(&mut self, allowed: bool) {
        regdma::set_modem_power_down(allowed && self.memory.is_some());
    }

    /// Releases what [`Self::enable`] built.
    pub(super) fn release(&mut self) {
        if self.memory.is_none() {
            return;
        }
        regdma::set_modem_power_down(false);
        regdma::destroy(Module::ClockModem);
        regdma::destroy(Module::PhyFe);
        self.memory = None;
    }
}
