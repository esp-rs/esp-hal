//! Powers the modem domain down in light sleep, when every radio in it can lose its state.
//!
//! Each radio that is up reports whether it can retain its MAC and baseband. The retention method
//! of the chip decides how the domain loses power, and how the PHY keeps its state.

use enumset::{EnumSet, EnumSetType};
use esp_sync::NonReentrantMutex;

#[cfg(modem_retention = "backup_dma")]
pub(crate) use self::method::{sleep, wake};

/// The retention method of the chip.
#[cfg_attr(modem_retention = "backup_dma", path = "backup_dma.rs")]
mod method;

/// A radio in the modem domain.
#[derive(EnumSetType)]
pub(crate) enum Radio {
    #[cfg(feature = "ble")]
    Ble,
    #[cfg(feature = "wifi")]
    Wifi,
}

struct State {
    /// The radios that are up.
    up: EnumSet<Radio>,
    /// The radios that can power the modem domain down.
    power_down: EnumSet<Radio>,
    method: method::State,
}

impl State {
    /// Returns whether every radio that is up can power the modem domain down.
    fn power_down_allowed(&self) -> bool {
        self.up.is_subset(self.power_down)
    }
}

static STATE: NonReentrantMutex<State> = NonReentrantMutex::new(State {
    up: EnumSet::empty(),
    power_down: EnumSet::empty(),
    method: method::State::new(),
});

/// Records that `radio` is up.
///
/// The radio keeps the modem domain powered, unless [`enable_power_down`] succeeds for it.
pub(crate) fn start(radio: Radio) {
    STATE.with(|state| state.up.insert(radio));
}

/// Lets `radio` power the modem domain down.
///
/// Returns whether the radio can. Call this after [`start`], once the radio can retain its MAC and
/// baseband.
pub(crate) fn enable_power_down(radio: Radio) -> bool {
    let enabled = STATE.with(|state| {
        let enabled = method::enable(&mut state.method);
        if enabled {
            state.power_down.insert(radio);
        }
        enabled
    });

    if !enabled {
        warn!("No memory to retain the PHY state, the modem stays powered in sleep");
    }
    enabled
}

/// Records that `radio` is down.
///
/// Call this while the radio does not sleep.
pub(crate) fn stop(radio: Radio) {
    STATE.with(|state| {
        state.up.remove(radio);
        state.power_down.remove(radio);
        if state.power_down.is_empty() {
            method::release(&mut state.method);
        }
    });
}
