use esp_sync::NonReentrantMutex;

#[cfg_attr(esp32, path = "clocks_ll/esp32.rs")]
#[cfg_attr(esp32c2, path = "clocks_ll/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "clocks_ll/esp32c3.rs")]
#[cfg_attr(esp32c5, path = "clocks_ll/esp32c5.rs")]
#[cfg_attr(esp32c6, path = "clocks_ll/esp32c6.rs")]
#[cfg_attr(esp32c61, path = "clocks_ll/esp32c61.rs")]
#[cfg_attr(esp32h2, path = "clocks_ll/esp32h2.rs")]
#[cfg_attr(esp32s2, path = "clocks_ll/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "clocks_ll/esp32s3.rs")]
#[cfg_attr(esp32s31, path = "clocks_ll/esp32s31.rs")]
#[allow(unused)]
pub(crate) mod clocks_ll;

#[inline]
/// Do any common initialization needed for the radio clocks
pub(crate) fn init_radio_clocks() {
    clocks_ll::init_clocks();
}

/// Undo the clock initialization done by [`init_radio_clocks`], gating the
/// modem clocks again (mirroring ESP-IDF's per-module clock disable).
#[inline]
pub(crate) fn deinit_radio_clocks() {
    clocks_ll::deinit_clocks();
}

// TODO: radio clocks should be modelled to some extent

struct Refcount {
    count: NonReentrantMutex<usize>,
}

impl Refcount {
    const fn new() -> Self {
        Self {
            count: NonReentrantMutex::new(0),
        }
    }

    fn update(&self, en: bool, f: fn(bool)) {
        self.count.with(|refcount| {
            let count = *refcount;

            let run = if en {
                *refcount = unwrap!(count.checked_add(1), "Clock ref count overflowed.");
                count == 0
            } else {
                // Asymmetric because currently we may disable clocks more times than we enable them
                // (e.g. radio deinit turns off BLE even if it wasn't enabled.).
                *refcount = count.saturating_sub(1);
                count == 1
            };

            if run {
                f(en);
            }
        });
    }
}

#[cfg(feature = "wifi")]
pub(crate) fn enable_wifi(en: bool) {
    static REFCOUNT: Refcount = Refcount::new();
    REFCOUNT.update(en, clocks_ll::enable_wifi);
}

#[cfg(feature = "ble")]
pub(crate) fn enable_bt(en: bool) {
    static REFCOUNT: Refcount = Refcount::new();
    REFCOUNT.update(en, clocks_ll::enable_bt);
}

#[cfg(feature = "ieee802154")]
pub(crate) fn enable_ieee802154(en: bool) {
    static REFCOUNT: Refcount = Refcount::new();
    REFCOUNT.update(en, clocks_ll::enable_ieee802154);
}
