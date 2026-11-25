//! USB Serial/JTAG handling across light sleep.
//!
//! A light sleep that stops USB Serial/JTAG leaves the D+ pull-up on. The host keeps the device
//! enumerated, and the link does not recover after the wake. Such a sleep disconnects USB
//! Serial/JTAG first, and reconnects it on wake.
//!
//! A host that attaches or resumes the bus during such sleeps enumerates the device only while the
//! chip stays awake for longer than the 100 ms attach debounce. A kept pull-up does not help,
//! because the host then finds a device that does not respond.

use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use super::{RtcSleepConfig, SleepKind};
use crate::{
    peripherals::USB_DEVICE,
    soc::usb_phy::{self, PhyPads},
    system::{Peripheral, PeripheralClockControl},
};

/// Returns whether USB Serial/JTAG is clocked and out of reset.
fn module_is_enabled() -> bool {
    cfg_select! {
        esp32p4 => {
            let aon = crate::peripherals::LP_AON_CLKRST::regs();
            crate::peripherals::HP_SYS_CLKRST::regs()
                .soc_clk_ctrl2()
                .read()
                .usb_device_apb_clk_en()
                .bit_is_set()
                && aon
                    .lp_aonclkrst_hp_usb_clkrst_ctrl0()
                    .read()
                    .lp_aonclkrst_usb_device_48m_clk_en()
                    .bit_is_set()
                && aon
                    .lp_aonclkrst_hp_usb_clkrst_ctrl1()
                    .read()
                    .lp_aonclkrst_rst_en_usb_device()
                    .bit_is_clear()
        }
        esp32s31 => {
            let cnnt = crate::peripherals::CNNT_SYS::regs()
                .sys_hp_usb_device_ctrl()
                .read();
            crate::peripherals::HP_SYS_CLKRST::regs()
                .usb_device_ctrl0()
                .read()
                .apb_clk_en()
                .bit_is_set()
                && cnnt.sys_usb_device_48m_clk_en().bit_is_set()
                && cnnt.sys_usb_device_rst_en().bit_is_clear()
        }
        esp32c3 => {
            let system = crate::peripherals::SYSTEM::regs();
            system
                .perip_clk_en0()
                .read()
                .usb_device_clk_en()
                .bit_is_set()
                && system
                    .perip_rst_en0()
                    .read()
                    .usb_device_rst()
                    .bit_is_clear()
        }
        esp32s3 => {
            let system = crate::peripherals::SYSTEM::regs();
            system
                .perip_clk_en1()
                .read()
                .usb_device_clk_en()
                .bit_is_set()
                && system
                    .perip_rst_en1()
                    .read()
                    .usb_device_rst()
                    .bit_is_clear()
        }
        _ => {
            let conf = crate::peripherals::PCR::regs().usb_device_conf().read();
            conf.usb_device_clk_en().bit_is_set() && conf.usb_device_rst_en().bit_is_clear()
        }
    }
}

fn pad_is_enabled() -> bool {
    USB_DEVICE::regs()
        .conf0()
        .read()
        .usb_pad_enable()
        .bit_is_set()
}

/// Returns whether the coming light sleep keeps USB Serial/JTAG running.
///
/// A light sleep keeps it running while a USB host is connected, unless the
/// `keep-usb-serial-jtag-connected` option is disabled.
#[cfg(sleep_keeps_usb_serial_jtag)]
pub(super) fn keep_in_light_sleep() -> bool {
    cfg!(keep_usb_serial_jtag_connected) && host_connected()
}

/// The GPIOs of the D- and D+ pads that USB Serial/JTAG drives, if they are known.
///
/// The `USB_EXCHG_PINS` eFuse swaps the two lines, so the sleep handles the pull-ups of both pads.
fn pads() -> Option<[u8; 2]> {
    usb_phy::usj_pads().map(PhyPads::gpios)
}

fn set_bus_clock_enabled(enabled: bool) {
    PeripheralClockControl::set_enabled_untracked(Peripheral::UsbDevice, enabled);
}

fn set_pad_enabled(enabled: bool) {
    USB_DEVICE::regs()
        .conf0()
        .modify(|_, w| w.usb_pad_enable().bit(enabled));
}

/// The USB Serial/JTAG state that [`disconnect`] changes.
pub(super) struct Disconnected {
    module_was_enabled: bool,
    conf0: u32,
    /// The pull-ups of [`pads`], if USB Serial/JTAG had the pads.
    pad_pullups: Option<[bool; 2]>,
}

/// Disconnects USB Serial/JTAG from the host if the coming sleep stops it.
///
/// A deep sleep powers USB Serial/JTAG down on its own.
#[inline(always)]
pub(super) fn disconnect_for_sleep(
    config: &RtcSleepConfig,
    kind: SleepKind,
) -> Option<Disconnected> {
    let light = kind == SleepKind::Light;
    cfg_select! {
        sleep_keeps_usb_serial_jtag => (light && !config.keep_usb_serial_jtag).then(disconnect),
        _ => {
            let _ = config;
            light.then(disconnect)
        }
    }
}

/// Disconnects USB Serial/JTAG from the host on light-sleep entry.
///
/// See <https://github.com/espressif/esp-idf/blob/4dbfecac7e/components/esp_hw_support/sleep_console.c#L17-L36>.
fn disconnect() -> Disconnected {
    let module_was_enabled = module_is_enabled();
    if !module_was_enabled {
        // The pad enable needs the register block clocked and out of reset.
        set_bus_clock_enabled(true);
        PeripheralClockControl::reset(Peripheral::UsbDevice);
        // The register reset enables the pad, but the pad was off before the sleep.
        set_pad_enabled(false);
    }
    let pad_enabled = pad_is_enabled();
    // The host sees a disconnect only if neither line is pulled up. Clearing `usb_pad_enable`
    // keeps the pull-ups of USB Serial/JTAG and of the pads, and the pull-up of the D+ pad is
    // enabled after reset.
    let conf0 = USB_DEVICE::regs().conf0();
    let saved_conf0 = conf0.read().bits();
    conf0.modify(|_, w| {
        w.pad_pull_override().set_bit();
        w.dp_pullup().clear_bit();
        w.dm_pullup().clear_bit()
    });
    let pad_pullups = pad_enabled.then(pads).flatten().map(|pads| {
        pads.map(|pad| {
            let reg = crate::gpio::io_mux_reg(pad);
            let pullup = reg.read().fun_wpu().bit();
            reg.modify(|_, w| w.fun_wpu().clear_bit());
            pullup
        })
    });
    set_pad_enabled(false);
    set_bus_clock_enabled(false);

    Disconnected {
        module_was_enabled,
        conf0: saved_conf0,
        pad_pullups,
    }
}

impl Disconnected {
    /// Reconnects USB Serial/JTAG to the host on wake.
    ///
    /// See <https://github.com/espressif/esp-idf/blob/4dbfecac7e/components/esp_hw_support/sleep_console.c#L38-L50>.
    pub(super) fn reconnect(self) {
        set_bus_clock_enabled(true);
        USB_DEVICE::regs()
            .conf0()
            .write(|w| unsafe { w.bits(self.conf0) });
        if let (Some(pullups), Some(pads)) = (self.pad_pullups, pads()) {
            for (pad, pullup) in pads.into_iter().zip(pullups) {
                crate::gpio::io_mux_reg(pad).modify(|_, w| w.fun_wpu().bit(pullup));
            }
        }
        if !self.module_was_enabled {
            set_bus_clock_enabled(false);
        }
    }
}

/// Returns whether a USB host talks to USB Serial/JTAG.
///
/// A host that has not suspended the bus sends a start-of-frame packet every millisecond, and each
/// packet advances the frame index. A change in the last few milliseconds counts, because two
/// calls within a millisecond see the same index. The SOF interrupt flag stays untouched, because
/// esp-println reads it to pick its output.
///
/// The 11-bit index wraps every 2048 ms, so a call a multiple of that after the previous one sees
/// the same index. After a call that found a host, an unchanged index is read again one frame
/// later. A call with no earlier index, because USB Serial/JTAG was off at [`init`], misses a host
/// for one sleep.
pub(super) fn host_connected() -> bool {
    const TOLERANCE_US: u32 = 3_000;
    // Longer than the 1 ms between two start-of-frame packets.
    const RESAMPLE_GAP_US: u32 = 1_100;

    if !module_is_enabled() || !pad_is_enabled() {
        return false;
    }

    let changed_recently =
        || now_us().wrapping_sub(LAST_CHANGE_US.load(Ordering::Relaxed)) < TOLERANCE_US;

    let frame = frame_index();
    let last_frame = LAST_FRAME.swap(frame, Ordering::Relaxed);
    if last_frame == NO_FRAME {
        return false;
    }

    if frame != last_frame {
        LAST_CHANGE_US.store(now_us(), Ordering::Relaxed);
    } else if WAS_CONNECTED.load(Ordering::Relaxed) && !changed_recently() {
        crate::rom::ets_delay_us(RESAMPLE_GAP_US);
        let frame = frame_index();
        if frame != last_frame {
            LAST_FRAME.store(frame, Ordering::Relaxed);
            LAST_CHANGE_US.store(now_us(), Ordering::Relaxed);
        }
    }

    let connected = changed_recently();
    WAS_CONNECTED.store(connected, Ordering::Relaxed);
    connected
}

/// Reads the first frame index, so that the first [`host_connected`] call needs no wait.
///
/// Runs before the time driver starts, so it stores no timestamp.
pub(super) fn init() {
    if module_is_enabled() && pad_is_enabled() {
        LAST_FRAME.store(frame_index(), Ordering::Relaxed);
    }
}

const NO_FRAME: u32 = u32::MAX;
static LAST_FRAME: AtomicU32 = AtomicU32::new(NO_FRAME);
static LAST_CHANGE_US: AtomicU32 = AtomicU32::new(0);
static WAS_CONNECTED: AtomicBool = AtomicBool::new(false);

fn frame_index() -> u32 {
    u32::from(
        USB_DEVICE::regs()
            .fram_num()
            .read()
            .sof_frame_index()
            .bits(),
    )
}

fn now_us() -> u32 {
    crate::time::Instant::now()
        .duration_since_epoch()
        .as_micros() as u32
}
