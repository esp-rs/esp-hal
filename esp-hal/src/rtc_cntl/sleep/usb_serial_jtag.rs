//! USB Serial/JTAG handling across light sleep.
//!
//! A chip that can keep USB Serial/JTAG running keeps it running while a USB host is connected.
//! Otherwise, a light sleep leaves the PHY driving the pads, including the D+ pull-up, while the
//! peripheral is stopped. The host keeps the device enumerated, and the link does not recover after
//! the wake. Such a sleep disconnects USB Serial/JTAG before it starts and reconnects it on wake.
//!
//! A host that attaches later, or that resumes a suspended bus, enumerates the device only while
//! the chip stays awake for longer than the 100 ms attach debounce. Keeping the pull-up during
//! such sleeps does not help: a host that resumes the bus then finds a device that does not
//! respond.

use crate::{
    peripherals::USB_DEVICE,
    system::{Peripheral, PeripheralClockControl},
};

/// Returns whether USB Serial/JTAG is clocked and out of reset.
fn module_is_enabled() -> bool {
    cfg_select! {
        esp32p4 => {
            crate::peripherals::HP_SYS_CLKRST::regs()
                .soc_clk_ctrl2()
                .read()
                .usb_device_apb_clk_en()
                .bit_is_set()
                && crate::peripherals::LP_AON_CLKRST::regs()
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

/// Returns whether the coming sleep keeps USB Serial/JTAG running.
///
/// A light sleep keeps it running while a USB host is connected, unless the
/// `keep-usb-serial-jtag-connected` option is disabled.
#[cfg(sleep_usb_serial_jtag_light_sleep = "keep")]
pub(super) fn keep_running_in_sleep(deep: bool) -> bool {
    cfg!(keep_usb_serial_jtag_connected) && !deep && host_connected()
}

/// The USB Serial/JTAG state that [`pad_backup_and_disable`] changes.
pub(super) struct PadBackup {
    clock_was_enabled: bool,
    conf0: u32,
    /// The pull-up of the D+ pad, if USB Serial/JTAG had the pad.
    dp_pad_pullup: Option<bool>,
}

/// The GPIO of the D+ pad.
#[allow(unused_assignments)]
const DP_PAD: u8 = {
    let mut pad = None;
    for_each_analog_function! {
        (USJ_DP, $gpio:ident) => { pad = Some(crate::peripherals::$gpio::NUMBER); };
    }
    // The ESP32-S3 shares the PHY with USB OTG, and the metadata names the pad after USB OTG.
    if pad.is_none() {
        for_each_analog_function! {
            (USB_FS_DP, $gpio:ident) => { pad = Some(crate::peripherals::$gpio::NUMBER); };
        }
    }
    pad.unwrap()
};

fn enable_bus_clock(enable: bool) {
    PeripheralClockControl::set_enabled_untracked(Peripheral::UsbDevice, enable);
}

fn set_pad_enable(enable: bool) {
    USB_DEVICE::regs()
        .conf0()
        .modify(|_, w| w.usb_pad_enable().bit(enable));
}

/// Disconnects USB Serial/JTAG on light-sleep entry.
///
/// See <https://github.com/espressif/esp-idf/blob/4dbfecac7e/components/esp_hw_support/sleep_console.c#L17-L36>.
pub(super) fn pad_backup_and_disable() -> PadBackup {
    let clock_was_enabled = module_is_enabled();
    if !clock_was_enabled {
        // The pad enable needs the register block clocked and out of reset.
        enable_bus_clock(true);
        PeripheralClockControl::reset(Peripheral::UsbDevice);
        // The register reset enables the pad, but the pad was off before the sleep.
        set_pad_enable(false);
    }
    let pad_enabled = pad_is_enabled();
    // Clearing `usb_pad_enable` alone keeps the D+ pull-up, so the host never sees a
    // disconnect and USB Serial/JTAG stops responding after the sleep. The pad pulls D+ up while
    // either the USB Serial/JTAG pull-up or the pull-up of the pad is enabled, and the pull-up
    // of the pad is enabled after reset.
    let conf0 = USB_DEVICE::regs().conf0();
    let saved_conf0 = conf0.read().bits();
    conf0.modify(|_, w| {
        w.pad_pull_override().set_bit();
        w.dp_pullup().clear_bit();
        w.dm_pullup().clear_bit()
    });
    let dp_pad_pullup = pad_enabled.then(|| {
        let dp = crate::gpio::io_mux_reg(DP_PAD);
        let pullup = dp.read().fun_wpu().bit();
        dp.modify(|_, w| w.fun_wpu().clear_bit());
        pullup
    });
    set_pad_enable(false);
    enable_bus_clock(false);

    PadBackup {
        clock_was_enabled,
        conf0: saved_conf0,
        dp_pad_pullup,
    }
}

/// Reconnects USB Serial/JTAG on wake.
///
/// See <https://github.com/espressif/esp-idf/blob/4dbfecac7e/components/esp_hw_support/sleep_console.c#L38-L50>.
pub(super) fn pad_restore(backup: PadBackup) {
    enable_bus_clock(true);
    USB_DEVICE::regs()
        .conf0()
        .write(|w| unsafe { w.bits(backup.conf0) });
    if let Some(pullup) = backup.dp_pad_pullup {
        crate::gpio::io_mux_reg(DP_PAD).modify(|_, w| w.fun_wpu().bit(pullup));
    }
    if !backup.clock_was_enabled {
        enable_bus_clock(false);
    }
}

/// Returns whether a USB host talks to USB Serial/JTAG.
///
/// A connected host that has not suspended the bus sends a start-of-frame packet every
/// millisecond, and each packet advances the frame index. Two calls within that millisecond see
/// the same index, so a change in the last few milliseconds counts. The SOF interrupt flag stays
/// untouched, because esp-println reads it to pick its output.
///
/// The first call after boot, and a call after the microsecond counter wraps, can report a host
/// that is not there. The next sleep then keeps USB Serial/JTAG running once without need.
pub(super) fn host_connected() -> bool {
    use core::sync::atomic::{AtomicU32, Ordering};

    const TOLERANCE_US: u32 = 3_000;
    static LAST_FRAME: AtomicU32 = AtomicU32::new(u32::MAX);
    static LAST_CHANGE_US: AtomicU32 = AtomicU32::new(0);

    if !module_is_enabled() || !pad_is_enabled() {
        return false;
    }

    let now = crate::time::Instant::now()
        .duration_since_epoch()
        .as_micros() as u32;
    let frame = u32::from(
        USB_DEVICE::regs()
            .fram_num()
            .read()
            .sof_frame_index()
            .bits(),
    );
    if LAST_FRAME.load(Ordering::Relaxed) != frame {
        LAST_FRAME.store(frame, Ordering::Relaxed);
        LAST_CHANGE_US.store(now, Ordering::Relaxed);
    }

    now.wrapping_sub(LAST_CHANGE_US.load(Ordering::Relaxed)) < TOLERANCE_US
}
