#![cfg_attr(docsrs, procmacros::doc_replace(
    "dm_pin" => {
        cfg(any(esp32s2, esp32s3)) => "GPIO19",
        cfg(esp32p4) => "GPIO26",
    },
    "dp_pin" => {
        cfg(any(esp32s2, esp32s3)) => "GPIO20",
        cfg(esp32p4) => "GPIO27",
    },
))]
//! USB On-The-Go (Synopsys DWC) — full-speed (`USB_FS`) and high-speed (`USB_HS`).
//!
//! `embassy-usb` / `embassy-usb-host` integration: [`embassy_usb_device`], [`embassy_usb_host`].
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::usb::otg::{Usb, embassy_usb_device};
//!
//! let usb = Usb::new_fs(
//!     peripherals.USB_FS,
//!     peripherals.__dp_pin__,
//!     peripherals.__dm_pin__,
//! );
//! # {after_snippet}
//! ```

use embassy_usb_synopsys_otg::{
    PhyType,
    State,
    StateStorage,
    host::{HostState, HostStateStorage, on_host_interrupt},
    on_interrupt,
    otg_v1::Otg,
};
use procmacros::handler;

#[cfg(usb_otg_driver_supported)]
use crate::peripherals::USB_FS;
#[cfg(usb_otg_hs_driver_supported)]
use crate::peripherals::USB_HS;
use crate::{
    gpio::{InputSignal, Level},
    interrupt::{InterruptHandler, Priority},
    peripherals,
    system::{Peripheral, PeripheralGuard},
};

pub(crate) enum AnyUsb<'d> {
    #[cfg(usb_otg_driver_supported)]
    Fs(USB_FS<'d>),
    #[cfg(usb_otg_hs_driver_supported)]
    Hs(USB_HS<'d>),
}

impl AnyUsb<'_> {
    delegate::delegate! {
        to match self {
            #[cfg(usb_otg_driver_supported)]
            AnyUsb::Fs(peri) => peri,
            #[cfg(usb_otg_hs_driver_supported)]
            AnyUsb::Hs(peri) => peri,
        } {
            fn info(&self) -> &'static UsbOtgInfo;
            fn bind_peri_interrupt(&self, handler: InterruptHandler);
            fn disable_peri_interrupt_on_all_cores(&self);
            fn embassy_device_state(&self) -> State<'static>;
            fn embassy_host_state(&self) -> HostState<'static>;
        }
    }
}

#[cfg(usb_otg_driver_supported)]
pub mod fs_pins;
#[cfg(usb_otg_driver_supported)]
pub use fs_pins::{UsbFsDm, UsbFsDp};

pub mod embassy_usb_device;
pub mod embassy_usb_host;

/// Static description of one USB OTG controller instance (register base, PHY, IRQ metadata).
#[doc(hidden)]
pub struct UsbOtgInfo {
    pub(crate) register_ptr: *const (),
    pub(crate) peripheral: Peripheral,
    pub(crate) fifo_depth_words: usize,
    pub(crate) phy_type: PhyType,
    pub(crate) rx_fifo_extra_words: u16,
    pub(crate) device_interrupt: InterruptHandler,
    pub(crate) host_interrupt: InterruptHandler,
    pub(crate) enable_device_mode: fn(),
    pub(crate) enable_host_mode: fn(),
    pub(crate) platform_bus_disable_on_drop: fn(),
}

unsafe impl Send for UsbOtgInfo {}
unsafe impl Sync for UsbOtgInfo {}

impl PartialEq for UsbOtgInfo {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_ptr, other.register_ptr)
    }
}

#[cfg(usb_otg_driver_supported)]
impl<'d> USB_FS<'d> {
    fn degrade(self) -> AnyUsb<'d> {
        AnyUsb::Fs(self)
    }

    fn device_state() -> State<'static> {
        static DEVICE: StateStorage<7> = StateStorage::new();
        DEVICE.as_state()
    }

    fn host_state() -> HostState<'static> {
        static HOST: HostStateStorage<8> = HostStateStorage::new();
        HOST.as_host_state()
    }

    fn info(&self) -> &'static UsbOtgInfo {
        #[handler(priority = Priority::max())]
        pub(crate) fn otg_fs_device_interrupt() {
            let state = USB_FS::device_state();
            unsafe { on_interrupt(Otg::from_ptr(USB_FS::PTR.cast_mut().cast::<()>()), &state) }
        }

        #[handler(priority = Priority::max())]
        pub(crate) fn otg_fs_host_interrupt() {
            let state = USB_FS::host_state();
            unsafe { on_host_interrupt(Otg::from_ptr(USB_FS::PTR.cast_mut().cast::<()>()), &state) }
        }

        fn common_init() {
            #[cfg(esp32p4)]
            {
                use crate::RegisterToggle;

                peripherals::LP_AON_CLKRST::regs()
                    .lp_aonclkrst_hp_usb_clkrst_ctrl0()
                    .modify(|_, w| w.lp_aonclkrst_usb_otg11_48m_clk_en().set_bit());

                peripherals::LP_AON_CLKRST::regs()
                    .lp_aonclkrst_hp_usb_clkrst_ctrl1()
                    .toggle(|w, en| w.lp_aonclkrst_rst_en_usb_otg11().bit(en));
            }

            #[cfg(esp32s3)]
            peripherals::LPWR::regs().usb_conf().modify(|_, w| {
                w.sw_hw_usb_phy_sel().set_bit();
                w.sw_usb_phy_sel().set_bit()
            });

            peripherals::USB_WRAP::regs().otg_conf().modify(|_, w| {
                w.usb_pad_enable().set_bit();
                w.phy_sel().clear_bit();
                w.clk_en().set_bit();
                w.ahb_clk_force_on().set_bit();
                w.phy_clk_force_on().set_bit();
                w
            });
        }

        fn fs_enable_device_mode() {
            common_init();

            peripherals::USB_WRAP::regs()
                .otg_conf()
                .modify(|_, w| w.pad_pull_override().clear_bit());

            InputSignal::USB_FS_IDDIG.connect_to(&Level::High);
            InputSignal::USB_FS_SRP_BVALID.connect_to(&Level::High);
            InputSignal::USB_FS_VBUSVALID.connect_to(&Level::High);
            InputSignal::USB_FS_AVALID.connect_to(&Level::Low);
        }

        fn fs_enable_host_mode() {
            common_init();

            peripherals::USB_WRAP::regs().otg_conf().modify(|_, w| {
                w.pad_pull_override().set_bit();
                w.dp_pulldown().set_bit();
                w.dm_pulldown().set_bit();
                w.dp_pullup().clear_bit();
                w.dm_pullup().clear_bit()
            });

            InputSignal::USB_FS_SRP_BVALID.connect_to(&Level::Low);
            InputSignal::USB_FS_IDDIG.connect_to(&Level::Low);
            InputSignal::USB_FS_VBUSVALID.connect_to(&Level::High);
            InputSignal::USB_FS_AVALID.connect_to(&Level::High);
        }

        static INFO: UsbOtgInfo = UsbOtgInfo {
            register_ptr: USB_FS::PTR.cast(),
            peripheral: Peripheral::UsbFs,
            fifo_depth_words: property!("usb_otg.fifo_depth_words"),
            phy_type: PhyType::InternalFullSpeed,
            rx_fifo_extra_words: 30,
            device_interrupt: otg_fs_device_interrupt,
            host_interrupt: otg_fs_host_interrupt,
            enable_device_mode: fs_enable_device_mode,
            enable_host_mode: fs_enable_host_mode,
            platform_bus_disable_on_drop: || {},
        };

        &INFO
    }

    fn embassy_device_state(&self) -> State<'static> {
        Self::device_state()
    }

    fn embassy_host_state(&self) -> HostState<'static> {
        Self::host_state()
    }
}

#[cfg(usb_otg_hs_driver_supported)]
impl<'d> USB_HS<'d> {
    fn degrade(self) -> AnyUsb<'d> {
        AnyUsb::Hs(self)
    }

    fn device_state() -> State<'static> {
        static DEVICE: StateStorage<16> = StateStorage::new();
        DEVICE.as_state()
    }

    fn host_state() -> HostState<'static> {
        static HOST: HostStateStorage<16> = HostStateStorage::new();
        HOST.as_host_state()
    }

    fn info(&self) -> &'static UsbOtgInfo {
        #[handler(priority = Priority::max())]
        pub(crate) fn otg_hs_device_interrupt() {
            let state = USB_HS::device_state();
            unsafe { on_interrupt(Otg::from_ptr(USB_HS::PTR.cast_mut().cast::<()>()), &state) }
        }

        #[handler(priority = Priority::max())]
        pub(crate) fn otg_hs_host_interrupt() {
            let state = USB_HS::host_state();
            unsafe { on_host_interrupt(Otg::from_ptr(USB_HS::PTR.cast_mut().cast::<()>()), &state) }
        }

        #[cfg(esp32p4)]
        fn p4_hs_init() {
            use crate::RegisterToggle;

            peripherals::LP_AON_CLKRST::regs()
                .lp_aonclkrst_hp_usb_clkrst_ctrl1()
                .modify(|_, w| w.lp_aonclkrst_usb_otg20_phyref_clk_en().set_bit());

            peripherals::LP_AON_CLKRST::regs()
                .lp_aonclkrst_hp_usb_clkrst_ctrl1()
                .toggle(|w, en| {
                    w.lp_aonclkrst_rst_en_usb_otg20().bit(en);
                    w.lp_aonclkrst_rst_en_usb_otg20_phy().bit(en);
                    w
                });

            peripherals::HP_SYS::regs()
                .usbotg20_ctrl()
                .modify(|_, w| w.otg_suspendm().set_bit());

            const USB_UTMI: usize = 0x5009C000;
            let fc_06 = (USB_UTMI as *mut u32).wrapping_add(6);
            let bits = unsafe { fc_06.read_volatile() };
            unsafe { fc_06.write_volatile(bits | (1 << 3) | (1 << 0)) };
        }

        #[cfg(esp32p4)]
        fn connect_hs_pulldowns(connect: bool) {
            peripherals::LP_SYS::regs()
                .hp_usb_otghs_phy_ctrl()
                .modify(|_, w| {
                    w.hp_utmiotg_dppulldown().bit(connect);
                    w.hp_utmiotg_dmpulldown().bit(connect);
                    w
                });
        }

        fn hs_enable_device_mode() {
            #[cfg(esp32p4)]
            {
                p4_hs_init();
                connect_hs_pulldowns(false);
            }
        }

        fn hs_enable_host_mode() {
            #[cfg(esp32p4)]
            {
                p4_hs_init();
                connect_hs_pulldowns(true);
            }
        }

        static INFO: UsbOtgInfo = UsbOtgInfo {
            register_ptr: USB_HS::PTR.cast(),
            peripheral: Peripheral::UsbHs,
            fifo_depth_words: property!("usb_otg_hs.fifo_depth_words"),
            phy_type: PhyType::InternalHighSpeed,
            rx_fifo_extra_words: 31,
            device_interrupt: otg_hs_device_interrupt,
            host_interrupt: otg_hs_host_interrupt,
            enable_device_mode: hs_enable_device_mode,
            enable_host_mode: hs_enable_host_mode,
            platform_bus_disable_on_drop: || {},
        };

        &INFO
    }

    fn embassy_device_state(&self) -> State<'static> {
        Self::device_state()
    }

    fn embassy_host_state(&self) -> HostState<'static> {
        Self::host_state()
    }
}

/// Type-erased USB OTG handle.
pub struct Usb<'d> {
    inner: AnyUsb<'d>,
    _guard: PeripheralGuard,
}

impl<'d> Usb<'d> {
    #[inline]
    pub(crate) fn info(&self) -> &'static UsbOtgInfo {
        self.inner.info()
    }

    pub(crate) fn bind_device_interrupt(&self) {
        self.inner.bind_peri_interrupt(self.info().device_interrupt);
    }

    pub(crate) fn disable_device_interrupt(&self) {
        self.inner.disable_peri_interrupt_on_all_cores();
    }

    pub(crate) fn bind_host_interrupt(&self) {
        self.inner.bind_peri_interrupt(self.info().host_interrupt);
    }

    pub(crate) fn disable_host_interrupt(&self) {
        self.inner.disable_peri_interrupt_on_all_cores();
    }

    pub(crate) fn embassy_device_state(&self) -> State<'static> {
        self.inner.embassy_device_state()
    }

    pub(crate) fn embassy_host_state(&self) -> HostState<'static> {
        self.inner.embassy_host_state()
    }

    #[cfg(usb_otg_driver_supported)]
    /// Creates a new full-speed USB OTG driver.
    pub fn new_fs(
        usb: USB_FS<'d>,
        usb_dp: impl fs_pins::UsbFsDp + 'd,
        usb_dm: impl fs_pins::UsbFsDm + 'd,
    ) -> Self {
        let guard = PeripheralGuard::new(usb.info().peripheral);
        usb_dp.configure();
        usb_dm.configure();

        Self {
            inner: usb.degrade(),
            _guard: guard,
        }
    }

    #[cfg(usb_otg_hs_driver_supported)]
    /// Creates a new high-speed USB OTG driver.
    pub fn new_hs(usb: USB_HS<'d>) -> Self {
        let guard = PeripheralGuard::new(usb.info().peripheral);

        Self {
            inner: usb.degrade(),
            _guard: guard,
        }
    }
}
