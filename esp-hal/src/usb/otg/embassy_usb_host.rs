//! USB OTG host driver for embassy-usb-host.

use embassy_usb_driver::host::{DeviceEvent, UsbHostController};
use embassy_usb_synopsys_otg::{
    host::{OtgHost as OtgHostDriver, OtgHostInstance},
    otg_v1::Otg,
};

use crate::usb::otg::Usb;

/// Asynchronous USB host controller.
pub struct Driver<'d> {
    inner: OtgHostDriver<'d>,
    _usb: Usb<'d>,
}

impl<'d> Driver<'d> {
    /// Creates a new host controller driver for embassy-usb-host.
    pub fn new(peri: Usb<'d>) -> Self {
        let i = peri.info();
        let instance = OtgHostInstance {
            regs: unsafe { Otg::from_ptr(i.register_ptr.cast_mut()) },
            state: peri.embassy_host_state(),
            fifo_depth_words: i.fifo_depth_words as u16,
            phy_type: i.phy_type,
        };

        (i.enable_host_mode)();
        peri.bind_host_interrupt();

        Self {
            inner: OtgHostDriver::new(instance),
            _usb: peri,
        }
    }
}

impl<'d> UsbHostController<'d> for Driver<'d> {
    type Allocator = <OtgHostDriver<'d> as UsbHostController<'d>>::Allocator;

    fn allocator(&self) -> Self::Allocator {
        self.inner.allocator()
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        self.inner.wait_for_device_event().await
    }

    async fn bus_reset(&mut self) {
        self.inner.bus_reset().await
    }
}

impl<'d> Drop for Driver<'d> {
    fn drop(&mut self) {
        self._usb.disable_host_interrupt();
        (self._usb.info().platform_bus_disable_on_drop)();
    }
}
