//! USB OTG device driver for embassy-usb (Synopsys DWC).

use embassy_usb_driver::{EndpointAddress, EndpointAllocError, EndpointType, Event, Unsupported};
pub use embassy_usb_synopsys_otg::Config;
use embassy_usb_synopsys_otg::{
    Bus as OtgBus,
    ControlPipe,
    Driver as OtgDriver,
    Endpoint,
    In,
    OtgInstance,
    Out,
    otg_v1::Otg,
};

use crate::usb::otg::Usb;

/// Asynchronous USB driver.
pub struct Driver<'d> {
    inner: OtgDriver<'d>,
    _usb: Usb<'d>,
}

impl<'d> Driver<'d> {
    /// Initializes USB OTG for asynchronous device operation.
    ///
    /// # Arguments
    ///
    /// * `ep_out_buffer` - Buffer for received OUT packets. Must be large enough for all OUT
    ///   endpoint max packet sizes.
    pub fn new(peri: Usb<'d>, ep_out_buffer: &'d mut [u8], config: Config) -> Self {
        let info = peri.info();
        let state = peri.embassy_device_state();
        let instance = OtgInstance {
            regs: unsafe { Otg::from_ptr(info.register_ptr.cast_mut()) },
            state,
            fifo_depth_words: info.fifo_depth_words as u16,
            extra_rx_fifo_words: info.rx_fifo_extra_words,
            endpoint_count: state.endpoint_count(),
            phy_type: info.phy_type,
            calculate_trdt_fn: |_| 5,
        };
        Self {
            inner: OtgDriver::new(ep_out_buffer, instance, config),
            _usb: peri,
        }
    }
}

impl<'d> embassy_usb_driver::Driver<'d> for Driver<'d> {
    type EndpointOut = Endpoint<'d, Out>;
    type EndpointIn = Endpoint<'d, In>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.inner
            .alloc_endpoint_in(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.inner
            .alloc_endpoint_out(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let (bus, cp) = self.inner.start(control_max_packet_size);

        let mut bus = Bus {
            inner: bus,
            _usb: self._usb,
        };

        bus.init();

        (bus, cp)
    }
}

/// Asynchronous USB bus used internally by embassy-usb.
pub struct Bus<'d> {
    inner: OtgBus<'d>,
    _usb: Usb<'d>,
}

impl<'d> Bus<'d> {
    fn init(&mut self) {
        let i = self._usb.info();
        (i.enable_device_mode)();

        let r = unsafe { Otg::from_ptr(i.register_ptr.cast_mut()) };

        self.inner.core_soft_reset();

        self.inner.configure_as_device();
        self.inner.config_v5();

        r.pcgcctl().write(|w| w.0 = 0);

        self._usb.bind_device_interrupt();
    }

    fn disable(&mut self) {
        self._usb.disable_device_interrupt();
        (self._usb.info().platform_bus_disable_on_drop)();
    }
}

impl<'d> embassy_usb_driver::Bus for Bus<'d> {
    async fn poll(&mut self) -> Event {
        self.inner.poll().await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        self.inner.endpoint_set_stalled(ep_addr, stalled)
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        self.inner.endpoint_is_stalled(ep_addr)
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        self.inner.endpoint_set_enabled(ep_addr, enabled)
    }

    async fn enable(&mut self) {
        self.inner.enable().await
    }

    async fn disable(&mut self) {
        self.inner.disable().await
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        self.inner.remote_wakeup().await
    }
}

impl<'d> Drop for Bus<'d> {
    fn drop(&mut self) {
        Bus::disable(self);
    }
}
