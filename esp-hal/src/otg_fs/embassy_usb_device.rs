//! USB OTG device driver for embassy-usb.

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
    PhyType,
    State,
    on_interrupt,
    otg_v1::Otg,
};
use procmacros::handler;

use crate::{interrupt::Priority, otg_fs::Usb};

static DEVICE_STATE: State<{ Usb::MAX_EP_COUNT }> = State::new();

/// Asynchronous USB driver.
pub struct Driver<'d> {
    inner: OtgDriver<'d, { Usb::MAX_EP_COUNT }>,
    _usb: Usb<'d>,
}

impl<'d> Driver<'d> {
    const REGISTERS: Otg = unsafe { Otg::from_ptr(Usb::REGISTERS.cast_mut()) };

    /// Initializes USB OTG peripheral with internal Full-Speed PHY, for
    /// asynchronous operation.
    ///
    /// # Arguments
    ///
    /// * `ep_out_buffer` - An internal buffer used to temporarily store received packets.
    ///
    /// Must be large enough to fit all OUT endpoint max packet sizes.
    /// Endpoint allocation will fail if it is too small.
    pub fn new(peri: Usb<'d>, ep_out_buffer: &'d mut [u8], config: Config) -> Self {
        // From `synopsys-usb-otg` crate:
        // This calculation doesn't correspond to one in a Reference Manual.
        // In fact, the required number of words is higher than indicated in RM.
        // The following numbers are pessimistic and were figured out empirically.
        const RX_FIFO_EXTRA_SIZE_WORDS: u16 = 30;

        let instance = OtgInstance {
            regs: Self::REGISTERS,
            state: &DEVICE_STATE,
            fifo_depth_words: Usb::FIFO_DEPTH_WORDS as u16,
            extra_rx_fifo_words: RX_FIFO_EXTRA_SIZE_WORDS,
            endpoint_count: Usb::MAX_EP_COUNT,
            phy_type: PhyType::InternalFullSpeed,
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

/// Asynchronous USB bus mainly used internally by `embassy-usb`.
// We need a custom wrapper implementation to handle custom initialization.
pub struct Bus<'d> {
    inner: OtgBus<'d, { Usb::MAX_EP_COUNT }>,
    _usb: Usb<'d>,
}

impl Bus<'_> {
    fn init(&mut self) {
        Usb::_enable();

        let r = Driver::REGISTERS;

        // Wait for AHB ready.
        while !r.grstctl().read().ahbidl() {}

        // Configure as device.
        r.gusbcfg().modify(|w| {
            // Force device mode
            w.set_fdmod(true);
            w.set_srpcap(false);
        });

        // Perform core soft-reset
        while !r.grstctl().read().ahbidl() {}
        r.grstctl().modify(|w| w.set_csrst(true));
        while r.grstctl().read().csrst() {}

        self.inner.config_v1();

        // Enable PHY clock
        r.pcgcctl().write(|w| w.0 = 0);

        self._usb._usb0.bind_peri_interrupt(interrupt_handler);
    }

    fn disable(&mut self) {
        self._usb._usb0.disable_peri_interrupt_on_all_cores();

        Usb::_disable();
    }
}

impl embassy_usb_driver::Bus for Bus<'_> {
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

impl Drop for Bus<'_> {
    fn drop(&mut self) {
        Bus::disable(self);
    }
}

#[handler(priority = Priority::max())]
fn interrupt_handler() {
    unsafe { on_interrupt(Driver::REGISTERS, &DEVICE_STATE, Usb::MAX_EP_COUNT) }
}
