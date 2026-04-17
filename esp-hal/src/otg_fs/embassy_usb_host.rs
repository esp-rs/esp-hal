//! USB OTG host driver for embassy-usb-host.
use embassy_usb_driver::{
    EndpointInfo,
    host::{
        DeviceEvent,
        HostError,
        UsbHostDriver,
        channel::{Direction, Type},
    },
};
use embassy_usb_synopsys_otg::{
    PhyType,
    host::{HostState, OtgHost as OtgHostDriver, OtgHostInstance, on_host_interrupt},
    otg_v1::Otg,
};

use crate::{handler, interrupt::Priority, otg_fs::Usb};

#[handler(priority = Priority::max())]
fn interrupt_handler() {
    unsafe {
        on_host_interrupt(
            Driver::REGISTERS,
            &HOST_STATE,
            Usb::MAX_EP_COUNT.min(Usb::MAX_HOST_CH_COUNT),
        )
    }
}

/// UsbHost
pub struct Driver<'d> {
    inner: OtgHostDriver<'d, { Usb::MAX_HOST_CH_COUNT }>,
    _usb: Usb<'d>,
}

static HOST_STATE: HostState<{ Usb::MAX_HOST_CH_COUNT }> = HostState::new();

impl<'d> Driver<'d> {
    const REGISTERS: Otg = unsafe { Otg::from_ptr(Usb::REGISTERS.cast_mut()) };

    /// Creates a new Driver for embassy-usb-host.
    pub fn new(peri: Usb<'d>) -> Self {
        let instance = OtgHostInstance {
            regs: Self::REGISTERS,
            state: &HOST_STATE,
            fifo_depth_words: Usb::FIFO_DEPTH_WORDS as u16,
            channel_count: Usb::MAX_EP_COUNT.min(Usb::MAX_HOST_CH_COUNT),
            phy_type: PhyType::InternalFullSpeed,
        };

        Usb::_enable_host();
        peri._usb0.bind_peri_interrupt(interrupt_handler);

        Self {
            inner: OtgHostDriver::new(instance),
            _usb: peri,
        }
    }
}

impl<'d> UsbHostDriver for Driver<'d> {
    type Channel<Ty: Type, D: Direction> =
        <OtgHostDriver<'d, { Usb::MAX_HOST_CH_COUNT }> as UsbHostDriver>::Channel<Ty, D>;

    async fn wait_for_device_event(&self) -> DeviceEvent {
        self.inner.wait_for_device_event().await
    }

    async fn bus_reset(&self) {
        self.inner.bus_reset().await
    }

    fn alloc_channel<Ty: Type, D: Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        pre: bool,
    ) -> Result<Self::Channel<Ty, D>, HostError> {
        self.inner.alloc_channel(addr, endpoint, pre)
    }
}

impl<'d> Drop for Driver<'d> {
    fn drop(&mut self) {
        self._usb._usb0.disable_peri_interrupt_on_all_cores();

        Usb::_disable();
    }
}
