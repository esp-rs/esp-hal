//! # USB OTG full-speed peripheral
//!
//! ## Overview
//! The USB OTG Full-speed peripheral driver provides support for the USB
//! On-The-Go (OTG) full-speed functionality on ESP chips, allows communication
//! with USB devices using either blocking (usb-device) or asynchronous
//! (embassy-usb) APIs.
//!
//! The blocking driver uses the `esp_synopsys_usb_otg` crate, which provides
//! the `USB bus` implementation and `USB peripheral traits`.
//!
//! The asynchronous driver uses the `embassy_usb_synopsys_otg` crate, which
//! provides the `USB bus` and `USB device` implementations.
//!
//! The module also relies on other peripheral modules, such as `GPIO`,
//! `system`, and `clock control`, to configure and enable the `USB` peripheral.
//!
//! To use the USB OTG Full-speed peripheral driver, you need to initialize the
//! peripheral and configure its settings. The [`Usb`] struct represents the USB
//! peripheral and requires the GPIO pins that implement [`UsbDp`], and
//! [`UsbDm`], which define the specific types used for USB pin selection.
//!
//! The returned `Usb` instance can be used with the `usb-device` crate, or it
//! can be further configured with [`asynch::Driver`] to be used with the
//! `embassy-usb` crate.

pub use esp_synopsys_usb_otg::UsbBus;
use esp_synopsys_usb_otg::UsbPeripheral;

use crate::{
    gpio::InputSignal,
    peripheral::{Peripheral, PeripheralRef},
    peripherals,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

#[doc(hidden)]
pub trait UsbDp: crate::private::Sealed {}

#[doc(hidden)]
pub trait UsbDm: crate::private::Sealed {}

pub struct Usb<'d> {
    _usb0: PeripheralRef<'d, peripherals::USB0>,
}

impl<'d> Usb<'d> {
    pub fn new<P, M>(
        usb0: impl Peripheral<P = peripherals::USB0> + 'd,
        _usb_dp: impl Peripheral<P = P> + 'd,
        _usb_dm: impl Peripheral<P = M> + 'd,
    ) -> Self
    where
        P: UsbDp + Send + Sync,
        M: UsbDm + Send + Sync,
    {
        PeripheralClockControl::enable(PeripheralEnable::Usb);

        Self {
            _usb0: usb0.into_ref(),
        }
    }

    fn _enable() {
        unsafe {
            let usb_wrap = &*peripherals::USB_WRAP::PTR;
            usb_wrap.otg_conf().modify(|_, w| {
                w.usb_pad_enable()
                    .set_bit()
                    .phy_sel()
                    .clear_bit()
                    .clk_en()
                    .set_bit()
                    .ahb_clk_force_on()
                    .set_bit()
                    .phy_clk_force_on()
                    .set_bit()
            });

            #[cfg(esp32s3)]
            {
                let rtc = &*peripherals::LPWR::PTR;
                rtc.usb_conf()
                    .modify(|_, w| w.sw_hw_usb_phy_sel().set_bit().sw_usb_phy_sel().set_bit());
            }

            crate::gpio::connect_high_to_peripheral(InputSignal::USB_OTG_IDDIG); // connected connector is mini-B side
            crate::gpio::connect_high_to_peripheral(InputSignal::USB_SRP_BVALID); // HIGH to force USB device mode
            crate::gpio::connect_high_to_peripheral(InputSignal::USB_OTG_VBUSVALID); // receiving a valid Vbus from device
            crate::gpio::connect_low_to_peripheral(InputSignal::USB_OTG_AVALID);
        }
    }

    fn _disable() {
        // TODO
    }
}

unsafe impl<'d> Sync for Usb<'d> {}

unsafe impl<'d> UsbPeripheral for Usb<'d> {
    const REGISTERS: *const () = peripherals::USB0::ptr() as *const ();

    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 256;
    const ENDPOINT_COUNT: usize = 5;

    fn enable() {
        Self::_enable();
    }

    fn ahb_frequency_hz(&self) -> u32 {
        // unused
        80_000_000
    }
}

#[cfg(feature = "async")]
pub mod asynch {
    use embassy_usb_driver::{
        EndpointAddress,
        EndpointAllocError,
        EndpointType,
        Event,
        Unsupported,
    };
    pub use embassy_usb_synopsys_otg::Config;
    use embassy_usb_synopsys_otg::{
        on_interrupt,
        otg_v1::Otg,
        Bus as OtgBus,
        ControlPipe,
        Driver as OtgDriver,
        Endpoint,
        In,
        OtgInstance,
        Out,
        PhyType,
        State,
    };
    use procmacros::handler;

    use super::*;
    use crate::Cpu;

    // From ESP32-S3 TRM:
    // Six additional endpoints (endpoint numbers 1 to 6), configurable as IN or OUT
    const MAX_EP_COUNT: usize = 7;

    static STATE: State<MAX_EP_COUNT> = State::new();

    /// Asynchronous USB driver.
    pub struct Driver<'d> {
        inner: OtgDriver<'d, MAX_EP_COUNT>,
    }

    impl<'d> Driver<'d> {
        const REGISTERS: Otg = unsafe { Otg::from_ptr(Usb::REGISTERS.cast_mut()) };

        /// Initializes USB OTG peripheral with internal Full-Speed PHY, for
        /// asynchronous operation.
        ///
        /// # Arguments
        ///
        /// * `ep_out_buffer` - An internal buffer used to temporarily store
        ///   received packets.
        /// Must be large enough to fit all OUT endpoint max packet sizes.
        /// Endpoint allocation will fail if it is too small.
        pub fn new(_peri: Usb<'d>, ep_out_buffer: &'d mut [u8], config: Config) -> Self {
            // From `synopsys-usb-otg` crate:
            // This calculation doesn't correspond to one in a Reference Manual.
            // In fact, the required number of words is higher than indicated in RM.
            // The following numbers are pessimistic and were figured out empirically.
            const RX_FIFO_EXTRA_SIZE_WORDS: u16 = 30;

            let instance = OtgInstance {
                regs: Self::REGISTERS,
                state: &STATE,
                fifo_depth_words: Usb::FIFO_DEPTH_WORDS as u16,
                extra_rx_fifo_words: RX_FIFO_EXTRA_SIZE_WORDS,
                endpoint_count: Usb::ENDPOINT_COUNT,
                phy_type: PhyType::InternalFullSpeed,
                quirk_setup_late_cnak: quirk_setup_late_cnak(),
                calculate_trdt_fn: |_| 5,
            };
            Self {
                inner: OtgDriver::new(ep_out_buffer, instance, config),
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
            max_packet_size: u16,
            interval_ms: u8,
        ) -> Result<Self::EndpointIn, EndpointAllocError> {
            self.inner
                .alloc_endpoint_in(ep_type, max_packet_size, interval_ms)
        }

        fn alloc_endpoint_out(
            &mut self,
            ep_type: EndpointType,
            max_packet_size: u16,
            interval_ms: u8,
        ) -> Result<Self::EndpointOut, EndpointAllocError> {
            self.inner
                .alloc_endpoint_out(ep_type, max_packet_size, interval_ms)
        }

        fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
            let (bus, cp) = self.inner.start(control_max_packet_size);

            (
                Bus {
                    inner: bus,
                    inited: false,
                },
                cp,
            )
        }
    }

    /// Asynchronous USB bus mainly used internally by `embassy-usb`.
    // We need a custom wrapper implementation to handle custom initialization.
    pub struct Bus<'d> {
        inner: OtgBus<'d, MAX_EP_COUNT>,
        inited: bool,
    }

    impl<'d> Bus<'d> {
        fn init(&mut self) {
            Usb::_enable();

            let r = Driver::REGISTERS;

            // Wait for AHB ready.
            while !r.grstctl().read().ahbidl() {}

            // Configure as device.
            r.gusbcfg().write(|w| {
                // Force device mode
                w.set_fdmod(true);
                w.set_srpcap(false);
                // Enable internal full-speed PHY
                w.set_physel(true);
            });
            self.inner.config_v1();

            // Perform core soft-reset
            r.grstctl().modify(|w| w.set_csrst(true));
            while r.grstctl().read().csrst() {}

            r.pcgcctl().modify(|w| {
                // Disable power down
                w.set_stppclk(false);
            });

            unsafe {
                crate::interrupt::bind_interrupt(
                    crate::peripherals::Interrupt::USB,
                    interrupt_handler.handler(),
                );
                crate::interrupt::enable(
                    crate::peripherals::Interrupt::USB,
                    interrupt_handler.priority(),
                )
                .unwrap();
            }
        }

        fn disable(&mut self) {
            crate::interrupt::disable(Cpu::ProCpu, crate::peripherals::Interrupt::USB);

            #[cfg(multi_core)]
            crate::interrupt::disable(Cpu::AppCpu, crate::peripherals::Interrupt::USB);

            Usb::_disable();
        }
    }

    impl<'d> embassy_usb_driver::Bus for Bus<'d> {
        async fn poll(&mut self) -> Event {
            if !self.inited {
                self.init();
                self.inited = true;
            }

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

    fn quirk_setup_late_cnak() -> bool {
        // Our CID register is 4 bytes offset from what's in embassy-usb-synopsys-otg
        let cid = unsafe {
            Driver::REGISTERS
                .as_ptr()
                .cast::<u32>()
                .add(0x40)
                .read_volatile()
        };
        // ESP32-Sx has a different CID register value, too
        cid == 0x4f54_400a || cid & 0xf000 == 0x1000
    }

    #[handler(priority = crate::interrupt::Priority::max())]
    fn interrupt_handler() {
        let setup_late_cnak = quirk_setup_late_cnak();

        unsafe {
            on_interrupt(
                Driver::REGISTERS,
                &STATE,
                Usb::ENDPOINT_COUNT,
                setup_late_cnak,
            )
        }
    }
}
