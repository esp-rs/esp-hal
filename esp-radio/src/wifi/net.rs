//! Implementation of adapters to support smoltcp and embassy-net.

use esp_config::esp_config_int;

use crate::wifi::{MTU, WifiDevice, WifiDeviceMode, dump_packet_info, esp_wifi_send_data};

#[doc(hidden)]
#[derive(Debug)]
pub struct WifiRxToken {
    mode: WifiDeviceMode,
}

impl WifiRxToken {
    /// Consumes the RX token and applies the callback function to the received
    /// data buffer.
    fn consume_token<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut data = self.mode.data_queue_rx().with(|queue| {
            unwrap!(
                queue.pop_front(),
                "unreachable: transmit()/receive() ensures there is a packet to process"
            )
        });

        // We handle the received data outside of the lock because
        // PacketBuffer::drop must not be called in a critical section.
        // Dropping an PacketBuffer will call `esp_wifi_internal_free_rx_buffer`
        // which will try to lock an internal mutex. If the mutex is already
        // taken, the function will try to trigger a context switch, which will
        // fail if we are in an interrupt-free context.
        let buffer = data.as_slice_mut();
        dump_packet_info(buffer);

        f(buffer)
    }
}

#[doc(hidden)]
#[derive(Debug)]
pub struct WifiTxToken {
    mode: WifiDeviceMode,
}

impl WifiTxToken {
    /// Consumes the TX token and applies the callback function to the received
    /// data buffer.
    fn consume_token<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.mode.increase_in_flight_counter();

        // (safety): creation of multiple Wi-Fi devices with the same mode is impossible
        // in safe Rust, therefore only smoltcp _or_ embassy-net can be used at
        // one time
        static mut BUFFER: [u8; MTU] = [0u8; MTU];

        let buffer = unsafe { &mut BUFFER[..len] };

        let res = f(buffer);

        esp_wifi_send_data(self.mode.interface(), buffer);

        res
    }
}

impl WifiDeviceMode {
    fn tx_token(&self) -> Option<WifiTxToken> {
        if !self.can_send() {
            // TODO: perhaps we can use a counting semaphore with a short blocking timeout
            crate::preempt::yield_task();
        }

        if self.can_send() {
            Some(WifiTxToken { mode: *self })
        } else {
            None
        }
    }

    fn rx_token(&self) -> Option<(WifiRxToken, WifiTxToken)> {
        let is_empty = self.data_queue_rx().with(|q| q.is_empty());
        if is_empty || !self.can_send() {
            // TODO: use an OS queue with a short timeout
            crate::preempt::yield_task();
        }

        let is_empty = is_empty && self.data_queue_rx().with(|q| q.is_empty());

        if !is_empty {
            self.tx_token().map(|tx| (WifiRxToken { mode: *self }, tx))
        } else {
            None
        }
    }
}

/// The embassy-net driver implementation.
pub mod embassy {
    use embassy_net_driver::{Capabilities, Driver, HardwareAddress, RxToken, TxToken};
    use esp_hal::asynch::AtomicWaker;

    use super::*;

    // We can get away with a single tx waker because the transmit queue is shared
    // between interfaces.
    pub(crate) static TRANSMIT_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) static AP_RECEIVE_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static AP_LINK_STATE_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) static STA_RECEIVE_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static STA_LINK_STATE_WAKER: AtomicWaker = AtomicWaker::new();

    impl RxToken for WifiRxToken {
        fn consume<R, F>(self, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            self.consume_token(f)
        }
    }

    impl TxToken for WifiTxToken {
        fn consume<R, F>(self, len: usize, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            self.consume_token(len, f)
        }
    }

    /// A [Driver] implementation.
    pub struct EmbassyNetAdapter<'d> {
        interface: &'d mut WifiDevice,
    }

    impl<'d> EmbassyNetAdapter<'d> {
        /// Creates a new [Driver] implementation.
        pub fn new(interface: &'d mut WifiDevice) -> Self {
            Self { interface }
        }
    }

    impl<'d> Driver for EmbassyNetAdapter<'d> {
        type RxToken<'a>
            = WifiRxToken
        where
            Self: 'a;
        type TxToken<'a>
            = WifiTxToken
        where
            Self: 'a;

        fn receive(
            &mut self,
            cx: &mut core::task::Context<'_>,
        ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
            self.interface.mode.register_receive_waker(cx);
            self.interface.mode.register_transmit_waker(cx);
            self.interface.mode.rx_token()
        }

        fn transmit(&mut self, cx: &mut core::task::Context<'_>) -> Option<Self::TxToken<'_>> {
            self.interface.mode.register_transmit_waker(cx);
            self.interface.mode.tx_token()
        }

        fn link_state(
            &mut self,
            cx: &mut core::task::Context<'_>,
        ) -> embassy_net_driver::LinkState {
            self.interface.mode.register_link_state_waker(cx);
            self.interface.mode.link_state()
        }

        fn capabilities(&self) -> Capabilities {
            let mut caps = Capabilities::default();
            caps.max_transmission_unit = MTU;
            caps.max_burst_size =
                if esp_config_int!(usize, "ESP_RADIO_CONFIG_WIFI_MAX_BURST_SIZE") == 0 {
                    None
                } else {
                    Some(esp_config_int!(
                        usize,
                        "ESP_RADIO_CONFIG_WIFI_MAX_BURST_SIZE"
                    ))
                };
            caps
        }

        fn hardware_address(&self) -> HardwareAddress {
            HardwareAddress::Ethernet(self.interface.mac_address())
        }
    }
}

/// The smoltcp device implementation.
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[cfg(all(feature = "smoltcp", feature = "unstable"))]
pub mod smoltcp_adapter {
    use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};

    use super::*;

    impl TxToken for WifiTxToken {
        fn consume<R, F>(self, len: usize, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            self.consume_token(len, f)
        }
    }

    impl RxToken for WifiRxToken {
        fn consume<R, F>(self, f: F) -> R
        where
            F: FnOnce(&[u8]) -> R,
        {
            self.consume_token(|t| f(t))
        }
    }

    /// A [Device] implementation.
    pub struct SmoltcpAdapter<'d> {
        interface: &'d mut WifiDevice,
    }

    impl<'d> SmoltcpAdapter<'d> {
        /// Creates a new [Device] implementation.
        pub fn new(interface: &'d mut WifiDevice) -> Self {
            Self { interface }
        }

        /// Return the MAC address.
        pub fn mac_address(&self) -> [u8; 6] {
            self.interface.mac_address()
        }
    }

    impl<'d> Device for SmoltcpAdapter<'d> {
        type RxToken<'a>
            = WifiRxToken
        where
            Self: 'a;
        type TxToken<'a>
            = WifiTxToken
        where
            Self: 'a;

        fn receive(
            &mut self,
            _instant: smoltcp::time::Instant,
        ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
            self.interface.mode.rx_token()
        }

        fn transmit(&mut self, _instant: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
            self.interface.mode.tx_token()
        }

        fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
            let mut caps = DeviceCapabilities::default();
            caps.max_transmission_unit = MTU;
            caps.max_burst_size =
                if esp_config_int!(usize, "ESP_RADIO_CONFIG_WIFI_MAX_BURST_SIZE") == 0 {
                    None
                } else {
                    Some(esp_config_int!(
                        usize,
                        "ESP_RADIO_CONFIG_WIFI_MAX_BURST_SIZE"
                    ))
                };
            caps
        }
    }
}
