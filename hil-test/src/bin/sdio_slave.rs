//! SDIO slave configuration and DMA shutdown tests without an SDIO host.
//!
//! No additional wiring is required. Do not connect an SDIO host to GPIO18–23.
//! These tests do not supply a bus clock or require a peer MCU.

//% CHIP_FILTER: soc_has_slc
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    Blocking,
    peripherals::{Peripherals, SLCHOST},
    sdio_slave::{Descriptor, Error, SdioSlave, State},
};
use hil_test as _;
use static_cell::StaticCell;

fn configured_slave(p: Peripherals) -> SdioSlave<'static, Blocking> {
    SdioSlave::new(p.HINF, p.SLC, p.SLCHOST)
        .with_cmd(p.GPIO18)
        .with_clk(p.GPIO19)
        .with_data0(p.GPIO20)
        .with_data1(p.GPIO21)
        .with_data2(p.GPIO22)
        .with_data3(p.GPIO23)
}

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn incomplete_bus_does_not_change_configuration(p: Peripherals) {
        let mut slave = SdioSlave::new(p.HINF, p.SLC, p.SLCHOST);
        let before = slave.snapshot();
        let edge_control = SLCHOST::regs().conf().read().bits();

        assert_eq!(slave.enable_bus(), Err(Error::MissingPins));
        assert_eq!(slave.state(), State::ControllerReady);
        assert_eq!(slave.snapshot().cfg_data1, before.cfg_data1);
        assert_eq!(slave.snapshot().cfg_data7, before.cfg_data7);
        assert_eq!(SLCHOST::regs().conf().read().bits(), edge_control);
    }

    #[test]
    fn idle_bus_can_reopen_and_advertises_high_speed(p: Peripherals) {
        let mut slave = configured_slave(p);
        let edge_control = SLCHOST::regs().conf().read().bits();

        for _ in 0..2 {
            slave.enable_bus().unwrap();
            assert_eq!(slave.state(), State::BusExposed);
            assert!(slave.snapshot().highspeed_advertised());
            assert!(!slave.snapshot().highspeed_selected());
            assert!(SLCHOST::regs().conf().read().hspeed_con_en().bit_is_set());
            assert_eq!(
                SLCHOST::regs().conf().read().bits() & !(1 << 27),
                edge_control & !(1 << 27)
            );

            slave.disable_bus().unwrap();
            assert_eq!(slave.state(), State::ControllerReady);
            assert!(slave.take_receive_buffers().unwrap().is_none());
            assert!(slave.take_transmit_buffers().unwrap().is_none());
        }
    }

    #[test]
    fn stopped_receive_returns_storage_and_invalidates_session(p: Peripherals) {
        static DESCRIPTOR: StaticCell<Descriptor> = StaticCell::new();
        static BUFFER: StaticCell<[u32; 32]> = StaticCell::new();

        let mut slave = configured_slave(p);
        slave.enable_bus().unwrap();
        let descriptor = DESCRIPTOR.init(Descriptor::new());
        let buffer = BUFFER.init([0; 32]);
        let descriptor_address = core::ptr::from_mut(descriptor);
        let buffer_address = buffer.as_mut_ptr();
        slave.arm_receive(descriptor, buffer).unwrap();

        assert_eq!(slave.state(), State::Function1Receiving);
        assert!(matches!(
            slave.take_receive_buffers(),
            Err(Error::InvalidState)
        ));
        slave.disable_bus().unwrap();
        assert_eq!(slave.state(), State::SessionInvalidated);
        let (descriptor, buffer) = slave.take_receive_buffers().unwrap().unwrap();
        assert_eq!(core::ptr::from_mut(descriptor), descriptor_address);
        assert_eq!(buffer.as_mut_ptr(), buffer_address);
        buffer.fill(0xA5A5_A5A5);
        assert!(slave.take_receive_buffers().unwrap().is_none());
        assert!(slave.take_transmit_buffers().unwrap().is_none());
        assert_eq!(slave.enable_bus(), Err(Error::SessionInvalidated));
    }

    #[test]
    fn stopped_transmit_returns_storage_and_invalidates_session(p: Peripherals) {
        static DESCRIPTOR: StaticCell<Descriptor> = StaticCell::new();
        static BUFFER: StaticCell<[u32; 32]> = StaticCell::new();

        let mut slave = configured_slave(p);
        slave.enable_bus().unwrap();
        let descriptor = DESCRIPTOR.init(Descriptor::new());
        let buffer = BUFFER.init([0x5A5A_5A5A; 32]);
        let descriptor_address = core::ptr::from_mut(descriptor);
        let buffer_address = buffer.as_mut_ptr();
        slave.arm_transmit(descriptor, buffer, 64).unwrap();

        assert_eq!(slave.state(), State::Function1Transmitting);
        assert!(matches!(
            slave.take_transmit_buffers(),
            Err(Error::InvalidState)
        ));
        slave.disable_bus().unwrap();
        assert_eq!(slave.state(), State::SessionInvalidated);
        let (descriptor, buffer) = slave.take_transmit_buffers().unwrap().unwrap();
        assert_eq!(core::ptr::from_mut(descriptor), descriptor_address);
        assert_eq!(buffer.as_mut_ptr(), buffer_address);
        buffer.fill(0xA5A5_A5A5);
        assert!(slave.take_transmit_buffers().unwrap().is_none());
        assert!(slave.take_receive_buffers().unwrap().is_none());
        assert_eq!(slave.enable_bus(), Err(Error::SessionInvalidated));
    }
}
