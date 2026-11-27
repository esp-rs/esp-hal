//! # Secure Digital Input Output (SDIO) Slave
//!
//! ## Overview
//!
//! The ESP32-C6 controller exchanges one packet at a time through SLC0 DMA.
//!
//! One descriptor receives one incremental-address packet on SLC0. The host
//! reads or writes CMD53 at `0x1F800 - length`, after function 1 is enabled and
//! ready. One direction is armed at a time. Rearming is explicit.
//!
//! ## Configuration
//!
//! Pads use fixed IO MUX function 0: CMD GPIO18, CLK GPIO19, DATA0..3 GPIO20..23.
//! Disabling the bus selects GPIO with input/output disabled. Previous pad
//! settings are not restored; pull resistors are unchanged.
//!
//! ## Usage
//!
//! Convert with [`SdioSlave::into_async`] for SLC0 IRQ completion waits. Dropping
//! a wait masks notifications but retains the DMA-owned storage. Stop DMA with
//! [`SdioSlave::disable_bus`] before reclaiming a cancelled transfer.
//!
//! [`SdioSlave::arm_receive`] consumes exclusive static storage in internal RAM.
//! Only a validated EOF with a parked DMA makes [`SdioSlave::received`] available.
//! [`SdioSlave::disable_bus`] clears readiness and waits for DMA idle before
//! releasing pads. [`SdioSlave::take_receive_buffers`] returns storage only after
//! this stop succeeds. A stop timeout retains ownership and holds the FSM reset.
//! If destruction cannot confirm idle, the clock reference and static storage
//! are retained instead of permitting unsafe reuse.
//!
//! The implementation uses HINF, SLC and SLCHOST (ESP32-C6 TRM chapter 34).
//! Use function 1 for incremental-address packets. Transmit length is published
//! as a cumulative 20-bit counter; the host subtracts its consumed-byte count
//! modulo 2^20. Receive credits are cumulative modulo 4096. Interrupting either
//! direction invalidates this session: shutdown can return memory safely, but
//! cannot retract a published length or a receive credit. Recover storage, drop
//! the driver, reset controller and host accounting, then create a new session.
//!
//! For register details, see the
#![doc = concat!(crate::trm_markdown_link!(), ".\n")]
//! ## Examples
//!
//! ### Configure the Native Pads
//!
//! Construct the native C6 bus with its fixed pads. Borrowing peripherals lets
//! the application construct another driver after dropping an invalidated one.
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::sdio_slave::SdioSlave;
//! let mut slave = SdioSlave::new(
//!     peripherals.HINF.reborrow(), peripherals.SLC.reborrow(),
//!     peripherals.SLCHOST.reborrow(),
//! )
//! .with_cmd(peripherals.GPIO18.reborrow())
//! .with_clk(peripherals.GPIO19.reborrow())
//! .with_data0(peripherals.GPIO20.reborrow())
//! .with_data1(peripherals.GPIO21.reborrow())
//! .with_data2(peripherals.GPIO22.reborrow())
//! .with_data3(peripherals.GPIO23.reborrow());
//! slave.enable_bus()?;
#![doc = crate::after_snippet!()]
//! ```
#![doc = "\n"]
//! A different GPIO cannot be assigned to a native signal:
//!
//! ```compile_fail
#![doc = crate::before_snippet!()]
//! use esp_hal::sdio_slave::SdioSlave;
//! let slave = SdioSlave::new(peripherals.HINF, peripherals.SLC, peripherals.SLCHOST);
//! let slave = slave.with_cmd(peripherals.GPIO19);
#![doc = crate::after_snippet!()]
//! ```
#![doc = "\n"]
//! ## Implementation State
//!
//! The driver supports single-packet SLC0 DMA in both directions, blocking
//! polling, and interrupt-driven asynchronous waits on ESP32-C6. It does not
//! support descriptor chains, SLC1 transfers, or automatic rearming.

use core::{
    marker::PhantomData,
    sync::atomic::{Ordering, compiler_fence},
};

use crate::{
    Async,
    Blocking,
    DriverMode,
    gpio::{AlternateFunction, AnyPin, Pin},
    peripherals::{GPIO18, GPIO19, GPIO20, GPIO21, GPIO22, GPIO23, HINF, SLC, SLCHOST},
    system::{Peripheral, PeripheralGuard},
};

mod events;
mod receive;
mod state;

use events::{Completion, DMA_EVENTS, RECEIVE_EVENTS, TRANSMIT_EVENTS, completion};
pub use receive::{ArmReceiveError, ArmTransmitError, Descriptor};
use receive::{Receive, Transmit};
use state::wait_for_idle;
pub use state::{Error, FunctionState, State};

/// Raw view of the receive path, for diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceiveDiagnostics {
    /// First word of the descriptor as the DMA left it (owner, eof, length, size).
    pub descriptor_dw0: u32,
    /// `SDIO_SLC0_TOKEN1`: accumulated number of receive buffers announced to the host.
    pub token: u32,
    /// `SDIO_SLC0INT_RAW_REG`.
    pub interrupt_raw: u32,
    /// `SDIO_SLC0_TXLINK_PARK`: the receive FSM is idle.
    pub parked: bool,
}

/// Raw outbound-path registers. These values do not validate a host payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct TransmitDiagnostics {
    /// Descriptor owner, EOF, length and capacity, read with a volatile access.
    pub descriptor_dw0: u32,
    /// Cumulative SLC0 length published to the host, modulo 20 bits.
    pub published_length: u32,
    /// SLC0 raw interrupt status, without acknowledging it.
    pub interrupt_raw: u32,
    /// Indicates whether the slave-to-host linked-list FSM is parked.
    pub parked: bool,
}

/// Raw diagnostic registers. The values are not proof of a successful
/// enumeration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RegisterSnapshot {
    /// `HINF_CFG_DATA1_REG` (CCCR configuration and status).
    pub cfg_data1: u32,
    /// `HINF_CFG_DATA7_REG` (function 0 ready, interrupt and capability bits).
    pub cfg_data7: u32,
    /// `HINF_CONF_STATUS_REG` (function 0 configuration status).
    pub function0_status: u32,
    /// `SDIO_SLC0INT_RAW_REG`: SLC0 raw interrupt status.
    pub slc0_interrupt_raw: u32,
    /// `SDIO_SLC1INT_RAW_REG`: SLC1 raw interrupt status.
    pub slc1_interrupt_raw: u32,
}

impl RegisterSnapshot {
    /// Returns whether the CCCR advertises high speed support (`SHS`).
    ///
    /// Reads `HINF_CFG_DATA1.HIGHSPEED_ENABLE` from the snapshot.
    pub const fn highspeed_advertised(&self) -> bool {
        state::highspeed_advertised(self.cfg_data1)
    }

    /// Returns whether the host has selected high speed (`EHS`).
    ///
    /// Reads `HINF_CFG_DATA1.HIGHSPEED_MODE` from the snapshot.
    pub const fn highspeed_selected(&self) -> bool {
        state::highspeed_selected(self.cfg_data1)
    }
}

/// SDIO Slave controller with exclusive ownership of one SLC0 packet.
pub struct SdioSlave<'d, Dm: DriverMode = Blocking> {
    _hinf: HINF<'d>,
    _slc: SLC<'d>,
    _slchost: SLCHOST<'d>,
    pads: [Option<AnyPin<'d>>; 6],
    state: State,
    receive: Option<Receive>,
    receive_error: Option<Error>,
    transmit: Option<Transmit>,
    transmit_error: Option<Error>,
    mode: PhantomData<Dm>,
    // Dropped last: the clock must outlive every other resource.
    _clock: Option<PeripheralGuard>,
}

impl<'d> SdioSlave<'d, Blocking> {
    /// Creates a new SDIO slave controller in [`Blocking`] mode.
    ///
    /// Enables the block clock. The first clock user also resets the block.
    /// No pad is configured or routed. All six native pads must be supplied
    /// through the `with_*` methods before calling [`Self::enable_bus`].
    pub fn new(hinf: HINF<'d>, slc: SLC<'d>, slchost: SLCHOST<'d>) -> Self {
        let clock = PeripheralGuard::new(Peripheral::SdioSlave);

        Self {
            _hinf: hinf,
            _slc: slc,
            _slchost: slchost,
            pads: [None, None, None, None, None, None],
            state: State::ControllerReady,
            receive: None,
            receive_error: None,
            transmit: None,
            transmit_error: None,
            mode: PhantomData,
            _clock: Some(clock),
        }
    }

    /// Binds the SLC0 handler for asynchronous completion waits.
    /// SLC1 remains disabled. Existing DMA ownership is preserved.
    #[instability::unstable]
    pub fn into_async(self) -> SdioSlave<'d, Async> {
        self.mask_dma_interrupts();
        self._slc.disable_ch0_interrupt_on_all_cores();
        self._slc.bind_ch0_interrupt(slc0_handler);
        self.map_mode()
    }
}

impl<'d> SdioSlave<'d, Async> {
    /// Removes the asynchronous IRQ handler and returns blocking operation.
    /// An active transfer remains owned and can be polled or stopped.
    #[instability::unstable]
    pub fn into_blocking(self) -> SdioSlave<'d, Blocking> {
        self.mask_dma_interrupts();
        self._slc.disable_ch0_interrupt_on_all_cores();
        self.map_mode()
    }

    /// Waits for an armed host-to-slave packet using SLC0 interrupts.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidState`] unless a receive is active, or a DMA error.
    ///
    /// # Cancellation Safety
    ///
    /// Dropping this future masks notification. It does not stop DMA or return
    /// storage. Resume the wait, poll completion, or call [`Self::disable_bus`].
    /// This permits cancellation with Embassy without returning DMA-owned RAM.
    /// Disabling an incomplete transfer invalidates its peer accounting; recover
    /// storage and create a new session after resetting controller and host.
    #[instability::unstable]
    pub async fn wait_received(&mut self) -> Result<usize, Error> {
        let mut source = CompletionWait {
            driver: self,
            transmitting: false,
        };
        events::Wait::new(&mut source).await
    }

    /// Waits for an armed slave-to-host packet using SLC0 interrupts.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidState`] unless a transmit is active, or a DMA error.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation masks notification and retains the active transfer and its
    /// storage. Use [`Self::disable_bus`] for explicit cancellation of DMA.
    /// Stopping an incomplete transfer requires a new controller/host session.
    #[instability::unstable]
    pub async fn wait_transmitted(&mut self) -> Result<usize, Error> {
        let mut source = CompletionWait {
            driver: self,
            transmitting: true,
        };
        events::Wait::new(&mut source).await
    }
}

impl<'d, Dm: DriverMode> SdioSlave<'d, Dm> {
    /// Assigns GPIO18 to the SDIO command signal.
    ///
    /// The native IO MUX connection requires this fixed GPIO. The pad is
    /// configured only when [`Self::enable_bus`] succeeds.
    ///
    /// # Panics
    ///
    /// Panics unless the controller is in [`State::ControllerReady`].
    pub fn with_cmd(self, pin: GPIO18<'d>) -> Self {
        self.with_pad(0, pin.degrade())
    }

    /// Assigns GPIO19 to the SDIO clock signal.
    ///
    /// The native IO MUX connection requires this fixed GPIO. The pad is
    /// configured only when [`Self::enable_bus`] succeeds.
    ///
    /// # Panics
    ///
    /// Panics unless the controller is in [`State::ControllerReady`].
    pub fn with_clk(self, pin: GPIO19<'d>) -> Self {
        self.with_pad(1, pin.degrade())
    }

    /// Assigns GPIO20 to the SDIO data 0 signal.
    ///
    /// The native IO MUX connection requires this fixed GPIO. The pad is
    /// configured only when [`Self::enable_bus`] succeeds.
    ///
    /// # Panics
    ///
    /// Panics unless the controller is in [`State::ControllerReady`].
    pub fn with_data0(self, pin: GPIO20<'d>) -> Self {
        self.with_pad(2, pin.degrade())
    }

    /// Assigns GPIO21 to the SDIO data 1 signal.
    ///
    /// The native IO MUX connection requires this fixed GPIO. The pad is
    /// configured only when [`Self::enable_bus`] succeeds.
    ///
    /// # Panics
    ///
    /// Panics unless the controller is in [`State::ControllerReady`].
    pub fn with_data1(self, pin: GPIO21<'d>) -> Self {
        self.with_pad(3, pin.degrade())
    }

    /// Assigns GPIO22 to the SDIO data 2 signal.
    ///
    /// The native IO MUX connection requires this fixed GPIO. The pad is
    /// configured only when [`Self::enable_bus`] succeeds.
    ///
    /// # Panics
    ///
    /// Panics unless the controller is in [`State::ControllerReady`].
    pub fn with_data2(self, pin: GPIO22<'d>) -> Self {
        self.with_pad(4, pin.degrade())
    }

    /// Assigns GPIO23 to the SDIO data 3 signal.
    ///
    /// The native IO MUX connection requires this fixed GPIO. The pad is
    /// configured only when [`Self::enable_bus`] succeeds.
    ///
    /// # Panics
    ///
    /// Panics unless the controller is in [`State::ControllerReady`].
    pub fn with_data3(self, pin: GPIO23<'d>) -> Self {
        self.with_pad(5, pin.degrade())
    }

    fn with_pad(mut self, index: usize, pad: AnyPin<'d>) -> Self {
        // Replacing a routed pad could interrupt a transfer and release a pin
        // still used by the peripheral. Configuration is restricted to idle.
        assert!(self.state == State::ControllerReady);
        self.pads[index] = Some(pad);
        self
    }

    fn map_mode<Other: DriverMode>(self) -> SdioSlave<'d, Other> {
        let this = core::mem::ManuallyDrop::new(self);
        // SAFETY: every owned field moves exactly once. ManuallyDrop prevents
        // the old driver from stopping DMA or releasing its clock and pads.
        unsafe {
            SdioSlave {
                _hinf: core::ptr::read(&this._hinf),
                _slc: core::ptr::read(&this._slc),
                _slchost: core::ptr::read(&this._slchost),
                pads: core::ptr::read(&this.pads),
                state: this.state,
                receive: core::ptr::read(&this.receive),
                receive_error: this.receive_error,
                transmit: core::ptr::read(&this.transmit),
                transmit_error: this.transmit_error,
                mode: PhantomData,
                _clock: core::ptr::read(&this._clock),
            }
        }
    }

    /// Returns the current controller state.
    pub fn state(&self) -> State {
        self.state
    }

    /// Exposes the SDIO controller on the bus.
    ///
    /// 1. Clears `IOREADY1` and `IOREADY2` (no function is ready).
    /// 2. Puts each pad in a defined state: output disabled, hold released, GPIO matrix output on
    ///    plain GPIO.
    /// 3. Selects IO MUX function 0 on the six pads and enables their input. The output direction
    ///    of the pads is left to the controller.
    ///
    /// The CCCR advertises high speed support (`SHS`, `HINF_CFG_DATA1.HIGHSPEED_ENABLE`) and the
    /// sending edge follows the high speed mode (`SLCHOST_CONF.HSPEED_CON_EN`) before the pads are
    /// routed; the host still has to select it (`EHS`). The other CCCR fields, the FBR and
    /// the CIS keep their reset values, and the interrupts and DMA are not touched. A successful
    /// return does not prove that a host can enumerate the card.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidState`] unless the state is
    /// [`State::ControllerReady`]. Returns [`Error::SessionInvalidated`] after
    /// an interrupted packet; this instance cannot resume the previous session.
    /// Returns [`Error::MissingPins`] if any of the six native pads is missing.
    /// No register or pad is changed when configuration is incomplete.
    pub fn enable_bus(&mut self) -> Result<(), Error> {
        state::validate_bus_configuration(self.state, &self.pads)?;
        let next = State::BusExposed;

        self.clear_function_ready();
        // The CCCR must advertise high speed, and the edge control be set, before a host can read
        // the card: the pads are handed over below, so this comes first.
        self.configure_highspeed();

        for pad in self.pads.iter().flatten() {
            // Known starting point: no output forced by the GPIO matrix, hold
            // released. Then hand the pad over to the controller.
            pad.init_gpio();
            pad.set_alternate_function(AlternateFunction::_0);
            pad.set_input_enable(true);
        }

        self.state = next;
        Ok(())
    }

    /// Takes the card off the bus.
    ///
    /// Clears `IOREADY1/2`, then puts the pads back in a defined GPIO state
    /// (output and input disabled, hold released). The previous pad
    /// configuration is not restored, the pulls are not changed, the clock is
    /// kept and the block is not reset.
    ///
    /// If either transfer direction has not reached validated completion, a
    /// successful stop enters [`State::SessionInvalidated`]. Its buffers can be
    /// reclaimed, but [`Self::enable_bus`] refuses to reopen this session. A
    /// pending receive credit or published outbound length cannot be revoked
    /// safely here. Reset controller and host accounting before a new session.
    /// After validated completion, stopping enters [`State::ControllerReady`].
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidState`] unless the pads are routed ([`State::BusExposed`] or
    /// [`State::Function1Receiving`], [`State::Function1Received`], or
    /// [`State::Function1ReceiveError`], or a transmit state). [`Error::DmaStopTimeout`] if DMA
    /// does not become idle. Storage, pads, and the clock remain owned on timeout.
    pub fn disable_bus(&mut self) -> Result<(), Error> {
        let next = self.state.after_disable();
        if !self.state.can_transition_to(next) {
            return Err(Error::InvalidState);
        }

        self.clear_function_ready();
        self.mask_dma_interrupts();
        self.stop_receive_dma()?;
        self.stop_transmit_dma()?;
        self.release_pads();
        self.state = next;
        Ok(())
    }

    /// State of I/O function 1, as seen by the host.
    pub fn function1(&self) -> FunctionState {
        let cfg = HINF::regs().cfg_data1().read();
        state::function_state(
            cfg.ioenable1().bit_is_set(),
            cfg.sdio_ioready1().bit_is_set(),
        )
    }

    /// State of I/O function 2, as seen by the host.
    pub fn function2(&self) -> FunctionState {
        let cfg = HINF::regs().cfg_data1().read();
        state::function_state(
            cfg.ioenable2().bit_is_set(),
            cfg.sdio_ioready2().bit_is_set(),
        )
    }

    /// Reads the diagnostic registers without clearing or acknowledging
    /// anything.
    ///
    /// The block is clocked for as long as the driver exists, so the read is
    /// always valid. Raw interrupt status is read: the masked status would
    /// read zero while the interrupts are disabled.
    pub fn snapshot(&self) -> RegisterSnapshot {
        RegisterSnapshot {
            cfg_data1: HINF::regs().cfg_data1().read().bits(),
            cfg_data7: HINF::regs().cfg_data7().read().bits(),
            function0_status: HINF::regs().conf_status().read().bits(),
            slc0_interrupt_raw: SLC::regs().slc0int_raw().read().bits(),
            slc1_interrupt_raw: SLC::regs().slc1int_raw().read().bits(),
        }
    }

    /// Arms the SLC0 receive DMA (host to slave) with one descriptor and one buffer, and
    /// announces one receive buffer to the host.
    ///
    /// 1. Initializes the DMA configuration bits specified by the TRM (`TOKEN_AUTO_CLR` 0,
    ///    `RX_NO_RESTART_CLR` 1, `TOKEN_NO_REPLACE` 1, the `SLCCONF1` bits 0).
    /// 2. Pulses the SLC0 TX (host to slave) FSM reset.
    /// 3. Writes the descriptor (`owner` = DMA, `eof` = 0, `length` = 0, `size` = buffer size).
    /// 4. Writes the descriptor address to `SDIO_SLC0TX_LINK_ADDR_REG` and sets `TXLINK_START`.
    /// 5. Adds one to the SLC0 token counter, so the host reads one more available buffer in
    ///    `SLCHOST_SLC0HOST_TOKEN_RDATA_REG`.
    ///
    /// The function is **not** declared ready here, see [`Self::set_function1_ready`].
    ///
    /// # Errors
    ///
    /// [`ArmReceiveError`] retains the original storage and contains
    /// [`Error::InvalidState`] unless the state is [`State::BusExposed`]; [`Error::InvalidBuffer`]
    /// if the buffer is empty or larger than 16380 bytes. [`Error::UnsupportedMemory`]
    /// if either allocation is outside internal RAM. Existing storage must be
    /// reclaimed with [`Self::take_receive_buffers`] before a new arm.
    pub fn arm_receive(
        &mut self,
        descriptor: &'static mut Descriptor,
        buffer: &'static mut [u32],
    ) -> Result<(), ArmReceiveError> {
        let next = State::Function1Receiving;
        if !self.state.can_transition_to(next) || self.state != State::BusExposed {
            return Err(ArmReceiveError::new(
                Error::InvalidState,
                descriptor,
                buffer,
            ));
        }
        if self.receive.is_some() || self.transmit.is_some() {
            return Err(ArmReceiveError::new(
                Error::InvalidState,
                descriptor,
                buffer,
            ));
        }
        if buffer.is_empty() || buffer.len() > 0x3ffc / 4 {
            return Err(ArmReceiveError::new(
                Error::InvalidBuffer,
                descriptor,
                buffer,
            ));
        }

        if !crate::soc::is_slice_in_dram(buffer)
            || !crate::soc::is_slice_in_dram(core::slice::from_ref(descriptor))
        {
            return Err(ArmReceiveError::new(
                Error::UnsupportedMemory,
                descriptor,
                buffer,
            ));
        }
        let receive = Receive::new(descriptor, buffer)?;
        let descriptor_address = receive.descriptor_address();
        self.receive = Some(receive);
        self.receive_error = None;

        let slc = SLC::regs();
        slc.slcconf0().modify(|_, w| {
            w.sdio_slc0_token_auto_clr().clear_bit();
            w.sdio_slc0_rx_no_restart_clr().set_bit();
            w.sdio_slc0_tx_loop_test().clear_bit()
        });
        // esp-pacs 61f9678 addresses bit 1 here. TRM 34.19 specifies bit 0.
        slc.slc_rx_dscr_conf()
            .modify(|r, w| unsafe { w.bits(receive::token_no_replace(r.bits())) });
        slc.slcconf1().modify(|_, w| {
            w.sdio_sdio_cmd_hold_en().clear_bit();
            w.sdio_slc0_len_auto_clr().clear_bit();
            w.sdio_slc0_tx_stitch_en().clear_bit();
            w.sdio_slc0_rx_stitch_en().clear_bit()
        });
        // Reset the SLC0 TX FSM, then release it.
        slc.slcconf0().modify(|_, w| w.sdio_slc0_tx_rst().set_bit());
        slc.slcconf0()
            .modify(|_, w| w.sdio_slc0_tx_rst().clear_bit());

        // Orders descriptor stores before link activation on the C6 AHB bus.
        dma_fence();

        self.clear_dma_events(RECEIVE_EVENTS);
        slc.slc0tx_link_addr()
            .write(|w| unsafe { w.sdio_slc0_txlink_addr().bits(descriptor_address) });
        slc.slc0tx_link()
            .modify(|_, w| w.sdio_slc0_txlink_start().set_bit());
        // The counter is cumulative: overwriting it with 1 on explicit reuse
        // would make the host's already-consumed credit disappear.
        slc.slc0token1()
            .write(|w| w.sdio_slc0_token1_inc().set_bit());

        self.state = next;
        Ok(())
    }

    /// Declares I/O function 1 ready (`IOREADY1`) once either DMA direction is
    /// armed. The host sees it in CCCR `IOR1` once it has enabled the function (`IOE1`).
    ///
    /// # Errors
    ///
    /// [`Error::InvalidState`] unless a receive or transmit is armed.
    pub fn set_function1_ready(&mut self) -> Result<(), Error> {
        if !self.state.function_ready_allowed() {
            return Err(Error::InvalidState);
        }
        HINF::regs()
            .cfg_data1()
            .modify(|_, w| w.sdio_ioready1().set_bit());
        Ok(())
    }

    /// Checks for one successfully received packet.
    ///
    /// The EOF event and parked DMA are both required. The descriptor must have
    /// CPU ownership, EOF, and a length within the buffer. A successful result
    /// makes [`Self::received`] available and withdraws function readiness.
    /// Subsequent polls return `Ok(None)`. No automatic rearming occurs.
    ///
    /// # Errors
    ///
    /// [`Error::DmaFault`] on a DMA overflow or descriptor error.
    /// [`Error::InvalidDescriptor`] on invalid write-back. Storage remains owned
    /// on error and must be stopped with [`Self::disable_bus`].
    pub fn poll_received(&mut self) -> Result<Option<usize>, Error> {
        if let Some(error) = self.receive_error {
            return Err(error);
        }
        if self.state != State::Function1Receiving {
            return Ok(None);
        }
        let slc = SLC::regs();
        let raw = slc.slc0int_raw().read();
        match completion(
            raw.bits(),
            false,
            slc.slc0tx_link()
                .read()
                .sdio_slc0_txlink_park()
                .bit_is_set(),
        ) {
            Completion::Fault => return self.receive_failed(Error::DmaFault),
            Completion::Pending | Completion::Draining => return Ok(None),
            Completion::Complete => {}
        }
        self.clear_function_ready();
        dma_fence();
        // SAFETY: successful EOF and parked DMA were observed above. The
        // acquire barrier precedes every CPU access to the received bytes.
        let result = unsafe { self.receive.as_mut().unwrap().finish() };
        let length = match result {
            Ok(length) => length,
            Err(error) => return self.receive_failed(error),
        };
        slc.slc0int_clr()
            .write(|w| w.sdio_slc0_tx_suc_eof_int_clr().set_bit());
        self.state = State::Function1Received;
        self.mask_dma_interrupts();
        Ok(Some(length))
    }

    /// Returns the validated packet after successful completion.
    ///
    /// Returns `None` before completion and after an error or bus disable.
    /// The length comes from validated DMA write-back, not from the caller.
    pub fn received(&self) -> Option<&[u8]> {
        if self.state != State::Function1Received {
            return None;
        }
        self.receive.as_ref()?.received()
    }

    /// Returns descriptor and buffer storage after disabling the bus.
    ///
    /// This operation permits explicit reuse of the static allocations. It does
    /// not arm DMA, expose pads, or publish a new token. After cancellation the
    /// allocations can be reused in a new, resynchronized session only.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidState`] unless [`Self::disable_bus`] has succeeded.
    pub fn take_receive_buffers(
        &mut self,
    ) -> Result<Option<(&'static mut Descriptor, &'static mut [u32])>, Error> {
        if !self.state.dma_stopped() {
            return Err(Error::InvalidState);
        }
        // SAFETY: both stopped states require confirmed DMA shutdown or no
        // previous transfer. take() prevents double restitution.
        Ok(self
            .receive
            .take()
            .map(|receive| unsafe { receive.into_buffers() }))
    }

    /// Reclaims a completed receive without disconnecting the bus pads.
    ///
    /// Function readiness stays cleared until the next explicit arm. The
    /// cumulative host credit is preserved. The returned storage can be reused
    /// in either direction once its CPU references have ended.
    ///
    /// # Errors
    /// Returns [`Error::InvalidState`] before successful receive completion,
    /// or [`Error::DmaStopTimeout`] if shutdown cannot be confirmed.
    #[instability::unstable]
    pub fn finish_receive(
        &mut self,
    ) -> Result<(&'static mut Descriptor, &'static mut [u32]), Error> {
        if self.state != State::Function1Received {
            return Err(Error::InvalidState);
        }
        self.mask_dma_interrupts();
        self.stop_receive_dma()?;
        let receive = self.receive.take().unwrap();
        self.state = State::BusExposed;
        // SAFETY: stop_receive_dma confirmed quiescence and ordered accesses.
        Ok(unsafe { receive.into_buffers() })
    }

    fn receive_failed(&mut self, error: Error) -> Result<Option<usize>, Error> {
        self.clear_function_ready();
        self.mask_dma_interrupts();
        self.receive_error = Some(error);
        self.state = State::Function1ReceiveError;
        Err(error)
    }

    /// Raw receive-path registers, read without clearing anything.
    pub fn receive_diagnostics(&self) -> ReceiveDiagnostics {
        let slc = SLC::regs();
        let descriptor_dw0 = self
            .receive
            .as_ref()
            .map(Receive::descriptor_word)
            .unwrap_or(0);
        ReceiveDiagnostics {
            descriptor_dw0,
            token: slc.slc0token1().read().sdio_slc0_token1().bits() as u32,
            interrupt_raw: slc.slc0int_raw().read().bits(),
            parked: slc
                .slc0tx_link()
                .read()
                .sdio_slc0_txlink_park()
                .bit_is_set(),
        }
    }

    /// Confirms idle before allowing resource reuse. A timeout holds TX reset
    /// asserted and preserves storage, pads, and clock ownership.
    fn stop_receive_dma(&mut self) -> Result<(), Error> {
        if self.receive.is_none() {
            return Ok(());
        }
        let slc = SLC::regs();
        slc.slc0tx_link()
            .modify(|_, w| w.sdio_slc0_txlink_stop().set_bit());
        dma_fence();
        let parked = || {
            let link = slc.slc0tx_link().read();
            link.sdio_slc0_txlink_park().bit_is_set()
                && link.sdio_slc0_txlink_start().bit_is_clear()
                && link.sdio_slc0_txlink_stop().bit_is_clear()
        };
        let mut idle = wait_for_idle(parked);
        slc.slcconf0().modify(|_, w| w.sdio_slc0_tx_rst().set_bit());
        dma_fence();
        if !idle {
            idle = wait_for_idle(parked);
        }
        if !idle {
            self.receive_error = Some(Error::DmaStopTimeout);
            self.state = State::Function1ReceiveError;
            return Err(Error::DmaStopTimeout);
        }
        dma_fence();
        slc.slcconf0()
            .modify(|_, w| w.sdio_slc0_tx_rst().clear_bit());
        slc.slc0int_clr().write(|w| {
            w.sdio_slc0_tx_suc_eof_int_clr().set_bit();
            w.sdio_slc0_tx_dscr_err_int_clr().set_bit();
            w.sdio_slc0_tx_ovf_int_clr().set_bit()
        });
        dma_fence();
        self.receive_error = None;
        Ok(())
    }

    /// Arms one slave-to-host packet on SLC0, without automatic rearming.
    ///
    /// The buffer is word aligned; `length` can end within its last word.
    /// The driver publishes the cumulative length before starting RXLINK.
    /// Function 1 readiness is set separately with [`Self::set_function1_ready`].
    ///
    /// # Errors
    /// Returns the unchanged storage on invalid state, length or memory range.
    /// Both directions must have returned their previous storage before arming.
    #[instability::unstable]
    pub fn arm_transmit(
        &mut self,
        descriptor: &'static mut Descriptor,
        buffer: &'static mut [u32],
        length: usize,
    ) -> Result<(), ArmTransmitError> {
        let error =
            if self.state != State::BusExposed || self.receive.is_some() || self.transmit.is_some()
            {
                Some(Error::InvalidState)
            } else if buffer.is_empty()
                || buffer.len() > 0x3ffc / 4
                || length == 0
                || length > buffer.len() * 4
            {
                Some(Error::InvalidBuffer)
            } else if !crate::soc::is_slice_in_dram(buffer)
                || !crate::soc::is_slice_in_dram(core::slice::from_ref(descriptor))
            {
                Some(Error::UnsupportedMemory)
            } else {
                None
            };
        if let Some(error) = error {
            return Err(ArmTransmitError::new(error, descriptor, buffer));
        }
        let transmit = Transmit::new(descriptor, buffer, length)?;
        let address = transmit.descriptor_address();
        self.transmit = Some(transmit);
        self.transmit_error = None;
        let slc = SLC::regs();
        slc.slcconf0().modify(|_, w| {
            w.sdio_slc0_rx_no_restart_clr().set_bit();
            w.sdio_slc0_rx_loop_test().clear_bit();
            w.sdio_slc0_rx_auto_wrback().set_bit();
            w.sdio_slc0_token_auto_clr().clear_bit()
        });
        slc.slc_rx_dscr_conf()
            .modify(|r, w| unsafe { w.bits(receive::token_no_replace(r.bits())) });
        slc.slcconf1().modify(|_, w| {
            w.sdio_sdio_cmd_hold_en().clear_bit();
            w.sdio_slc0_len_auto_clr().clear_bit();
            w.sdio_slc0_tx_stitch_en().clear_bit();
            w.sdio_slc0_rx_stitch_en().clear_bit()
        });
        slc.slcconf0().modify(|_, w| w.sdio_slc0_rx_rst().set_bit());
        slc.slcconf0()
            .modify(|_, w| w.sdio_slc0_rx_rst().clear_bit());
        self.clear_dma_events(TRANSMIT_EVENTS);
        dma_fence();
        slc.slc0rx_link_addr()
            .write(|w| unsafe { w.sdio_slc0_rxlink_addr().bits(address) });
        // INC_MORE preserves the host's cumulative byte accounting across packets.
        slc.slc0_len_conf().write(|w| unsafe {
            w.sdio_slc0_len_wdata().bits(length as u32);
            w.sdio_slc0_len_inc_more().set_bit()
        });
        dma_fence();
        slc.slc0rx_link()
            .modify(|_, w| w.sdio_slc0_rxlink_start().set_bit());
        self.state = State::Function1Transmitting;
        Ok(())
    }

    /// Polls the outbound packet for EOF, parked DMA and CPU descriptor ownership.
    ///
    /// Returns `Some(length)` once, then `None`. An error retains all storage.
    /// DMA completion does not prove that host software validated the payload.
    ///
    /// # Errors
    /// Reports DMA faults or invalid descriptor write-back. Disable the bus
    /// before reclaiming storage after an error.
    #[instability::unstable]
    pub fn poll_transmitted(&mut self) -> Result<Option<usize>, Error> {
        if let Some(error) = self.transmit_error {
            return Err(error);
        }
        if self.state != State::Function1Transmitting {
            return Ok(None);
        }
        let slc = SLC::regs();
        let event = completion(
            slc.slc0int_raw().read().bits(),
            true,
            slc.slc0rx_link()
                .read()
                .sdio_slc0_rxlink_park()
                .bit_is_set(),
        );
        match event {
            Completion::Pending | Completion::Draining => return Ok(None),
            Completion::Fault => return self.transmit_failed(Error::DmaFault),
            Completion::Complete => {}
        }
        self.clear_function_ready();
        dma_fence();
        // SAFETY: successful EOF, parked DMA and an acquire barrier precede access.
        let result = unsafe { self.transmit.as_mut().unwrap().finish() };
        let length = match result {
            Ok(length) => length,
            Err(error) => return self.transmit_failed(error),
        };
        self.clear_dma_events(TRANSMIT_EVENTS);
        self.mask_dma_interrupts();
        self.state = State::Function1Transmitted;
        Ok(Some(length))
    }

    /// Returns outbound storage after a successful bus disable.
    ///
    /// # Errors
    /// Returns [`Error::InvalidState`] before DMA shutdown is confirmed.
    #[instability::unstable]
    pub fn take_transmit_buffers(
        &mut self,
    ) -> Result<Option<(&'static mut Descriptor, &'static mut [u32])>, Error> {
        if !self.state.dma_stopped() {
            return Err(Error::InvalidState);
        }
        // SAFETY: both stopped states require successful DMA shutdown or no
        // previous transfer. take() prevents double restitution.
        Ok(self.transmit.take().map(|tx| unsafe { tx.into_buffers() }))
    }

    /// Reads outbound DMA status without acknowledging any event.
    #[instability::unstable]
    pub fn transmit_diagnostics(&self) -> TransmitDiagnostics {
        let slc = SLC::regs();
        TransmitDiagnostics {
            descriptor_dw0: self
                .transmit
                .as_ref()
                .map(Transmit::descriptor_word)
                .unwrap_or(0),
            published_length: slc.slc0_length().read().sdio_slc0_len().bits(),
            interrupt_raw: slc.slc0int_raw().read().bits(),
            parked: slc
                .slc0rx_link()
                .read()
                .sdio_slc0_rxlink_park()
                .bit_is_set(),
        }
    }

    /// Reclaims a completed outbound packet while keeping the bus pads routed.
    /// The cumulative published length remains unchanged for the next packet.
    ///
    /// # Errors
    /// Returns [`Error::InvalidState`] before successful transmit completion,
    /// or [`Error::DmaStopTimeout`] if shutdown cannot be confirmed.
    #[instability::unstable]
    pub fn finish_transmit(
        &mut self,
    ) -> Result<(&'static mut Descriptor, &'static mut [u32]), Error> {
        if self.state != State::Function1Transmitted {
            return Err(Error::InvalidState);
        }
        self.mask_dma_interrupts();
        self.stop_transmit_dma()?;
        let transmit = self.transmit.take().unwrap();
        self.state = State::BusExposed;
        // SAFETY: stop_transmit_dma confirmed quiescence and ordered accesses.
        Ok(unsafe { transmit.into_buffers() })
    }

    fn transmit_failed(&mut self, error: Error) -> Result<Option<usize>, Error> {
        self.clear_function_ready();
        self.mask_dma_interrupts();
        self.transmit_error = Some(error);
        self.state = State::Function1TransmitError;
        Err(error)
    }

    fn stop_transmit_dma(&mut self) -> Result<(), Error> {
        if self.transmit.is_none() {
            return Ok(());
        }
        let slc = SLC::regs();
        slc.slc0rx_link()
            .modify(|_, w| w.sdio_slc0_rxlink_stop().set_bit());
        dma_fence();
        let parked = || {
            let link = slc.slc0rx_link().read();
            link.sdio_slc0_rxlink_park().bit_is_set()
                && link.sdio_slc0_rxlink_start().bit_is_clear()
                && link.sdio_slc0_rxlink_stop().bit_is_clear()
        };
        let mut idle = wait_for_idle(parked);
        slc.slcconf0().modify(|_, w| w.sdio_slc0_rx_rst().set_bit());
        dma_fence();
        if !idle {
            idle = wait_for_idle(parked);
        }
        if !idle {
            self.transmit_error = Some(Error::DmaStopTimeout);
            self.state = State::Function1TransmitError;
            return Err(Error::DmaStopTimeout);
        }
        dma_fence();
        slc.slcconf0()
            .modify(|_, w| w.sdio_slc0_rx_rst().clear_bit());
        self.clear_dma_events(TRANSMIT_EVENTS);
        self.transmit_error = None;
        Ok(())
    }

    fn mask_dma_interrupts(&self) {
        SLC::regs()
            .slc0int_ena()
            .modify(|r, w| unsafe { w.bits(r.bits() & !DMA_EVENTS) });
    }

    fn clear_dma_events(&self, mask: u32) {
        // Only documented SLC0 DMA W1C bits. Never acknowledge SLC1 or host vectors.
        SLC::regs()
            .slc0int_clr()
            .write(|w| unsafe { w.bits(mask & DMA_EVENTS) });
    }

    /// Prepares the high speed capability, two fields each written on its own with a
    /// read-modify-write that leaves every other field of the register unchanged:
    ///
    /// - `HINF_CFG_DATA1.HIGHSPEED_ENABLE`: the CCCR advertises high speed support (`SHS`).
    /// - `SLCHOST_CONF.HSPEED_CON_EN`: the sending edge follows the high speed mode (`EHS`) that
    ///   the host selects. The force bits of the same register (`FRC_SDIO11`, `FRC_SDIO20`,
    ///   `FRC_NEG_SAMP`, `FRC_POS_SAMP`, `FRC_QUICK_IN`) are not touched and keep priority.
    ///
    /// Advertising the capability does not make the host use it, and does not prove that the
    /// timing of the bus is valid at a high frequency.
    fn configure_highspeed(&self) {
        HINF::regs()
            .cfg_data1()
            .modify(|_, w| w.highspeed_enable().set_bit());
        SLCHOST::regs()
            .conf()
            .modify(|_, w| w.hspeed_con_en().set_bit());
    }

    fn clear_function_ready(&self) {
        HINF::regs().cfg_data1().modify(|_, w| {
            w.sdio_ioready1().clear_bit();
            w.sdio_ioready2().clear_bit()
        });
    }

    fn release_pads(&self) {
        for pad in self.pads.iter().flatten() {
            pad.init_gpio();
        }
    }
}

static COMPLETION_WAKER: crate::asynch::AtomicWaker = crate::asynch::AtomicWaker::new();

#[crate::handler]
fn slc0_handler() {
    // Preserve RAW until the task validates DMA and descriptor state. Masking
    // stops a level IRQ from repeating while PARK has not caught up with EOF.
    SLC::regs()
        .slc0int_ena()
        .modify(|r, w| unsafe { w.bits(r.bits() & !DMA_EVENTS) });
    COMPLETION_WAKER.wake();
}

struct CompletionWait<'a, 'd> {
    driver: &'a mut SdioSlave<'d, Async>,
    transmitting: bool,
}

impl events::CompletionSource for CompletionWait<'_, '_> {
    fn register(&self, waker: &core::task::Waker) {
        COMPLETION_WAKER.register(waker);
    }

    fn poll_completion(&mut self) -> Result<Option<usize>, Error> {
        let expected = if self.transmitting {
            State::Function1Transmitting
        } else {
            State::Function1Receiving
        };
        let result = if self.transmitting {
            self.driver.poll_transmitted()
        } else {
            self.driver.poll_received()
        };
        match result {
            Ok(None) if self.driver.state != expected => Err(Error::InvalidState),
            result => result,
        }
    }

    fn pending_event(&self) -> bool {
        let mask = if self.transmitting {
            TRANSMIT_EVENTS
        } else {
            RECEIVE_EVENTS
        };
        SLC::regs().slc0int_raw().read().bits() & mask != 0
    }

    fn arm_notification(&self) {
        let mask = if self.transmitting {
            TRANSMIT_EVENTS
        } else {
            RECEIVE_EVENTS
        };
        SLC::regs()
            .slc0int_ena()
            .modify(|r, w| unsafe { w.bits(r.bits() | mask) });
    }

    fn mask_notification(&self) {
        self.driver.mask_dma_interrupts();
    }
}

impl<Dm: DriverMode> Drop for SdioSlave<'_, Dm> {
    fn drop(&mut self) {
        self.mask_dma_interrupts();
        self._slc.disable_ch0_interrupt_on_all_cores();
        if self.state.pads_routed() {
            self.clear_function_ready();
            if self
                .stop_receive_dma()
                .and_then(|_| self.stop_transmit_dma())
                .is_err()
            {
                // Reset stays asserted. Leaking one clock reference and static
                // storage is safer than permitting DMA to access reused memory.
                if let Some(clock) = self._clock.take() {
                    core::mem::forget(clock);
                }
                return;
            }
            self.release_pads();
        }
    }
}

/// Orders CPU memory and peripheral I/O accesses at DMA ownership boundaries.
fn dma_fence() {
    compiler_fence(Ordering::SeqCst);
    // SAFETY: this instruction only orders accesses; it touches no registers or
    // stack memory. No `nomem` option: the compiler must also respect the barrier.
    unsafe {
        core::arch::asm!("fence iorw, iorw", options(nostack));
    }
    compiler_fence(Ordering::SeqCst);
}

impl<Dm: DriverMode> core::fmt::Debug for SdioSlave<'_, Dm> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SdioSlave")
            .field("state", &self.state)
            .field("receive_error", &self.receive_error)
            .field("transmit_error", &self.transmit_error)
            .finish_non_exhaustive()
    }
}

#[cfg(feature = "defmt")]
impl<Dm: DriverMode> defmt::Format for SdioSlave<'_, Dm> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "SdioSlave {{ state: {:?}, receive_error: {:?} }}",
            self.state,
            self.receive_error
        );
    }
}
