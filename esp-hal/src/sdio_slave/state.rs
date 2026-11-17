//! Register-free state and invariant logic of the SDIO slave controller.
//!
//! Nothing in this file touches hardware, so it is compiled by the host tests
//! as well (see `esp-hal-host-tests`). Keep it free of `esp-hal` imports.

/// State of the controller, as tracked by the driver.
///
/// The states a host can observe are modeled separately, see
/// [`FunctionState`]: whether a function has been enabled by the host, or is
/// ready, is hardware state that the driver observes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum State {
    /// The SDIO block is clocked and out of reset. The SDIO pads are not routed
    /// to the controller, so the host cannot see the card.
    ControllerReady,
    /// DMA shutdown succeeded and the pads are released, but an interrupted
    /// packet leaves peer byte or credit accounting ambiguous. Recover storage,
    /// drop this driver, reset both peers' accounting and create a new session.
    SessionInvalidated,
    /// The SDIO pads are routed to the controller and the function-ready bits
    /// `IOREADY1/2` are cleared. The clock is running and the block is out of
    /// reset.
    ///
    /// This state only describes what the driver has configured. It does **not**
    /// mean that a host has enumerated the card. No function is ready to transfer
    /// data.
    BusExposed,
    /// The SLC0 receive DMA (host to slave) is armed with a linked list and
    /// a buffer, and the driver may declare function 1 ready. The pads are routed.
    ///
    /// The opposite direction cannot be armed until this packet is reclaimed.
    Function1Receiving,
    /// One packet completed. The receive buffer is owned by the CPU.
    Function1Received,
    /// A DMA error or stop timeout. Resources remain owned by the driver.
    Function1ReceiveError,
    /// One slave-to-host packet is published and its DMA owns the storage.
    Function1Transmitting,
    /// The slave-to-host packet reached EOF and its DMA is parked.
    Function1Transmitted,
    /// An outbound DMA fault or stop timeout. Storage remains owned.
    Function1TransmitError,
}

/// State of one SDIO I/O function, as observable on the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FunctionState {
    /// The host has not enabled the function (CCCR `IOEx` is 0).
    Disabled,
    /// The host has enabled the function but the slave has not declared it
    /// ready (CCCR `IORx` is 0).
    EnabledByHost,
    /// The function is enabled and declared ready to transfer.
    ///
    /// Function 1 can enter this state after either DMA direction is armed.
    Ready,
}

/// Error returned by state-dependent operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The operation is not allowed in the current [`State`].
    InvalidState,
    /// One or more native SDIO pads have not been assigned.
    MissingPins,
    /// An interrupted transfer requires a new controller/host session.
    SessionInvalidated,
    /// A DMA buffer is empty or larger than 16380 bytes.
    InvalidBuffer,
    /// A descriptor or buffer is outside internal DMA-accessible RAM.
    UnsupportedMemory,
    /// DMA write-back has an invalid owner, EOF, size, or length.
    InvalidDescriptor,
    /// DMA reported an overflow, underflow, or descriptor error.
    DmaFault,
    /// DMA did not become idle within the polling bound.
    DmaStopTimeout,
}

impl State {
    /// Returns whether the driver may move from `self` to `next`.
    pub const fn can_transition_to(self, next: State) -> bool {
        matches!(
            (self, next),
            (State::ControllerReady, State::BusExposed)
                | (State::BusExposed, State::ControllerReady)
                | (State::BusExposed, State::Function1Receiving)
                | (State::Function1Receiving, State::Function1Received)
                | (State::Function1Receiving, State::Function1ReceiveError)
                | (State::Function1Received, State::ControllerReady)
                | (State::Function1Received, State::BusExposed)
                | (State::Function1Received, State::Function1ReceiveError)
                | (State::Function1ReceiveError, State::SessionInvalidated)
                | (State::Function1Receiving, State::SessionInvalidated)
                | (State::BusExposed, State::Function1Transmitting)
                | (State::Function1Transmitting, State::Function1Transmitted)
                | (State::Function1Transmitting, State::Function1TransmitError)
                | (State::Function1Transmitted, State::Function1TransmitError)
                | (State::Function1Transmitting, State::SessionInvalidated)
                | (State::Function1Transmitted, State::ControllerReady)
                | (State::Function1Transmitted, State::BusExposed)
                | (State::Function1TransmitError, State::SessionInvalidated)
        )
    }

    /// Returns whether the SDIO pads are routed to the controller in this state.
    pub const fn pads_routed(self) -> bool {
        !self.dma_stopped()
    }

    /// This classification is used only after both hardware stop checks succeed.
    pub(super) const fn after_disable(self) -> Self {
        match self {
            Self::Function1Receiving
            | Self::Function1ReceiveError
            | Self::Function1Transmitting
            | Self::Function1TransmitError
            | Self::SessionInvalidated => Self::SessionInvalidated,
            _ => Self::ControllerReady,
        }
    }

    pub(super) const fn dma_stopped(self) -> bool {
        matches!(self, Self::ControllerReady | Self::SessionInvalidated)
    }

    /// Returns whether the driver may declare a function ready (`IOREADY1`) in this
    /// state. Advertising readiness requires an armed DMA direction.
    pub const fn function_ready_allowed(self) -> bool {
        matches!(
            self,
            State::Function1Receiving | State::Function1Transmitting
        )
    }
}

/// Checks the complete bus configuration before any register or pad access.
pub(super) fn validate_bus_configuration<P>(
    state: State,
    pads: &[Option<P>; 6],
) -> Result<(), Error> {
    if state == State::SessionInvalidated {
        return Err(Error::SessionInvalidated);
    }
    if state != State::ControllerReady {
        return Err(Error::InvalidState);
    }
    if pads.iter().any(Option::is_none) {
        return Err(Error::MissingPins);
    }
    Ok(())
}

/// Derives the observable state of a function from the two hardware bits.
///
/// `ready` without `enabled` is reported as [`FunctionState::Disabled`]: a
/// function the host has not enabled cannot be used.
pub const fn function_state(enabled_by_host: bool, ready: bool) -> FunctionState {
    match (enabled_by_host, ready) {
        (false, _) => FunctionState::Disabled,
        (true, false) => FunctionState::EnabledByHost,
        (true, true) => FunctionState::Ready,
    }
}

/// Bounds polling without interpreting a request to stop as confirmation.
pub(super) fn wait_for_idle(mut parked: impl FnMut() -> bool) -> bool {
    for _ in 0..10_000 {
        if parked() {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::InvalidState => "operation is not allowed in this state",
            Self::MissingPins => "all six native SDIO pads must be assigned",
            Self::SessionInvalidated => {
                "transfer interrupted; create a new session and resynchronize the host"
            }
            Self::InvalidBuffer => {
                "buffer must contain 1..=4095 words and transmit length must fit"
            }
            Self::UnsupportedMemory => "DMA storage must be in internal DRAM",
            Self::InvalidDescriptor => "invalid DMA descriptor write-back",
            Self::DmaFault => "DMA overflow or descriptor fault",
            Self::DmaStopTimeout => "DMA stop did not reach idle within the polling bound",
        })
    }
}

impl core::error::Error for Error {}

/// `HIGHSPEED_ENABLE` in `HINF_CFG_DATA1_REG` (bit 2, read/write, reset value 0): makes the CCCR
/// advertise high speed support (`SHS`) to the host (ESP32-C6 TRM, chapter 34).
pub const CFG_DATA1_HIGHSPEED_ENABLE: u32 = 1 << 2;

/// `HIGHSPEED_MODE` in `HINF_CFG_DATA1_REG` (bit 3, read only): reflects the high speed mode
/// (`EHS`) that the host selected in the CCCR.
pub const CFG_DATA1_HIGHSPEED_MODE: u32 = 1 << 3;

/// Returns whether `cfg_data1` makes the card advertise high speed support (`SHS`).
pub const fn highspeed_advertised(cfg_data1: u32) -> bool {
    cfg_data1 & CFG_DATA1_HIGHSPEED_ENABLE != 0
}

/// Returns whether the host has selected high speed (`EHS`) according to `cfg_data1`.
pub const fn highspeed_selected(cfg_data1: u32) -> bool {
    cfg_data1 & CFG_DATA1_HIGHSPEED_MODE != 0
}
