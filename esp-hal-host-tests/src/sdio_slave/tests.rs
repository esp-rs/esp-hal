use super::state::*;

const STATES: [State; 9] = [
    State::ControllerReady,
    State::SessionInvalidated,
    State::BusExposed,
    State::Function1Receiving,
    State::Function1Received,
    State::Function1ReceiveError,
    State::Function1Transmitting,
    State::Function1Transmitted,
    State::Function1TransmitError,
];

#[test]
fn function_ready_is_only_allowed_with_an_armed_transfer() {
    for s in STATES {
        assert_eq!(
            s.function_ready_allowed(),
            matches!(s, State::Function1Receiving | State::Function1Transmitting),
            "{s:?}"
        );
    }
}

#[test]
fn pads_are_released_in_both_stopped_states() {
    assert!(!State::ControllerReady.pads_routed());
    assert!(!State::SessionInvalidated.pads_routed());
    assert!(State::BusExposed.pads_routed());
    assert!(State::Function1Receiving.pads_routed());
}

#[test]
fn transitions_are_limited_to_the_documented_graph() {
    use State::*;
    let allowed = [
        (ControllerReady, BusExposed),
        (BusExposed, ControllerReady),
        (BusExposed, Function1Receiving),
        (Function1Receiving, Function1Received),
        (Function1Receiving, Function1ReceiveError),
        (Function1Received, ControllerReady),
        (Function1Received, BusExposed),
        (Function1Received, Function1ReceiveError),
        (Function1ReceiveError, SessionInvalidated),
        (Function1Receiving, SessionInvalidated),
        (BusExposed, Function1Transmitting),
        (Function1Transmitting, Function1Transmitted),
        (Function1Transmitting, Function1TransmitError),
        (Function1Transmitted, Function1TransmitError),
        (Function1Transmitting, SessionInvalidated),
        (Function1Transmitted, ControllerReady),
        (Function1Transmitted, BusExposed),
        (Function1TransmitError, SessionInvalidated),
    ];
    for from in STATES {
        for to in STATES {
            assert_eq!(
                from.can_transition_to(to),
                allowed.contains(&(from, to)),
                "{from:?} -> {to:?}"
            );
        }
    }
}

#[test]
fn function_state_distinguishes_enabled_from_ready() {
    assert_eq!(function_state(false, false), FunctionState::Disabled);
    assert_eq!(function_state(true, false), FunctionState::EnabledByHost);
    assert_eq!(function_state(true, true), FunctionState::Ready);
}

#[test]
fn ready_without_host_enable_is_not_usable() {
    assert_eq!(function_state(false, true), FunctionState::Disabled);
}

#[test]
fn only_an_armed_transfer_can_yield_a_ready_function() {
    for s in STATES {
        for enabled in [false, true] {
            let st = function_state(enabled, s.function_ready_allowed());
            assert_eq!(
                st == FunctionState::Ready,
                enabled && matches!(s, State::Function1Receiving | State::Function1Transmitting),
                "{s:?} enabled={enabled}"
            );
        }
    }
}

#[test]
fn error_is_comparable() {
    assert_eq!(Error::InvalidState, Error::InvalidState);
    assert_ne!(Error::InvalidState, Error::InvalidBuffer);
}

#[test]
fn stop_waits_for_idle_and_reports_timeout() {
    let mut polls = 0;
    assert!(wait_for_idle(|| {
        polls += 1;
        polls == 4
    }));
    assert_eq!(polls, 4);
    let mut polls = 0;
    assert!(!wait_for_idle(|| {
        polls += 1;
        false
    }));
    assert_eq!(polls, 10_000);
}

#[test]
fn token_no_replace_sets_bit_zero_without_changing_other_bits() {
    for bits in [0, 2, 0x0203_701a, u32::MAX] {
        let updated = super::receive::token_no_replace(bits);
        assert_eq!(updated & 1, 1);
        assert_eq!(updated & !1, bits & !1);
    }
}

#[test]
fn receive_cancellation_cannot_publish_a_second_credit_in_the_same_session() {
    let mut state = State::BusExposed;
    let mut announced = 0;
    assert!(state.can_transition_to(State::Function1Receiving));
    state = State::Function1Receiving;
    announced += 1;
    let consumed = 0;
    // This is the exact state selection used by disable_bus after stop succeeds.
    state = state.after_disable();
    assert_eq!(state, State::SessionInvalidated);
    assert!(state.dma_stopped()); // Memory restitution is still permitted.
    assert!(!state.pads_routed());
    assert!(!state.can_transition_to(State::BusExposed));
    assert!(!state.can_transition_to(State::Function1Receiving));
    assert_eq!(announced - consumed, 1); // No second arm can increment the counter.
}

#[test]
fn outbound_cancellation_requires_a_new_session_too() {
    for state in [
        State::Function1Transmitting,
        State::Function1TransmitError,
        State::Function1Receiving,
        State::Function1ReceiveError,
    ] {
        assert!(!state.dma_stopped());
        let stopped = state.after_disable();
        assert!(state.can_transition_to(stopped));
        assert_eq!(stopped, State::SessionInvalidated);
        for next in STATES {
            assert!(!stopped.can_transition_to(next));
        }
    }
}

#[test]
fn completed_packets_can_be_reclaimed_and_explicitly_rearmed() {
    for state in [State::Function1Received, State::Function1Transmitted] {
        assert!(state.can_transition_to(State::BusExposed));
        assert_eq!(state.after_disable(), State::ControllerReady);
        assert!(State::ControllerReady.can_transition_to(State::BusExposed));
        assert!(State::BusExposed.can_transition_to(State::Function1Receiving));
        assert!(State::BusExposed.can_transition_to(State::Function1Transmitting));
    }
}

#[test]
fn storage_restitution_is_permitted_only_in_confirmed_stopped_states() {
    for state in STATES {
        assert_eq!(
            state.dma_stopped(),
            matches!(state, State::ControllerReady | State::SessionInvalidated)
        );
    }
}

#[test]
fn cmd5_voltage_argument_selects_only_ocr_bits_twenty_and_twenty_one() {
    use super::voltage::voltage_window;
    assert_eq!(voltage_window(1 << 20), 0x20);
    assert_eq!(voltage_window(1 << 21), 0x40);
    assert_eq!(voltage_window(u32::MAX), 0x60);
    for bit in 0..32 {
        let expected = if matches!(bit, 20 | 21) { 1 << bit } else { 0 };
        assert_eq!((voltage_window(1 << bit) as u32) << 15, expected);
    }
}

#[test]
fn bus_enable_rejects_every_incomplete_pad_configuration() {
    for mask in 0u8..64 {
        let pads = core::array::from_fn(|index| (mask & (1 << index) != 0).then_some(index));
        let expected = if mask == 63 {
            Ok(())
        } else {
            Err(Error::MissingPins)
        };
        assert_eq!(
            validate_bus_configuration(State::ControllerReady, &pads),
            expected,
            "{mask:06b}"
        );
    }
}

#[test]
fn pin_configuration_never_reopens_an_active_or_invalidated_session() {
    for state in STATES {
        if state == State::ControllerReady {
            continue;
        }
        for mask in 0u8..64 {
            let pads = core::array::from_fn(|index| (mask & (1 << index) != 0).then_some(index));
            let error = if state == State::SessionInvalidated {
                Error::SessionInvalidated
            } else {
                Error::InvalidState
            };
            assert_eq!(validate_bus_configuration(state, &pads), Err(error));
        }
    }
}

#[test]
fn highspeed_bits_are_the_documented_ones() {
    // ESP32-C6 TRM chapter 34, HINF_CFG_DATA1_REG: HIGHSPEED_ENABLE is bit 2, HIGHSPEED_MODE bit 3.
    assert_eq!(CFG_DATA1_HIGHSPEED_ENABLE, 0b0100);
    assert_eq!(CFG_DATA1_HIGHSPEED_MODE, 0b1000);
}

#[test]
fn highspeed_is_advertised_only_by_the_enable_bit() {
    assert!(!highspeed_advertised(0));
    assert!(highspeed_advertised(CFG_DATA1_HIGHSPEED_ENABLE));
    // the read-only mode bit and the neighbours are not an advertisement
    assert!(!highspeed_advertised(CFG_DATA1_HIGHSPEED_MODE));
    assert!(!highspeed_advertised(!CFG_DATA1_HIGHSPEED_ENABLE));
    assert!(highspeed_advertised(u32::MAX));
}

#[test]
fn highspeed_is_selected_only_by_the_mode_bit() {
    assert!(!highspeed_selected(0));
    assert!(highspeed_selected(CFG_DATA1_HIGHSPEED_MODE));
    assert!(!highspeed_selected(CFG_DATA1_HIGHSPEED_ENABLE));
    assert!(!highspeed_selected(!CFG_DATA1_HIGHSPEED_MODE));
}
