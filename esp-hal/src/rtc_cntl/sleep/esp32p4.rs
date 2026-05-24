//! Sleep mode for ESP32-P4X (chip revision v3.x / eco5).
//!
//! Current status: Not implemented. Sleep mode requires PMU eco5 registers
//! which have different layouts from eco4.
//!
//! The original florianL21 sleep code
//! It references types and PMU registers that don't exist in current PAC:
//!   - crate::clock::Clock, crate::efuse::Efuse (old API pattern)
//!   - PMU hp_active/hp_modem/hp_sleep register groups

// TODO: Implement sleep mode once PMU eco5 registers are validated.
// TODO: reference florianL21's implementation
