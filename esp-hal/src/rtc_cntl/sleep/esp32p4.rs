//! Sleep mode for ESP32-P4X (chip revision v3.x / eco5).
//!
//! Current status: Not implemented. Sleep mode requires PMU eco5 registers
//! which have different layouts from eco4.
//!
//! The original florianL21 sleep code (1064 lines) is preserved as
//! esp32p4_florianl21_original.rs.bak in this directory.
//! It references types and PMU registers that don't exist in current PAC:
//!   - crate::clock::Clock, crate::efuse::Efuse (old API pattern)
//!   - PMU hp_active/hp_modem/hp_sleep register groups
//!
//! TODO(P4X): Implement sleep mode once PMU eco5 registers are validated.
//! Ref: esp-idf pmu_sleep.c, pmu_eco5_struct.h
//!      TRM v0.5 Ch 16 (Low-Power Management)
//!      .investigation/ESP32P4_ECO5_PERIPHERAL_AUDIT.md

// florianL21 original: see esp32p4_florianl21_original.rs.bak (1064 lines)
