//! # GPIO configuration module (ESP32-S3)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32-S3` chip. Pins can be
//! configured as inputs or outputs, and their state can be set or read.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
//!   - `gpio` block:
//!       * Defines the pin configurations for various GPIO pins. Each line represents a pin and its
//!         associated options such as input/output mode, analog capability, and corresponding
//!         functions.
//!   - `analog` block:
//!       * Block defines the analog capabilities of various GPIO pins. Each line represents a pin
//!         and its associated options such as mux selection, function selection, and input enable.
//!   - `enum InputSignal`:
//!       * This enumeration defines input signals for the GPIO mux. Each input signal is assigned a
//!         specific value.
//!   - `enum OutputSignal`:
//!       * This enumeration defines output signals for the GPIO mux. Each output signal is assigned
//!         a specific value.
//!
//! Functions to read the interrupt status and NMI status
//! registers for both the `PRO CPU` and `APP CPU`. The implementation uses the
//! `gpio` peripheral to access the appropriate registers.
