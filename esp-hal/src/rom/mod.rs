//! # ESP ROM libraries
//!
//! ## Overview
//! The `rom` driver provides functionality related to the ROM (Read-Only
//! Memory) on ESP chips. It includes implementations for the [CRC (Cyclic
//! Redundancy Check)] and [MD5 (Message Digest 5)] algorithms.
//!
//! The driver's functionality allows users to perform CRC calculations and MD5
//! hashing using the ROM functions provided by the ESP chip. This can be useful
//! for various applications that require data integrity checks or cryptographic
//! operations.
//!
//! It uses `CRC` error-checking techniques to detect changes in data during
//! transmission or storage.
//!
//! This module also implements the `MD5` algorithm, which is widely used for
//! cryptographic hash function. It's commonly used to verify data integrity and
//! to check whether the data has been modified.
//!
//! Safe abstractions to the additional libraries provided in the ESP's
//! Read-Only Memory.
//!
//! [CRC (Cyclic Redundancy Check)]: ./crc/index.html
//! [MD5 (Message Digest 5)]: ./md5/index.html

pub use esp_rom_sys::rom::*;

pub(crate) mod regi2c;
