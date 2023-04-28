//! ECC Accelerator
//!
//! # Overview
//!
//! Elliptic Curve Cryptography (ECC) is an approach to public-key cryptography
//! based on the algebraic structure of elliptic curves. ECC allows smaller
//! keys compared to RSA cryptography while providing equivalent security.
//!
//! ECC Accelerator can complete various calculation based on different
//! elliptic curves, thus accelerating ECC algorithm and ECC-derived
//! algorithms (such as ECDSA).
//!
//! # Main features
//!
//! ECC Accelerator supports:
//! - Two different elliptic curves, namely P-192 and P-256 defined in FIPS 186-3.
//! - Seven working modes.
//! - Interrupt upon completion of calculation.
//!
//! # Availability on ESP32 family
//!
//! The accelerator is available on ESP32-C2 and ESP32-C6.
//! 
//! # Data representation
//! 
//! Inputs of the ECC hardware accelerator must be provided in big-endian representation.
//! The driver handles the inner representation of the blocks.

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::ECC,
    reg_access::AlignmentHelper,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

pub struct Ecc<'d> {
    ecc: PeripheralRef<'d, ECC>,
    alignment_helper: AlignmentHelper,
}

/// ECC interface error
#[derive(Debug)]
pub enum Error {
    /// It means the purpose of the selected block does not match the
    /// configured key purpose and the calculation will not proceed.
    SizeMismatchCurve,
    /// It means that the point is not on the curve.
    PointNotOnSelectedCurve,
}

pub enum EllipticCurve {
    P192 = 0,
    P256 = 1,
}

enum WorkMode {
    PointMultiMode = 0,
    #[cfg(esp32c2)]
    DivisionMode = 1,
    PointVerif = 2,
    PointVerifMulti = 3,
    JacobianPointMulti = 4,
    JacobianPointVerif = 6,
    PointVerifJacobianMulti = 7,
}

impl<'d> Ecc<'d> {
    pub fn new(
        ecc: impl Peripheral<P = ECC> + 'd,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(ecc);

        peripheral_clock_control.enable(PeripheralEnable::Ecc);

        Self {
            ecc,
            alignment_helper: AlignmentHelper::default(),
        }
    }

    pub fn free(self) -> PeripheralRef<'d, ECC> {
        self.ecc
    }

    pub fn reset(&mut self) {
        self.ecc.mult_conf.reset()
    }

    /// # Base point multiplication
    ///
    /// Base Point Multiplication can be represented as:
    /// (Q_x, Q_y) = k * (P_x, P_y)
    /// 
    /// # Error
    /// 
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    pub fn affine_point_multiplication(
        &mut self,
        curve: &EllipticCurve,
        k: &[u8],
        x: &mut [u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if k.len() != 24 || x.len() != 24 || y.len() != 24 {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if k.len() != 32 || x.len() != 32 || y.len() != 32 {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };
        let mode = WorkMode::PointMultiMode;

        let mut tmp = [0_u8; 32];
        self.reverse_words(k, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.k_mem[0], tmp.as_ref(), 8);
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.px_mem[0], tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.py_mem[0], tmp.as_ref(), 8);

        self.ecc.mult_conf.write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        while self.is_busy() {}

        self.alignment_helper
            .volatile_read_regset(&self.ecc.px_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), x);
        self.alignment_helper
            .volatile_read_regset(&self.ecc.py_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), y);

        Ok(())
    }

    #[cfg(esp32c2)]
    /// # Finite Field Division
    ///
    /// Finite Field Division can be represented as:
    /// Result = P_y * k^{−1} mod p
    /// 
    /// # Error
    /// 
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    pub fn finite_field_division(
        &mut self,
        curve: &EllipticCurve,
        k: &[u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if k.len() != 24 || y.len() != 24 {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if k.len() != 32 || y.len() != 32 {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };
        let mode = WorkMode::DivisionMode;

        let mut tmp = [0_u8; 32];
        self.reverse_words(k, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.k_mem[0], tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.py_mem[0], tmp.as_ref(), 8);

        self.ecc.mult_conf.write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        while self.is_busy() {}

        self.alignment_helper
            .volatile_read_regset(&self.ecc.py_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), y);

        Ok(())
    }

    /// # Base Point Verification
    ///
    /// Base Point Verification can be used to verify if a point (Px, Py) is
    /// on a selected elliptic curve.
    /// 
    /// # Error
    /// 
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    /// 
    /// This function will return an error if the point is not on the selected
    /// elliptic curve.
    pub fn affine_point_verification(
        &mut self,
        curve: &EllipticCurve,
        x: &[u8],
        y: &[u8],
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if x.len() != 24 || y.len() != 24 {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if x.len() != 32 || y.len() != 32 {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };
        let mode = WorkMode::PointVerif;

        let mut tmp = [0_u8; 32];
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.px_mem[0], tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.py_mem[0], tmp.as_ref(), 8);

        self.ecc.mult_conf.write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        while self.is_busy() {}

        if !self.ecc.mult_conf.read().verification_result().bit() {
            self.ecc.mult_conf.reset();
            return Err(Error::PointNotOnSelectedCurve);
        }

        Ok(())
    }

    /// # Base Point Verification + Base Point Multiplication
    ///
    /// In this working mode, ECC first verifies if Point (P_x, P_y) is on the
    /// selected elliptic curve or not. If yes, then perform the multiplication:
    /// (Q_x, Q_y) = k * (P_x, P_y)
    /// 
    /// # Error
    /// 
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    /// 
    /// This function will return an error if the point is not on the selected
    /// elliptic curve.
    pub fn affine_point_verification_multiplication(
        &mut self,
        curve: &EllipticCurve,
        k: &[u8],
        x: &mut [u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if k.len() != 24 || x.len() != 24 || y.len() != 24 {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if k.len() != 32 || x.len() != 32 || y.len() != 32 {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };
        let mode = WorkMode::PointVerifMulti;

        let mut tmp = [0_u8; 32];
        self.reverse_words(k, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.k_mem[0], tmp.as_ref(), 8);
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.px_mem[0], tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.py_mem[0], tmp.as_ref(), 8);

        self.ecc.mult_conf.write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        while self.is_busy() {}

        if !self.ecc.mult_conf.read().verification_result().bit() {
            self.ecc.mult_conf.reset();
            return Err(Error::PointNotOnSelectedCurve);
        }

        self.alignment_helper
            .volatile_read_regset(&self.ecc.px_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), x);
        self.alignment_helper
            .volatile_read_regset(&self.ecc.py_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), y);

        Ok(())
    }

    /// # Jacobian Point Multiplication
    ///
    /// Jacobian Point Multiplication can be represented as:
    /// (Q_x, Q_y, Q_z) = k * (P_x, P_y, 1)
    /// 
    /// # Error
    /// 
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    pub fn jacobian_point_multication(
        &mut self,
        curve: &EllipticCurve,
        k: &mut [u8],
        x: &mut [u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if k.len() != 24 || x.len() != 24 || y.len() != 24 {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if k.len() != 32 || x.len() != 32 || y.len() != 32 {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };
        let mode = WorkMode::JacobianPointMulti;

        let mut tmp = [0_u8; 32];
        self.reverse_words(k, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.k_mem[0], tmp.as_ref(), 8);
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.px_mem[0], tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.py_mem[0], tmp.as_ref(), 8);

        self.ecc.mult_conf.write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        while self.is_busy() {}

        self.alignment_helper
            .volatile_read_regset(&self.ecc.px_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), x);
        self.alignment_helper
            .volatile_read_regset(&self.ecc.py_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), y);
        self.alignment_helper
            .volatile_read_regset(&self.ecc.k_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), k);

        Ok(())
    }

    /// # Jacobian Point Verification
    ///
    /// Jacobian Point Verification can be used to verify if a point (Q_x, Q_y,
    /// Q_z) is on a selected elliptic curve.
    /// 
    /// # Error
    /// 
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    /// 
    /// This function will return an error if the point is not on the selected
    /// elliptic curve.
    pub fn jacobian_point_verification(
        &mut self,
        curve: &EllipticCurve,
        x: &[u8],
        y: &[u8],
        z: &[u8],
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if x.len() != 24 || y.len() != 24 || z.len() != 24 {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if x.len() != 32 || y.len() != 32 || z.len() != 32 {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };
        let mode = WorkMode::JacobianPointVerif;

        let mut tmp = [0_u8; 32];
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.px_mem[0], tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.py_mem[0], tmp.as_ref(), 8);
        self.reverse_words(z, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.k_mem[0], tmp.as_ref(), 8);

        self.ecc.mult_conf.write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        while self.is_busy() {}

        if !self.ecc.mult_conf.read().verification_result().bit() {
            self.ecc.mult_conf.reset();
            return Err(Error::PointNotOnSelectedCurve);
        }

        Ok(())
    }

    /// # Base Point Verification + Jacobian Point Multiplication
    ///
    /// In this working mode, ECC first verifies if Point (Px, Py) is on the
    /// selected elliptic curve or not. If yes, then perform the multiplication:
    /// (Q_x, Q_y, Q_z) = k * (P_x, P_y, 1)
    /// 
    /// # Error
    /// 
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    /// 
    /// This function will return an error if the point is not on the selected
    /// elliptic curve.
    pub fn affine_point_verification_jacobian_multiplication(
        &mut self,
        curve: &EllipticCurve,
        k: &mut [u8],
        x: &mut [u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if k.len() != 24 || x.len() != 24 || y.len() != 24 {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if k.len() != 32 || x.len() != 32 || y.len() != 32 {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };
        let mode = WorkMode::PointVerifJacobianMulti;

        let mut tmp = [0_u8; 32];
        self.reverse_words(k, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.k_mem[0], tmp.as_ref(), 8);
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.px_mem[0], tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(&mut self.ecc.py_mem[0], tmp.as_ref(), 8);

        self.ecc.mult_conf.write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        while self.is_busy() {}

        if !self.ecc.mult_conf.read().verification_result().bit() {
            self.ecc.mult_conf.reset();
            return Err(Error::PointNotOnSelectedCurve);
        }

        self.alignment_helper
            .volatile_read_regset(&self.ecc.px_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), x);
        self.alignment_helper
            .volatile_read_regset(&self.ecc.py_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), y);
        self.alignment_helper
            .volatile_read_regset(&self.ecc.k_mem[0], &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), k);

        Ok(())
    }

    fn is_busy(&self) -> bool {
        self.ecc.mult_conf.read().start().bit_is_set()
    }

    fn reverse_words(&self, src: &[u8], dst: &mut [u8]) {
        let n = core::cmp::min(src.len(), dst.len());
        let nsrc = if src.len() > n {
            src.split_at(n).0
        } else {
            src
        };
        let ndst = if dst.len() > n {
            dst.split_at_mut(n).0
        } else {
            dst
        };
        for (a, b) in nsrc.chunks_exact(4).zip(ndst.rchunks_exact_mut(4)) {
            b.copy_from_slice(&u32::from_be_bytes(a.try_into().unwrap()).to_ne_bytes());
        }
    }
}
