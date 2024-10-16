//! # Elliptic Curve Cryptography (ECC) Accelerator
//!
//! ## Overview
//!
//! Elliptic Curve Cryptography (ECC) is an approach to public-key cryptography
//! based on the algebraic structure of elliptic curves. ECC allows smaller
//! keys compared to RSA cryptography while providing equivalent security.
//!
//! ECC Accelerator can complete various calculation based on different
//! elliptic curves, thus accelerating ECC algorithm and ECC-derived
//! algorithms (such as ECDSA).
//!
//! ## Configuration
//! ECC Accelerator supports:
//! - Two different elliptic curves, namely P-192 and P-256 defined in FIPS
//!   186-3.
//! - Seven working modes.
//! - Interrupt upon completion of calculation.
//!
//! Inputs of the ECC hardware accelerator must be provided in big-endian
//! representation. The driver handles the inner representation of the blocks.
//!
//! ## Examples
//! Visit the [ECC] test for an example of using the ECC Accelerator.
//!
//! [ECC]: https://github.com/esp-rs/esp-hal/blob/main/hil-test/tests/ecc.rs

use core::marker::PhantomData;

use crate::{
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::ECC,
    reg_access::{AlignmentHelper, SocDependentEndianess},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
    InterruptConfigurable,
};

/// The ECC Accelerator driver instance
pub struct Ecc<'d, DM: crate::Mode> {
    ecc: PeripheralRef<'d, ECC>,
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    phantom: PhantomData<DM>,
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

/// Represents supported elliptic curves for cryptographic operations.
pub enum EllipticCurve {
    /// The P-192 elliptic curve, a 192-bit curve.
    P192 = 0,
    /// The P-256 elliptic curve. a 256-bit curve.
    P256 = 1,
}

#[derive(Clone)]
/// Represents the operational modes for elliptic curve or modular arithmetic
/// computations.
pub enum WorkMode {
    /// Point multiplication mode.
    PointMultiMode          = 0,
    #[cfg(esp32c2)]
    /// Division mode.
    DivisionMode            = 1,
    /// Point verification mode.
    PointVerif              = 2,
    /// Point verification and multiplication mode.
    PointVerifMulti         = 3,
    /// Jacobian point multiplication mode.
    JacobianPointMulti      = 4,
    #[cfg(esp32h2)]
    /// Point addition mode.
    PointAdd                = 5,
    /// Jacobian point verification mode.
    JacobianPointVerif      = 6,
    /// Point verification and multiplication in Jacobian coordinates.
    PointVerifJacobianMulti = 7,
    #[cfg(esp32h2)]
    /// Modular addition mode.
    ModAdd                  = 8,
    #[cfg(esp32h2)]
    /// Modular subtraction mode.
    ModSub                  = 9,
    #[cfg(esp32h2)]
    /// Modular multiplication mode.
    ModMulti                = 10,
    #[cfg(esp32h2)]
    /// Modular division mode.
    ModDiv                  = 11,
}

impl<'d> Ecc<'d, crate::Blocking> {
    /// Create a new instance in [crate::Blocking] mode.
    pub fn new(ecc: impl Peripheral<P = ECC> + 'd) -> Self {
        crate::into_ref!(ecc);

        PeripheralClockControl::reset(PeripheralEnable::Ecc);
        PeripheralClockControl::enable(PeripheralEnable::Ecc);

        Self {
            ecc,
            alignment_helper: AlignmentHelper::default(),
            phantom: PhantomData,
        }
    }
}

impl crate::private::Sealed for Ecc<'_, crate::Blocking> {}

impl InterruptConfigurable for Ecc<'_, crate::Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(crate::peripherals::Interrupt::ECC, handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::ECC, handler.priority())
                .unwrap();
        }
    }
}

impl<DM: crate::Mode> Ecc<'_, DM> {
    /// Resets the ECC peripheral.
    pub fn reset(&mut self) {
        self.ecc.mult_conf().reset()
    }

    /// # Base point multiplication
    ///
    /// Base Point Multiplication can be represented as:
    /// (Q_x, Q_y) = k * (P_x, P_y)
    ///
    /// Output is stored in `x` and `y`.
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
            .volatile_write_regset(self.ecc.k_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.px_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), tmp.as_ref(), 8);

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        // wait for interrupt
        while self.is_busy() {}

        self.alignment_helper
            .volatile_read_regset(self.ecc.px_mem(0).as_ptr(), &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), x);
        self.alignment_helper
            .volatile_read_regset(self.ecc.py_mem(0).as_ptr(), &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), y);

        Ok(())
    }

    /// # Finite Field Division
    ///
    /// Finite Field Division can be represented as:
    /// Result = P_y * k^{−1} mod p
    ///
    /// Output is stored in `y`.
    ///
    /// # Error
    ///
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    #[cfg(esp32c2)]
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
            .volatile_write_regset(self.ecc.k_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), tmp.as_ref(), 8);

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        // wait for interrupt
        while self.is_busy() {}

        self.alignment_helper
            .volatile_read_regset(self.ecc.py_mem(0).as_ptr(), &mut tmp, 8);
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
            .volatile_write_regset(self.ecc.px_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), tmp.as_ref(), 8);

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        // wait for interrupt
        while self.is_busy() {}

        if !self.ecc.mult_conf().read().verification_result().bit() {
            self.ecc.mult_conf().reset();
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
    /// Output is stored in `x` and `y`.
    ///
    /// # Error
    ///
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    ///
    /// This function will return an error if the point is not on the selected
    /// elliptic curve.
    #[cfg(not(esp32h2))]
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
            .volatile_write_regset(self.ecc.k_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.px_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), tmp.as_ref(), 8);

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        // wait for interrupt
        while self.is_busy() {}

        if !self.ecc.mult_conf().read().verification_result().bit() {
            self.ecc.mult_conf().reset();
            return Err(Error::PointNotOnSelectedCurve);
        }

        self.alignment_helper
            .volatile_read_regset(self.ecc.px_mem(0).as_ptr(), &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), x);
        self.alignment_helper
            .volatile_read_regset(self.ecc.py_mem(0).as_ptr(), &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), y);

        Ok(())
    }

    /// # Base Point Verification + Base Point Multiplication
    ///
    /// In this working mode, ECC first verifies if Point (P_x, P_y) is on the
    /// selected elliptic curve or not. If yes, then perform the multiplication:
    /// (Q_x, Q_y) = (J_x, J_y, J_z) = k * (P_x, P_y)
    ///
    /// The affine point representation output is stored in `px` and `py`.
    /// The Jacobian point representation output is stored in `qx`, `qy`, and
    /// `qz`.
    ///
    /// # Error
    ///
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    ///
    /// This function will return an error if the point is not on the selected
    /// elliptic curve.
    #[allow(clippy::too_many_arguments)]
    #[cfg(esp32h2)]
    pub fn affine_point_verification_multiplication(
        &mut self,
        curve: &EllipticCurve,
        k: &[u8],
        px: &mut [u8],
        py: &mut [u8],
        qx: &mut [u8],
        qy: &mut [u8],
        qz: &mut [u8],
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if k.len() != 24 || px.len() != 24 || py.len() != 24 {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if k.len() != 32 || px.len() != 32 || py.len() != 32 {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };
        let mode = WorkMode::PointVerifMulti;

        let mut tmp = [0_u8; 32];
        self.reverse_words(k, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.k_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(px, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.px_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(py, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), tmp.as_ref(), 8);

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        // wait for interrupt
        while self.is_busy() {}

        if !self.ecc.mult_conf().read().verification_result().bit() {
            self.ecc.mult_conf().reset();
            return Err(Error::PointNotOnSelectedCurve);
        }

        self.alignment_helper
            .volatile_read_regset(self.ecc.px_mem(0).as_ptr(), &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), px);
        self.alignment_helper
            .volatile_read_regset(self.ecc.py_mem(0).as_ptr(), &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), py);
        self.alignment_helper
            .volatile_read_regset(self.ecc.qx_mem(0).as_ptr(), &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), qx);
        self.alignment_helper
            .volatile_read_regset(self.ecc.qy_mem(0).as_ptr(), &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), qy);
        self.alignment_helper
            .volatile_read_regset(self.ecc.qz_mem(0).as_ptr(), &mut tmp, 8);
        self.reverse_words(tmp.as_ref(), qz);

        Ok(())
    }

    /// # Jacobian Point Multiplication
    ///
    /// Jacobian Point Multiplication can be represented as:
    /// (Q_x, Q_y, Q_z) = k * (P_x, P_y, 1)
    ///
    /// Output is stored in `x`, `y`, and `k`.
    ///
    /// # Error
    ///
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    pub fn jacobian_point_multiplication(
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
            .volatile_write_regset(self.ecc.k_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.px_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), tmp.as_ref(), 8);

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        while self.is_busy() {}

        cfg_if::cfg_if! {
            if #[cfg(not(esp32h2))] {
            self.alignment_helper
                .volatile_read_regset(self.ecc.px_mem(0).as_ptr(), &mut tmp, 8);
            self.reverse_words(tmp.as_ref(), x);
            self.alignment_helper
                .volatile_read_regset(self.ecc.py_mem(0).as_ptr(), &mut tmp, 8);
            self.reverse_words(tmp.as_ref(), y);
            self.alignment_helper
                .volatile_read_regset(self.ecc.k_mem(0).as_ptr(), &mut tmp, 8);
            self.reverse_words(tmp.as_ref(), k);
            } else {
            self.alignment_helper
                .volatile_read_regset(self.ecc.qx_mem(0).as_ptr(), &mut tmp, 8);
            self.reverse_words(tmp.as_ref(), x);
            self.alignment_helper
                .volatile_read_regset(self.ecc.qy_mem(0).as_ptr(), &mut tmp, 8);
            self.reverse_words(tmp.as_ref(), y);
            self.alignment_helper
                .volatile_read_regset(self.ecc.qz_mem(0).as_ptr(), &mut tmp, 8);
            self.reverse_words(tmp.as_ref(), k);
            }
        }

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

        cfg_if::cfg_if! {
            if #[cfg(not(esp32h2))] {
                self.alignment_helper
                    .volatile_write_regset(self.ecc.px_mem(0).as_ptr(), tmp.as_ref(), 8);
                self.reverse_words(y, &mut tmp);
                self.alignment_helper
                    .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), tmp.as_ref(), 8);
                self.reverse_words(z, &mut tmp);
                self.alignment_helper
                    .volatile_write_regset(self.ecc.k_mem(0).as_ptr(), tmp.as_ref(), 8);
            } else {
                self.alignment_helper
                    .volatile_write_regset(self.ecc.qx_mem(0).as_ptr(), tmp.as_ref(), 8);
                self.reverse_words(y, &mut tmp);
                self.alignment_helper
                    .volatile_write_regset(self.ecc.qy_mem(0).as_ptr(), tmp.as_ref(), 8);
                self.reverse_words(z, &mut tmp);
                self.alignment_helper
                    .volatile_write_regset(self.ecc.qz_mem(0).as_ptr(), tmp.as_ref(), 8);
            }
        }

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        // wait for interrupt
        while self.is_busy() {}

        if !self.ecc.mult_conf().read().verification_result().bit() {
            self.ecc.mult_conf().reset();
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
    /// Output is stored in `x`, `y`, and `k`.
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
            .volatile_write_regset(self.ecc.k_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(x, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.px_mem(0).as_ptr(), tmp.as_ref(), 8);
        self.reverse_words(y, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), tmp.as_ref(), 8);

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        // wait for interrupt
        while self.is_busy() {}

        if !self.ecc.mult_conf().read().verification_result().bit() {
            self.ecc.mult_conf().reset();
            return Err(Error::PointNotOnSelectedCurve);
        }

        if !self.ecc.mult_conf().read().verification_result().bit() {
            self.ecc.mult_conf().reset();
            return Err(Error::PointNotOnSelectedCurve);
        }

        cfg_if::cfg_if! {
            if #[cfg(not(esp32h2))] {
                self.alignment_helper
                    .volatile_read_regset(self.ecc.px_mem(0).as_ptr(), &mut tmp, 8);
                self.reverse_words(tmp.as_ref(), x);
                self.alignment_helper
                    .volatile_read_regset(self.ecc.py_mem(0).as_ptr(), &mut tmp, 8);
                self.reverse_words(tmp.as_ref(), y);
                self.alignment_helper
                    .volatile_read_regset(self.ecc.k_mem(0).as_ptr(), &mut tmp, 8);
                self.reverse_words(tmp.as_ref(), k);
            } else {
                self.alignment_helper
                    .volatile_read_regset(self.ecc.qx_mem(0).as_ptr(), &mut tmp, 8);
                self.reverse_words(tmp.as_ref(), x);
                self.alignment_helper
                    .volatile_read_regset(self.ecc.qy_mem(0).as_ptr(), &mut tmp, 8);
                self.reverse_words(tmp.as_ref(), y);
                self.alignment_helper
                    .volatile_read_regset(self.ecc.qz_mem(0).as_ptr(), &mut tmp, 8);
                self.reverse_words(tmp.as_ref(), k);
            }
        }

        Ok(())
    }

    /// # Point Addition
    ///
    /// In this working mode, ECC first verifies if Point (Px, Py) is on the
    /// selected elliptic curve or not. If yes, then perform the addition:
    /// (R_x, R_y) = (J_x, J_y, J_z) = (P_x, P_y, 1) + (Q_x, Q_y, Q_z)
    ///
    /// This functions requires data in Little Endian.
    /// The affine point representation output is stored in `px` and `py`.
    /// The Jacobian point representation output is stored in `qx`, `qy`, and
    /// `qz`.
    ///
    /// # Error
    ///
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    ///
    /// This function will return an error if the point is not on the selected
    /// elliptic curve.
    #[cfg(esp32h2)]
    pub fn affine_point_addition(
        &mut self,
        curve: &EllipticCurve,
        px: &mut [u8],
        py: &mut [u8],
        qx: &mut [u8],
        qy: &mut [u8],
        qz: &mut [u8],
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if px.len() != 24
                    || py.len() != 24
                    || qx.len() != 24
                    || qy.len() != 24
                    || qz.len() != 24
                {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if px.len() != 32
                    || py.len() != 32
                    || qx.len() != 32
                    || qy.len() != 32
                    || qz.len() != 32
                {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };
        let mode = WorkMode::PointAdd;

        let mut tmp = [0_u8; 32];

        tmp[0..px.len()].copy_from_slice(px);
        self.alignment_helper
            .volatile_write_regset(self.ecc.px_mem(0).as_ptr(), &tmp, 8);
        tmp[0..py.len()].copy_from_slice(py);
        self.alignment_helper
            .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), &tmp, 8);
        tmp[0..qx.len()].copy_from_slice(qx);
        self.alignment_helper
            .volatile_write_regset(self.ecc.qx_mem(0).as_ptr(), &tmp, 8);
        tmp[0..qy.len()].copy_from_slice(qy);
        self.alignment_helper
            .volatile_write_regset(self.ecc.qy_mem(0).as_ptr(), &tmp, 8);
        tmp[0..qz.len()].copy_from_slice(qz);
        self.alignment_helper
            .volatile_write_regset(self.ecc.qz_mem(0).as_ptr(), &tmp, 8);

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(mode as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        // wait for interrupt
        while self.is_busy() {}

        self.alignment_helper
            .volatile_read_regset(self.ecc.px_mem(0).as_ptr(), &mut tmp, 8);
        let mut tmp_len = px.len();
        px[..].copy_from_slice(&tmp[..tmp_len]);
        self.alignment_helper
            .volatile_read_regset(self.ecc.py_mem(0).as_ptr(), &mut tmp, 8);
        tmp_len = py.len();
        py[..].copy_from_slice(&tmp[..tmp_len]);
        self.alignment_helper
            .volatile_read_regset(self.ecc.qx_mem(0).as_ptr(), &mut tmp, 8);
        tmp_len = qx.len();
        qx[..].copy_from_slice(&tmp[..tmp_len]);
        self.alignment_helper
            .volatile_read_regset(self.ecc.qy_mem(0).as_ptr(), &mut tmp, 8);
        tmp_len = qy.len();
        qy[..].copy_from_slice(&tmp[..tmp_len]);
        self.alignment_helper
            .volatile_read_regset(self.ecc.qz_mem(0).as_ptr(), &mut tmp, 8);
        tmp_len = qz.len();
        qz[..].copy_from_slice(&tmp[..tmp_len]);

        Ok(())
    }

    /// # Mod Operations (+-*/)
    ///
    /// In this working mode, ECC first verifies if Point (A, B) is on the
    /// selected elliptic curve or not. If yes, then perform single mod
    /// operation: R = A (+-*/) B mod N
    ///
    /// This functions requires data in Little Endian.
    /// Output is stored in `a` (+-) and in `b` (*/).
    ///
    /// # Error
    ///
    /// This function will return an error if any bitlength value is different
    /// from the bitlength of the prime fields of the curve.
    ///
    /// This function will return an error if the point is not on the selected
    /// elliptic curve.
    #[cfg(esp32h2)]
    pub fn mod_operations(
        &mut self,
        curve: &EllipticCurve,
        a: &mut [u8],
        b: &mut [u8],
        work_mode: WorkMode,
    ) -> Result<(), Error> {
        let curve = match curve {
            EllipticCurve::P192 => {
                if a.len() != 24 || b.len() != 24 {
                    return Err(Error::SizeMismatchCurve);
                }
                false
            }
            EllipticCurve::P256 => {
                if a.len() != 32 || b.len() != 32 {
                    return Err(Error::SizeMismatchCurve);
                }
                true
            }
        };

        let mut tmp = [0_u8; 32];
        tmp[0..a.len()].copy_from_slice(a);
        self.alignment_helper
            .volatile_write_regset(self.ecc.px_mem(0).as_ptr(), &tmp, 8);
        tmp[0..b.len()].copy_from_slice(b);
        self.alignment_helper
            .volatile_write_regset(self.ecc.py_mem(0).as_ptr(), &tmp, 8);

        self.ecc.mult_conf().write(|w| unsafe {
            w.work_mode()
                .bits(work_mode.clone() as u8)
                .key_length()
                .bit(curve)
                .start()
                .set_bit()
        });

        // wait for interrupt
        while self.is_busy() {}

        match work_mode {
            WorkMode::ModAdd | WorkMode::ModSub => {
                self.alignment_helper.volatile_read_regset(
                    self.ecc.px_mem(0).as_ptr(),
                    &mut tmp,
                    8,
                );
                let tmp_len = a.len();
                a[..].copy_from_slice(&tmp[..tmp_len]);
            }
            WorkMode::ModMulti | WorkMode::ModDiv => {
                self.alignment_helper.volatile_read_regset(
                    self.ecc.py_mem(0).as_ptr(),
                    &mut tmp,
                    8,
                );
                let tmp_len = b.len();
                b[..].copy_from_slice(&tmp[..tmp_len]);
            }
            _ => unreachable!(),
        }

        Ok(())
    }

    fn is_busy(&self) -> bool {
        self.ecc.mult_conf().read().start().bit_is_set()
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
