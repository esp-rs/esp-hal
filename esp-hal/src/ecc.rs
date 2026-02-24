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
//! - Two different elliptic curves, namely P-192 and P-256 defined in FIPS 186-3.
//! - Seven working modes.
//! - Interrupt upon completion of calculation.
//!
//! Inputs of the ECC hardware accelerator must be provided in big-endian
//! representation. The driver handles the inner representation of the blocks.

use core::marker::PhantomData;

use crate::{
    Blocking,
    DriverMode,
    interrupt::InterruptHandler,
    pac::{self, ecc::mult_conf::KEY_LENGTH},
    peripherals::{ECC, Interrupt},
    reg_access::{AlignmentHelper, SocDependentEndianess},
    system::{self, GenericPeripheralGuard},
};

const MEM_BLOCK_SIZE: usize = 32;

/// The ECC Accelerator driver instance
pub struct Ecc<'d, Dm: DriverMode> {
    ecc: ECC<'d>,
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    phantom: PhantomData<Dm>,
    _memory_guard: EccMemoryPowerGuard,
    _guard: GenericPeripheralGuard<{ system::Peripheral::Ecc as u8 }>,
}

struct EccMemoryPowerGuard;

impl EccMemoryPowerGuard {
    fn new() -> Self {
        #[cfg(soc_has_pcr)]
        crate::peripherals::PCR::regs()
            .ecc_pd_ctrl()
            .modify(|_, w| {
                w.ecc_mem_force_pd().clear_bit();
                w.ecc_mem_force_pu().set_bit();
                w.ecc_mem_pd().clear_bit()
            });
        Self
    }
}

impl Drop for EccMemoryPowerGuard {
    fn drop(&mut self) {
        #[cfg(soc_has_pcr)]
        crate::peripherals::PCR::regs()
            .ecc_pd_ctrl()
            .modify(|_, w| {
                w.ecc_mem_force_pd().clear_bit();
                w.ecc_mem_force_pu().clear_bit();
                w.ecc_mem_pd().set_bit()
            });
    }
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
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EllipticCurve {
    /// The P-192 elliptic curve, a 192-bit curve.
    P192,
    /// The P-256 elliptic curve. a 256-bit curve.
    P256,
}
impl EllipticCurve {
    fn size_check<const N: usize>(&self, params: [&[u8]; N]) -> Result<(), Error> {
        let bytes = match self {
            EllipticCurve::P192 => 24,
            EllipticCurve::P256 => 32,
        };

        if params.iter().any(|p| p.len() != bytes) {
            return Err(Error::SizeMismatchCurve);
        }

        Ok(())
    }
}

#[derive(Clone, Copy)]
/// Represents the operational modes for elliptic curve or modular arithmetic
/// computations.
pub enum WorkMode {
    /// Point multiplication mode.
    PointMultiMode          = 0,
    #[cfg(ecc_working_modes = "7")]
    /// Division mode.
    DivisionMode            = 1,
    /// Point verification mode.
    PointVerif              = 2,
    /// Point verification and multiplication mode.
    PointVerifMulti         = 3,
    /// Jacobian point multiplication mode.
    JacobianPointMulti      = 4,
    #[cfg(ecc_working_modes = "11")]
    /// Point addition mode.
    PointAdd                = 5,
    /// Jacobian point verification mode.
    JacobianPointVerif      = 6,
    /// Point verification and multiplication in Jacobian coordinates.
    PointVerifJacobianMulti = 7,
    #[cfg(ecc_working_modes = "11")]
    /// Modular addition mode.
    ModAdd                  = 8,
    #[cfg(ecc_working_modes = "11")]
    /// Modular subtraction mode.
    ModSub                  = 9,
    #[cfg(ecc_working_modes = "11")]
    /// Modular multiplication mode.
    ModMulti                = 10,
    #[cfg(ecc_working_modes = "11")]
    /// Modular division mode.
    ModDiv                  = 11,
}

impl<'d> Ecc<'d, Blocking> {
    /// Create a new instance in [Blocking] mode.
    pub fn new(ecc: ECC<'d>) -> Self {
        let guard = GenericPeripheralGuard::new();

        Self {
            ecc,
            alignment_helper: AlignmentHelper::default(),
            phantom: PhantomData,
            _memory_guard: EccMemoryPowerGuard::new(),
            _guard: guard,
        }
    }
}

impl crate::private::Sealed for Ecc<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Ecc<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

impl<Dm: DriverMode> Ecc<'_, Dm> {
    fn regs(&self) -> &pac::ecc::RegisterBlock {
        self.ecc.register_block()
    }

    /// Resets the ECC peripheral.
    pub fn reset(&mut self) {
        self.regs().mult_conf().reset()
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
        curve: EllipticCurve,
        k: &[u8],
        x: &mut [u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        curve.size_check([k, x, y])?;

        self.write_mem_reversed(self.k_mem(), k);
        self.write_mem_reversed(self.px_mem(), x);
        self.write_mem_reversed(self.py_mem(), y);

        self.start_operation(WorkMode::PointMultiMode, curve);
        while self.is_busy() {}

        self.read_mem_reversed(self.px_mem(), x);
        self.read_mem_reversed(self.py_mem(), y);

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
        curve: EllipticCurve,
        k: &[u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        curve.size_check([k, y])?;

        self.write_mem_reversed(self.k_mem(), k);
        self.write_mem_reversed(self.py_mem(), y);

        self.start_operation(WorkMode::DivisionMode, curve);

        // wait for interrupt
        while self.is_busy() {}

        self.read_mem_reversed(self.py_mem(), y);

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
        curve: EllipticCurve,
        x: &[u8],
        y: &[u8],
    ) -> Result<(), Error> {
        curve.size_check([x, y])?;

        self.write_mem_reversed(self.px_mem(), x);
        self.write_mem_reversed(self.py_mem(), y);

        self.start_operation(WorkMode::PointVerif, curve);

        // wait for interrupt
        while self.is_busy() {}
        self.check_point_verification_result()?;

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
    #[cfg(not(ecc_working_modes = "11"))]
    pub fn affine_point_verification_multiplication(
        &mut self,
        curve: EllipticCurve,
        k: &[u8],
        x: &mut [u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        curve.size_check([k, x, y])?;

        self.write_mem_reversed(self.k_mem(), k);
        self.write_mem_reversed(self.px_mem(), x);
        self.write_mem_reversed(self.py_mem(), y);

        self.start_operation(WorkMode::PointVerifMulti, curve);

        // wait for interrupt
        while self.is_busy() {}
        self.check_point_verification_result()?;

        self.read_mem_reversed(self.px_mem(), x);
        self.read_mem_reversed(self.py_mem(), y);

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
    #[expect(clippy::too_many_arguments)]
    #[cfg(ecc_working_modes = "11")]
    pub fn affine_point_verification_multiplication(
        &mut self,
        curve: EllipticCurve,
        k: &[u8],
        px: &mut [u8],
        py: &mut [u8],
        qx: &mut [u8],
        qy: &mut [u8],
        qz: &mut [u8],
    ) -> Result<(), Error> {
        curve.size_check([k, px, py])?; //Q?

        self.write_mem_reversed(self.k_mem(), k);
        self.write_mem_reversed(self.px_mem(), px);
        self.write_mem_reversed(self.py_mem(), py);

        self.start_operation(WorkMode::PointVerifMulti, curve);

        // wait for interrupt
        while self.is_busy() {}
        self.check_point_verification_result()?;

        self.read_mem_reversed(self.px_mem(), px);
        self.read_mem_reversed(self.py_mem(), py);
        self.read_mem_reversed(self.qx_mem(), qx);
        self.read_mem_reversed(self.qy_mem(), qy);
        self.read_mem_reversed(self.qz_mem(), qz);

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
        curve: EllipticCurve,
        k: &mut [u8],
        x: &mut [u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        curve.size_check([k, x, y])?;

        self.write_mem_reversed(self.k_mem(), k);
        self.write_mem_reversed(self.px_mem(), x);
        self.write_mem_reversed(self.py_mem(), y);

        self.start_operation(WorkMode::JacobianPointMulti, curve);

        while self.is_busy() {}

        cfg_if::cfg_if! {
            if #[cfg(not(ecc_working_modes = "11"))] {
                self.read_mem_reversed(self.px_mem(), x);
                self.read_mem_reversed(self.py_mem(), y);
                self.read_mem_reversed(self.k_mem(), k);
            } else {
                self.read_mem_reversed(self.qx_mem(), x);
                self.read_mem_reversed(self.qy_mem(), y);
                self.read_mem_reversed(self.qz_mem(), k);
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
        curve: EllipticCurve,
        x: &[u8],
        y: &[u8],
        z: &[u8],
    ) -> Result<(), Error> {
        curve.size_check([x, y, z])?;

        cfg_if::cfg_if! {
            if #[cfg(not(ecc_working_modes = "11"))] {
                self.write_mem_reversed(self.px_mem(), x);
                self.write_mem_reversed(self.py_mem(), y);
                self.write_mem_reversed(self.k_mem(), z);
            } else {
                self.write_mem_reversed(self.qx_mem(), x);
                self.write_mem_reversed(self.qy_mem(), y);
                self.write_mem_reversed(self.qz_mem(), z);
            }
        }

        self.start_operation(WorkMode::JacobianPointVerif, curve);

        // wait for interrupt
        while self.is_busy() {}
        self.check_point_verification_result()?;

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
        curve: EllipticCurve,
        k: &mut [u8],
        x: &mut [u8],
        y: &mut [u8],
    ) -> Result<(), Error> {
        curve.size_check([k, x, y])?;

        self.write_mem_reversed(self.k_mem(), k);
        self.write_mem_reversed(self.px_mem(), x);
        self.write_mem_reversed(self.py_mem(), y);

        self.start_operation(WorkMode::PointVerifJacobianMulti, curve);

        // wait for interrupt
        while self.is_busy() {}
        self.check_point_verification_result()?;

        cfg_if::cfg_if! {
            if #[cfg(not(ecc_working_modes = "11"))] {
                self.read_mem_reversed(self.px_mem(), x);
                self.read_mem_reversed(self.py_mem(), y);
                self.read_mem_reversed(self.k_mem(), k);
            } else {
                self.read_mem_reversed(self.qx_mem(), x);
                self.read_mem_reversed(self.qy_mem(), y);
                self.read_mem_reversed(self.qz_mem(), k);
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
    #[cfg(ecc_working_modes = "11")]
    pub fn affine_point_addition(
        &mut self,
        curve: EllipticCurve,
        px: &mut [u8],
        py: &mut [u8],
        qx: &mut [u8],
        qy: &mut [u8],
        qz: &mut [u8],
    ) -> Result<(), Error> {
        curve.size_check([px, py, qx, qy, qz])?;

        self.write_mem(self.px_mem(), px);
        self.write_mem(self.py_mem(), py);
        self.write_mem(self.qx_mem(), qx);
        self.write_mem(self.qy_mem(), qy);
        self.write_mem(self.qz_mem(), qz);

        self.start_operation(WorkMode::PointAdd, curve);

        // wait for interrupt
        while self.is_busy() {}

        self.read_mem(self.px_mem(), px);
        self.read_mem(self.py_mem(), py);
        self.read_mem(self.qx_mem(), qx);
        self.read_mem(self.qy_mem(), qy);
        self.read_mem(self.qz_mem(), qz);

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
    #[cfg(ecc_working_modes = "11")]
    pub fn mod_operations(
        &mut self,
        curve: EllipticCurve,
        a: &mut [u8],
        b: &mut [u8],
        work_mode: WorkMode,
    ) -> Result<(), Error> {
        curve.size_check([a, b])?;

        self.write_mem(self.px_mem(), a);
        self.write_mem(self.py_mem(), b);

        self.start_operation(work_mode, curve);

        // wait for interrupt
        while self.is_busy() {}

        match work_mode {
            WorkMode::ModAdd | WorkMode::ModSub => self.read_mem(self.px_mem(), a),
            WorkMode::ModMulti | WorkMode::ModDiv => self.read_mem(self.py_mem(), b),
            _ => unreachable!(),
        }

        Ok(())
    }

    /// Register an interrupt handler for the ECC peripheral.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::ECC);
        }
        crate::interrupt::bind_handler(Interrupt::ECC, handler);
    }

    fn is_busy(&self) -> bool {
        self.regs().mult_conf().read().start().bit_is_set()
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

    fn start_operation(&self, mode: WorkMode, curve: EllipticCurve) {
        self.regs().mult_conf().write(|w| unsafe {
            w.work_mode().bits(mode as u8);
            w.key_length().variant(match curve {
                EllipticCurve::P192 => KEY_LENGTH::P192,
                EllipticCurve::P256 => KEY_LENGTH::P256,
            });
            w.start().set_bit()
        });
    }

    fn check_point_verification_result(&self) -> Result<(), Error> {
        if self
            .regs()
            .mult_conf()
            .read()
            .verification_result()
            .bit_is_set()
        {
            Ok(())
        } else {
            self.regs().mult_conf().reset();
            Err(Error::PointNotOnSelectedCurve)
        }
    }

    #[cfg(ecc_working_modes = "11")]
    fn write_mem(&mut self, ptr: *mut u32, data: &[u8]) {
        self.alignment_helper
            .volatile_write_regset(ptr, data, data.len());
        #[cfg(ecc_zero_extend_writes)]
        if data.len() < MEM_BLOCK_SIZE {
            let pad = MEM_BLOCK_SIZE - data.len();
            self.alignment_helper.volatile_write_regset(
                ptr.wrapping_byte_add(data.len()),
                &[0; MEM_BLOCK_SIZE][..pad],
                pad,
            );
        }
    }

    fn write_mem_reversed(&mut self, ptr: *mut u32, data: &[u8]) {
        let mut tmp = [0_u8; MEM_BLOCK_SIZE];
        self.reverse_words(data, &mut tmp);
        self.alignment_helper
            .volatile_write_regset(ptr, tmp.as_ref(), MEM_BLOCK_SIZE);
    }

    #[cfg(ecc_working_modes = "11")]
    fn read_mem(&mut self, reg: *const u32, out: &mut [u8]) {
        self.alignment_helper
            .volatile_read_regset(reg, out, out.len());
    }

    fn read_mem_reversed(&mut self, reg: *const u32, out: &mut [u8]) {
        let mut tmp = [0_u8; MEM_BLOCK_SIZE];
        self.alignment_helper
            .volatile_read_regset(reg, &mut tmp, MEM_BLOCK_SIZE);
        self.reverse_words(tmp.as_ref(), out);
    }

    fn k_mem(&self) -> *mut u32 {
        self.regs().k_mem(0).as_ptr()
    }

    fn px_mem(&self) -> *mut u32 {
        self.regs().px_mem(0).as_ptr()
    }

    fn py_mem(&self) -> *mut u32 {
        self.regs().py_mem(0).as_ptr()
    }

    #[cfg(ecc_working_modes = "11")]
    fn qx_mem(&self) -> *mut u32 {
        self.regs().qx_mem(0).as_ptr()
    }

    #[cfg(ecc_working_modes = "11")]
    fn qy_mem(&self) -> *mut u32 {
        self.regs().qy_mem(0).as_ptr()
    }

    #[cfg(ecc_working_modes = "11")]
    fn qz_mem(&self) -> *mut u32 {
        self.regs().qz_mem(0).as_ptr()
    }
}
