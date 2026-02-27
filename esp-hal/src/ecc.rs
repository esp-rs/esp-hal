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

use core::marker::PhantomData;

use procmacros::BuilderLite;

use crate::{
    Blocking,
    DriverMode,
    interrupt::InterruptHandler,
    pac::{self, ecc::mult_conf::KEY_LENGTH},
    peripherals::{ECC, Interrupt},
    private::Sealed,
    system::{self, GenericPeripheralGuard},
};

const MEM_BLOCK_SIZE: usize = property!("ecc.mem_block_size");

/// The ECC Accelerator driver.
///
/// Note that as opposed to commonly used standards, this driver operates on
/// **little-endian** data.
pub struct Ecc<'d, Dm: DriverMode> {
    _ecc: ECC<'d>,
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

/// ECC peripheral configuration.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Force enable register clock.
    force_enable_reg_clock: bool,

    /// Force enable memory clock.
    #[cfg(ecc_has_memory_clock_gate)]
    force_enable_mem_clock: bool,

    /// Enable constant time operation and minimized power consumption variation for
    /// point-multiplication operations.
    #[cfg_attr(
        esp32h2,
        doc = r"

Only available on chip revision 1.2 and above."
    )]
    #[cfg(ecc_supports_enhanced_security)]
    enhanced_security: bool,
}

/// The length of the arguments do not match the length required by the curve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyLengthMismatch;

/// ECC operation error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperationError {
    /// The length of the arguments do not match the length required by the curve.
    ParameterLengthMismatch,

    /// Point verification failed.
    PointNotOnCurve,
}

/// Modulus base.
#[cfg(ecc_has_modular_arithmetic)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EccModBase {
    /// The order of the curve.
    OrderOfCurve = 0,

    /// Prime modulus.
    PrimeModulus = 1,
}

impl From<KeyLengthMismatch> for OperationError {
    fn from(_: KeyLengthMismatch) -> Self {
        OperationError::ParameterLengthMismatch
    }
}

for_each_ecc_curve! {
    (all $(( $id:literal, $name:ident, $bits:literal )),*) => {
        /// Represents supported elliptic curves for cryptographic operations.
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum EllipticCurve {
            $(
                #[doc = concat!("The ", stringify!($name), " elliptic curve, a ", $bits, "-bit curve.")]
                $name,
            )*
        }
        impl EllipticCurve {
            fn size_check<const N: usize>(&self, params: [&[u8]; N]) -> Result<(), KeyLengthMismatch> {
                let bytes = self.size();

                if params.iter().any(|p| p.len() != bytes) {
                    return Err(KeyLengthMismatch);
                }

                Ok(())
            }

            /// Returns the size of the elliptic curve in bytes.
            pub const fn size(self) -> usize {
                match self {
                    $(
                        EllipticCurve::$name => $bits / 8,
                    )*
                }
            }
        }
    };
}

/// This macro defines 3 other macros:
/// - `doc_summary` that takes the first line of the documentation and returns it as a string
/// - `result_type` that generates the return types for each operation
/// - `operation` that generates the operation function
///
/// These generated macros can then be fed to `for_each_ecc_working_mode!` to generate operations
/// the device supports.
macro_rules! define_operations {
    ($($op:tt {
        // The first line is used for summary, and it is prepended with `# ` on the driver method.
        docs: [$first_line:literal $(, $lines:literal)*],
        // The driver method name
        function: $function:ident,
        // Whether the operation is modular, i.e. whether it needs a modulus argument.
        $(modular_arithmetic_method: $is_modular:literal,)?
        // Whether the operation does point verification first.
        verifies_point: $verifies_point:tt,
        // Input parameters. This determines the name and order of the function arguments,
        // as well as which memory block they will be written to. Depending on the value of
        // cfg(ecc_separate_jacobian_point_memory), qx, qy and qz may be mapped to px, py and k.
        inputs: [$($input:ident),*],
        // What data does the output contain?
        // - Scalar (and which memory block contains the scalar)
        // - AffinePoint
        // - JacobianPoint
        returns: [
            $(
                // What data is computed may be device specific.
                $(#[$returns_meta:meta])*
                $returns:ident $({ const $c:ident: $t:ty = $v:expr })?
            ),*
        ]
    }),*) => {
        macro_rules! doc_summary {
            $(
                ($op) => { $first_line };
            )*
        }
        macro_rules! result_type {
            $(
                ($op) => {
                    #[doc = concat!("A marker type representing ", doc_summary!($op))]
                    #[non_exhaustive]
                    pub struct $op;

                    impl crate::private::Sealed for $op {}

                    impl EccOperation for $op {
                        const WORK_MODE: WorkMode = WorkMode::$op;
                        const VERIFIES_POINT: bool = $verifies_point;
                    }

                    paste::paste! {
                        $(
                            $(#[$returns_meta])*
                            impl [<OperationReturns $returns>] for $op {
                                $(
                                    const $c: $t = $v;
                                )?
                            }
                        )*
                    }
                };
            )*
        }
        macro_rules! driver_method {
            $(
                ($op) => {
                    #[doc = concat!("# ", $first_line)]
                    $(#[doc = $lines])*
                    #[doc = r"

## Errors

This function will return an error if the bitlength of the parameters is different
from the bitlength of the prime fields of the curve."]
                    #[inline]
                    pub fn $function<'op>(
                        &'op mut self,
                        curve: EllipticCurve,
                        $(#[cfg($is_modular)] modulus: EccModBase,)?
                        $($input: &[u8],)*
                    ) -> Result<EccResultHandle<'op, $op>, KeyLengthMismatch> {
                        curve.size_check([$($input),*])?;

                        paste::paste! {
                            $(
                                self.info().write_mem(self.info().[<$input _mem>](), $input);
                            )*
                        };

                        #[cfg(ecc_has_modular_arithmetic)]
                        let mod_base = $crate::if_set! {
                            $(
                                {
                                    $crate::ignore!($is_modular);
                                    modulus
                                }
                            )?,
                            // else
                            EccModBase::OrderOfCurve
                        };

                        Ok(self.run_operation::<$op>(
                            curve,
                            #[cfg(ecc_has_modular_arithmetic)] mod_base,
                        ))
                    }
                };
            )*
        }
    }
}

define_operations! {
    AffinePointMultiplication {
        docs: [
            "Base point multiplication",
            "",
            "This method performs `(Qx, Qy) = k * (Px, Py)`."
        ],
        function: affine_point_multiplication,
        verifies_point: false,
        inputs: [k, px, py],
        returns: [AffinePoint]
    },

    AffinePointVerification {
        docs: [
            "Base Point Verification",
            "",
            "This method verifies whether Point (Px, Py) is on the selected elliptic curve."
        ],
        function: affine_point_verification,
        verifies_point: true,
        inputs: [px, py],
        returns: []
    },

    AffinePointVerificationAndMultiplication {
        docs: [
            "Base Point Verification and Multiplication",
            "",
            "This method verifies whether Point (Px, Py) is on the selected elliptic curve and performs `(Qx, Qy) = k * (Px, Py)`."
        ],
        function: affine_point_verification_multiplication,
        verifies_point: true,
        inputs: [k, px, py],
        returns: [
            AffinePoint,
            #[cfg(ecc_separate_jacobian_point_memory)]
            JacobianPoint
        ]
    },

    AffinePointAddition {
        docs: [
            "Point Addition",
            "",
            "This method performs `(Rx, Ry) = (Jx, Jy, Jz) = (Px, Py, 1) + (Qx, Qy, Qz)`."
        ],
        function: affine_point_addition,
        verifies_point: false,
        inputs: [px, py, qx, qy, qz],
        returns: [
            AffinePoint,
            #[cfg(ecc_separate_jacobian_point_memory)]
            JacobianPoint
        ]
    },

    JacobianPointMultiplication {
        docs: [
            "Jacobian Point Multiplication",
            "",
            "This method performs `(Qx, Qy, Qz) = k * (Px, Py, 1)`."
        ],
        function: jacobian_point_multiplication,
        verifies_point: false,
        inputs: [k, px, py],
        returns: [
            JacobianPoint
        ]
    },

    JacobianPointVerification {
        docs: [
            "Jacobian Point Verification",
            "",
            "This method verifies whether Point (Qx, Qy, Qz) is on the selected elliptic curve."
        ],
        function: jacobian_point_verification,
        verifies_point: true,
        inputs: [qx, qy, qz],
        returns: [
            JacobianPoint
        ]
    },

    AffinePointVerificationAndJacobianPointMultiplication {
        docs: [
            "Base Point Verification + Jacobian Point Multiplication",
            "",
            "This method first verifies whether Point (Px, Py) is on the selected elliptic curve. If yes, it performs `(Qx, Qy, Qz) = k * (Px, Py, 1)`."
        ],
        function: affine_point_verification_jacobian_multiplication,
        verifies_point: true,
        inputs: [k, px, py],
        returns: [
            JacobianPoint
        ]
    },

    FiniteFieldDivision {
        docs: [
            "Finite Field Division",
            "",
            "This method performs `R = Py * k^{−1} mod p`."
        ],
        function: finite_field_division,
        verifies_point: false,
        inputs: [k, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Py }
        ]
    },

    ModularAddition {
        docs: [
            "Modular Addition",
            "",
            "This method performs `R = Px + Py mod p`."
        ],
        function: modular_addition,
        modular_arithmetic_method: true,
        verifies_point: false,
        inputs: [px, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Px }
        ]
    },

    ModularSubtraction {
        docs: [
            "Modular Subtraction",
            "",
            "This method performs `R = Px - Py mod p`."
        ],
        function: modular_subtraction,
        modular_arithmetic_method: true,
        verifies_point: false,
        inputs: [px, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Px }
        ]
    },

    ModularMultiplication {
        docs: [
            "Modular Multiplication",
            "",
            "This method performs `R = Px * Py mod p`."
        ],
        function: modular_multiplication,
        modular_arithmetic_method: true,
        verifies_point: false,
        inputs: [px, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Py }
        ]
    },

    ModularDivision {
        docs: [
            "Modular Division",
            "",
            "This method performs `R = Px * Py^{−1} mod p`."
        ],
        function: modular_division,
        modular_arithmetic_method: true,
        verifies_point: false,
        inputs: [px, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Py }
        ]
    }
}

impl<'d> Ecc<'d, Blocking> {
    /// Create a new instance in [Blocking] mode.
    pub fn new(ecc: ECC<'d>, config: Config) -> Self {
        let this = Self {
            _ecc: ecc,
            phantom: PhantomData,
            _memory_guard: EccMemoryPowerGuard::new(),
            _guard: GenericPeripheralGuard::new(),
        };

        this.info().apply_config(&config);

        this
    }
}

impl crate::private::Sealed for Ecc<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Ecc<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

struct Info {
    regs: &'static pac::ecc::RegisterBlock,
}

impl Info {
    fn reset(&self) {
        self.regs.mult_conf().reset()
    }

    fn apply_config(&self, config: &Config) {
        self.regs.mult_conf().modify(|_, w| {
            w.clk_en().bit(config.force_enable_reg_clock);

            #[cfg(ecc_has_memory_clock_gate)]
            w.mem_clock_gate_force_on()
                .bit(config.force_enable_mem_clock);

            #[cfg(ecc_supports_enhanced_security)]
            if !cfg!(esp32h2) || crate::soc::chip_revision_above(102) {
                w.security_mode().bit(config.enhanced_security);
            }

            w
        });
    }

    fn check_point_verification_result(&self) -> Result<(), OperationError> {
        if self
            .regs
            .mult_conf()
            .read()
            .verification_result()
            .bit_is_set()
        {
            Ok(())
        } else {
            Err(OperationError::PointNotOnCurve)
        }
    }

    #[inline]
    fn write_mem(&self, mut word_ptr: *mut u32, data: &[u8]) {
        // Note that at least the C2 requires writing this memory in words.

        debug_assert!(data.len() <= MEM_BLOCK_SIZE);

        #[cfg(ecc_zero_extend_writes)]
        let end = word_ptr.wrapping_byte_add(MEM_BLOCK_SIZE);

        let (chunks, remainder) = data.as_chunks::<4>();
        debug_assert!(remainder.is_empty());

        for word_bytes in chunks {
            unsafe { word_ptr.write_volatile(u32::from_le_bytes(*word_bytes)) };
            word_ptr = word_ptr.wrapping_add(1);
        }

        #[cfg(ecc_zero_extend_writes)]
        while word_ptr < end {
            unsafe { word_ptr.write_volatile(0) };
            word_ptr = word_ptr.wrapping_add(1);
        }
    }

    #[inline]
    fn read_mem(&self, mut word_ptr: *const u32, out: &mut [u8]) {
        for word_bytes in out.chunks_exact_mut(4) {
            let word = unsafe { word_ptr.read_volatile() };
            word_ptr = word_ptr.wrapping_add(1);
            word_bytes.copy_from_slice(&word.to_le_bytes());
        }
    }

    fn k_mem(&self) -> *mut u32 {
        self.regs.k_mem(0).as_ptr()
    }

    fn px_mem(&self) -> *mut u32 {
        self.regs.px_mem(0).as_ptr()
    }

    fn py_mem(&self) -> *mut u32 {
        self.regs.py_mem(0).as_ptr()
    }

    fn qx_mem(&self) -> *mut u32 {
        cfg_if::cfg_if! {
            if #[cfg(ecc_separate_jacobian_point_memory)] {
                self.regs.qx_mem(0).as_ptr()
            } else {
                self.regs.px_mem(0).as_ptr()
            }
        }
    }

    fn qy_mem(&self) -> *mut u32 {
        cfg_if::cfg_if! {
            if #[cfg(ecc_separate_jacobian_point_memory)] {
                self.regs.qy_mem(0).as_ptr()
            } else {
                self.regs.py_mem(0).as_ptr()
            }
        }
    }

    fn qz_mem(&self) -> *mut u32 {
        cfg_if::cfg_if! {
            if #[cfg(ecc_separate_jacobian_point_memory)] {
                self.regs.qz_mem(0).as_ptr()
            } else {
                self.regs.k_mem(0).as_ptr()
            }
        }
    }

    fn read_point_result(&self, x: &mut [u8], y: &mut [u8]) {
        self.read_mem(self.px_mem(), x);
        self.read_mem(self.py_mem(), y);
    }

    fn read_jacobian_result(&self, qx: &mut [u8], qy: &mut [u8], qz: &mut [u8]) {
        self.read_mem(self.qx_mem(), qx);
        self.read_mem(self.qy_mem(), qy);
        self.read_mem(self.qz_mem(), qz);
    }

    fn is_busy(&self) -> bool {
        self.regs.mult_conf().read().start().bit_is_set()
    }

    fn start_operation(
        &self,
        mode: WorkMode,
        curve: EllipticCurve,
        #[cfg(ecc_has_modular_arithmetic)] mod_base: EccModBase,
    ) {
        let curve_variant;
        for_each_ecc_curve! {
            (all $(($_id:tt, $name:ident, $_bits:tt)),*) => {
                curve_variant = match curve {
                    $(EllipticCurve::$name => KEY_LENGTH::$name,)*
                }
            };
        };
        self.regs.mult_conf().modify(|_, w| unsafe {
            w.work_mode().bits(mode as u8);
            w.key_length().variant(curve_variant);

            #[cfg(ecc_has_modular_arithmetic)]
            w.mod_base().bit(mod_base as u8 == 1);

            w.start().set_bit()
        });
    }
}

// Broken into separate macro invocations per item, to make the "Expand macro" LSP output more
// readable

for_each_ecc_working_mode! {
    (all $(( $id:literal, $mode:tt )),*) => {
        #[derive(Clone, Copy)]
        #[doc(hidden)]
        /// Represents the operational modes for elliptic curve or modular arithmetic
        /// computations.
        pub enum WorkMode {
            $(
                $mode = $id,
            )*
        }
    };
}

// Result type for each operation
for_each_ecc_working_mode! {
    (all $(( $id:literal, $mode:tt )),*) => {
        $(
            result_type!($mode);
        )*
    };
}

// The main driver implementation
for_each_ecc_working_mode! {
    (all $(( $id:literal, $mode:tt )),*) => {
        impl<'d, Dm: DriverMode> Ecc<'d, Dm> {
            fn info(&self) -> Info {
                Info { regs: ECC::regs() }
            }

            fn run_operation<'op, O: EccOperation>(
                &'op mut self,
                curve: EllipticCurve,
                #[cfg(ecc_has_modular_arithmetic)] mod_base: EccModBase,
            ) -> EccResultHandle<'op, O> {
                self.info().start_operation(
                    O::WORK_MODE,
                    curve,
                    #[cfg(ecc_has_modular_arithmetic)] mod_base,
                );

                // wait for interrupt
                while self.info().is_busy() {}

                EccResultHandle::new(curve, self)
            }

            /// Applies the given configuration to the ECC peripheral.
            pub fn apply_config(&mut self, config: &Config) {
                self.info().apply_config(config);
            }

            /// Resets the ECC peripheral.
            pub fn reset(&mut self) {
                self.info().reset()
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

            $(
                driver_method!($mode);
            )*
        }
    };
}

/// Marks an ECC operation.
pub trait EccOperation: Sealed {
    /// Whether the operation verifies that the input point is on the curve.
    const VERIFIES_POINT: bool;

    /// Work mode
    #[doc(hidden)]
    const WORK_MODE: WorkMode;
}

/// Scalar result location.
#[doc(hidden)]
pub enum ScalarResultLocation {
    /// The scalar value is stored in the `Px` memory location.
    Px,
    /// The scalar value is stored in the `Py` memory location.
    Py,
    /// The scalar value is stored in the `k` memory location.
    K,
}

/// Marks operations that return a scalar value.
pub trait OperationReturnsScalar: EccOperation {
    /// Where the scalar value is stored.
    #[doc(hidden)]
    const LOCATION: ScalarResultLocation;
}

/// Marks operations that return a point in affine format.
pub trait OperationReturnsAffinePoint: EccOperation {}

/// Marks operations that return a point in Jacobian format.
pub trait OperationReturnsJacobianPoint: EccOperation {}

/// The result of an ECC operation.
///
/// This struct can be used to read the result of an ECC operation. The methods which can be used
/// depend on the operation. An operation can compute multiple values, such as an affine point and
/// a Jacobian point at the same time.
#[must_use]
pub struct EccResultHandle<'op, O>
where
    O: EccOperation,
{
    curve: EllipticCurve,
    info: Info,
    _marker: PhantomData<(&'op mut (), O)>,
}

impl<'op, O> EccResultHandle<'op, O>
where
    O: EccOperation,
{
    fn new<'d, Dm: DriverMode>(curve: EllipticCurve, driver: &'op mut Ecc<'d, Dm>) -> Self {
        Self {
            curve,
            info: driver.info(),
            _marker: PhantomData,
        }
    }

    fn run_checks<const N: usize>(&self, params: [&[u8]; N]) -> Result<(), OperationError> {
        self.curve.size_check(params)?;
        if O::VERIFIES_POINT {
            self.info.check_point_verification_result()?;
        }
        Ok(())
    }

    /// Returns whether the operation was successful.
    ///
    /// For operations that only perform point verification, this method returns whether the point
    /// is on the curve. For operations that do not perform point verification, this method always
    /// returns true.
    pub fn success(&self) -> bool {
        if O::VERIFIES_POINT {
            self.info.check_point_verification_result().is_ok()
        } else {
            true
        }
    }

    /// Retrieve the scalar result of the operation.
    ///
    /// ## Errors
    ///
    /// Returns an error if point verification failed, or if `out` is not the correct size.
    pub fn read_scalar_result(&self, out: &mut [u8]) -> Result<(), OperationError>
    where
        O: OperationReturnsScalar,
    {
        self.run_checks([out])?;

        match O::LOCATION {
            ScalarResultLocation::Px => self.info.read_mem(self.info.px_mem(), out),
            ScalarResultLocation::Py => self.info.read_mem(self.info.py_mem(), out),
            ScalarResultLocation::K => self.info.read_mem(self.info.k_mem(), out),
        }

        Ok(())
    }

    /// Retrieve the affine point result of the operation.
    ///
    /// ## Errors
    ///
    /// Returns an error if point verification failed, or if `x` or `y` are not the correct size.
    pub fn read_affine_point_result(&self, x: &mut [u8], y: &mut [u8]) -> Result<(), OperationError>
    where
        O: OperationReturnsAffinePoint,
    {
        self.run_checks([x, y])?;
        self.info.read_point_result(x, y);
        Ok(())
    }

    /// Retrieve the Jacobian point result of the operation.
    ///
    /// ## Errors
    ///
    /// Returns an error if point verification failed, or if `x`, `y`, or `z` are not the correct
    /// size.
    pub fn read_jacobian_point_result(
        &self,
        x: &mut [u8],
        y: &mut [u8],
        z: &mut [u8],
    ) -> Result<(), OperationError>
    where
        O: OperationReturnsJacobianPoint,
    {
        self.run_checks([x, y, z])?;
        self.info.read_jacobian_result(x, y, z);
        Ok(())
    }
}
