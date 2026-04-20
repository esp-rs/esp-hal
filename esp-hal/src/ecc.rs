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

use core::{marker::PhantomData, ptr::NonNull};

use procmacros::BuilderLite;

#[cfg(ecc_supports_enhanced_security)]
use crate::efuse::ChipRevision;
use crate::{
    Blocking,
    DriverMode,
    interrupt::InterruptHandler,
    pac::{self, ecc::mult_conf::KEY_LENGTH},
    peripherals::{ECC, Interrupt},
    private::Sealed,
    system::{self, GenericPeripheralGuard},
    work_queue::{Handle, Poll, Status, VTable, WorkQueue, WorkQueueDriver, WorkQueueFrontend},
};

/// This macro defines 4 other macros:
/// - `doc_summary` that takes the first line of the documentation and returns it as a string
/// - `result_type` that generates the return types for each operation
/// - `operation` that generates the operation function
/// - `backend_operation` that generates the backend operation function
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
        $(verifies_point: $verifies_point:literal,)?
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
                $returns:ident $({ const $c:ident: $t:tt = $v:expr })?
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
                        const VERIFIES_POINT: bool = $crate::if_set!($($verifies_point)?, false);
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

                        $(
                            const _: bool = $verifies_point; // I just need this ignored.
                            impl OperationVerifiesPoint for $op {}
                        )?
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

        macro_rules! backend_operation {
            $(
                ($op) => {
                    #[doc = concat!("Configures a new ", $first_line, " operation with the given inputs, to be executed on [`EccBackend`].")]
                    ///
                    /// Outputs need to be assigned separately before executing the operation.
                    pub fn $function<'op>(
                        self,
                        $(#[cfg($is_modular)] modulus: EccModBase,)?
                        $($input: &'op [u8],)*
                    ) -> Result<EccBackendOperation<'op, $op>, KeyLengthMismatch> {
                        self.size_check([&$($input,)*])?;

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

                        let work_item = EccWorkItem {
                            curve: self,
                            operation: WorkMode::$op,
                            cancelled: false,
                            #[cfg(ecc_has_modular_arithmetic)]
                            mod_base,
                            inputs: {
                                let mut inputs = MemoryPointers::default();
                                $(
                                    paste::paste! {
                                        inputs.[<set_ $input>](NonNull::from($input));
                                    };
                                )*
                                inputs
                            },
                            point_verification_result: false,
                            outputs: MemoryPointers::default(),
                        };

                        Ok(EccBackendOperation::new(work_item))
                    }
                };
            )*
        }
    }
}

define_operations! {
    AffinePointMultiplication {
        docs: [
            "Base Point Multiplication",
            "",
            "This operation performs `(Qx, Qy) = k * (Px, Py)`."
        ],
        function: affine_point_multiplication,
        inputs: [k, px, py],
        returns: [AffinePoint]
    },

    AffinePointVerification {
        docs: [
            "Base Point Verification",
            "",
            "This operation verifies whether Point (Px, Py) is on the selected elliptic curve."
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
            "This operation verifies whether Point (Px, Py) is on the selected elliptic curve and performs `(Qx, Qy) = k * (Px, Py)`."
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
            "This operation performs `(Rx, Ry) = (Jx, Jy, Jz) = (Px, Py, 1) + (Qx, Qy, Qz)`."
        ],
        function: affine_point_addition,
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
            "This operation performs `(Qx, Qy, Qz) = k * (Px, Py, 1)`."
        ],
        function: jacobian_point_multiplication,
        inputs: [k, px, py],
        returns: [
            JacobianPoint
        ]
    },

    JacobianPointVerification {
        docs: [
            "Jacobian Point Verification",
            "",
            "This operation verifies whether Point (Qx, Qy, Qz) is on the selected elliptic curve."
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
            "This operation first verifies whether Point (Px, Py) is on the selected elliptic curve. If yes, it performs `(Qx, Qy, Qz) = k * (Px, Py, 1)`."
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
            "This operation performs `R = Py * k^{−1} mod p`."
        ],
        function: finite_field_division,
        inputs: [k, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Py }
        ]
    },

    ModularAddition {
        docs: [
            "Modular Addition",
            "",
            "This operation performs `R = Px + Py mod p`."
        ],
        function: modular_addition,
        modular_arithmetic_method: true,
        inputs: [px, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Px }
        ]
    },

    ModularSubtraction {
        docs: [
            "Modular Subtraction",
            "",
            "This operation performs `R = Px - Py mod p`."
        ],
        function: modular_subtraction,
        modular_arithmetic_method: true,
        inputs: [px, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Px }
        ]
    },

    ModularMultiplication {
        docs: [
            "Modular Multiplication",
            "",
            "This operation performs `R = Px * Py mod p`."
        ],
        function: modular_multiplication,
        modular_arithmetic_method: true,
        inputs: [px, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Py }
        ]
    },

    ModularDivision {
        docs: [
            "Modular Division",
            "",
            "This operation performs `R = Px * Py^{−1} mod p`."
        ],
        function: modular_division,
        modular_arithmetic_method: true,
        inputs: [px, py],
        returns: [
            Scalar { const LOCATION: ScalarResultLocation = ScalarResultLocation::Py }
        ]
    }
}

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
        ///
        /// The methods that represent operations require the `EccBackend` to be started before use.
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum EllipticCurve {
            $(
                #[doc = concat!("The ", stringify!($name), " elliptic curve, a ", $bits, "-bit curve.")]
                $name,
            )*
        }
        impl EllipticCurve {
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

for_each_ecc_working_mode! {
    (all $(($wm_id:literal, $op:tt)),*) => {
        impl EllipticCurve {
            fn size_check<const N: usize>(&self, params: [&[u8]; N]) -> Result<(), KeyLengthMismatch> {
                let bytes = self.size();

                if params.iter().any(|p| p.len() != bytes) {
                    return Err(KeyLengthMismatch);
                }

                Ok(())
            }

            $(
                // Macro defined by `define_operations`
                backend_operation!($op);
            )*
        }
    };
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
            if !cfg!(esp32h2) || crate::soc::chip_revision_above(ChipRevision::from_combined(102)) {
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

/// Marks operations that verify that the input point is on the curve.
pub trait OperationVerifiesPoint: EccOperation {}

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

struct EccWorkItem {
    curve: EllipticCurve,
    operation: WorkMode,
    cancelled: bool,
    #[cfg(ecc_has_modular_arithmetic)]
    mod_base: EccModBase,
    inputs: MemoryPointers,
    point_verification_result: bool,
    outputs: MemoryPointers,
}

#[derive(Default)]
struct MemoryPointers {
    // All of these pointers point to slices with curve-appropriate lengths.
    k: Option<NonNull<u8>>,
    px: Option<NonNull<u8>>,
    py: Option<NonNull<u8>>,
    #[cfg(ecc_separate_jacobian_point_memory)]
    qx: Option<NonNull<u8>>,
    #[cfg(ecc_separate_jacobian_point_memory)]
    qy: Option<NonNull<u8>>,
    #[cfg(ecc_separate_jacobian_point_memory)]
    qz: Option<NonNull<u8>>,
}

impl MemoryPointers {
    fn set_scalar(&mut self, location: ScalarResultLocation, ptr: NonNull<[u8]>) {
        match location {
            ScalarResultLocation::Px => self.set_px(ptr),
            ScalarResultLocation::Py => self.set_py(ptr),
            ScalarResultLocation::K => self.set_k(ptr),
        }
    }

    fn set_k(&mut self, ptr: NonNull<[u8]>) {
        self.k = Some(ptr.cast());
    }

    fn set_px(&mut self, ptr: NonNull<[u8]>) {
        self.px = Some(ptr.cast());
    }

    fn set_py(&mut self, ptr: NonNull<[u8]>) {
        self.py = Some(ptr.cast());
    }

    fn set_qx(&mut self, ptr: NonNull<[u8]>) {
        cfg_if::cfg_if! {
            if #[cfg(ecc_separate_jacobian_point_memory)] {
                self.qx = Some(ptr.cast());
            } else {
                self.px = Some(ptr.cast());
            }
        }
    }

    fn set_qy(&mut self, ptr: NonNull<[u8]>) {
        cfg_if::cfg_if! {
            if #[cfg(ecc_separate_jacobian_point_memory)] {
                self.qy = Some(ptr.cast());
            } else {
                self.py = Some(ptr.cast());
            }
        }
    }

    fn set_qz(&mut self, ptr: NonNull<[u8]>) {
        cfg_if::cfg_if! {
            if #[cfg(ecc_separate_jacobian_point_memory)] {
                self.qz = Some(ptr.cast());
            } else {
                self.k = Some(ptr.cast());
            }
        }
    }
}

// Safety: MemoryPointers is safe to share between threads, in the context of a WorkQueue. The
// WorkQueue ensures that only a single location can access the data. All the internals, except
// for the pointers, are Sync. The pointers are safe to share because they point at data that the
// ECC driver ensures can be accessed safely and soundly.
unsafe impl Sync for MemoryPointers {}
// Safety: we will not hold on to the pointers when the work item leaves the queue.
unsafe impl Send for MemoryPointers {}

static ECC_WORK_QUEUE: WorkQueue<EccWorkItem> = WorkQueue::new();

const ECC_VTABLE: VTable<EccWorkItem> = VTable {
    post: |driver, item| {
        let driver = unsafe { EccBackend::from_raw(driver) };

        // Ensure driver is initialized
        if let DriverState::Uninitialized(ecc) = &driver.driver {
            let mut ecc = Ecc::new(unsafe { ecc.clone_unchecked() }, driver.config);
            ecc.set_interrupt_handler(ecc_work_queue_handler);
            driver.driver = DriverState::Initialized(ecc);
        };

        Some(driver.process(item))
    },
    poll: |driver, item| {
        let driver = unsafe { EccBackend::from_raw(driver) };
        driver.poll(item)
    },
    cancel: |driver, item| {
        let driver = unsafe { EccBackend::from_raw(driver) };
        driver.cancel(item);
    },
    stop: |driver| {
        let driver = unsafe { EccBackend::from_raw(driver) };
        driver.deinitialize()
    },
};

enum DriverState<'d> {
    Uninitialized(ECC<'d>),
    Initialized(Ecc<'d, Blocking>),
}

/// ECC processing backend.
///
/// This struct enables shared access to the device's ECC hardware using a work queue.
pub struct EccBackend<'d> {
    driver: DriverState<'d>,
    config: Config,
}

impl<'d> EccBackend<'d> {
    /// Creates a new ECC backend.
    ///
    /// The backend needs to be [`start`][Self::start]ed before it can execute ECC operations.
    pub fn new(ecc: ECC<'d>, config: Config) -> Self {
        Self {
            driver: DriverState::Uninitialized(ecc),
            config,
        }
    }

    /// Registers the ECC driver to process ECC operations.
    ///
    /// The driver stops operating when the returned object is dropped.
    pub fn start(&mut self) -> EccWorkQueueDriver<'_, 'd> {
        EccWorkQueueDriver {
            inner: WorkQueueDriver::new(self, ECC_VTABLE, &ECC_WORK_QUEUE),
        }
    }

    // WorkQueue callbacks. They may run in any context.

    unsafe fn from_raw<'any>(ptr: NonNull<()>) -> &'any mut Self {
        unsafe { ptr.cast::<EccBackend<'_>>().as_mut() }
    }

    fn process(&mut self, item: &mut EccWorkItem) -> Poll {
        let DriverState::Initialized(driver) = &mut self.driver else {
            unreachable!()
        };

        let bytes = item.curve.size();

        macro_rules! set_input {
            ($input:ident, $input_mem:ident) => {
                if let Some($input) = item.inputs.$input {
                    driver.info().write_mem(driver.info().$input_mem(), unsafe {
                        core::slice::from_raw_parts($input.as_ptr(), bytes)
                    });
                }
            };
        }

        set_input!(k, k_mem);
        set_input!(px, px_mem);
        set_input!(py, py_mem);

        #[cfg(ecc_separate_jacobian_point_memory)]
        {
            set_input!(qx, qx_mem);
            set_input!(qy, qy_mem);
            set_input!(qz, qz_mem);
        }

        driver.info().start_operation(
            item.operation,
            item.curve,
            #[cfg(ecc_has_modular_arithmetic)]
            item.mod_base,
        );
        Poll::Pending(false)
    }

    fn poll(&mut self, item: &mut EccWorkItem) -> Poll {
        let DriverState::Initialized(driver) = &mut self.driver else {
            unreachable!()
        };

        if driver.info().is_busy() {
            return Poll::Pending(false);
        }
        if item.cancelled {
            return Poll::Ready(Status::Cancelled);
        }

        let bytes = item.curve.size();

        macro_rules! read_output {
            ($output:ident, $output_mem:ident) => {
                if let Some($output) = item.outputs.$output {
                    driver.info().read_mem(driver.info().$output_mem(), unsafe {
                        core::slice::from_raw_parts_mut($output.as_ptr(), bytes)
                    });
                }
            };
        }

        read_output!(k, k_mem);
        read_output!(px, px_mem);
        read_output!(py, py_mem);

        #[cfg(ecc_separate_jacobian_point_memory)]
        {
            read_output!(qx, qx_mem);
            read_output!(qy, qy_mem);
            read_output!(qz, qz_mem);
        }

        item.point_verification_result = driver.info().check_point_verification_result().is_ok();

        Poll::Ready(Status::Completed)
    }

    fn cancel(&mut self, item: &mut EccWorkItem) {
        let DriverState::Initialized(driver) = &mut self.driver else {
            unreachable!()
        };
        driver.reset();
        item.cancelled = true;
    }

    fn deinitialize(&mut self) {
        if let DriverState::Initialized(ref ecc) = self.driver {
            self.driver = DriverState::Uninitialized(unsafe { ecc._ecc.clone_unchecked() });
        }
    }
}

/// An active work queue driver.
///
/// This object must be kept around, otherwise ECC operations will never complete.
pub struct EccWorkQueueDriver<'t, 'd> {
    inner: WorkQueueDriver<'t, EccBackend<'d>, EccWorkItem>,
}

impl<'t, 'd> EccWorkQueueDriver<'t, 'd> {
    /// Finishes processing the current work queue item, then stops the driver.
    pub fn stop(self) -> impl Future<Output = ()> {
        self.inner.stop()
    }
}

#[crate::ram]
#[crate::handler]
fn ecc_work_queue_handler() {
    if !ECC_WORK_QUEUE.process() {
        // The queue may indicate that it needs to be polled again. In this case, we do not clear
        // the interrupt bit, which causes the interrupt to be re-handled.
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c5, esp32c61))] {
                let reg = ECC::regs().int_clr();
            } else {
                let reg = ECC::regs().mult_int_clr();
            }
        }
        reg.write(|w| w.calc_done().clear_bit_by_one());
    }
}

/// An ECC operation that can be enqueued on the work queue.
pub struct EccBackendOperation<'op, O: EccOperation> {
    frontend: WorkQueueFrontend<EccWorkItem>,
    _marker: PhantomData<(&'op mut (), O)>,
}

impl<'op, O: EccOperation> EccBackendOperation<'op, O> {
    fn new(work_item: EccWorkItem) -> Self {
        Self {
            frontend: WorkQueueFrontend::new(work_item),
            _marker: PhantomData,
        }
    }

    /// Designate a buffer for the scalar result of the operation.
    ///
    /// Once the operation is processed, the result can be retrieved from the designated buffer.
    ///
    /// ## Errors
    ///
    /// Returns an error if `out` is not the correct size.
    pub fn with_scalar_result(mut self, out: &'op mut [u8]) -> Result<Self, KeyLengthMismatch>
    where
        O: OperationReturnsScalar,
    {
        self.frontend.data().curve.size_check([out])?;

        self.frontend
            .data_mut()
            .outputs
            .set_scalar(O::LOCATION, NonNull::from(out));

        Ok(self)
    }

    /// Designate buffers for the affine point result of the operation.
    ///
    /// Once the operation is processed, the result can be retrieved from the designated buffers.
    ///
    /// ## Errors
    ///
    /// Returns an error if `x` or `y` are not the correct size.
    pub fn with_affine_point_result(
        mut self,
        px: &'op mut [u8],
        py: &'op mut [u8],
    ) -> Result<Self, KeyLengthMismatch>
    where
        O: OperationReturnsAffinePoint,
    {
        self.frontend.data().curve.size_check([px, py])?;

        self.frontend.data_mut().outputs.set_px(NonNull::from(px));
        self.frontend.data_mut().outputs.set_py(NonNull::from(py));

        Ok(self)
    }

    /// Designate buffers for the Jacobian point result of the operation.
    ///
    /// Once the operation is processed, the result can be retrieved from the designated buffers.
    ///
    /// ## Errors
    ///
    /// Returns an error if `x`, `y`, or `z` are not the correct size.
    pub fn with_jacobian_point_result(
        mut self,
        qx: &'op mut [u8],
        qy: &'op mut [u8],
        qz: &'op mut [u8],
    ) -> Result<Self, KeyLengthMismatch>
    where
        O: OperationReturnsJacobianPoint,
    {
        self.frontend.data().curve.size_check([qx, qy, qz])?;

        self.frontend.data_mut().outputs.set_qx(NonNull::from(qx));
        self.frontend.data_mut().outputs.set_qy(NonNull::from(qy));
        self.frontend.data_mut().outputs.set_qz(NonNull::from(qz));

        Ok(self)
    }

    /// Returns `true` if the input point is on the curve.
    ///
    /// The operation must be processed before this method returns a meaningful value.
    pub fn point_on_curve(&self) -> bool
    where
        O: OperationVerifiesPoint,
    {
        self.frontend.data().point_verification_result
    }

    /// Starts processing the operation.
    ///
    /// The returned [`EccHandle`] must be polled to completion before the operation is considered
    /// complete.
    pub fn process(&mut self) -> EccHandle<'_> {
        EccHandle(self.frontend.post(&ECC_WORK_QUEUE))
    }
}

/// A handle for an in-progress operation.
#[must_use]
pub struct EccHandle<'t>(Handle<'t, EccWorkItem>);

impl EccHandle<'_> {
    /// Polls the status of the work item.
    ///
    /// This function returns `true` if the item has been processed.
    #[inline]
    pub fn poll(&mut self) -> bool {
        self.0.poll()
    }

    /// Polls the work item to completion, by busy-looping.
    ///
    /// This function returns immediately if `poll` returns `true`.
    #[inline]
    pub fn wait_blocking(self) -> Status {
        self.0.wait_blocking()
    }

    /// Waits until the work item is completed.
    #[inline]
    pub fn wait(&mut self) -> impl Future<Output = Status> {
        self.0.wait()
    }

    /// Cancels the work item and asynchronously waits until it is removed from the work queue.
    #[inline]
    pub fn cancel(&mut self) -> impl Future<Output = ()> {
        self.0.cancel()
    }
}
