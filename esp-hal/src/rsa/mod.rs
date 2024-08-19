//! # Rivest–Shamir–Adleman (RSA) Accelerator.
//!
//! ## Overview
//! The RSA Accelerator provides hardware support for high precision computation
//! used in various RSA asymmetric cipher algorithms by significantly reducing
//! their software complexity. Compared with RSA algorithms implemented solely
//! in software, this hardware accelerator can speed up RSA algorithms
//! significantly.
//!
//! ## Configuration
//! The RSA Accelerator also supports operands of different lengths, which
//! provides more flexibility during the computation.
//!
//! ## Usage
//! Implementation details;
//!    * The driver uses low-level peripheral access to read and write data
//!      from/to the `RSA` peripheral.
//!    * The driver contains `unsafe` code blocks as it directly manipulates
//!      memory addresses for data transfer.
//!    * The driver supports different sizes of operands based on the generic
//!      types provided during instantiation.
//!    * The [nb] crate is used to handle non-blocking operations.
//!    * The driver provides a set of high-level abstractions to simplify `RSA`
//!      cryptographic operations on `ESP` chips, allowing developers to
//!      leverage the `RSA accelerator` for improved performance.
//!
//! This peripheral supports `async` on every available chip except of `esp32`
//! (to be solved).
//!
//! ## Examples
//! ### Modular Exponentiation, Modular Multiplication, and Multiplication
//! Visit the [RSA] test for an example of using the peripheral.
//!
//! [nb]: https://docs.rs/nb/1.1.0/nb/
//! [the repository with corresponding example]: https://github.com/esp-rs/esp-hal/blob/main/hil-test/tests/rsa.rs

use core::{marker::PhantomData, ptr::copy_nonoverlapping};

use crate::{
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::RSA,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
    InterruptConfigurable,
};

#[cfg_attr(esp32s2, path = "esp32sX.rs")]
#[cfg_attr(esp32s3, path = "esp32sX.rs")]
#[cfg_attr(esp32c3, path = "esp32cX.rs")]
#[cfg_attr(esp32c6, path = "esp32cX.rs")]
#[cfg_attr(esp32h2, path = "esp32cX.rs")]
#[cfg_attr(esp32, path = "esp32.rs")]
mod rsa_spec_impl;

pub use rsa_spec_impl::operand_sizes;

/// RSA peripheral container
pub struct Rsa<'d, DM: crate::Mode> {
    rsa: PeripheralRef<'d, RSA>,
    phantom: PhantomData<DM>,
}

impl<'d, DM: crate::Mode> Rsa<'d, DM> {
    fn internal_set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(crate::peripherals::Interrupt::RSA, handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::RSA, handler.priority())
                .unwrap();
        }
    }
}

impl<'d> Rsa<'d, crate::Blocking> {
    /// Create a new instance in [crate::Blocking] mode.
    ///
    /// Optionally an interrupt handler can be bound.
    pub fn new(rsa: impl Peripheral<P = RSA> + 'd) -> Self {
        Self::new_internal(rsa)
    }
}

impl<'d> crate::private::Sealed for Rsa<'d, crate::Blocking> {}

impl<'d> InterruptConfigurable for Rsa<'d, crate::Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.internal_set_interrupt_handler(handler);
    }
}

#[cfg(feature = "async")]
impl<'d> Rsa<'d, crate::Async> {
    /// Create a new instance in [crate::Blocking] mode.
    pub fn new_async(rsa: impl Peripheral<P = RSA> + 'd) -> Self {
        let mut this = Self::new_internal(rsa);
        this.internal_set_interrupt_handler(asynch::rsa_interrupt_handler);
        this
    }
}

impl<'d, DM: crate::Mode> Rsa<'d, DM> {
    fn new_internal(rsa: impl Peripheral<P = RSA> + 'd) -> Self {
        crate::into_ref!(rsa);

        PeripheralClockControl::reset(PeripheralEnable::Rsa);
        PeripheralClockControl::enable(PeripheralEnable::Rsa);

        Self {
            rsa,
            phantom: PhantomData,
        }
    }

    unsafe fn write_operand_b<const N: usize>(&mut self, operand_b: &[u32; N]) {
        copy_nonoverlapping(operand_b.as_ptr(), self.rsa.y_mem(0).as_ptr(), N);
    }

    unsafe fn write_modulus<const N: usize>(&mut self, modulus: &[u32; N]) {
        copy_nonoverlapping(modulus.as_ptr(), self.rsa.m_mem(0).as_ptr(), N);
    }

    fn write_mprime(&mut self, m_prime: u32) {
        self.rsa.m_prime().write(|w| unsafe { w.bits(m_prime) });
    }

    unsafe fn write_operand_a<const N: usize>(&mut self, operand_a: &[u32; N]) {
        copy_nonoverlapping(operand_a.as_ptr(), self.rsa.x_mem(0).as_ptr(), N);
    }

    unsafe fn write_r<const N: usize>(&mut self, r: &[u32; N]) {
        copy_nonoverlapping(r.as_ptr(), self.rsa.z_mem(0).as_ptr(), N);
    }

    unsafe fn read_out<const N: usize>(&mut self, outbuf: &mut [u32; N]) {
        copy_nonoverlapping(
            self.rsa.z_mem(0).as_ptr() as *const u32,
            outbuf.as_ptr() as *mut u32,
            N,
        );
    }
}

/// Defines an RSA operation mode.
pub trait RsaMode: crate::private::Sealed {
    /// The input data type used for the RSA operation.
    type InputType;
}

/// A trait for RSA operations that involve multiple inputs and outputs.
pub trait Multi: RsaMode {
    /// The type of the output produced by the RSA operation.
    type OutputType;
}

macro_rules! implement_op {
    (($x:literal, multi)) => {
        paste! {
            /// Represents an RSA operation for the given bit size with multi-output.
            pub struct [<Op $x>];
        }
        paste! {
            impl Multi for [<Op $x>] {
                type OutputType = [u32; $x * 2 / 32];
            }
        }
        paste! {
            impl crate::private::Sealed for [<Op $x>] {}
        }
        paste! {
            impl RsaMode for [<Op $x>] {
                type InputType = [u32; $x / 32];
            }
        }
    };

    (($x:literal)) => {
        paste! {
            /// Represents an RSA operation for the given bit size.
            pub struct [<Op $x>];
        }
        paste! {
            impl crate::private::Sealed for [<Op $x>] {}
        }
        paste! {
            impl RsaMode for [<Op $x>] {
                type InputType = [u32; $x / 32];
            }
        }
    };

    ($x:tt, $($y:tt),+) => {
        implement_op!($x);
        implement_op!($($y),+);
    };
}

use implement_op;

/// Support for RSA peripheral's modular exponentiation feature that could be
/// used to find the `(base ^ exponent) mod modulus`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaModularExponentiation<'a, 'd, T: RsaMode, DM: crate::Mode> {
    rsa: &'a mut Rsa<'d, DM>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode, DM: crate::Mode, const N: usize> RsaModularExponentiation<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// starts the modular exponentiation operation. `r` could be calculated
    /// using `2 ^ ( bitlength * 2 ) mod modulus`, for more information
    /// check 24.3.2 in the <https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf>
    pub fn start_exponentiation(&mut self, base: &T::InputType, r: &T::InputType) {
        unsafe {
            self.rsa.write_operand_a(base);
            self.rsa.write_r(r);
        }
        self.set_start();
    }

    /// reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully. `start_exponentiation` must be
    /// called before calling this function.
    pub fn read_results(&mut self, outbuf: &mut T::InputType) {
        loop {
            if self.rsa.is_idle() {
                unsafe {
                    self.rsa.read_out(outbuf);
                }
                self.rsa.clear_interrupt();
                break;
            }
        }
    }
}

/// Support for RSA peripheral's modular multiplication feature that could be
/// used to find the `(operand a * operand b) mod modulus`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaModularMultiplication<'a, 'd, T: RsaMode, DM: crate::Mode> {
    rsa: &'a mut Rsa<'d, DM>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode, DM: crate::Mode, const N: usize> RsaModularMultiplication<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully.
    pub fn read_results(&mut self, outbuf: &mut T::InputType) {
        loop {
            if self.rsa.is_idle() {
                unsafe {
                    self.rsa.read_out(outbuf);
                }
                self.rsa.clear_interrupt();
                break;
            }
        }
    }
}

/// Support for RSA peripheral's large number multiplication feature that could
/// be used to find the `operand a * operand b`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaMultiplication<'a, 'd, T: RsaMode + Multi, DM: crate::Mode> {
    rsa: &'a mut Rsa<'d, DM>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode + Multi, DM: crate::Mode, const N: usize> RsaMultiplication<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully. `start_multiplication` must be
    /// called before calling this function.
    pub fn read_results<const O: usize>(&mut self, outbuf: &mut T::OutputType)
    where
        T: Multi<OutputType = [u32; O]>,
    {
        loop {
            if self.rsa.is_idle() {
                unsafe {
                    self.rsa.read_out(outbuf);
                }
                self.rsa.clear_interrupt();
                break;
            }
        }
    }
}

/// Async functionality
#[cfg(feature = "async")]
pub(crate) mod asynch {
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::handler;

    use crate::rsa::{
        Multi,
        RsaMode,
        RsaModularExponentiation,
        RsaModularMultiplication,
        RsaMultiplication,
    };

    static WAKER: AtomicWaker = AtomicWaker::new();

    /// `Future` that waits for the RSA operation to complete.
    pub(crate) struct RsaFuture<'d> {
        instance: &'d crate::peripherals::RSA,
    }

    impl<'d> RsaFuture<'d> {
        /// Asynchronously initializes the RSA peripheral.
        pub async fn new(instance: &'d crate::peripherals::RSA) -> Self {
            #[cfg(not(any(esp32, esp32s2, esp32s3)))]
            instance.int_ena().modify(|_, w| w.int_ena().set_bit());

            #[cfg(any(esp32s2, esp32s3))]
            instance
                .interrupt_ena()
                .modify(|_, w| w.interrupt_ena().set_bit());

            #[cfg(esp32)]
            instance.interrupt().modify(|_, w| w.interrupt().set_bit());

            Self { instance }
        }

        fn event_bit_is_clear(&self) -> bool {
            #[cfg(not(any(esp32, esp32s2, esp32s3)))]
            return self.instance.int_ena().read().int_ena().bit_is_clear();

            #[cfg(any(esp32s2, esp32s3))]
            return self
                .instance
                .interrupt_ena()
                .read()
                .interrupt_ena()
                .bit_is_clear();

            #[cfg(esp32)]
            return self.instance.interrupt().read().interrupt().bit_is_clear();
        }
    }

    impl<'d> core::future::Future for RsaFuture<'d> {
        type Output = ();

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            WAKER.register(cx.waker());
            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<'a, 'd, T: RsaMode, const N: usize> RsaModularExponentiation<'a, 'd, T, crate::Async>
    where
        T: RsaMode<InputType = [u32; N]>,
    {
        /// Asynchronously performs an RSA modular exponentiation operation.
        pub async fn exponentiation(
            &mut self,
            base: &T::InputType,
            r: &T::InputType,
            outbuf: &mut T::InputType,
        ) {
            self.start_exponentiation(base, r);
            RsaFuture::new(&self.rsa.rsa).await;
            self.read_results(outbuf);
        }
    }

    impl<'a, 'd, T: RsaMode, const N: usize> RsaModularMultiplication<'a, 'd, T, crate::Async>
    where
        T: RsaMode<InputType = [u32; N]>,
    {
        #[cfg(not(esp32))]
        /// Asynchronously performs an RSA modular multiplication operation.
        pub async fn modular_multiplication(
            &mut self,
            r: &T::InputType,
            outbuf: &mut T::InputType,
        ) {
            self.start_modular_multiplication(r);
            RsaFuture::new(&self.rsa.rsa).await;
            self.read_results(outbuf);
        }

        #[cfg(esp32)]
        /// Asynchronously performs an RSA modular multiplication operation.
        pub async fn modular_multiplication(
            &mut self,
            operand_a: &T::InputType,
            operand_b: &T::InputType,
            r: &T::InputType,
            outbuf: &mut T::InputType,
        ) {
            self.start_step1(operand_a, r);
            self.start_step2(operand_b);
            RsaFuture::new(&self.rsa.rsa).await;
            self.read_results(outbuf);
        }
    }

    impl<'a, 'd, T: RsaMode + Multi, const N: usize> RsaMultiplication<'a, 'd, T, crate::Async>
    where
        T: RsaMode<InputType = [u32; N]>,
    {
        #[cfg(not(esp32))]
        /// Asynchronously performs an RSA multiplication operation.
        pub async fn multiplication<'b, const O: usize>(
            &mut self,
            operand_b: &T::InputType,
            outbuf: &mut T::OutputType,
        ) where
            T: Multi<OutputType = [u32; O]>,
        {
            self.start_multiplication(operand_b);
            RsaFuture::new(&self.rsa.rsa).await;
            self.read_results(outbuf);
        }

        #[cfg(esp32)]
        /// Asynchronously performs an RSA multiplication operation.
        pub async fn multiplication<'b, const O: usize>(
            &mut self,
            operand_a: &T::InputType,
            operand_b: &T::InputType,
            outbuf: &mut T::OutputType,
        ) where
            T: Multi<OutputType = [u32; O]>,
        {
            self.start_multiplication(operand_a, operand_b);
            RsaFuture::new(&self.rsa.rsa).await;
            self.read_results(outbuf);
        }
    }

    #[handler]
    /// Interrupt handler for RSA.
    pub(super) fn rsa_interrupt_handler() {
        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        unsafe { &*crate::peripherals::RSA::ptr() }
            .int_ena()
            .modify(|_, w| w.int_ena().clear_bit());

        #[cfg(esp32)]
        unsafe { &*crate::peripherals::RSA::ptr() }
            .interrupt()
            .modify(|_, w| w.interrupt().clear_bit());

        #[cfg(any(esp32s2, esp32s3))]
        unsafe { &*crate::peripherals::RSA::ptr() }
            .interrupt_ena()
            .modify(|_, w| w.interrupt_ena().clear_bit());

        WAKER.wake();
    }
}
