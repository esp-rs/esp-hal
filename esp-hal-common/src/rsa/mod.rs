//! # RSA Accelerator support.
//!
//! ## Overview
//! The `RSA` driver provides a set of functions to accelerate `RSA
//! (Rivest–Shamir–Adleman)` cryptographic operations on ESP chips. `RSA` is a
//! widely used `public-key` cryptographic algorithm that involves complex
//! mathematical computations, and the `RSA` accelerator on `ESP` chips is
//! designed to optimize these computations for faster performance.
//!
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
//! ## Examples
//! ### Initialization
//! ```no_run
//! let peripherals = Peripherals::take();
//! let mut system = peripherals.PCR.split();
//!
//! let mut rsa = Rsa::new(peripherals.RSA);
//! ```
//!
//! ⚠️: The examples for RSA peripheral are quite extensive, so for a more
//! detailed study of how to use this driver please visit [the repository
//! with corresponding example].
//!
//! [nb]: https://docs.rs/nb/1.1.0/nb/
//! [the repository with corresponding example]: https://github.com/esp-rs/esp-hal/blob/main/esp32-hal/examples/rsa.rs

use core::{convert::Infallible, marker::PhantomData, ptr::copy_nonoverlapping};

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::RSA,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
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
pub struct Rsa<'d> {
    rsa: PeripheralRef<'d, RSA>,
}

impl<'d> Rsa<'d> {
    pub fn new(rsa: impl Peripheral<P = RSA> + 'd) -> Self {
        crate::into_ref!(rsa);

        PeripheralClockControl::enable(PeripheralEnable::Rsa);

        Self { rsa }
    }

    unsafe fn write_operand_b<const N: usize>(&mut self, operand_b: &[u8; N]) {
        copy_nonoverlapping(
            operand_b.as_ptr(),
            self.rsa.y_mem.as_mut_ptr() as *mut u8,
            N,
        );
    }

    unsafe fn write_modulus<const N: usize>(&mut self, modulus: &[u8; N]) {
        copy_nonoverlapping(modulus.as_ptr(), self.rsa.m_mem.as_mut_ptr() as *mut u8, N);
    }

    fn write_mprime(&mut self, m_prime: u32) {
        self.rsa.m_prime.write(|w| unsafe { w.bits(m_prime) });
    }

    unsafe fn write_operand_a<const N: usize>(&mut self, operand_a: &[u8; N]) {
        copy_nonoverlapping(
            operand_a.as_ptr(),
            self.rsa.x_mem.as_mut_ptr() as *mut u8,
            N,
        );
    }

    unsafe fn write_r<const N: usize>(&mut self, r: &[u8; N]) {
        copy_nonoverlapping(r.as_ptr(), self.rsa.z_mem.as_mut_ptr() as *mut u8, N);
    }

    unsafe fn read_out<const N: usize>(&mut self, outbuf: &mut [u8; N]) {
        copy_nonoverlapping(self.rsa.z_mem.as_ptr() as *const u8, outbuf.as_mut_ptr(), N);
    }
}

mod sealed {
    pub trait RsaMode {
        type InputType;
    }
    pub trait Multi: RsaMode {
        type OutputType;
    }
}

pub(self) use sealed::*;

macro_rules! implement_op {

    (($x:literal, multi)) => {
    paste! {pub struct [<Op $x>];}
    paste! {
        impl Multi for [<Op $x>] {
        type OutputType = [u8; $x*2 / 8];
    }}
    paste!{
    impl RsaMode for [<Op $x>] {
        type InputType = [u8; $x / 8];
    }}
    };

    (($x:literal)) => {
        paste! {pub struct [<Op $x>];}
    paste!{
    impl RsaMode for [<Op $x>] {
        type InputType =  [u8; $x / 8];
    }}
     };

    ($x:tt, $($y:tt),+) => {
        implement_op!($x);
        implement_op!($($y),+);
    };
}

pub(self) use implement_op;

#[cfg(feature = "async")]
pub(crate) mod asynch {
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::interrupt;

    static WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) struct RsaFuture<'d> {
        instance: &'d crate::peripherals::RSA,
    }

    impl<'d> RsaFuture<'d> {
        pub fn new(instance: &'d crate::peripherals::RSA) -> Self {
            #[cfg(not(any(esp32, esp32s2, esp32s3)))]
            instance.int_ena.modify(|_, w| w.int_ena().set_bit());

            #[cfg(any(esp32s2, esp32s3))]
            instance
                .interrupt_ena
                .modify(|_, w| w.interrupt_ena().set_bit());

            #[cfg(esp32)]
            instance.interrupt.modify(|_, w| w.interrupt().set_bit());

            Self { instance }
        }

        fn event_bit_is_clear(&self) -> bool {
            #[cfg(not(any(esp32, esp32s2, esp32s3)))]
            return self.instance.int_ena.read().int_ena().bit_is_clear();

            #[cfg(any(esp32s2, esp32s3))]
            return self
                .instance
                .interrupt_ena
                .read()
                .interrupt_ena()
                .bit_is_clear();

            #[cfg(esp32)]
            return self.instance.interrupt.read().interrupt().bit_is_clear();
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

    #[interrupt]
    fn RSA() {
        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        unsafe { &*crate::peripherals::RSA::ptr() }
            .int_ena
            .modify(|_, w| w.int_ena().clear_bit());

        #[cfg(esp32)]
        unsafe { &*crate::peripherals::RSA::ptr() }
            .interrupt
            .modify(|_, w| w.interrupt().clear_bit());

        #[cfg(any(esp32s2, esp32s3))]
        unsafe { &*crate::peripherals::RSA::ptr() }
            .interrupt_ena
            .modify(|_, w| w.interrupt_ena().clear_bit());

        WAKER.wake();
    }
}

/// Support for RSA peripheral's modular exponentiation feature that could be
/// used to find the `(base ^ exponent) mod modulus`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaModularExponentiation<'a, 'd, T: RsaMode> {
    rsa: &'a mut Rsa<'d>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode, const N: usize> RsaModularExponentiation<'a, 'd, T>
where
    T: RsaMode<InputType = [u8; N]>,
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
    pub fn read_results(&mut self, outbuf: &mut T::InputType) -> nb::Result<(), Infallible> {
        if !self.rsa.is_idle() {
            return Err(nb::Error::WouldBlock);
        }
        unsafe {
            self.rsa.read_out(outbuf);
        }
        self.rsa.clear_interrupt();
        Ok(())
    }

    #[cfg(feature = "async")]
    pub async fn async_exponentiation(
        &mut self,
        base: &T::InputType,
        r: &T::InputType,
        outbuf: &mut T::InputType,
    ) {
        self.start_exponentiation(&base, &r);
        asynch::RsaFuture::new(&self.rsa.rsa);
        nb::block!(self.read_results(outbuf)).unwrap();
    }
}

/// Support for RSA peripheral's modular multiplication feature that could be
/// used to find the `(operand a * operand b) mod modulus`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaModularMultiplication<'a, 'd, T: RsaMode> {
    rsa: &'a mut Rsa<'d>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode, const N: usize> RsaModularMultiplication<'a, 'd, T>
where
    T: RsaMode<InputType = [u8; N]>,
{
    /// Reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully.
    pub fn read_results(&mut self, outbuf: &mut T::InputType) -> nb::Result<(), Infallible> {
        if !self.rsa.is_idle() {
            return Err(nb::Error::WouldBlock);
        }
        unsafe {
            self.rsa.read_out(outbuf);
        }
        self.rsa.clear_interrupt();
        Ok(())
    }

    #[cfg(feature = "async")]
    #[cfg(not(esp32))]
    pub async fn async_modular_multiplication(
        &mut self,
        r: &T::InputType,
        outbuf: &mut T::InputType,
    ) {
        self.start_modular_multiplication(r);

        asynch::RsaFuture::new(&self.rsa.rsa);

        nb::block!(self.read_results(outbuf)).unwrap();
    }

    #[cfg(feature = "async")]
    #[cfg(esp32)]
    pub async fn async_modular_multiplication(
        &mut self,
        operand_a: &T::InputType,
        operand_b: &T::InputType,
        r: &T::InputType,
        outbuf: &mut T::InputType,
    ) {
        self.start_step1(operand_a, r);
        nb::block!(self.start_step2(operand_b)).unwrap();
        asynch::RsaFuture::new(&self.rsa.rsa);
        nb::block!(self.read_results(outbuf)).unwrap();
    }
}

/// Support for RSA peripheral's large number multiplication feature that could
/// be used to find the `operand a * operand b`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaMultiplication<'a, 'd, T: RsaMode + Multi> {
    rsa: &'a mut Rsa<'d>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode + Multi, const N: usize> RsaMultiplication<'a, 'd, T>
where
    T: RsaMode<InputType = [u8; N]>,
{
    /// Reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully. `start_multiplication` must be
    /// called before calling this function.
    pub fn read_results<'b, const O: usize>(
        &mut self,
        outbuf: &mut T::OutputType,
    ) -> nb::Result<(), Infallible>
    where
        T: Multi<OutputType = [u8; O]>,
    {
        if !self.rsa.is_idle() {
            return Err(nb::Error::WouldBlock);
        }
        unsafe {
            self.rsa.read_out(outbuf);
        }
        self.rsa.clear_interrupt();
        Ok(())
    }

    #[cfg(feature = "async")]
    #[cfg(not(esp32))]
    pub async fn async_multiplication<'b, const O: usize>(
        &mut self,
        operand_b: &T::InputType,
        outbuf: &mut T::OutputType,
    ) where
        T: Multi<OutputType = [u8; O]>,
    {
        self.start_multiplication(operand_b);

        asynch::RsaFuture::new(&self.rsa.rsa);

        nb::block!(self.read_results(outbuf)).unwrap();
    }

    #[cfg(feature = "async")]
    #[cfg(esp32)]
    pub async fn async_multiplication<'b, const O: usize>(
        &mut self,
        operand_a: &T::InputType,
        operand_b: &T::InputType,
        outbuf: &mut T::OutputType,
    ) where
        T: Multi<OutputType = [u8; O]>,
    {
        self.start_multiplication(operand_a, operand_b);

        asynch::RsaFuture::new(&self.rsa.rsa);

        nb::block!(self.read_results(outbuf)).unwrap();
    }
}
