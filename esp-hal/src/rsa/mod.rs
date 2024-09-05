//! # RSA (Rivest–Shamir–Adleman) accelerator.
//!
//! ## Overview
//!
//! The RSA accelerator provides hardware support for high precision computation
//! used in various RSA asymmetric cipher algorithms by significantly reducing
//! their software complexity. Compared with RSA algorithms implemented solely
//! in software, this hardware accelerator can speed up RSA algorithms
//! significantly.
//!
//! ## Configuration
//!
//! The RSA accelerator also supports operands of different lengths, which
//! provides more flexibility during the computation.
//!
//! ## Examples
//!
//! ### Modular Exponentiation, Modular Multiplication, and Multiplication
//! Visit the [RSA test suite] for an example of using the peripheral.
//!
//! [nb]: https://docs.rs/nb/1.1.0/nb/
//! [RSA test suite]: https://github.com/esp-rs/esp-hal/blob/main/hil-test/tests/rsa.rs

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

    fn write_operand_b<const N: usize>(&mut self, operand_b: &[u32; N]) {
        unsafe {
            copy_nonoverlapping(operand_b.as_ptr(), self.rsa.y_mem(0).as_ptr(), N);
        }
    }

    fn write_modulus<const N: usize>(&mut self, modulus: &[u32; N]) {
        unsafe {
            copy_nonoverlapping(modulus.as_ptr(), self.rsa.m_mem(0).as_ptr(), N);
        }
    }

    fn write_mprime(&mut self, m_prime: u32) {
        self.rsa.m_prime().write(|w| unsafe { w.bits(m_prime) });
    }

    fn write_operand_a<const N: usize>(&mut self, operand_a: &[u32; N]) {
        unsafe {
            copy_nonoverlapping(operand_a.as_ptr(), self.rsa.x_mem(0).as_ptr(), N);
        }
    }

    fn write_multi_operand_b<const N: usize>(&mut self, operand_b: &[u32; N]) {
        unsafe {
            copy_nonoverlapping(operand_b.as_ptr(), self.rsa.z_mem(0).as_ptr().add(N), N);
        }
    }

    fn write_r<const N: usize>(&mut self, r: &[u32; N]) {
        unsafe {
            copy_nonoverlapping(r.as_ptr(), self.rsa.z_mem(0).as_ptr(), N);
        }
    }

    fn read_out<const N: usize>(&self, outbuf: &mut [u32; N]) {
        unsafe {
            copy_nonoverlapping(
                self.rsa.z_mem(0).as_ptr() as *const u32,
                outbuf.as_ptr() as *mut u32,
                N,
            );
        }
    }

    fn internal_set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(crate::peripherals::Interrupt::RSA, handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::RSA, handler.priority())
                .unwrap();
        }
    }

    fn wait_for_idle(&mut self) {
        while !self.is_idle() {}
        self.clear_interrupt();
    }

    fn read_results<const N: usize>(&mut self, outbuf: &mut [u32; N]) {
        self.wait_for_idle();
        self.read_out(outbuf);
    }
}

/// Defines the input size of an RSA operation.
pub trait RsaMode: crate::private::Sealed {
    /// The input data type used for the operation.
    type InputType;
}

/// Defines the output type of RSA multiplications.
pub trait Multi: RsaMode {
    /// The type of the output produced by the operation.
    type OutputType;
}

macro_rules! implement_op {
    (($x:literal, multi)) => {
        paste! {
            #[doc = concat!($x, "-bit RSA operation.")]
            pub struct [<Op $x>];

            impl Multi for [<Op $x>] {
                type OutputType = [u32; $x * 2 / 32];
            }

            impl crate::private::Sealed for [<Op $x>] {}

            impl RsaMode for [<Op $x>] {
                type InputType = [u32; $x / 32];
            }
        }
    };

    (($x:literal)) => {
        paste! {
            /// Represents an RSA operation for the given bit size.
            pub struct [<Op $x>];

            impl crate::private::Sealed for [<Op $x>] {}

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
    /// Creates an instance of `RsaModularExponentiation`.
    ///
    /// `m_prime` could be calculated using `-(modular multiplicative inverse of
    /// modulus) mod 2^32`.
    ///
    /// For more information refer to 24.3.2 of <https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf>.
    pub fn new(
        rsa: &'a mut Rsa<'d, DM>,
        exponent: &T::InputType,
        modulus: &T::InputType,
        m_prime: u32,
    ) -> Self {
        Self::write_mode(rsa);
        rsa.write_operand_b(exponent);
        rsa.write_modulus(modulus);
        rsa.write_mprime(m_prime);

        #[cfg(not(esp32))]
        if rsa.is_search_enabled() {
            rsa.write_search_position(Self::find_search_pos(exponent));
        }

        Self {
            rsa,
            phantom: PhantomData,
        }
    }

    fn set_up_exponentiation(&mut self, base: &T::InputType, r: &T::InputType) {
        self.rsa.write_operand_a(base);
        self.rsa.write_r(r);
    }

    /// Starts the modular exponentiation operation.
    ///
    /// `r` can be calculated using `2 ^ ( bitlength * 2 ) mod modulus`.
    ///
    /// For more information refer to 24.3.2 of <https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf>.
    pub fn start_exponentiation(&mut self, base: &T::InputType, r: &T::InputType) {
        self.set_up_exponentiation(base, r);
        self.rsa.write_modexp_start();
    }

    /// Reads the result to the given buffer.
    ///
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully. `start_exponentiation` must be
    /// called before calling this function.
    pub fn read_results(&mut self, outbuf: &mut T::InputType) {
        self.rsa.read_results(outbuf);
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
    /// Creates an instance of `RsaModularMultiplication`.
    ///
    /// - `r` can be calculated using `2 ^ ( bitlength * 2 ) mod modulus`.
    /// - `m_prime` can be calculated using `-(modular multiplicative inverse of
    ///   modulus) mod 2^32`.
    ///
    /// For more information refer to 20.3.1 of <https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf>.
    pub fn new(
        rsa: &'a mut Rsa<'d, DM>,
        operand_a: &T::InputType,
        modulus: &T::InputType,
        r: &T::InputType,
        m_prime: u32,
    ) -> Self {
        Self::write_mode(rsa);
        rsa.write_mprime(m_prime);
        rsa.write_modulus(modulus);
        rsa.write_operand_a(operand_a);
        rsa.write_r(r);

        Self {
            rsa,
            phantom: PhantomData,
        }
    }

    /// Starts the modular multiplication operation.
    ///
    /// For more information refer to 19.3.1 of <https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf>.
    pub fn start_modular_multiplication(&mut self, operand_b: &T::InputType) {
        self.set_up_modular_multiplication(operand_b);
        self.rsa.write_modmulti_start();
    }

    /// Reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully.
    pub fn read_results(&mut self, outbuf: &mut T::InputType) {
        self.rsa.read_results(outbuf);
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
    /// Creates an instance of `RsaMultiplication`.
    pub fn new(rsa: &'a mut Rsa<'d, DM>, operand_a: &T::InputType) -> Self {
        Self::write_mode(rsa);
        rsa.write_operand_a(operand_a);

        Self {
            rsa,
            phantom: PhantomData,
        }
    }

    /// Starts the multiplication operation.
    pub fn start_multiplication(&mut self, operand_b: &T::InputType) {
        self.set_up_multiplication(operand_b);
        self.rsa.write_multi_start();
    }

    /// Reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully. `start_multiplication` must be
    /// called before calling this function.
    pub fn read_results<const O: usize>(&mut self, outbuf: &mut T::OutputType)
    where
        T: Multi<OutputType = [u32; O]>,
    {
        self.rsa.read_results(outbuf);
    }
}

/// Async functionality
pub(crate) mod asynch {
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use portable_atomic::{AtomicBool, Ordering};
    use procmacros::handler;

    use crate::{
        rsa::{
            Multi,
            Rsa,
            RsaMode,
            RsaModularExponentiation,
            RsaModularMultiplication,
            RsaMultiplication,
        },
        Async,
    };

    static WAKER: AtomicWaker = AtomicWaker::new();

    static SIGNALED: AtomicBool = AtomicBool::new(false);

    /// `Future` that waits for the RSA operation to complete.
    #[must_use = "futures do nothing unless you `.await` or poll them"]
    struct RsaFuture<'a, 'd> {
        #[cfg_attr(esp32, allow(dead_code))]
        instance: &'a Rsa<'d, Async>,
    }

    impl<'a, 'd> RsaFuture<'a, 'd> {
        fn new(instance: &'a Rsa<'d, Async>) -> Self {
            SIGNALED.store(false, Ordering::Relaxed);

            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                } else if #[cfg(any(esp32s2, esp32s3))] {
                    instance.rsa.interrupt_ena().write(|w| w.interrupt_ena().set_bit());
                } else {
                    instance.rsa.int_ena().write(|w| w.int_ena().set_bit());
                }
            }

            Self { instance }
        }

        fn is_done(&self) -> bool {
            SIGNALED.load(Ordering::Acquire)
        }
    }

    impl Drop for RsaFuture<'_, '_> {
        fn drop(&mut self) {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                } else if #[cfg(any(esp32s2, esp32s3))] {
                    self.instance.rsa.interrupt_ena().write(|w| w.interrupt_ena().clear_bit());
                } else {
                    self.instance.rsa.int_ena().write(|w| w.int_ena().clear_bit());
                }
            }
        }
    }

    impl core::future::Future for RsaFuture<'_, '_> {
        type Output = ();

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            WAKER.register(cx.waker());
            if self.is_done() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<'a, 'd, T: RsaMode, const N: usize> RsaModularExponentiation<'a, 'd, T, Async>
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
            self.set_up_exponentiation(base, r);
            let fut = RsaFuture::new(self.rsa);
            self.rsa.write_modexp_start();
            fut.await;
            self.rsa.read_out(outbuf);
        }
    }

    impl<'a, 'd, T: RsaMode, const N: usize> RsaModularMultiplication<'a, 'd, T, Async>
    where
        T: RsaMode<InputType = [u32; N]>,
    {
        /// Asynchronously performs an RSA modular multiplication operation.
        pub async fn modular_multiplication(
            &mut self,
            operand_b: &T::InputType,
            outbuf: &mut T::InputType,
        ) {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    let fut = RsaFuture::new(self.rsa);
                    self.rsa.write_multi_start();
                    fut.await;

                    self.rsa.write_operand_a(operand_b);
                } else {
                    self.set_up_modular_multiplication(operand_b);
                }
            }

            let fut = RsaFuture::new(self.rsa);
            self.rsa.write_modmulti_start();
            fut.await;
            self.rsa.read_out(outbuf);
        }
    }

    impl<'a, 'd, T: RsaMode + Multi, const N: usize> RsaMultiplication<'a, 'd, T, Async>
    where
        T: RsaMode<InputType = [u32; N]>,
    {
        /// Asynchronously performs an RSA multiplication operation.
        pub async fn multiplication<'b, const O: usize>(
            &mut self,
            operand_b: &T::InputType,
            outbuf: &mut T::OutputType,
        ) where
            T: Multi<OutputType = [u32; O]>,
        {
            self.set_up_multiplication(operand_b);
            let fut = RsaFuture::new(self.rsa);
            self.rsa.write_multi_start();
            fut.await;
            self.rsa.read_out(outbuf);
        }
    }

    #[handler]
    /// Interrupt handler for RSA.
    pub(super) fn rsa_interrupt_handler() {
        let rsa = unsafe { &*crate::peripherals::RSA::ptr() };
        SIGNALED.store(true, Ordering::Release);
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                rsa.interrupt().write(|w| w.interrupt().set_bit());
            } else if #[cfg(any(esp32s2, esp32s3))] {
                rsa.clear_interrupt().write(|w| w.clear_interrupt().set_bit());
            } else  {
                rsa.int_clr().write(|w| w.clear_interrupt().set_bit());
            }
        }

        WAKER.wake();
    }
}
