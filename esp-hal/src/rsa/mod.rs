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
//! [RSA test suite]: https://github.com/esp-rs/esp-hal/blob/main/hil-test/tests/rsa.rs

use core::{marker::PhantomData, task::Poll};

use portable_atomic::{AtomicBool, Ordering};
use procmacros::handler;

use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    interrupt::InterruptHandler,
    pac,
    peripherals::{Interrupt, RSA},
    system::{Cpu, GenericPeripheralGuard, Peripheral as PeripheralEnable},
    trm_markdown_link,
};

/// RSA peripheral driver.
pub struct Rsa<'d, Dm: DriverMode> {
    rsa: RSA<'d>,
    phantom: PhantomData<Dm>,
    _guard: GenericPeripheralGuard<{ PeripheralEnable::Rsa as u8 }>,
}

// There are two distinct peripheral versions: ESP32, and all else. There is a naming split in the
// later devices, and they use different (memory size, operand size increment) parameters, but they
// are largely the same.

/// How many words are there in an operand size increment.
///
/// I.e. if the RSA hardware works with operands of 512, 1024, 1536, ... bits, the increment is 512
/// bits, or 16 words.
const WORDS_PER_INCREMENT: u32 = property!("rsa.size_increment") / 32;

impl<'d> Rsa<'d, Blocking> {
    /// Create a new instance in [Blocking] mode.
    ///
    /// Optionally an interrupt handler can be bound.
    pub fn new(rsa: RSA<'d>) -> Self {
        let guard = GenericPeripheralGuard::new();

        let this = Self {
            rsa,
            phantom: PhantomData,
            _guard: guard,
        };

        while !this.ready() {}

        this
    }

    /// Reconfigures the RSA driver to operate in asynchronous mode.
    pub fn into_async(mut self) -> Rsa<'d, Async> {
        self.set_interrupt_handler(rsa_interrupt_handler);
        self.enable_disable_interrupt(true);

        Rsa {
            rsa: self.rsa,
            phantom: PhantomData,
            _guard: self._guard,
        }
    }

    /// Enables/disables rsa interrupt.
    ///
    /// When enabled rsa peripheral would generate an interrupt when a operation
    /// is finished.
    pub fn enable_disable_interrupt(&mut self, enable: bool) {
        self.internal_enable_disable_interrupt(enable);
    }

    /// Registers an interrupt handler for the RSA peripheral.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.rsa.disable_peri_interrupt();

        self.rsa.bind_peri_interrupt(handler.handler());
        self.rsa.enable_peri_interrupt(handler.priority());
    }
}

impl crate::private::Sealed for Rsa<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Rsa<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

impl<'d> Rsa<'d, Async> {
    /// Create a new instance in [crate::Blocking] mode.
    pub fn into_blocking(self) -> Rsa<'d, Blocking> {
        self.internal_enable_disable_interrupt(false);
        self.rsa.disable_peri_interrupt();

        crate::interrupt::disable(Cpu::current(), Interrupt::RSA);
        Rsa {
            rsa: self.rsa,
            phantom: PhantomData,
            _guard: self._guard,
        }
    }
}

impl<'d, Dm: DriverMode> Rsa<'d, Dm> {
    fn internal_enable_disable_interrupt(&self, enable: bool) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                // Can't seem to actually disable the interrupt, but esp-idf still writes the register
                self.regs().interrupt().write(|w| w.interrupt().bit(enable));
            } else {
                self.regs().int_ena().write(|w| w.int_ena().bit(enable));
            }
        }
    }

    fn regs(&self) -> &pac::rsa::RegisterBlock {
        self.rsa.register_block()
    }

    /// After the RSA accelerator is released from reset, the memory blocks
    /// needs to be initialized, only after that peripheral should be used.
    /// This function would return without an error if the memory is
    /// initialized.
    fn ready(&self) -> bool {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2, esp32s3))] {
                self.regs().clean().read().clean().bit_is_set()
            } else {
                self.regs().query_clean().read().query_clean().bit_is_set()
            }
        }
    }

    /// Starts the modular exponentiation operation.
    fn start_modexp(&self) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2, esp32s3))] {
                self.regs()
                    .modexp_start()
                    .write(|w| w.modexp_start().set_bit());
            } else {
                self.regs()
                    .set_start_modexp()
                    .write(|w| w.set_start_modexp().set_bit());
            }
        }
    }

    /// Starts the multiplication operation.
    fn start_multi(&self) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2, esp32s3))] {
                self.regs().mult_start().write(|w| w.mult_start().set_bit());
            } else {
                self.regs()
                    .set_start_mult()
                    .write(|w| w.set_start_mult().set_bit());
            }
        }
    }

    /// Starts the modular multiplication operation.
    fn start_modmulti(&self) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                // modular-ness is encoded in the multi_mode register value
                self.start_multi();
            } else if #[cfg(any(esp32s2, esp32s3))] {
                self.regs()
                    .modmult_start()
                    .write(|w| w.modmult_start().set_bit());
            } else {
                self.regs()
                    .set_start_modmult()
                    .write(|w| w.set_start_modmult().set_bit());
            }
        }
    }

    /// Clears the RSA interrupt flag.
    fn clear_interrupt(&mut self) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                self.regs().interrupt().write(|w| w.interrupt().set_bit());
            } else {
                self.regs().int_clr().write(|w| w.int_clr().set_bit());
            }
        }
    }

    /// Checks if the RSA peripheral is idle.
    fn is_idle(&self) -> bool {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                self.regs().interrupt().read().interrupt().bit_is_set()
            } else if #[cfg(any(esp32s2, esp32s3))] {
                self.regs().idle().read().idle().bit_is_set()
            } else {
                self.regs().query_idle().read().query_idle().bit_is_set()
            }
        }
    }

    fn wait_for_idle(&mut self) {
        while !self.is_idle() {}
        self.clear_interrupt();
    }

    /// Writes the result size of the multiplication.
    fn write_multi_mode(&mut self, mode: u32, modular: bool) {
        let mode = if cfg!(esp32) && !modular {
            const NON_MODULAR: u32 = 8;
            mode | NON_MODULAR
        } else {
            mode
        };

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                self.regs().mult_mode().write(|w| unsafe { w.bits(mode) });
            } else {
                self.regs().mode().write(|w| unsafe { w.bits(mode) });
            }
        }
    }

    /// Writes the result size of the modular exponentiation.
    fn write_modexp_mode(&mut self, mode: u32) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                self.regs().modexp_mode().write(|w| unsafe { w.bits(mode) });
            } else {
                self.regs().mode().write(|w| unsafe { w.bits(mode) });
            }
        }
    }

    fn write_operand_b(&mut self, operand: &[u32]) {
        for (reg, op) in self.regs().y_mem_iter().zip(operand.iter().copied()) {
            reg.write(|w| unsafe { w.bits(op) });
        }
    }

    fn write_modulus(&mut self, modulus: &[u32]) {
        for (reg, op) in self.regs().m_mem_iter().zip(modulus.iter().copied()) {
            reg.write(|w| unsafe { w.bits(op) });
        }
    }

    fn write_mprime(&mut self, m_prime: u32) {
        self.regs().m_prime().write(|w| unsafe { w.bits(m_prime) });
    }

    fn write_operand_a(&mut self, operand: &[u32]) {
        for (reg, op) in self.regs().x_mem_iter().zip(operand.iter().copied()) {
            reg.write(|w| unsafe { w.bits(op) });
        }
    }

    fn write_multi_operand_b(&mut self, operand: &[u32]) {
        for (reg, op) in self
            .regs()
            .z_mem_iter()
            .skip(operand.len())
            .zip(operand.iter().copied())
        {
            reg.write(|w| unsafe { w.bits(op) });
        }
    }

    fn write_r(&mut self, r: &[u32]) {
        for (reg, op) in self.regs().z_mem_iter().zip(r.iter().copied()) {
            reg.write(|w| unsafe { w.bits(op) });
        }
    }

    fn read_out(&self, outbuf: &mut [u32]) {
        for (reg, op) in self.regs().z_mem_iter().zip(outbuf.iter_mut()) {
            *op = reg.read().bits();
        }
    }

    fn read_results(&mut self, outbuf: &mut [u32]) {
        self.wait_for_idle();
        self.read_out(outbuf);
    }

    /// Enables/disables constant time acceleration.
    ///
    /// When enabled it would increase the performance of modular
    /// exponentiation by simplifying the calculation concerning the 0 bits
    /// of the exponent. I.e. the less the Hamming weight, the greater the
    /// performance.
    ///
    /// Note: this compromises security by enabling timing-based side-channel attacks.
    ///
    /// For more information refer to the
    #[doc = trm_markdown_link!("rsa")]
    #[cfg(not(esp32))]
    pub fn disable_constant_time(&mut self, accelerate: bool) {
        self.regs()
            .constant_time()
            .write(|w| w.constant_time().bit(accelerate));
    }

    /// Enables/disables search acceleration.
    ///
    /// When enabled it would increase the performance of modular
    /// exponentiation by discarding the exponent's bits before the most
    /// significant set bit.
    ///
    /// Note: this compromises security by effectively decreasing the key length.
    ///
    /// For more information refer to the
    #[doc = trm_markdown_link!("rsa")]
    #[cfg(not(esp32))]
    pub fn search_acceleration(&mut self, enable: bool) {
        self.regs()
            .search_enable()
            .write(|w| w.search_enable().bit(enable));
    }

    /// Checks if the search functionality is enabled in the RSA hardware.
    #[cfg(not(esp32))]
    fn is_search_enabled(&mut self) -> bool {
        self.regs()
            .search_enable()
            .read()
            .search_enable()
            .bit_is_set()
    }

    /// Sets the search position in the RSA hardware.
    #[cfg(not(esp32))]
    fn write_search_position(&mut self, search_position: u32) {
        self.regs()
            .search_pos()
            .write(|w| unsafe { w.bits(search_position) });
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

/// Defines the exponentiation and multiplication lengths for RSA operations.
pub mod operand_sizes {
    for_each_rsa_exponentiation!(
        ($x:literal) => {
            paste::paste! {
                #[doc = concat!(stringify!($x), "-bit RSA operation.")]
                pub struct [<Op $x>];

                impl crate::private::Sealed for [<Op $x>] {}
                impl crate::rsa::RsaMode for [<Op $x>] {
                    type InputType = [u32; $x / 32];
                }
            }
        };
    );

    for_each_rsa_multiplication!(
        ($x:literal) => {
            impl crate::rsa::Multi for paste::paste!( [<Op $x>] ) {
                type OutputType = [u32; $x * 2 / 32];
            }
        };
    );
}

/// Support for RSA peripheral's modular exponentiation feature that could be
/// used to find the `(base ^ exponent) mod modulus`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaModularExponentiation<'a, 'd, T: RsaMode, Dm: DriverMode> {
    rsa: &'a mut Rsa<'d, Dm>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode, Dm: DriverMode, const N: usize> RsaModularExponentiation<'a, 'd, T, Dm>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Creates an instance of `RsaModularExponentiation`.
    ///
    /// `m_prime` could be calculated using `-(modular multiplicative inverse of
    /// modulus) mod 2^32`.
    ///
    /// For more information refer to the
    #[doc = trm_markdown_link!("rsa")]
    pub fn new(
        rsa: &'a mut Rsa<'d, Dm>,
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
    /// For more information refer to the
    #[doc = trm_markdown_link!("rsa")]
    pub fn start_exponentiation(&mut self, base: &T::InputType, r: &T::InputType) {
        self.set_up_exponentiation(base, r);
        self.rsa.start_modexp();
    }

    /// Reads the result to the given buffer.
    ///
    /// This is a blocking function. `start_exponentiation` must be
    /// called before calling this function.
    pub fn read_results(&mut self, outbuf: &mut T::InputType) {
        self.rsa.read_results(outbuf);
    }

    #[cfg(not(esp32))]
    fn find_search_pos(exponent: &T::InputType) -> u32 {
        for (i, byte) in exponent.iter().rev().enumerate() {
            if *byte == 0 {
                continue;
            }
            return (exponent.len() * 32) as u32 - (byte.leading_zeros() + i as u32 * 32) - 1;
        }
        0
    }

    /// Sets the modular exponentiation mode for the RSA hardware.
    fn write_mode(rsa: &mut Rsa<'d, Dm>) {
        rsa.write_modexp_mode(N as u32 / WORDS_PER_INCREMENT - 1);
    }
}

/// Support for RSA peripheral's modular multiplication feature that could be
/// used to find the `(operand a * operand b) mod modulus`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaModularMultiplication<'a, 'd, T, Dm>
where
    T: RsaMode,
    Dm: DriverMode,
{
    rsa: &'a mut Rsa<'d, Dm>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T, Dm, const N: usize> RsaModularMultiplication<'a, 'd, T, Dm>
where
    T: RsaMode<InputType = [u32; N]>,
    Dm: DriverMode,
{
    /// Creates an instance of `RsaModularMultiplication`.
    ///
    /// - `r` can be calculated using `2 ^ ( bitlength * 2 ) mod modulus`.
    /// - `m_prime` can be calculated using `-(modular multiplicative inverse of modulus) mod 2^32`.
    ///
    /// For more information refer to the
    #[doc = trm_markdown_link!("rsa")]
    pub fn new(
        rsa: &'a mut Rsa<'d, Dm>,
        operand_a: &T::InputType,
        modulus: &T::InputType,
        r: &T::InputType,
        m_prime: u32,
    ) -> Self {
        rsa.write_multi_mode(N as u32 / WORDS_PER_INCREMENT - 1, true);

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
    /// For more information refer to the
    #[doc = trm_markdown_link!("rsa")]
    pub fn start_modular_multiplication(&mut self, operand_b: &T::InputType) {
        self.set_up_modular_multiplication(operand_b);
        self.rsa.start_modmulti();
    }

    /// Reads the result to the given buffer.
    ///
    /// This is a blocking function. `start_modular_multiplication` must be
    /// called before calling this function.
    pub fn read_results(&mut self, outbuf: &mut T::InputType) {
        self.rsa.read_results(outbuf);
    }

    fn set_up_modular_multiplication(&mut self, operand_b: &T::InputType) {
        if cfg!(esp32) {
            self.rsa.start_multi();
            self.rsa.wait_for_idle();

            self.rsa.write_operand_a(operand_b);
        } else {
            self.rsa.write_operand_b(operand_b);
        }
    }
}

/// Support for RSA peripheral's large number multiplication feature that could
/// be used to find the `operand a * operand b`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaMultiplication<'a, 'd, T, Dm>
where
    T: RsaMode + Multi,
    Dm: DriverMode,
{
    rsa: &'a mut Rsa<'d, Dm>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T, Dm, const N: usize> RsaMultiplication<'a, 'd, T, Dm>
where
    T: RsaMode<InputType = [u32; N]>,
    T: Multi,
    Dm: DriverMode,
{
    /// Creates an instance of `RsaMultiplication`.
    pub fn new(rsa: &'a mut Rsa<'d, Dm>, operand_a: &T::InputType) -> Self {
        // Non-modular multiplication result is twice as wide as its operands.
        rsa.write_multi_mode(2 * N as u32 / WORDS_PER_INCREMENT - 1, false);
        rsa.write_operand_a(operand_a);

        Self {
            rsa,
            phantom: PhantomData,
        }
    }

    /// Starts the multiplication operation.
    pub fn start_multiplication(&mut self, operand_b: &T::InputType) {
        self.set_up_multiplication(operand_b);
        self.rsa.start_multi();
    }

    /// Reads the result to the given buffer.
    ///
    /// This is a blocking function. `start_multiplication` must be
    /// called before calling this function.
    pub fn read_results<const O: usize>(&mut self, outbuf: &mut T::OutputType)
    where
        T: Multi<OutputType = [u32; O]>,
    {
        self.rsa.read_results(outbuf);
    }

    fn set_up_multiplication(&mut self, operand_b: &T::InputType) {
        self.rsa.write_multi_operand_b(operand_b);
    }
}

static WAKER: AtomicWaker = AtomicWaker::new();
// TODO: this should only be needed for ESP32
static SIGNALED: AtomicBool = AtomicBool::new(false);

/// `Future` that waits for the RSA operation to complete.
#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RsaFuture<'a, 'd> {
    driver: &'a Rsa<'d, Async>,
}

impl<'a, 'd> RsaFuture<'a, 'd> {
    fn new(driver: &'a Rsa<'d, Async>) -> Self {
        SIGNALED.store(false, Ordering::Relaxed);

        driver.internal_enable_disable_interrupt(true);

        Self { driver }
    }

    fn is_done(&self) -> bool {
        SIGNALED.load(Ordering::Acquire)
    }
}

impl Drop for RsaFuture<'_, '_> {
    fn drop(&mut self) {
        self.driver.internal_enable_disable_interrupt(false);
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

impl<T: RsaMode, const N: usize> RsaModularExponentiation<'_, '_, T, Async>
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
        self.rsa.start_modexp();
        fut.await;
        self.rsa.read_out(outbuf);
    }
}

impl<T: RsaMode, const N: usize> RsaModularMultiplication<'_, '_, T, Async>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Asynchronously performs an RSA modular multiplication operation.
    pub async fn modular_multiplication(
        &mut self,
        operand_b: &T::InputType,
        outbuf: &mut T::InputType,
    ) {
        if cfg!(esp32) {
            let fut = RsaFuture::new(self.rsa);
            self.rsa.start_multi();
            fut.await;

            self.rsa.write_operand_a(operand_b);
        } else {
            self.set_up_modular_multiplication(operand_b);
        }

        let fut = RsaFuture::new(self.rsa);
        self.rsa.start_modmulti();
        fut.await;
        self.rsa.read_out(outbuf);
    }
}

impl<T: RsaMode + Multi, const N: usize> RsaMultiplication<'_, '_, T, Async>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Asynchronously performs an RSA multiplication operation.
    pub async fn multiplication<const O: usize>(
        &mut self,
        operand_b: &T::InputType,
        outbuf: &mut T::OutputType,
    ) where
        T: Multi<OutputType = [u32; O]>,
    {
        self.set_up_multiplication(operand_b);
        let fut = RsaFuture::new(self.rsa);
        self.rsa.start_multi();
        fut.await;
        self.rsa.read_out(outbuf);
    }
}

#[handler]
/// Interrupt handler for RSA.
pub(super) fn rsa_interrupt_handler() {
    let rsa = RSA::regs();
    SIGNALED.store(true, Ordering::Release);
    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            rsa.interrupt().write(|w| w.interrupt().set_bit());
        } else  {
            rsa.int_clr().write(|w| w.int_clr().set_bit());
        }
    }

    WAKER.wake();
}
