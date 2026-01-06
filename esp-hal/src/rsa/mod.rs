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

use core::{marker::PhantomData, ptr::NonNull, task::Poll};

use portable_atomic::{AtomicBool, Ordering};
use procmacros::{handler, ram};

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
    work_queue::{self, Status, VTable, WorkQueue, WorkQueueDriver, WorkQueueFrontend},
};

/// RSA peripheral driver.
pub struct Rsa<'d, Dm: DriverMode> {
    rsa: RSA<'d>,
    phantom: PhantomData<Dm>,
    #[cfg(not(esp32))]
    _memory_guard: RsaMemoryPowerGuard,
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

#[cfg(not(esp32))]
struct RsaMemoryPowerGuard;

#[cfg(not(esp32))]
impl RsaMemoryPowerGuard {
    fn new() -> Self {
        crate::peripherals::SYSTEM::regs()
            .rsa_pd_ctrl()
            .modify(|_, w| w.rsa_mem_pd().clear_bit());
        Self
    }
}

#[cfg(not(esp32))]
impl Drop for RsaMemoryPowerGuard {
    fn drop(&mut self) {
        crate::peripherals::SYSTEM::regs()
            .rsa_pd_ctrl()
            .modify(|_, w| w.rsa_mem_pd().set_bit());
    }
}

impl<'d> Rsa<'d, Blocking> {
    /// Create a new instance in [Blocking] mode.
    ///
    /// Optionally an interrupt handler can be bound.
    pub fn new(rsa: RSA<'d>) -> Self {
        let guard = GenericPeripheralGuard::new();

        let this = Self {
            rsa,
            phantom: PhantomData,
            #[cfg(not(esp32))]
            _memory_guard: RsaMemoryPowerGuard::new(),
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
            #[cfg(not(esp32))]
            _memory_guard: self._memory_guard,
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
            #[cfg(not(esp32))]
            _memory_guard: self._memory_guard,
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

    /// Enables/disables constant time operation.
    ///
    /// Disabling constant time operation increases the performance of modular
    /// exponentiation by simplifying the calculation concerning the 0 bits
    /// of the exponent. I.e. the less the Hamming weight, the greater the
    /// performance.
    ///
    /// Note: this compromises security by enabling timing-based side-channel attacks.
    ///
    /// For more information refer to the
    #[doc = trm_markdown_link!("rsa")]
    #[cfg(not(esp32))]
    pub fn disable_constant_time(&mut self, disable: bool) {
        self.regs()
            .constant_time()
            .write(|w| w.constant_time().bit(disable));
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
    type InputType: AsRef<[u32]> + AsMut<[u32]>;
}

/// Defines the output type of RSA multiplications.
pub trait Multi: RsaMode {
    /// The type of the output produced by the operation.
    type OutputType: AsRef<[u32]> + AsMut<[u32]>;
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
    /// This is a blocking function: it waits for the RSA operation to complete,
    /// then reads the results into the provided buffer. `start_exponentiation` must be
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
    /// This is a blocking function: it waits for the RSA operation to complete,
    /// then reads the results into the provided buffer. `start_modular_multiplication` must be
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
    /// This is a blocking function: it waits for the RSA operation to complete,
    /// then reads the results into the provided buffer. `start_multiplication` must be
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

static RSA_WORK_QUEUE: WorkQueue<RsaWorkItem> = WorkQueue::new();
const RSA_VTABLE: VTable<RsaWorkItem> = VTable {
    post: |driver, item| {
        // Start processing immediately.
        let driver = unsafe { RsaBackend::from_raw(driver) };
        Some(driver.process_item(item))
    },
    poll: |driver, item| {
        let driver = unsafe { RsaBackend::from_raw(driver) };
        driver.process_item(item)
    },
    cancel: |driver, item| {
        let driver = unsafe { RsaBackend::from_raw(driver) };
        driver.cancel(item)
    },
    stop: |driver| {
        let driver = unsafe { RsaBackend::from_raw(driver) };
        driver.deinitialize()
    },
};

#[derive(Default)]
enum RsaBackendState<'d> {
    #[default]
    Idle,
    Initializing(Rsa<'d, Blocking>),
    Ready(Rsa<'d, Blocking>),
    #[cfg(esp32)]
    ModularMultiplicationRoundOne(Rsa<'d, Blocking>),
    Processing(Rsa<'d, Blocking>),
}

#[procmacros::doc_replace]
/// RSA processing backend.
///
/// The backend processes work items placed in the RSA work queue. The backend needs to be created
/// and started for operations to be processed. This allows you to perform operations on the RSA
/// accelerator without carrying around the peripheral singleton, or the driver.
///
/// The [`RsaContext`] struct can enqueue work items that this backend will process.
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::rsa::{RsaBackend, RsaContext, operand_sizes::Op512};
/// #
/// let mut rsa_backend = RsaBackend::new(peripherals.RSA);
/// let _driver = rsa_backend.start();
///
/// async fn perform_512bit_big_number_multiplication(
///     operand_a: &[u32; 16],
///     operand_b: &[u32; 16],
///     result: &mut [u32; 32],
/// ) {
///     let mut rsa = RsaContext::new();
///
///     let mut handle = rsa.multiply::<Op512>(operand_a, operand_b, result);
///     handle.wait().await;
/// }
/// # {after_snippet}
/// ```
pub struct RsaBackend<'d> {
    peri: RSA<'d>,
    state: RsaBackendState<'d>,
}

impl<'d> RsaBackend<'d> {
    #[procmacros::doc_replace]
    /// Creates a new RSA backend.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::rsa::RsaBackend;
    /// #
    /// let mut rsa = RsaBackend::new(peripherals.RSA);
    /// # {after_snippet}
    /// ```
    pub fn new(rsa: RSA<'d>) -> Self {
        Self {
            peri: rsa,
            state: RsaBackendState::Idle,
        }
    }

    #[procmacros::doc_replace]
    /// Registers the RSA driver to process RSA operations.
    ///
    /// The driver stops operating when the returned object is dropped.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::rsa::RsaBackend;
    /// #
    /// let mut rsa = RsaBackend::new(peripherals.RSA);
    /// // Start the backend, which allows processing RSA operations.
    /// let _backend = rsa.start();
    /// # {after_snippet}
    /// ```
    pub fn start(&mut self) -> RsaWorkQueueDriver<'_, 'd> {
        RsaWorkQueueDriver {
            inner: WorkQueueDriver::new(self, RSA_VTABLE, &RSA_WORK_QUEUE),
        }
    }

    // WorkQueue callbacks. They may run in any context.

    unsafe fn from_raw<'any>(ptr: NonNull<()>) -> &'any mut Self {
        unsafe { ptr.cast::<RsaBackend<'_>>().as_mut() }
    }

    fn process_item(&mut self, item: &mut RsaWorkItem) -> work_queue::Poll {
        match core::mem::take(&mut self.state) {
            RsaBackendState::Idle => {
                let driver = Rsa {
                    rsa: unsafe { self.peri.clone_unchecked() },
                    phantom: PhantomData,
                    #[cfg(not(esp32))]
                    _memory_guard: RsaMemoryPowerGuard::new(),
                    _guard: GenericPeripheralGuard::new(),
                };
                self.state = RsaBackendState::Initializing(driver);
                work_queue::Poll::Pending(true)
            }
            RsaBackendState::Initializing(mut rsa) => {
                // Wait for the peripheral to finish initializing. Ideally we need a way to
                // instruct the work queue to wake the polling task immediately.
                self.state = if rsa.ready() {
                    rsa.set_interrupt_handler(rsa_work_queue_handler);
                    rsa.enable_disable_interrupt(true);
                    RsaBackendState::Ready(rsa)
                } else {
                    RsaBackendState::Initializing(rsa)
                };
                work_queue::Poll::Pending(true)
            }
            RsaBackendState::Ready(mut rsa) => {
                #[cfg(not(esp32))]
                {
                    rsa.disable_constant_time(!item.constant_time);
                    rsa.search_acceleration(item.search_acceleration);
                }

                match item.operation {
                    RsaOperation::Multiplication { x, y } => {
                        let n = x.len() as u32;
                        rsa.write_operand_a(unsafe { x.as_ref() });

                        // Non-modular multiplication result is twice as wide as its operands.
                        rsa.write_multi_mode(2 * n / WORDS_PER_INCREMENT - 1, false);
                        rsa.write_multi_operand_b(unsafe { y.as_ref() });
                        rsa.start_multi();
                    }

                    RsaOperation::ModularMultiplication {
                        x,
                        #[cfg(not(esp32))]
                        y,
                        m,
                        m_prime,
                        r: r_inv,
                        ..
                    } => {
                        let n = x.len() as u32;
                        rsa.write_operand_a(unsafe { x.as_ref() });

                        rsa.write_multi_mode(n / WORDS_PER_INCREMENT - 1, true);

                        #[cfg(not(esp32))]
                        rsa.write_operand_b(unsafe { y.as_ref() });

                        rsa.write_modulus(unsafe { m.as_ref() });
                        rsa.write_mprime(m_prime);
                        rsa.write_r(unsafe { r_inv.as_ref() });

                        rsa.start_modmulti();

                        #[cfg(esp32)]
                        {
                            // ESP32 requires a two-step process where Y needs to be written to the
                            // X memory.
                            self.state = RsaBackendState::ModularMultiplicationRoundOne(rsa);

                            return work_queue::Poll::Pending(false);
                        }
                    }
                    RsaOperation::ModularExponentiation {
                        x,
                        y,
                        m,
                        m_prime,
                        r_inv,
                    } => {
                        let n = x.len() as u32;
                        rsa.write_operand_a(unsafe { x.as_ref() });

                        rsa.write_modexp_mode(n / WORDS_PER_INCREMENT - 1);
                        rsa.write_operand_b(unsafe { y.as_ref() });
                        rsa.write_modulus(unsafe { m.as_ref() });
                        rsa.write_mprime(m_prime);
                        rsa.write_r(unsafe { r_inv.as_ref() });

                        #[cfg(not(esp32))]
                        if item.search_acceleration {
                            fn find_search_pos(exponent: &[u32]) -> u32 {
                                for (i, byte) in exponent.iter().rev().enumerate() {
                                    if *byte == 0 {
                                        continue;
                                    }
                                    return (exponent.len() * 32) as u32
                                        - (byte.leading_zeros() + i as u32 * 32)
                                        - 1;
                                }
                                0
                            }
                            rsa.write_search_position(find_search_pos(unsafe { y.as_ref() }));
                        }

                        rsa.start_modexp();
                    }
                }

                self.state = RsaBackendState::Processing(rsa);

                work_queue::Poll::Pending(false)
            }

            #[cfg(esp32)]
            RsaBackendState::ModularMultiplicationRoundOne(mut rsa) => {
                if rsa.is_idle() {
                    let RsaOperation::ModularMultiplication { y, .. } = item.operation else {
                        unreachable!();
                    };

                    // Y needs to be written to the X memory.
                    rsa.write_operand_a(unsafe { y.as_ref() });
                    rsa.start_modmulti();

                    self.state = RsaBackendState::Processing(rsa);
                } else {
                    // Wait for the operation to complete
                    self.state = RsaBackendState::ModularMultiplicationRoundOne(rsa);
                }
                work_queue::Poll::Pending(false)
            }

            RsaBackendState::Processing(rsa) => {
                if rsa.is_idle() {
                    rsa.read_out(unsafe { item.result.as_mut() });

                    self.state = RsaBackendState::Ready(rsa);
                    work_queue::Poll::Ready(Status::Completed)
                } else {
                    self.state = RsaBackendState::Processing(rsa);
                    work_queue::Poll::Pending(false)
                }
            }
        }
    }

    fn cancel(&mut self, _item: &mut RsaWorkItem) {
        // Drop the driver to reset it. We don't read the result, so the work item remains
        // unchanged, effectively cancelling it.
        self.state = RsaBackendState::Idle;
    }

    fn deinitialize(&mut self) {
        self.state = RsaBackendState::Idle;
    }
}

/// An active work queue driver.
///
/// This object must be kept around, otherwise RSA operations will never complete.
///
/// For a usage example, see [`RsaBackend`].
pub struct RsaWorkQueueDriver<'t, 'd> {
    inner: WorkQueueDriver<'t, RsaBackend<'d>, RsaWorkItem>,
}

impl<'t, 'd> RsaWorkQueueDriver<'t, 'd> {
    /// Finishes processing the current work queue item, then stops the driver.
    pub fn stop(self) -> impl Future<Output = ()> {
        self.inner.stop()
    }
}

#[derive(Clone)]
struct RsaWorkItem {
    // Acceleration options
    #[cfg(not(esp32))]
    search_acceleration: bool,
    #[cfg(not(esp32))]
    constant_time: bool,

    // The operation to execute.
    operation: RsaOperation,
    result: NonNull<[u32]>,
}

unsafe impl Sync for RsaWorkItem {}
unsafe impl Send for RsaWorkItem {}

#[derive(Clone)]
enum RsaOperation {
    // Z = X * Y
    // len(Z) = len(X) + len(Y)
    Multiplication {
        x: NonNull<[u32]>,
        y: NonNull<[u32]>,
    },
    // Z = X * Y mod M
    ModularMultiplication {
        x: NonNull<[u32]>,
        y: NonNull<[u32]>,
        m: NonNull<[u32]>,
        r: NonNull<[u32]>,
        m_prime: u32,
    },
    // Z = X ^ Y mod M
    ModularExponentiation {
        x: NonNull<[u32]>,
        y: NonNull<[u32]>,
        m: NonNull<[u32]>,
        r_inv: NonNull<[u32]>,
        m_prime: u32,
    },
}

#[handler]
#[ram]
fn rsa_work_queue_handler() {
    if !RSA_WORK_QUEUE.process() {
        // The queue may indicate that it needs to be polled again. In this case, we do not clear
        // the interrupt bit, which causes the interrupt to be re-handled.
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                RSA::regs().interrupt().write(|w| w.interrupt().set_bit());
            } else {
                RSA::regs().int_clr().write(|w| w.int_clr().set_bit());
            }
        }
    }
}

/// An RSA work queue user.
///
/// This object allows performing [big number multiplication][Self::multiply], [big number modular
/// multiplication][Self::modular_multiply] and [big number modular
/// exponentiation][Self::modular_exponentiate] with hardware acceleration. To perform these
/// operations, the [`RsaBackend`] must be started, otherwise these operations will never complete.
#[cfg_attr(
    not(esp32),
    doc = " \nThe context is created with a secure configuration by default. You can enable hardware acceleration
    options using [enable_search_acceleration][Self::enable_search_acceleration] and
    [enable_acceleration][Self::enable_acceleration] when appropriate."
)]
#[derive(Clone)]
pub struct RsaContext {
    frontend: WorkQueueFrontend<RsaWorkItem>,
}

impl Default for RsaContext {
    fn default() -> Self {
        Self::new()
    }
}

impl RsaContext {
    /// Creates a new context.
    pub fn new() -> Self {
        Self {
            frontend: WorkQueueFrontend::new(RsaWorkItem {
                #[cfg(not(esp32))]
                search_acceleration: false,
                #[cfg(not(esp32))]
                constant_time: true,
                operation: RsaOperation::Multiplication {
                    x: NonNull::from(&[]),
                    y: NonNull::from(&[]),
                },
                result: NonNull::from(&mut []),
            }),
        }
    }

    #[cfg(not(esp32))]
    /// Enables search acceleration.
    ///
    /// When enabled it would increase the performance of modular
    /// exponentiation by discarding the exponent's bits before the most
    /// significant set bit.
    ///
    /// > ⚠️ Note: this compromises security by effectively decreasing the key length.
    ///
    /// For more information refer to the
    #[doc = trm_markdown_link!("rsa")]
    pub fn enable_search_acceleration(&mut self) {
        self.frontend.data_mut().search_acceleration = true;
    }

    #[cfg(not(esp32))]
    /// Enables acceleration by disabling constant time operation.
    ///
    /// Disabling constant time operation increases the performance of modular
    /// exponentiation by simplifying the calculation concerning the 0 bits
    /// of the exponent. I.e. the less the Hamming weight, the greater the
    /// performance.
    ///
    /// > ⚠️ Note: this compromises security by enabling timing-based side-channel attacks.
    ///
    /// For more information refer to the
    #[doc = trm_markdown_link!("rsa")]
    pub fn enable_acceleration(&mut self) {
        self.frontend.data_mut().constant_time = false;
    }

    fn post(&mut self) -> RsaHandle<'_> {
        RsaHandle(self.frontend.post(&RSA_WORK_QUEUE))
    }

    #[procmacros::doc_replace]
    /// Starts a modular exponentiation operation, performing `Z = X ^ Y mod M`.
    ///
    /// Software needs to pre-calculate the following values:
    ///
    /// - `r`: `2 ^ ( bitlength * 2 ) mod M`.
    /// - `m_prime` can be calculated using `-(modular multiplicative inverse of M) mod 2^32`.
    ///
    /// It is relatively easy to calculate these values using the `crypto-bigint` crate:
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// use crypto_bigint::{U512, Uint};
    /// const fn compute_r(modulus: &U512) -> U512 {
    ///     let mut d = [0_u32; U512::LIMBS * 2 + 1];
    ///     d[d.len() - 1] = 1;
    ///     let d = Uint::from_words(d);
    ///     d.const_rem(&modulus.resize()).0.resize()
    /// }
    ///
    /// const fn compute_mprime(modulus: &U512) -> u32 {
    ///     let m_inv = modulus.inv_mod2k(32).to_words()[0];
    ///     (-1 * m_inv as i64 & (u32::MAX as i64)) as u32
    /// }
    ///
    /// // Inputs
    /// const X: U512 = Uint::from_be_hex(
    ///     "c7f61058f96db3bd87dbab08ab03b4f7f2f864eac249144adea6a65f97803b719d8ca980b7b3c0389c1c7c6\
    ///    7dc353c5e0ec11f5fc8ce7f6073796cc8f73fa878",
    /// );
    /// const Y: U512 = Uint::from_be_hex(
    ///     "1763db3344e97be15d04de4868badb12a38046bb793f7630d87cf100aa1c759afac15a01f3c4c83ec2d2f66\
    ///    6bd22f71c3c1f075ec0e2cb0cb29994d091b73f51",
    /// );
    /// const M: U512 = Uint::from_be_hex(
    ///     "6b6bb3d2b6cbeb45a769eaa0384e611e1b89b0c9b45a045aca1c5fd6e8785b38df7118cf5dd45b9b63d293b\
    ///    67aeafa9ba25feb8712f188cb139b7d9b9af1c361",
    /// );
    ///
    /// // Values derived using the functions we defined above:
    /// let r = compute_r(&M);
    /// let mprime = compute_mprime(&M);
    ///
    /// use esp_hal::rsa::{RsaContext, operand_sizes::Op512};
    ///
    /// // Now perform the actual computation:
    /// let mut rsa = RsaContext::new();
    /// let mut outbuf = [0; 16];
    /// let mut handle = rsa.modular_multiply::<Op512>(
    ///     X.as_words(),
    ///     Y.as_words(),
    ///     M.as_words(),
    ///     r.as_words(),
    ///     mprime,
    ///     &mut outbuf,
    /// );
    /// handle.wait_blocking();
    /// # {after_snippet}
    /// ```
    ///
    /// The calculation is done asynchronously. This function returns an [`RsaHandle`] that can be
    /// used to poll the status of the calculation, to wait for it to finish, or to cancel the
    /// operation (by dropping the handle).
    ///
    /// When the operation is completed, the result will be stored in `result`.
    pub fn modular_exponentiate<'t, OP>(
        &'t mut self,
        x: &'t OP::InputType,
        y: &'t OP::InputType,
        m: &'t OP::InputType,
        r: &'t OP::InputType,
        m_prime: u32,
        result: &'t mut OP::InputType,
    ) -> RsaHandle<'t>
    where
        OP: RsaMode,
    {
        self.frontend.data_mut().operation = RsaOperation::ModularExponentiation {
            x: NonNull::from(x.as_ref()),
            y: NonNull::from(y.as_ref()),
            m: NonNull::from(m.as_ref()),
            r_inv: NonNull::from(r.as_ref()),
            m_prime,
        };
        self.frontend.data_mut().result = NonNull::from(result.as_mut());
        self.post()
    }

    /// Starts a modular multiplication operation, performing `Z = X * Y mod M`.
    ///
    /// Software needs to pre-calculate the following values:
    ///
    /// - `r`: `2 ^ ( bitlength * 2 ) mod M`.
    /// - `m_prime` can be calculated using `-(modular multiplicative inverse of M) mod 2^32`.
    ///
    /// For an example how these values can be calculated and used, see
    /// [Self::modular_exponentiate].
    ///
    /// The calculation is done asynchronously. This function returns an [`RsaHandle`] that can be
    /// used to poll the status of the calculation, to wait for it to finish, or to cancel the
    /// operation (by dropping the handle).
    ///
    /// When the operation is completed, the result will be stored in `result`.
    pub fn modular_multiply<'t, OP>(
        &'t mut self,
        x: &'t OP::InputType,
        y: &'t OP::InputType,
        m: &'t OP::InputType,
        r: &'t OP::InputType,
        m_prime: u32,
        result: &'t mut OP::InputType,
    ) -> RsaHandle<'t>
    where
        OP: RsaMode,
    {
        self.frontend.data_mut().operation = RsaOperation::ModularMultiplication {
            x: NonNull::from(x.as_ref()),
            y: NonNull::from(y.as_ref()),
            m: NonNull::from(m.as_ref()),
            r: NonNull::from(r.as_ref()),
            m_prime,
        };
        self.frontend.data_mut().result = NonNull::from(result.as_mut());
        self.post()
    }

    #[procmacros::doc_replace]
    /// Starts a multiplication operation, performing `Z = X * Y`.
    ///
    /// The calculation is done asynchronously. This function returns an [`RsaHandle`] that can be
    /// used to poll the status of the calculation, to wait for it to finish, or to cancel the
    /// operation (by dropping the handle).
    ///
    /// When the operation is completed, the result will be stored in `result`. The `result` is
    /// twice as wide as the inputs.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    ///
    /// // Inputs
    /// # let x: [u32; 16] = [0; 16];
    /// # let y: [u32; 16] = [0; 16];
    /// // let x: [u32; 16] = [...];
    /// // let y: [u32; 16] = [...];
    /// let mut outbuf = [0; 32];
    ///
    /// use esp_hal::rsa::{RsaContext, operand_sizes::Op512};
    ///
    /// // Now perform the actual computation:
    /// let mut rsa = RsaContext::new();
    /// let mut handle = rsa.multiply::<Op512>(&x, &y, &mut outbuf);
    /// handle.wait_blocking();
    /// # {after_snippet}
    /// ```
    pub fn multiply<'t, OP>(
        &'t mut self,
        x: &'t OP::InputType,
        y: &'t OP::InputType,
        result: &'t mut OP::OutputType,
    ) -> RsaHandle<'t>
    where
        OP: Multi,
    {
        self.frontend.data_mut().operation = RsaOperation::Multiplication {
            x: NonNull::from(x.as_ref()),
            y: NonNull::from(y.as_ref()),
        };
        self.frontend.data_mut().result = NonNull::from(result.as_mut());
        self.post()
    }
}

/// The handle to the pending RSA operation.
pub struct RsaHandle<'t>(work_queue::Handle<'t, RsaWorkItem>);

impl RsaHandle<'_> {
    /// Polls the status of the work item.
    #[inline]
    pub fn poll(&mut self) -> bool {
        self.0.poll()
    }

    /// Blocks until the work item to be processed.
    #[inline]
    pub fn wait_blocking(mut self) {
        while !self.poll() {}
    }

    /// Waits for the work item to be processed.
    #[inline]
    pub fn wait(&mut self) -> impl Future<Output = Status> {
        self.0.wait()
    }
}
