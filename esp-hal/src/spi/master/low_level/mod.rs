#[cfg(spi_master_version = "1")]
use core::cell::Cell;
use core::{
    cell::UnsafeCell,
    future::Future,
    mem::MaybeUninit,
    pin::Pin,
    sync::atomic::{AtomicUsize, Ordering},
    task::{Context, Poll},
};

use enumset::{EnumSet, enum_set};

use super::{
    Address,
    AnySpi,
    Command,
    Config,
    ConfigError,
    DataMode,
    EMPTY_WRITE_PAD,
    FIFO_SIZE,
    SpiInterrupt,
    SpiPinGuard,
    any,
};
use crate::{
    asynch::AtomicWaker,
    gpio::{InputSignal, OutputSignal},
    handler,
    interrupt::InterruptHandler,
    pac::spi2::RegisterBlock,
    private::{self, DropGuard},
    ram,
    spi::{BitOrder, Error, Mode},
    system::PeripheralGuard,
};

#[cfg_attr(spi_master_version = "1", path = "v1.rs")]
#[cfg_attr(spi_master_version = "2", path = "v2.rs")]
#[cfg_attr(spi_master_version = "3", path = "v3.rs")]
mod version;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) struct SpiWrapper<'d> {
    pub(super) spi: AnySpi<'d>,
    _guard: PeripheralGuard,
}

impl<'d> SpiWrapper<'d> {
    pub(super) fn new(spi: impl Instance + 'd) -> Self {
        let p = spi.info().peripheral;
        let this = Self {
            spi: spi.degrade(),
            _guard: PeripheralGuard::new(p),
        };

        // Initialize state
        unsafe {
            this.state()
                .pins
                .get()
                .write(MaybeUninit::new(SpiPinGuard::new_unconnected()))
        }

        this
    }

    pub(super) fn info(&self) -> &'static Info {
        self.spi.info()
    }

    pub(super) fn state(&self) -> &'static State {
        self.spi.state()
    }

    pub(super) fn disable_peri_interrupt_on_all_cores(&self) {
        self.spi.disable_peri_interrupt_on_all_cores();
    }

    pub(super) fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.spi.set_interrupt_handler(handler);
    }

    pub(super) fn pins(&mut self) -> &mut SpiPinGuard {
        unsafe {
            // SAFETY: we "own" the state, we are allowed to borrow it mutably
            self.state().pins()
        }
    }
}

impl Drop for SpiWrapper<'_> {
    fn drop(&mut self) {
        unsafe {
            // SAFETY: we "own" the state, we are allowed to deinit it
            self.spi.state().deinit();
        }
    }
}

/// SPI peripheral instance.
pub trait Instance: private::Sealed + any::Degrade {
    #[doc(hidden)]
    /// Returns the peripheral data and state describing this instance.
    fn parts(&self) -> (&'static Info, &'static State);

    /// Returns the peripheral data describing this instance.
    #[doc(hidden)]
    #[inline(always)]
    fn info(&self) -> &'static Info {
        self.parts().0
    }

    /// Returns the peripheral state for this instance.
    #[doc(hidden)]
    #[inline(always)]
    fn state(&self) -> &'static State {
        self.parts().1
    }
}

/// Marker trait for QSPI-capable SPI peripherals.
#[doc(hidden)]
pub trait QspiInstance: Instance {}

/// Peripheral data describing a particular SPI instance.
#[doc(hidden)]
#[non_exhaustive]
#[allow(private_interfaces, reason = "Unstable details")]
pub struct Info {
    /// Pointer to the register block for this SPI instance.
    ///
    /// Use [Self::register_block] to access the register block.
    pub register_block: *const RegisterBlock,

    /// The system peripheral marker.
    pub peripheral: crate::system::Peripheral,

    /// Interrupt handler for the asynchronous operations.
    pub async_handler: InterruptHandler,

    /// SCLK signal.
    pub sclk: OutputSignal,

    /// Chip select signals.
    pub cs: &'static [OutputSignal],

    pub sio_inputs: &'static [InputSignal],
    pub sio_outputs: &'static [OutputSignal],
}

impl Info {
    pub(super) fn cs(&self, n: usize) -> OutputSignal {
        *unwrap!(self.cs.get(n), "CS{} is not defined", n)
    }

    pub(super) fn opt_sio_input(&self, n: usize) -> Option<InputSignal> {
        self.sio_inputs.get(n).copied()
    }

    pub(super) fn opt_sio_output(&self, n: usize) -> Option<OutputSignal> {
        self.sio_outputs.get(n).copied()
    }

    pub(super) fn sio_input(&self, n: usize) -> InputSignal {
        unwrap!(self.opt_sio_input(n), "SIO{} is not defined", n)
    }

    pub(super) fn sio_output(&self, n: usize) -> OutputSignal {
        unwrap!(self.opt_sio_output(n), "SIO{} is not defined", n)
    }
}

pub(super) struct Driver {
    pub(super) info: &'static Info,
    pub(super) state: &'static State,
}

// Private implementation bits.
impl Driver {
    /// Returns the register block for this SPI instance.
    pub(super) fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.info.register_block }
    }

    pub(super) fn abort_transfer(&self) {
        version::abort_transfer(self);
        self.update();
    }

    /// Initialize for full-duplex 1 bit mode
    pub(super) fn init(&self) {
        self.regs().user().modify(|_, w| {
            w.usr_miso_highpart().clear_bit();
            w.usr_mosi_highpart().clear_bit();
            w.doutdin().set_bit();
            w.usr_miso().set_bit();
            w.usr_mosi().set_bit();
            w.cs_hold().set_bit();
            w.usr_dummy_idle().set_bit();
            w.usr_addr().clear_bit();
            w.usr_command().clear_bit()
        });

        #[cfg(spi_master_version = "3")]
        self.regs().clk_gate().modify(|_, w| {
            w.clk_en().set_bit();
            w.mst_clk_active().set_bit();
            w.mst_clk_sel().set_bit()
        });

        #[cfg(soc_has_pcr)]
        unsafe {
            // use default clock source
            crate::peripherals::PCR::regs()
                .spi2_clkm_conf()
                .modify(|_, w| {
                    #[cfg(spi_master_has_clk_pre_div)]
                    w.spi2_clkm_div_num().bits(1);
                    w.spi2_clkm_sel().bits(1);
                    w.spi2_clkm_en().set_bit()
                });
        }

        version::init(self);

        self.regs().slave().write(|w| unsafe { w.bits(0) });
    }

    fn init_spi_data_mode(
        &self,
        cmd_mode: DataMode,
        address_mode: DataMode,
        data_mode: DataMode,
    ) -> Result<(), Error> {
        version::init_spi_data_mode(self, cmd_mode, address_mode, data_mode)
    }

    /// Enable or disable listening for the given interrupts.
    #[cfg_attr(not(feature = "unstable"), allow(dead_code))]
    pub(super) fn enable_listen(&self, interrupts: EnumSet<SpiInterrupt>, enable: bool) {
        version::enable_listen(self, interrupts, enable);
    }

    /// Gets asserted interrupts
    #[cfg_attr(not(feature = "unstable"), allow(dead_code))]
    pub(super) fn interrupts(&self) -> EnumSet<SpiInterrupt> {
        version::interrupts(self)
    }

    /// Resets asserted interrupts
    pub(super) fn clear_interrupts(&self, interrupts: EnumSet<SpiInterrupt>) {
        version::clear_interrupts(self, interrupts);
    }

    pub(super) fn apply_config(&self, config: &Config) -> Result<(), ConfigError> {
        config.validate()?;
        self.ch_bus_freq(config)?;
        self.set_bit_order(config.read_bit_order, config.write_bit_order);
        self.set_data_mode(config.mode);
        self.state
            .min_async_transfer_size
            .store(config.min_async_transfer_size, Ordering::Relaxed);

        version::apply_config(self, config);

        Ok(())
    }

    fn set_data_mode(&self, data_mode: Mode) {
        version::set_data_mode(self, data_mode);
    }

    fn ch_bus_freq(&self, bus_clock_config: &Config) -> Result<(), ConfigError> {
        fn enable_clocks(_reg_block: &RegisterBlock, _enable: bool) {
            #[cfg(spi_master_version = "3")]
            _reg_block.clk_gate().modify(|_, w| {
                w.clk_en().bit(_enable);
                w.mst_clk_active().bit(_enable);
                w.mst_clk_sel().bit(true) // TODO: support XTAL clock source
            });
        }

        // Change clock frequency
        let raw = bus_clock_config.raw_clock_reg_value()?;

        enable_clocks(self.regs(), false);
        self.regs().clock().write(|w| unsafe { w.bits(raw) });
        enable_clocks(self.regs(), true);

        Ok(())
    }

    #[cfg(not(spi_master_bit_order_is_bool))]
    fn set_bit_order(&self, read_order: BitOrder, write_order: BitOrder) {
        let read_value = match read_order {
            BitOrder::MsbFirst => 0,
            BitOrder::LsbFirst => 1,
        };
        let write_value = match write_order {
            BitOrder::MsbFirst => 0,
            BitOrder::LsbFirst => 1,
        };
        self.regs().ctrl().modify(|_, w| unsafe {
            w.rd_bit_order().bits(read_value);
            w.wr_bit_order().bits(write_value);
            w
        });
    }

    #[cfg(spi_master_bit_order_is_bool)]
    fn set_bit_order(&self, read_order: BitOrder, write_order: BitOrder) {
        let read_value = match read_order {
            BitOrder::MsbFirst => false,
            BitOrder::LsbFirst => true,
        };
        let write_value = match write_order {
            BitOrder::MsbFirst => false,
            BitOrder::LsbFirst => true,
        };
        self.regs().ctrl().modify(|_, w| {
            w.rd_bit_order().bit(read_value);
            w.wr_bit_order().bit(write_value);
            w
        });
    }

    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn fill_fifo(&self, chunk: &[u8]) {
        // TODO: replace with `array_chunks` and `from_le_bytes`
        let mut c_iter = chunk.chunks_exact(4);
        let mut w_iter = self.regs().w_iter();
        for c in c_iter.by_ref() {
            if let Some(w_reg) = w_iter.next() {
                let word = (c[0] as u32)
                    | ((c[1] as u32) << 8)
                    | ((c[2] as u32) << 16)
                    | ((c[3] as u32) << 24);
                w_reg.write(|w| w.buf().set(word));
            }
        }
        let rem = c_iter.remainder();
        if !rem.is_empty()
            && let Some(w_reg) = w_iter.next()
        {
            let word = match rem.len() {
                3 => (rem[0] as u32) | ((rem[1] as u32) << 8) | ((rem[2] as u32) << 16),
                2 => (rem[0] as u32) | ((rem[1] as u32) << 8),
                1 => rem[0] as u32,
                _ => unreachable!(),
            };
            w_reg.write(|w| w.buf().set(word));
        }
    }

    /// Write bytes to SPI.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn write_one(&self, words: &[u8]) -> Result<(), Error> {
        if words.len() > FIFO_SIZE {
            return Err(Error::FifoSizeExeeded);
        }
        self.configure_datalen(0, words.len());
        self.fill_fifo(words);
        self.start_operation();
        Ok(())
    }

    /// Write bytes to SPI.
    ///
    /// This function will return before all bytes of the last chunk to transmit
    /// have been sent to the wire. If you must ensure that the whole
    /// messages was written correctly, use [`Self::flush`].
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn write(&self, words: &[u8]) -> Result<(), Error> {
        for chunk in words.chunks(FIFO_SIZE) {
            self.flush()?;
            self.write_one(chunk)?;
        }
        Ok(())
    }

    /// Write bytes to SPI.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) async fn write_async(&self, words: &[u8]) -> Result<(), Error> {
        for chunk in words.chunks(FIFO_SIZE) {
            self.write_one(chunk)?;
            self.flush_async().await;
        }
        Ok(())
    }

    /// Read bytes from SPI.
    ///
    /// Sends out a stuffing byte for every byte to read. This function doesn't
    /// perform flushing. If you want to read the response to something you
    /// have written before, consider using [`Self::transfer`] instead.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn read(&self, words: &mut [u8]) -> Result<(), Error> {
        let empty_array = [EMPTY_WRITE_PAD; FIFO_SIZE];

        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.write_one(&empty_array[0..chunk.len()])?;
            self.flush()?;
            self.read_from_fifo(chunk)?;
        }
        Ok(())
    }

    /// Read bytes from SPI.
    ///
    /// Sends out a stuffing byte for every byte to read. If you want to read
    /// the response to something you have written before, consider using
    /// [`Self::transfer`] instead.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) async fn read_async(&self, words: &mut [u8]) -> Result<(), Error> {
        let empty_array = [EMPTY_WRITE_PAD; FIFO_SIZE];

        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.write_one(&empty_array[0..chunk.len()])?;
            self.flush_async().await;
            self.read_from_fifo(chunk)?;
        }
        Ok(())
    }

    /// Read received bytes from SPI FIFO.
    ///
    /// Copies the contents of the SPI receive FIFO into `words`. This function
    /// doesn't perform any data transfer. If you want to read the response to
    /// something you have written before, consider using [`Self::transfer`]
    /// instead.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn read_from_fifo(&self, words: &mut [u8]) -> Result<(), Error> {
        if words.len() > FIFO_SIZE {
            return Err(Error::FifoSizeExeeded);
        }

        for (chunk, w_reg) in words.chunks_mut(4).zip(self.regs().w_iter()) {
            let reg_val = w_reg.read().bits();
            let bytes = reg_val.to_le_bytes();

            let len = chunk.len();
            chunk.copy_from_slice(&bytes[..len]);
        }

        Ok(())
    }

    pub(super) fn busy(&self) -> bool {
        self.regs().cmd().read().usr().bit_is_set()
    }

    // Check if the bus is busy and if it is wait for it to be idle
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn flush_async(&self) -> impl Future<Output = ()> {
        SpiFuture { driver: self }
    }

    // Check if the bus is busy and if it is wait for it to be idle
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn flush(&self) -> Result<(), Error> {
        while self.busy() {
            // wait for bus to be clear
        }
        Ok(())
    }

    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn transfer_in_place(&self, words: &mut [u8]) -> Result<(), Error> {
        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.write_one(chunk)?;
            self.flush()?;
            self.read_from_fifo(chunk)?;
        }

        Ok(())
    }

    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn transfer(&self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        let mut write_from = 0;
        let mut read_from = 0;

        loop {
            // How many bytes we write in this chunk
            let write_inc = core::cmp::min(FIFO_SIZE, write.len() - write_from);
            // How many bytes we read in this chunk
            let read_inc = core::cmp::min(FIFO_SIZE, read.len() - read_from);

            if (write_inc == 0) && (read_inc == 0) {
                break;
            }

            self.flush()?;

            if write_inc < read_inc {
                // Read more than we write, must pad writing part with zeros
                let mut empty = [EMPTY_WRITE_PAD; FIFO_SIZE];
                empty[0..write_inc].copy_from_slice(&write[write_from..][..write_inc]);
                self.write_one(&empty[..read_inc])?;
            } else {
                self.write_one(&write[write_from..][..write_inc])?;
            }

            if read_inc > 0 {
                self.flush()?;
                self.read_from_fifo(&mut read[read_from..][..read_inc])?;
            }

            write_from += write_inc;
            read_from += read_inc;
        }
        Ok(())
    }

    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) async fn transfer_in_place_async(&self, words: &mut [u8]) -> Result<(), Error> {
        for chunk in words.chunks_mut(FIFO_SIZE) {
            // Cut the transfer short if the future is dropped. We'll block for a short
            // while to ensure the peripheral is idle.
            let cancel_on_drop = DropGuard::new((), |_| {
                self.abort_transfer();
                while self.busy() {}
            });
            let res = self.write_one(chunk);
            self.flush_async().await;
            cancel_on_drop.defuse();
            res?;

            self.read_from_fifo(chunk)?;
        }

        Ok(())
    }

    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) async fn transfer_async(&self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        let mut write_from = 0;
        let mut read_from = 0;

        loop {
            // How many bytes we write in this chunk
            let write_inc = core::cmp::min(FIFO_SIZE, write.len() - write_from);
            // How many bytes we read in this chunk
            let read_inc = core::cmp::min(FIFO_SIZE, read.len() - read_from);

            if (write_inc == 0) && (read_inc == 0) {
                break;
            }

            self.flush_async().await;

            if write_inc < read_inc {
                // Read more than we write, must pad writing part with zeros
                let mut empty = [EMPTY_WRITE_PAD; FIFO_SIZE];
                empty[0..write_inc].copy_from_slice(&write[write_from..][..write_inc]);
                self.write_one(&empty[..read_inc])?;
            } else {
                self.write_one(&write[write_from..][..write_inc])?;
            }

            // Preserve previous semantics - see https://github.com/esp-rs/esp-hal/issues/5257
            self.flush_async().await;
            if read_inc > 0 {
                self.read_from_fifo(&mut read[read_from..][..read_inc])?;
            }

            write_from += write_inc;
            read_from += read_inc;
        }
        Ok(())
    }

    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    pub(super) fn start_operation(&self) {
        self.update();
        self.clear_interrupts(SpiInterrupt::TransferDone.into());
        self.regs().cmd().modify(|_, w| w.usr().set_bit());
    }

    pub(super) fn setup_full_duplex(&self) -> Result<(), Error> {
        self.regs().user().modify(|_, w| {
            w.usr_miso().set_bit();
            w.usr_mosi().set_bit();
            w.doutdin().set_bit();
            w.usr_dummy().clear_bit();
            w.sio().clear_bit()
        });

        self.init_spi_data_mode(
            DataMode::SingleTwoDataLines,
            DataMode::SingleTwoDataLines,
            DataMode::SingleTwoDataLines,
        )?;

        version::setup_full_duplex(self);

        Ok(())
    }

    #[expect(clippy::too_many_arguments)]
    pub(super) fn setup_half_duplex(
        &self,
        is_write: bool,
        cmd: Command,
        address: Address,
        dummy_idle: bool,
        dummy: u8,
        no_mosi_miso: bool,
        data_mode: DataMode,
    ) -> Result<(), Error> {
        self.init_spi_data_mode(cmd.mode(), address.mode(), data_mode)?;

        let dummy = version::prepare_half_duplex(self, is_write, dummy);

        let reg_block = self.regs();
        reg_block.user().modify(|_, w| {
            w.usr_miso_highpart().clear_bit();
            w.usr_mosi_highpart().clear_bit();
            // This bit tells the hardware whether we use Single or SingleTwoDataLines
            w.sio().bit(data_mode == DataMode::Single);
            w.doutdin().clear_bit();
            w.usr_miso().bit(!is_write && !no_mosi_miso);
            w.usr_mosi().bit(is_write && !no_mosi_miso);
            w.cs_hold().set_bit();
            w.usr_dummy_idle().bit(dummy_idle);
            w.usr_dummy().bit(dummy != 0);
            w.usr_addr().bit(!address.is_none());
            w.usr_command().bit(!cmd.is_none())
        });

        // FIXME why is clock config even here?
        #[cfg(spi_master_version = "3")]
        reg_block.clk_gate().modify(|_, w| {
            w.clk_en().set_bit();
            w.mst_clk_active().set_bit();
            w.mst_clk_sel().set_bit()
        });

        #[cfg(soc_has_pcr)]
        // use default clock source
        crate::peripherals::PCR::regs()
            .spi2_clkm_conf()
            .modify(|_, w| unsafe { w.spi2_clkm_sel().bits(1) });

        version::setup_half_duplex(self);

        reg_block.slave().write(|w| unsafe { w.bits(0) });

        self.update();

        // set cmd, address, dummy cycles
        self.set_up_common_phases(cmd, address, dummy);

        Ok(())
    }

    pub(super) fn set_up_common_phases(&self, cmd: Command, address: Address, dummy: u8) {
        let reg_block = self.regs();
        if !cmd.is_none() {
            reg_block.user2().modify(|_, w| unsafe {
                w.usr_command_bitlen().bits((cmd.width() - 1) as u8);
                w.usr_command_value().bits(cmd.value())
            });
        }

        if !address.is_none() {
            reg_block
                .user1()
                .modify(|_, w| unsafe { w.usr_addr_bitlen().bits((address.width() - 1) as u8) });

            version::write_address(self, address.value() << (32 - address.width()));
        }

        if dummy > 0 {
            reg_block
                .user1()
                .modify(|_, w| unsafe { w.usr_dummy_cyclelen().bits(dummy - 1) });
        }
    }

    pub(super) fn update(&self) {
        cfg_if::cfg_if! {
            if #[cfg(spi_master_version = "3")] {
                let reg_block = self.regs();

                reg_block.cmd().modify(|_, w| w.update().set_bit());

                while reg_block.cmd().read().update().bit_is_set() {
                    // wait
                }
            } else {
                // Doesn't seem to be needed for ESP32 and ESP32-S2
            }
        }
    }

    pub(super) fn configure_datalen(&self, rx_len_bytes: usize, tx_len_bytes: usize) {
        let rx_len = rx_len_bytes as u32 * 8;
        let tx_len = tx_len_bytes as u32 * 8;

        version::configure_datalen(self, rx_len.saturating_sub(1), tx_len.saturating_sub(1));
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_block, other.register_block)
    }
}

unsafe impl Sync for Info {}

for_each_spi_master! {
    ($peri:ident, $sys:ident, $sclk:ident [$($cs:ident),+] [$($sio:ident),*] $(, $is_qspi:tt)?) => {
        impl Instance for crate::peripherals::$peri<'_> {
            #[inline(always)]
            fn parts(&self) -> (&'static Info, &'static State) {
                #[handler]
                #[ram]
                fn irq_handler() {
                    handle_async(&INFO, &STATE)
                }

                static INFO: Info = Info {
                    register_block: crate::peripherals::$peri::regs(),
                    peripheral: crate::system::Peripheral::$sys,
                    async_handler: irq_handler,
                    sclk: OutputSignal::$sclk,
                    cs: &[$(OutputSignal::$cs),+],
                    sio_inputs: &[$(InputSignal::$sio),*],
                    sio_outputs: &[$(OutputSignal::$sio),*],
                };

                static STATE: State = State {
                    waker: AtomicWaker::new(),
                    pins: UnsafeCell::new(MaybeUninit::uninit()),
                    min_async_transfer_size: AtomicUsize::new(0),

                    #[cfg(spi_master_version = "1")]
                    esp32_hack: Esp32Hack {
                        timing_miso_delay: Cell::new(None),
                        extra_dummy: Cell::new(0),
                    },
                };

                (&INFO, &STATE)
            }
        }

        $(
            // If the extra pins are set, implement QspiInstance
            $crate::ignore!($is_qspi);
            impl QspiInstance for crate::peripherals::$peri<'_> {}
        )?
    };
}

#[doc(hidden)]
pub struct State {
    pub(super) waker: AtomicWaker,
    pins: UnsafeCell<MaybeUninit<SpiPinGuard>>,
    pub(super) min_async_transfer_size: AtomicUsize,

    #[cfg(spi_master_version = "1")]
    esp32_hack: Esp32Hack,
}

impl State {
    // Syntactic helper to get a mutable reference to the pin guard.
    //
    // Intended to be called in `SpiWrapper::pins` only
    //
    // # Safety
    //
    // The caller must ensure that Rust's aliasing rules are upheld.
    #[allow(
        clippy::mut_from_ref,
        reason = "Safety requirements ensure this is okay"
    )]
    pub(super) unsafe fn pins(&self) -> &mut SpiPinGuard {
        unsafe { (&mut *self.pins.get()).assume_init_mut() }
    }

    unsafe fn deinit(&self) {
        unsafe {
            let mut old = self.pins.get().replace(MaybeUninit::uninit());
            old.assume_init_drop();
        }
    }
}

#[cfg(spi_master_version = "1")]
pub(super) struct Esp32Hack {
    timing_miso_delay: Cell<Option<u8>>,
    extra_dummy: Cell<u8>,
}

unsafe impl Sync for State {}

#[ram]
pub(super) fn handle_async(info: &'static Info, state: &'static State) {
    let driver = Driver { info, state };
    if driver.interrupts().contains(SpiInterrupt::TransferDone) {
        driver.enable_listen(SpiInterrupt::TransferDone.into(), false);
        state.waker.wake();
    }
}

struct SpiFuture<'a> {
    driver: &'a Driver,
}

impl SpiFuture<'_> {
    const EVENTS: EnumSet<SpiInterrupt> = enum_set!(SpiInterrupt::TransferDone);
}

impl Future for SpiFuture<'_> {
    type Output = ();

    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.driver.busy() {
            self.driver.state.waker.register(cx.waker());
            self.driver.enable_listen(Self::EVENTS, true);

            Poll::Pending
        } else {
            self.driver.clear_interrupts(Self::EVENTS);
            Poll::Ready(())
        }
    }
}

impl Drop for SpiFuture<'_> {
    fn drop(&mut self) {
        self.driver.enable_listen(Self::EVENTS, false);
    }
}
