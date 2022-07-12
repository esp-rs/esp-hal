//! # Serial Peripheral Interface
//! To construct the SPI instances, use the `Spi::new` functions.
//!
//! ## Initialisation example
//! ```rust
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let sclk = io.pins.gpio12;
//! let miso = io.pins.gpio11;
//! let mosi = io.pins.gpio13;
//! let cs = io.pins.gpio10;
//!
//! let mut spi = hal::spi::Spi::new(
//!     peripherals.SPI2,
//!     sclk,
//!     mosi,
//!     Some(miso),
//!     Some(cs),
//!     100u32.kHz(),
//!     SpiMode::Mode0,
//!     &mut peripheral_clock_control,
//!     &mut clocks,
//! );
//! ```

use core::convert::Infallible;

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    pac::spi2::RegisterBlock,
    system::PeripheralClockControl,
    types::{InputSignal, OutputSignal},
    InputPin,
    OutputPin,
};

#[derive(Debug, Clone, Copy)]
pub enum SpiMode {
    Mode0,
    Mode1,
    Mode2,
    Mode3,
}

pub struct Spi<T> {
    spi: T,
}

impl<T> Spi<T>
where
    T: Instance,
{
    /// Constructs an SPI instance in 8bit dataframe mode.
    pub fn new<SCK: OutputPin, MOSI: OutputPin, MISO: InputPin, CS: OutputPin>(
        spi: T,
        mut sck: SCK,
        mut mosi: MOSI,
        miso: Option<MISO>,
        cs: Option<CS>,
        frequency: HertzU32,
        mode: SpiMode,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        sck.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.sclk_signal());

        mosi.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.mosi_signal());

        if let Some(mut miso) = miso {
            miso.set_to_input()
                .connect_input_to_peripheral(spi.miso_signal());
        }

        if let Some(mut cs) = cs {
            cs.set_to_push_pull_output()
                .connect_peripheral_to_output(spi.cs_signal());
        }

        spi.enable_peripheral(peripheral_clock_control);

        let mut spi = Self { spi };
        spi.spi.setup(frequency, clocks);
        spi.spi.init();
        spi.spi.set_data_mode(mode);

        spi
    }

    /// Return the raw interface to the underlying peripheral instance
    pub fn free(self) -> T {
        self.spi
    }
}

impl<T> embedded_hal::spi::FullDuplex<u8> for Spi<T>
where
    T: Instance,
{
    type Error = Infallible;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.spi.read_byte()
    }

    fn send(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.spi.write_byte(word)
    }
}

impl<T> embedded_hal::blocking::spi::Transfer<u8> for Spi<T>
where
    T: Instance,
{
    type Error = Infallible;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi.transfer(words)
    }
}

impl<T> embedded_hal::blocking::spi::Write<u8> for Spi<T>
where
    T: Instance,
{
    type Error = Infallible;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write_bytes(words)
    }
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::spi::ErrorType for Spi<T> {
    type Error = Infallible;
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::spi::nb::FullDuplex for Spi<T>
where
    T: Instance,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.spi.read_byte()
    }

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.spi.write_byte(word)
    }
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::spi::blocking::SpiBusWrite for Spi<T>
where
    T: Instance,
{
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.send_bytes(words)
    }
}

#[cfg(feature = "eh1")]
impl<T> embedded_hal_1::spi::blocking::SpiBusFlush for Spi<T>
where
    T: Instance,
{
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.spi.flush()
    }
}

pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn sclk_signal(&self) -> OutputSignal;

    fn mosi_signal(&self) -> OutputSignal;

    fn miso_signal(&self) -> InputSignal;

    fn cs_signal(&self) -> OutputSignal;

    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl);

    fn init(&mut self) {
        let reg_block = self.register_block();
        reg_block.user.modify(|_, w| {
            w.usr_miso_highpart()
                .clear_bit()
                .usr_miso_highpart()
                .clear_bit()
                .doutdin()
                .set_bit()
                .usr_miso()
                .set_bit()
                .usr_mosi()
                .set_bit()
                .cs_hold()
                .set_bit()
                .usr_dummy_idle()
                .set_bit()
                .usr_addr()
                .clear_bit()
                .usr_command()
                .clear_bit()
        });

        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        reg_block.clk_gate.modify(|_, w| {
            w.clk_en()
                .set_bit()
                .mst_clk_active()
                .set_bit()
                .mst_clk_sel()
                .set_bit()
        });

        reg_block.ctrl.write(|w| unsafe { w.bits(0) });

        #[cfg(not(feature = "esp32"))]
        reg_block.misc.write(|w| unsafe { w.bits(0) });

        reg_block.slave.write(|w| unsafe { w.bits(0) });
    }

    // taken from https://github.com/apache/incubator-nuttx/blob/8267a7618629838231256edfa666e44b5313348e/arch/risc-v/src/esp32c3/esp32c3_spi.c#L496
    fn setup(&mut self, frequency: HertzU32, clocks: &Clocks) {
        // FIXME: this might not be always true
        let apb_clk_freq: HertzU32 = HertzU32::Hz(clocks.apb_clock.to_Hz());

        let reg_val: u32;
        let duty_cycle = 128;

        // In HW, n, h and l fields range from 1 to 64, pre ranges from 1 to 8K.
        // The value written to register is one lower than the used value.

        if frequency > ((apb_clk_freq / 4) * 3) {
            // Using APB frequency directly will give us the best result here.
            reg_val = 1 << 31;
        } else {
            /* For best duty cycle resolution, we want n to be as close to 32 as
            * possible, but we also need a pre/n combo that gets us as close as
            * possible to the intended frequency. To do this, we bruteforce n and
            * calculate the best pre to go along with that. If there's a choice
            * between pre/n combos that give the same result, use the one with the
            * higher n.
            */

            let mut pre: i32;
            let mut bestn: i32 = -1;
            let mut bestpre: i32 = -1;
            let mut besterr: i32 = 0;
            let mut errval: i32;

            /* Start at n = 2. We need to be able to set h/l so we have at least
            * one high and one low pulse.
            */

            for  n in 2..64 {
                /* Effectively, this does:
                *   pre = round((APB_CLK_FREQ / n) / frequency)
                */

                pre = ((apb_clk_freq.raw() as i32/ n) + (frequency.raw() as i32 / 2)) / frequency.raw() as i32;

                if pre <= 0 {
                    pre = 1;
                }

                if pre > 16 {
                    pre = 16;
                }

                errval = (apb_clk_freq.raw() as i32 / (pre as i32 * n as i32) - frequency.raw() as i32).abs();
                if bestn == -1 || errval <= besterr {
                    besterr = errval;
                    bestn = n as i32;
                    bestpre = pre as i32;
                }
            }

            let n: i32 = bestn;
            pre = bestpre as i32;
            let l: i32 = n;

            /* Effectively, this does:
            *   h = round((duty_cycle * n) / 256)
            */

            let mut h: i32 = (duty_cycle * n + 127) / 256;
            if h <= 0 {
                h = 1;
            }

            reg_val = (l as u32 - 1) |
                    ((h as u32 - 1) << 6) |
                    ((n as u32 - 1) << 12) |
                    ((pre as u32 - 1) << 18);
        }

        self.register_block()
            .clock
            .write(|w| unsafe { w.bits(reg_val) });
    }

    #[cfg(not(feature = "esp32"))]
    fn set_data_mode(&mut self, data_mode: SpiMode) -> &mut Self {
        let reg_block = self.register_block();

        match data_mode {
            SpiMode::Mode0 => {
                reg_block.misc.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().clear_bit());
            }
            SpiMode::Mode1 => {
                reg_block.misc.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode2 => {
                reg_block.misc.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode3 => {
                reg_block.misc.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().clear_bit());
            }
        }
        self
    }

    #[cfg(feature = "esp32")]
    fn set_data_mode(&mut self, data_mode: SpiMode) -> &mut Self {
        let reg_block = self.register_block();

        match data_mode {
            SpiMode::Mode0 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().clear_bit());
            }
            SpiMode::Mode1 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode2 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode3 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().clear_bit());
            }
        }
        self
    }

    fn read_byte(&mut self) -> nb::Result<u8, Infallible> {
        let reg_block = self.register_block();

        if reg_block.cmd.read().usr().bit_is_set() {
            return Err(nb::Error::WouldBlock);
        }

        Ok(u32::try_into(reg_block.w0.read().bits()).unwrap_or_default())
    }

    fn write_byte(&mut self, word: u8) -> nb::Result<(), Infallible> {
        let reg_block = self.register_block();

        if reg_block.cmd.read().usr().bit_is_set() {
            return Err(nb::Error::WouldBlock);
        }

        self.configure_datalen(8);

        reg_block.w0.write(|w| unsafe { w.bits(word.into()) });

        self.update();

        reg_block.cmd.modify(|_, w| w.usr().set_bit());

        Ok(())
    }

    fn write_bytes(&mut self, words: &[u8]) -> Result<(), Infallible> {
        let mut words_mut = [0u8; 256];
        words_mut[0..words.len()].clone_from_slice(&words[0..words.len()]);

        self.transfer(&mut words_mut[0..words.len()])?;

        Ok(())
    }

    fn send_bytes(&mut self, words: &[u8]) -> Result<(), Infallible> {
        let reg_block = self.register_block();
        let num_chuncks = words.len() / 64;

        // The fifo has a total of 16 32 bit registers (64 bytes) so the data
        // must be chunked and then transmitted
        for (i, chunk) in words.chunks(64).enumerate() {
            self.configure_datalen(chunk.len() as u32 * 8);

            let mut fifo_ptr = reg_block.w0.as_ptr();
            for chunk in chunk.chunks(4) {
                let mut u32_as_bytes = [0u8; 4];
                unsafe {
                    let ptr = u32_as_bytes.as_mut_ptr();
                    ptr.copy_from(chunk.as_ptr(), chunk.len());
                }
                let reg_val: u32 = u32::from_le_bytes(u32_as_bytes);

                unsafe {
                    *fifo_ptr = reg_val;
                    fifo_ptr = fifo_ptr.offset(1);
                };
            }

            self.update();

            reg_block.cmd.modify(|_, w| w.usr().set_bit());

            // Wait for all chunks to complete except the last one.
            // The function is allowed to return before the bus is idle.
            // see [embedded-hal flushing](https://docs.rs/embedded-hal/1.0.0-alpha.8/embedded_hal/spi/blocking/index.html#flushing)
            if i < num_chuncks {
                while reg_block.cmd.read().usr().bit_is_set() {
                    // wait
                }
            }
        }
        Ok(())
    }

    // Check if the bus is busy and if it is wait for it to be idle
    fn flush(&mut self) -> Result<(), Infallible> {
        let reg_block = self.register_block();

        while reg_block.cmd.read().usr().bit_is_set() {
            // wait for bus to be clear
        }
        Ok(())
    }

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Infallible> {
        let reg_block = self.register_block();

        for chunk in words.chunks_mut(64) {
            self.configure_datalen(chunk.len() as u32 * 8);

            let mut fifo_ptr = reg_block.w0.as_ptr();
            for chunk in chunk.chunks(4) {
                let mut u32_as_bytes = [0u8; 4];
                u32_as_bytes[0..(chunk.len())].clone_from_slice(chunk);
                let reg_val: u32 = u32::from_le_bytes(u32_as_bytes);

                unsafe {
                    *fifo_ptr = reg_val;
                    fifo_ptr = fifo_ptr.offset(1);
                };
            }

            self.update();

            reg_block.cmd.modify(|_, w| w.usr().set_bit());

            while reg_block.cmd.read().usr().bit_is_set() {
                // wait
            }

            let mut fifo_ptr = reg_block.w0.as_ptr();
            for index in (0..chunk.len()).step_by(4) {
                let reg_val = unsafe { *fifo_ptr };
                let bytes = reg_val.to_le_bytes();

                let len = usize::min(chunk.len(), index + 4) - index;
                chunk[index..(index + len)].clone_from_slice(&bytes[0..len]);

                unsafe {
                    fifo_ptr = fifo_ptr.offset(1);
                };
            }
        }

        Ok(words)
    }

    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    fn update(&self) {
        let reg_block = self.register_block();

        reg_block.cmd.modify(|_, w| w.update().set_bit());

        while reg_block.cmd.read().update().bit_is_set() {
            // wait
        }
    }

    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    fn update(&self) {
        // not need/available on ESP32/ESP32S2
    }

    fn configure_datalen(&self, len: u32) {
        let reg_block = self.register_block();

        #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
        reg_block
            .ms_dlen
            .write(|w| unsafe { w.ms_data_bitlen().bits(len - 1) });

        #[cfg(not(any(feature = "esp32c3", feature = "esp32s3")))]
        {
            reg_block
                .mosi_dlen
                .write(|w| unsafe { w.usr_mosi_dbitlen().bits(len - 1) });

            reg_block
                .miso_dlen
                .write(|w| unsafe { w.usr_miso_dbitlen().bits(len - 1) });
        }
    }
}

#[cfg(any(feature = "esp32c3"))]
impl Instance for crate::pac::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::FSPICLK_MUX
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::FSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::FSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::FSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi2);
    }
}

#[cfg(any(feature = "esp32"))]
impl Instance for crate::pac::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::HSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::HSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::HSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::HSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi2);
    }
}

#[cfg(any(feature = "esp32"))]
impl Instance for crate::pac::SPI3 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::VSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::VSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::VSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::VSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi3)
    }
}

#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
impl Instance for crate::pac::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::FSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::FSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::FSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::FSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi2)
    }
}

#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
impl Instance for crate::pac::SPI3 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_CLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_D
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::SPI3_Q
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_CS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi3)
    }
}
