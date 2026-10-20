use enumset::EnumSet;

use super::{I2sClockSource, I2sInterrupt, private::I2sClockDividers};
#[cfg(not(i2s_version = "1"))]
use crate::clock::ll::I2sClkConfig as ModuleClockConfig;
#[cfg(i2s_version = "1")]
use crate::clock::ll::I2sMclkConfig as ModuleClockConfig;
use crate::{
    clock::ll::{ClockTree, I2sInstance},
    gpio::{InputSignal, OutputSignal},
    pac::i2s0::RegisterBlock,
};

#[cfg_attr(i2s_version = "1", path = "v1.rs")]
#[cfg_attr(i2s_version = "2", path = "v2.rs")]
#[cfg_attr(i2s_version = "3", path = "v3.rs")]
mod version;

/// Peripheral data describing a particular I2S instance.
///
/// All the per-instance data (register block pointer, system peripheral marker and the
/// GPIO matrix signals) is stored here, so that the driver can operate on a single
/// type-erased `&'static Info` regardless of which concrete I2S peripheral is used.
#[doc(hidden)]
#[non_exhaustive]
pub struct Info {
    /// Pointer to the register block for this I2S instance.
    pub register_block: *const RegisterBlock,

    /// The system peripheral marker.
    pub peripheral: crate::system::Peripheral,

    /// MCLK output signal.
    #[cfg(not(esp32))] // MCLK on ESP32 requires special handling
    pub mclk: OutputSignal,

    /// BCLK (TX) output signal.
    pub bclk: OutputSignal,

    /// WS (TX) output signal.
    pub ws: OutputSignal,

    /// BCLK (RX) output signal.
    pub bclk_rx: OutputSignal,

    /// WS (RX) output signal.
    pub ws_rx: OutputSignal,

    /// Data out signals.
    pub dout_lines: &'static [OutputSignal],

    /// Data in signals.
    pub din_lines: &'static [InputSignal],

    /// PDM TX supported.
    pub pdm_tx: bool,

    /// PDM RX supported.
    pub pdm_rx: bool,

    /// Hardware PCM-to-PDM format conversion filter supported on TX.
    pub pcm2pdm: bool,

    /// Hardware PDM-to-PCM format conversion filter supported on RX.
    pub pdm2pcm: bool,

    /// Clock tree instance for this I2S peripheral.
    pub clock_instance: I2sInstance,
}

// SAFETY: The register block pointer refers to a static peripheral memory region.
unsafe impl Sync for Info {}

impl Info {
    pub(crate) fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    pub(crate) fn dout(&self, line: u8) -> Option<OutputSignal> {
        // (line 1 for two-line DAC).
        self.dout_lines.get(line as usize).copied()
    }

    pub(crate) fn din(&self, line: u8) -> Option<InputSignal> {
        // (line 0 is the default DIN signal).
        self.din_lines.get(line as usize).copied()
    }

    pub(crate) fn interrupts(&self) -> EnumSet<I2sInterrupt> {
        let mut res = EnumSet::new();
        let ints = self.regs().int_st().read();

        if ints.rx_hung().bit() {
            res.insert(I2sInterrupt::RxHung);
        }
        if ints.tx_hung().bit() {
            res.insert(I2sInterrupt::TxHung);
        }
        #[cfg(not(i2s_version = "1"))]
        if ints.rx_done().bit() {
            res.insert(I2sInterrupt::RxDone);
        }
        #[cfg(not(i2s_version = "1"))]
        if ints.tx_done().bit() {
            res.insert(I2sInterrupt::TxDone);
        }

        res
    }

    pub(crate) fn enable_listen(&self, interrupts: EnumSet<I2sInterrupt>, enable: bool) {
        self.regs().int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    I2sInterrupt::RxHung => w.rx_hung().bit(enable),
                    I2sInterrupt::TxHung => w.tx_hung().bit(enable),
                    #[cfg(not(i2s_version = "1"))]
                    I2sInterrupt::RxDone => w.rx_done().bit(enable),
                    #[cfg(not(i2s_version = "1"))]
                    I2sInterrupt::TxDone => w.tx_done().bit(enable),
                };
            }
            w
        });
    }

    pub(crate) fn clear_interrupts(&self, interrupts: EnumSet<I2sInterrupt>) {
        self.regs().int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    I2sInterrupt::RxHung => w.rx_hung().clear_bit_by_one(),
                    I2sInterrupt::TxHung => w.tx_hung().clear_bit_by_one(),
                    #[cfg(not(i2s_version = "1"))]
                    I2sInterrupt::RxDone => w.rx_done().clear_bit_by_one(),
                    #[cfg(not(i2s_version = "1"))]
                    I2sInterrupt::TxDone => w.tx_done().clear_bit_by_one(),
                };
            }
            w
        });
    }
}

/// Describes a module clock for the clock tree: the source to select, and the MCLK divider.
fn clock_config(source: I2sClockSource, dividers: &I2sClockDividers) -> ModuleClockConfig {
    ModuleClockConfig::new(
        source,
        dividers.mclk_divider,
        dividers.denominator,
        dividers.numerator,
    )
}

impl Info {
    /// Configures the module clock that both directions share.
    #[cfg(i2s_version = "1")]
    pub(crate) fn configure_mclk(&self, source: I2sClockSource, dividers: &I2sClockDividers) {
        ClockTree::with(|clocks| {
            self.clock_instance
                .configure_mclk(clocks, clock_config(source, dividers));
        });
    }

    /// Configures the module clock of the transmitter.
    #[cfg(not(i2s_version = "1"))]
    pub(crate) fn configure_tx_mclk(&self, source: I2sClockSource, dividers: &I2sClockDividers) {
        ClockTree::with(|clocks| {
            self.clock_instance
                .configure_tx_clk(clocks, clock_config(source, dividers));
        });
    }

    /// Configures the module clock of the receiver.
    #[cfg(not(i2s_version = "1"))]
    pub(crate) fn configure_rx_mclk(&self, source: I2sClockSource, dividers: &I2sClockDividers) {
        ClockTree::with(|clocks| {
            self.clock_instance
                .configure_rx_clk(clocks, clock_config(source, dividers));
        });
    }

    /// Selects the module clock that drives the MCLK pad.
    #[cfg(not(i2s_version = "1"))]
    pub(crate) fn configure_mclk_pad(&self, sel: super::MclkOut) {
        ClockTree::with(|clocks| {
            self.clock_instance.configure_mclk_out(clocks, sel);
        });
    }
}
