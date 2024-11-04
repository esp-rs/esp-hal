use core::{
    pin::Pin,
    task::{Context, Poll},
};

use crate::i2c::master::*;

pub enum Event {
    EndDetect,
    TxComplete,
    #[cfg(not(esp32s2))]
    TxFifoWatermark,
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct I2cFuture<'a> {
    event: Event,
    info: &'a Info,
    state: &'a State,
}

impl<'a> I2cFuture<'a> {
    pub fn new(event: Event, info: &'a Info, state: &'a State) -> Self {
        info.register_block().int_ena().modify(|_, w| {
            let w = match event {
                Event::EndDetect => w.end_detect().set_bit(),
                Event::TxComplete => w.trans_complete().set_bit(),
                #[cfg(not(esp32s2))]
                Event::TxFifoWatermark => w.txfifo_wm().set_bit(),
            };

            w.arbitration_lost().set_bit();
            w.time_out().set_bit();
            w.nack().set_bit()
        });

        Self { event, state, info }
    }

    fn event_bit_is_clear(&self) -> bool {
        let r = self.info.register_block().int_ena().read();

        match self.event {
            Event::EndDetect => r.end_detect().bit_is_clear(),
            Event::TxComplete => r.trans_complete().bit_is_clear(),
            #[cfg(not(esp32s2))]
            Event::TxFifoWatermark => r.txfifo_wm().bit_is_clear(),
        }
    }

    fn check_error(&self) -> Result<(), Error> {
        let r = self.info.register_block().int_raw().read();

        if r.arbitration_lost().bit_is_set() {
            return Err(Error::ArbitrationLost);
        }

        if r.time_out().bit_is_set() {
            return Err(Error::TimeOut);
        }

        if r.nack().bit_is_set() {
            return Err(Error::AckCheckFailed);
        }

        if r.trans_complete().bit_is_set()
            && self
                .info
                .register_block()
                .sr()
                .read()
                .resp_rec()
                .bit_is_clear()
        {
            return Err(Error::AckCheckFailed);
        }

        Ok(())
    }
}

impl core::future::Future for I2cFuture<'_> {
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        self.state.waker.register(ctx.waker());

        let error = self.check_error();

        if error.is_err() {
            return Poll::Ready(error);
        }

        if self.event_bit_is_clear() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

impl Driver<'_> {
    pub(crate) async fn wait_for_completion(&self, end_only: bool) -> Result<(), Error> {
        self.check_errors()?;

        if end_only {
            I2cFuture::new(Event::EndDetect, self.info, self.state).await?;
        } else {
            let res = embassy_futures::select::select(
                I2cFuture::new(Event::TxComplete, self.info, self.state),
                I2cFuture::new(Event::EndDetect, self.info, self.state),
            )
            .await;

            match res {
                embassy_futures::select::Either::First(res) => res?,
                embassy_futures::select::Either::Second(res) => res?,
            }
        }
        self.check_all_commands_done()?;

        Ok(())
    }
}

impl Driver<'_> {
    /// Checks for I2C transmission errors and handles them.
    ///
    /// This function inspects specific I2C-related interrupts to detect errors
    /// during communication, such as timeouts, failed acknowledgments, or
    /// arbitration loss. If an error is detected, the function handles it
    /// by resetting the I2C peripheral to clear the error condition and then
    /// returns an appropriate error.
    pub(crate) fn check_errors(&self) -> Result<(), Error> {
        let interrupts = self.info.register_block().int_raw().read();

        // Handle error cases
        let retval = if interrupts.time_out().bit_is_set() {
            Error::TimeOut
        } else if interrupts.nack().bit_is_set() {
            Error::AckCheckFailed
        } else if interrupts.arbitration_lost().bit_is_set() {
            Error::ArbitrationLost
        } else if interrupts.trans_complete().bit_is_set()
            && self
                .info
                .register_block()
                .sr()
                .read()
                .resp_rec()
                .bit_is_clear()
        {
            Error::AckCheckFailed
        } else {
            return Ok(());
        };

        self.reset();

        Err(retval)
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    pub(crate) fn reset(&self) {
        // Reset the FSM
        self.info
            .register_block()
            .ctr()
            .modify(|_, w| w.fsm_rst().set_bit());

        // Clear all I2C interrupts
        self.info
            .register_block()
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });

        // Reset fifo
        self.reset_fifo();

        // Reset the command list
        self.reset_command_list();
    }

    /// Resets the transmit and receive FIFO buffers
    pub(crate) fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.info
            .register_block()
            .fifo_conf()
            .modify(|_, w| unsafe {
                w.tx_fifo_rst().set_bit();
                w.rx_fifo_rst().set_bit();
                w.nonfifo_en().clear_bit();
                w.fifo_prt_en().set_bit();
                w.rxfifo_wm_thrhd().bits(1);
                w.txfifo_wm_thrhd().bits(8)
            });

        self.info.register_block().fifo_conf().modify(|_, w| {
            w.tx_fifo_rst().clear_bit();
            w.rx_fifo_rst().clear_bit()
        });

        self.info.register_block().int_clr().write(|w| {
            w.rxfifo_wm().clear_bit_by_one();
            w.txfifo_wm().clear_bit_by_one()
        });

        self.update_config();
    }

    pub(crate) fn write_fifo(&self, data: u8) {
        self.info
            .register_block()
            .data()
            .write(|w| unsafe { w.fifo_rdata().bits(data) });
    }
}
