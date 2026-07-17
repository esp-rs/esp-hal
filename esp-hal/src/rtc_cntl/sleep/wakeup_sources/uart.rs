use crate::rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers, WakeupSource};

for_each_uart! {
    ($id:literal, $inst:ident, $peri:ident, $rxd:ident, $txd:ident, $cts:ident, $rts:ident, wakeup_source = true) => {
        paste::paste! {
            #[doc = concat!("UART", $id, " wakeup source")]
            ///
            /// The chip can be woken up by reverting RXD for multiple cycles until the
            /// number of rising edges is equal to or greater than the given value.
            ///
            /// Note that the character which triggers wakeup (and any characters before
            /// it) will not be received by the UART after wakeup. This means that the
            /// external device typically needs to send an extra character to trigger
            /// wakeup before sending the data.
            ///
            /// After waking-up from UART, you should send some extra data through the UART
            /// port in Active mode, so that the internal wakeup indication signal can be
            /// cleared. Otherwise, the next UART wake-up would trigger with two less
            /// rising edges than the configured threshold value.
            ///
            /// Wakeup from light sleep takes some time, so not every character sent to the
            /// UART can be received by the application.
            ///
            /// This wakeup source can be used to wake up from light sleep only.
            #[instability::unstable]
            pub struct [< Uart $id WakeupSource >] {
                threshold: u16,
            }

            impl [< Uart $id WakeupSource >] {
                #[doc = concat!("Create a new instance of UART", $id, " wakeup source>") ]
                ///
                /// # Panics
                ///
                /// Panics if `threshold` is out of bounds.
                #[instability::unstable]
                pub fn new(threshold: u16) -> Self {
                    if threshold > 1023 {
                        panic!("Invalid threshold");
                    }
                    Self { threshold }
                }
            }

            #[instability::unstable]
            impl WakeSource for [< Uart $id WakeupSource >] {
                fn apply(&self, _rtc: &Rtc<'_>, triggers: &mut WakeTriggers, _sleep_config: &mut RtcSleepConfig) {
                    triggers.insert(WakeupSource::[< Uart $id >]);
                    let uart = crate::peripherals::$inst::regs();

                    cfg_select! {
                        any(esp32, esp32s2, esp32s3, esp32c2, esp32c3) => {
                            uart.sleep_conf()
                                .modify(|_, w| unsafe { w.active_threshold().bits(self.threshold) });
                        }
                        _ => {
                            uart.sleep_conf2().modify(|_, w| unsafe {
                                w.wk_mode_sel().bits(0);
                                w.active_threshold().bits(self.threshold)
                            });
                        }
                    }
                }
            }
        }
    };
}
