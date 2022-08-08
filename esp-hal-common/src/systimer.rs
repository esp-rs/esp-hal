use core::{intrinsics::transmute, marker::PhantomData};

use fugit::MillisDurationU32;

use crate::pac::{
    generic::Reg,
    systimer::{
        target0_conf::TARGET0_CONF_SPEC,
        target0_hi::TARGET0_HI_SPEC,
        target0_lo::TARGET0_LO_SPEC,
    },
    SYSTIMER,
};

// TODO this only handles unit0 of the systimer

#[derive(Debug)]
pub struct SystemTimer {
    _inner: SYSTIMER,
    pub alarm0: Alarm<Target, 0>,
    pub alarm1: Alarm<Target, 1>,
    pub alarm2: Alarm<Target, 2>,
}

impl SystemTimer {
    pub fn new(p: SYSTIMER) -> Self {
        Self {
            _inner: p,
            alarm0: Alarm::new(),
            alarm1: Alarm::new(),
            alarm2: Alarm::new(),
        }
    }

    // TODO use fugit types
    pub fn now() -> u64 {
        // This should be safe to access from multiple contexts
        // worst case scenario the second accesor ends up reading
        // an older time stamp
        let systimer = unsafe { &*SYSTIMER::ptr() };
        systimer
            .unit0_op
            .modify(|_, w| w.timer_unit0_update().set_bit());

        while !systimer
            .unit0_op
            .read()
            .timer_unit0_value_valid()
            .bit_is_set()
        {}

        let value_lo = systimer.unit0_value_lo.read().bits();
        let value_hi = systimer.unit0_value_hi.read().bits();

        ((value_hi as u64) << 32) | value_lo as u64
    }
}

#[derive(Debug)]
pub struct Target;
#[derive(Debug)]
pub struct Periodic; // TODO, also impl e-h timer traits

#[derive(Debug)]
pub struct Alarm<MODE, const CHANNEL: u8> {
    _pd: PhantomData<MODE>,
}

impl<T, const CHANNEL: u8> Alarm<T, CHANNEL> {

    #[cfg(feature = "esp32s2")]
    pub const TICKS_PER_SECOND: u32 = 40_000_000;
    #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
    pub const TICKS_PER_SECOND: u32 = 16_000_000;

    // private constructor
    fn new() -> Self {
        Self { _pd: PhantomData }
    }

    pub fn enable_interrupt(&self) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        match CHANNEL {
            0 => systimer
                .int_ena
                .modify(|_, w| w.target0_int_ena().set_bit()),
            1 => systimer
                .int_ena
                .modify(|_, w| w.target1_int_ena().set_bit()),
            2 => systimer
                .int_ena
                .modify(|_, w| w.target2_int_ena().set_bit()),
            _ => unreachable!(),
        }
    }

    pub fn clear_interrupt(&self) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        match CHANNEL {
            0 => systimer.int_clr.write(|w| w.target0_int_clr().set_bit()),
            1 => systimer.int_clr.write(|w| w.target1_int_clr().set_bit()),
            2 => systimer.int_clr.write(|w| w.target2_int_clr().set_bit()),
            _ => unreachable!(),
        }
    }
}

impl<const CHANNEL: u8> Alarm<Target, CHANNEL> {
    pub fn set_target(&self, timestamp: u64) {
        unsafe {
            let systimer = &*SYSTIMER::ptr();
            let (tconf, hi, lo): (
                &Reg<TARGET0_CONF_SPEC>,
                &Reg<TARGET0_HI_SPEC>,
                &Reg<TARGET0_LO_SPEC>,
            ) = match CHANNEL {
                0 => (
                    &systimer.target0_conf,
                    &systimer.target0_hi,
                    &systimer.target0_lo,
                ),
                1 => (
                    transmute(&systimer.target1_conf),
                    transmute(&systimer.target1_hi),
                    transmute(&systimer.target1_lo),
                ),
                2 => (
                    transmute(&systimer.target2_conf),
                    transmute(&systimer.target2_hi),
                    transmute(&systimer.target2_lo),
                ),
                _ => unreachable!(),
            };

            #[cfg(feature = "esp32s2")]
            systimer.step.write(|w| w.timer_xtal_step().bits(0x1)); // run at XTAL freq, not 80 * XTAL freq

            #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
            {
                tconf.write(|w| w.target0_timer_unit_sel().clear_bit()); // default, use unit 0
                systimer
                    .conf
                    .modify(|_, w| w.timer_unit0_core0_stall_en().clear_bit());
            }

            tconf.write(|w| w.target0_period_mode().clear_bit()); // target mode
            hi.write(|w| w.timer_target0_hi().bits((timestamp >> 32) as u32));
            lo.write(|w| w.timer_target0_lo().bits((timestamp & 0xFFFF_FFFF) as u32));

            #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
            {
                match CHANNEL {
                    0 => {
                        systimer
                            .comp0_load
                            .write(|w| w.timer_comp0_load().set_bit());
                    }
                    1 => systimer
                        .comp1_load
                        .write(|w| w.timer_comp1_load().set_bit()),
                    2 => systimer
                        .comp2_load
                        .write(|w| w.timer_comp2_load().set_bit()),
                    _ => unreachable!(),
                }

                systimer.conf.modify(|_r, w| match CHANNEL {
                    0 => w.target0_work_en().set_bit(),
                    1 => w.target1_work_en().set_bit(),
                    2 => w.target2_work_en().set_bit(),
                    _ => unreachable!(),
                });
            }

            #[cfg(feature = "esp32s2")]
            tconf.modify(|_r, w| match CHANNEL {
                0 => w.target0_work_en().set_bit(),
                1 => w.target0_work_en().set_bit(),
                2 => w.target0_work_en().set_bit(),
                _ => unreachable!(),
            });
        }
    }

    pub fn into_periodic(self) -> Alarm<Periodic, CHANNEL> {
        Alarm { _pd: PhantomData }
    }
}

impl<const CHANNEL: u8> Alarm<Periodic, CHANNEL> {
    pub fn set_period(&self, period: fugit::HertzU32) {
        let time_period: MillisDurationU32 = period.into_duration();
        let cycles = time_period.ticks();
        unsafe {
            let systimer = &*SYSTIMER::ptr();
            let (tconf, hi, lo): (
                &Reg<TARGET0_CONF_SPEC>,
                &Reg<TARGET0_HI_SPEC>,
                &Reg<TARGET0_LO_SPEC>,
            ) = match CHANNEL {
                0 => (
                    &systimer.target0_conf,
                    &systimer.target0_hi,
                    &systimer.target0_lo,
                ),
                1 => (
                    transmute(&systimer.target1_conf),
                    transmute(&systimer.target1_hi),
                    transmute(&systimer.target1_lo),
                ),
                2 => (
                    transmute(&systimer.target2_conf),
                    transmute(&systimer.target2_hi),
                    transmute(&systimer.target2_lo),
                ),
                _ => unreachable!(),
            };

            #[cfg(feature = "esp32s2")]
            systimer.step.write(|w| w.timer_xtal_step().bits(0x1)); // run at XTAL freq, not 80 * XTAL freq

            #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
            {
                tconf.write(|w| w.target0_timer_unit_sel().clear_bit()); // default, use unit 0
                systimer
                    .conf
                    .modify(|_, w| w.timer_unit0_core0_stall_en().clear_bit());
            }

            tconf.write(|w| {
                // periodic mode, period in cycles
                w.target0_period_mode().set_bit().target0_period().bits(cycles * (Self::TICKS_PER_SECOND / 1000))
            }); 
            hi.write(|w| w.timer_target0_hi().bits(0));
            lo.write(|w| w.timer_target0_lo().bits(0));

            #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
            {
                match CHANNEL {
                    0 => {
                        systimer
                            .comp0_load
                            .write(|w| w.timer_comp0_load().set_bit());
                    }
                    1 => systimer
                        .comp1_load
                        .write(|w| w.timer_comp1_load().set_bit()),
                    2 => systimer
                        .comp2_load
                        .write(|w| w.timer_comp2_load().set_bit()),
                    _ => unreachable!(),
                }

                systimer.conf.modify(|_r, w| match CHANNEL {
                    0 => w.target0_work_en().set_bit(),
                    1 => w.target1_work_en().set_bit(),
                    2 => w.target2_work_en().set_bit(),
                    _ => unreachable!(),
                });
            }

            #[cfg(feature = "esp32s2")]
            tconf.modify(|_r, w| match CHANNEL {
                0 => w.target0_work_en().set_bit(),
                1 => w.target0_work_en().set_bit(),
                2 => w.target0_work_en().set_bit(),
                _ => unreachable!(),
            });
        }
    }

    pub fn into_target(self) -> Alarm<Target, CHANNEL> {
        Alarm { _pd: PhantomData }
    }
}
