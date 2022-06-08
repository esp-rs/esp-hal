use core::{intrinsics::transmute, marker::PhantomData};

use esp32c3_pac::{
    generic::Reg,
    systimer::{
        comp0_load::COMP0_LOAD_SPEC,
        target0_conf::TARGET0_CONF_SPEC,
        target0_hi::TARGET0_HI_SPEC,
        target0_lo::TARGET0_LO_SPEC,
    },
};

use crate::pac::SYSTIMER;

// TODO this only handles unit0 of the systimer

#[derive(Debug)]
pub struct SystemTimer {
    inner: SYSTIMER,
    alrm0: Option<Alarm<Target, 0>>,
    alrm1: Option<Alarm<Target, 1>>,
    alrm2: Option<Alarm<Target, 2>>,
}

impl SystemTimer {
    pub fn new(p: SYSTIMER) -> Self {
        // TODO enable
        Self {
            inner: p,
            alrm0: Some(Alarm::new()),
            alrm1: Some(Alarm::new()),
            alrm2: Some(Alarm::new()),
        }
    }

    // TODO use fugit types
    pub fn now(&self) -> u64 {
        self.inner
            .unit0_op
            .write(|w| w.timer_unit0_update().set_bit());

        while !self
            .inner
            .unit0_op
            .read()
            .timer_unit0_value_valid()
            .bit_is_set()
        {}

        let value_lo = self.inner.unit0_value_lo.read().bits();
        let value_hi = self.inner.unit0_value_hi.read().bits();

        ((value_hi as u64) << 32) | value_lo as u64
    }

    pub fn alarm0(&mut self) -> Option<Alarm<Target, 0>> {
        self.alrm0.take()
    }

    pub fn alarm1(&mut self) -> Option<Alarm<Target, 1>> {
        self.alrm1.take()
    }

    pub fn alarm2(&mut self) -> Option<Alarm<Target, 2>> {
        self.alrm2.take()
    }
}

#[derive(Debug)]
pub struct Target;
// pub struct Periodic; // TODO

#[derive(Debug)]
pub struct Alarm<MODE, const CHANNEL: u8> {
    _pd: PhantomData<MODE>,
}

impl<const CHANNEL: u8> Alarm<Target, CHANNEL> {
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

    pub fn set_target(&self, timestamp: u64) {
        unsafe {
            let systimer = &*SYSTIMER::ptr();
            let (tconf, hi, lo, comp): (
                &Reg<TARGET0_CONF_SPEC>,
                &Reg<TARGET0_HI_SPEC>,
                &Reg<TARGET0_LO_SPEC>,
                &Reg<COMP0_LOAD_SPEC>,
            ) = match CHANNEL {
                0 => (
                    &systimer.target0_conf,
                    &systimer.target0_hi,
                    &systimer.target0_lo,
                    &systimer.comp0_load,
                ),
                1 => (
                    transmute(&systimer.target1_conf),
                    transmute(&systimer.target1_hi),
                    transmute(&systimer.target1_lo),
                    transmute(&systimer.comp1_load),
                ),
                2 => (
                    transmute(&systimer.target2_conf),
                    transmute(&systimer.target2_hi),
                    transmute(&systimer.target2_lo),
                    transmute(&systimer.comp2_load),
                ),
                _ => unreachable!(),
            };

            tconf.write(|w| {
                w.target0_timer_unit_sel()
                    .clear_bit()
                    .target0_period_mode()
                    .clear_bit()
            });

            hi.write(|w| w.timer_target0_hi().bits((timestamp >> 32) as u32));
            lo.write(|w| w.timer_target0_lo().bits((timestamp & 0xFFFF_FFFF) as u32));

            comp.write(|w| w.timer_comp0_load().set_bit());

            systimer.conf.modify(|_r, w| match CHANNEL {
                0 => w
                    .target0_work_en()
                    .set_bit()
                    .timer_unit0_core0_stall_en()
                    .clear_bit(),
                1 => w
                    .target1_work_en()
                    .set_bit()
                    .timer_unit0_core0_stall_en()
                    .clear_bit(),
                2 => w
                    .target2_work_en()
                    .set_bit()
                    .timer_unit0_core0_stall_en()
                    .clear_bit(),
                _ => unreachable!(),
            });
        }
    }
}
