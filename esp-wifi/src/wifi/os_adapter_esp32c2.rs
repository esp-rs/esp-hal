use log::trace;

pub(crate) fn chip_ints_on(mask: u32) {
    let cpuint = match mask {
        2 => 1,
        _ => panic!("ints_on mask {} not handled", mask),
    };

    trace!("ints_on n={}", cpuint);

    unsafe {
        (*esp32c2::INTERRUPT_CORE0::PTR)
            .cpu_int_enable
            .modify(|r, w| w.bits(r.bits() | 1 << cpuint));
    }
}

pub(crate) unsafe extern "C" fn wifi_int_disable(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
) -> u32 {
    let res = if riscv::register::mstatus::read().mie() {
        1
    } else {
        0
    };
    riscv::interrupt::disable();

    trace!(
        "wifi_int_disable wifi_int_mux {:p} - return {}",
        wifi_int_mux,
        res,
    );

    res
}

pub(crate) unsafe extern "C" fn wifi_int_restore(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
    tmp: u32,
) {
    trace!(
        "wifi_int_restore wifi_int_mux {:p} tmp {}",
        wifi_int_mux,
        tmp
    );

    if tmp == 1 {
        riscv::interrupt::enable();
    }
}

pub(crate) unsafe extern "C" fn set_intr(
    cpu_no: i32,
    intr_source: u32,
    intr_num: u32,
    intr_prio: i32,
) {
    trace!(
        "set_intr {} {} {} {}",
        cpu_no,
        intr_source,
        intr_num,
        intr_prio
    );

    // this gets called with
    // INFO - set_intr 0 2 1 1 (WIFI_PWR)
    // INFO - set_intr 0 0 1 1 (WIFI_MAC)

    // we do nothing here since all the interrupts are already
    // configured in `setup_timer_isr` and messing with the interrupts will
    // get us into trouble
}

pub(crate) unsafe extern "C" fn wifi_clock_enable() {
    trace!("wifi_clock_enable");

    const SYSCON_WIFI_CLK_EN_REG: *mut u32 = (0x60026000 + 0x14) as *mut u32;
    const SYSTEM_WIFI_CLK_WIFI_EN_M: u32 = 0;
    const SYSTEM_CORE_RST_EN_REG: *mut u32 = (0x60026000 + 0x18) as *mut u32;

    SYSCON_WIFI_CLK_EN_REG
        .write_volatile(SYSCON_WIFI_CLK_EN_REG.read_volatile() | SYSTEM_WIFI_CLK_WIFI_EN_M);
    SYSTEM_CORE_RST_EN_REG.write_volatile(SYSTEM_CORE_RST_EN_REG.read_volatile() & !0);
}

pub(crate) unsafe extern "C" fn wifi_clock_disable() {
    trace!("wifi_clock_disable");

    const SYSCON_WIFI_CLK_EN_REG: *mut u32 = (0x60026000 + 0x14) as *mut u32;
    const SYSTEM_WIFI_CLK_WIFI_EN_M: u32 = 0;
    const SYSTEM_CORE_RST_EN_REG: *mut u32 = (0x60026000 + 0x18) as *mut u32;

    SYSCON_WIFI_CLK_EN_REG
        .write_volatile(SYSCON_WIFI_CLK_EN_REG.read_volatile() & !SYSTEM_WIFI_CLK_WIFI_EN_M);
    SYSTEM_CORE_RST_EN_REG.write_volatile(SYSTEM_CORE_RST_EN_REG.read_volatile() | 0);
}
