use esp32c6_hal::riscv;
use log::trace;

pub(crate) fn chip_ints_on(mask: u32) {
    let cpuint = match mask {
        2 => 1,
        _ => panic!("ints_on mask {} not handled", mask),
    };

    trace!("ints_on n={}", cpuint);

    unsafe {
        (*esp32c6::INTPRI::PTR)
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

    const MODEM_SYSCON: u32 = 0x600A9800;
    const CLK_CONF1: u32 = 5 * 4;
    const MODEM_LPCON: u32 = 0x600AF000;
    const CLK_CONF: u32 = 6 * 4;

    unsafe {
        let clk_conf1 = (MODEM_SYSCON + CLK_CONF1) as *mut u32;
        clk_conf1.write_volatile(
            clk_conf1.read_volatile() |
                1 << 10 // clk_wifi_apb_en
                | 1 << 9 // clk_wifimac_en
                | 1 << 16 // fe_apb_clock
                | 1 << 15 // fe_cal_160m_clock
                | 1 << 14 // fe_160m_clock
                | 1 << 13 // fe_80m_clock
                | 1 << 8 // wifibb_160x1_clock
                | 1 << 7 // wifibb_80x1_clock
                | 1 << 6 // wifibb_40x1_clock
                | 1 << 5 // wifibb_80x_clock
                | 1 << 4 // wifibb_40x_clock
                | 1 << 3 // wifibb_80m_clock
                | 1 << 2 // wifibb_44m_clock
                | 1 << 1 // wifibb_40m_clock
                | 1 << 0, // wifibb_22m_clock
        );

        let clk_conf = (MODEM_LPCON + CLK_CONF) as *mut u32;
        clk_conf.write_volatile(
            clk_conf.read_volatile() |
                1 << 0 // clk_wifipwr_en
                | 1 << 1, // enable_coex_clock
        );
    }
}

pub(crate) unsafe extern "C" fn wifi_clock_disable() {
    trace!("wifi_clock_disable");

    const MODEM_SYSCON: u32 = 0x600A9800;
    const CLK_CONF1: u32 = 5 * 4;
    const MODEM_LPCON: u32 = 0x600AF000;
    const CLK_CONF: u32 = 6 * 4;

    unsafe {
        let clk_conf1 = (MODEM_SYSCON + CLK_CONF1) as *mut u32;
        clk_conf1.write_volatile(
            clk_conf1.read_volatile()
                & !(
                    1 << 10 // clk_wifi_apb_en
                | 1 << 9 // clk_wifimac_en
                | 1 << 16 // fe_apb_clock
                | 1 << 15 // fe_cal_160m_clock
                | 1 << 14 // fe_160m_clock
                | 1 << 13 // fe_80m_clock
                | 1 << 8 // wifibb_160x1_clock
                | 1 << 7 // wifibb_80x1_clock
                | 1 << 6 // wifibb_40x1_clock
                | 1 << 5 // wifibb_80x_clock
                | 1 << 4 // wifibb_40x_clock
                | 1 << 3 // wifibb_80m_clock
                | 1 << 2 // wifibb_44m_clock
                | 1 << 1 // wifibb_40m_clock
                | 1 << 0
                    // wifibb_22m_clock
                ),
        );

        let clk_conf = (MODEM_LPCON + CLK_CONF) as *mut u32;
        clk_conf.write_volatile(
            clk_conf.read_volatile()
                & !(
                    1 << 0 // clk_wifipwr_en
                | 1 << 1
                    // enable_coex_clock
                ),
        );
    }
}

pub(crate) unsafe extern "C" fn regdma_link_set_write_wait_content_dummy(
    _arg1: *mut esp_wifi_sys::c_types::c_void,
    _arg2: u32,
    _arg3: u32,
) {
    todo!()
}

pub(crate) unsafe extern "C" fn sleep_retention_find_link_by_id_dummy(
    _arg1: esp_wifi_sys::c_types::c_int,
) -> *mut esp_wifi_sys::c_types::c_void {
    todo!()
}

pub(crate) unsafe extern "C" fn sleep_retention_entries_create_dummy(
    _arg1: *const esp_wifi_sys::c_types::c_void,
    _arg2: esp_wifi_sys::c_types::c_int,
    _arg3: esp_wifi_sys::c_types::c_int,
    _arg4: esp_wifi_sys::c_types::c_int,
) -> esp_wifi_sys::c_types::c_int {
    todo!()
}

pub(crate) unsafe extern "C" fn sleep_retention_entries_destroy_dummy(
    _arg1: esp_wifi_sys::c_types::c_int,
) {
    todo!()
}
