use core::cell::RefCell;

use critical_section::Mutex;
use log::trace;

use crate::ble::btdm::ble_os_adapter_chip_specific::G_OSI_FUNCS;
use crate::ble::HciOutCollector;
use crate::ble::HCI_OUT_COLLECTOR;
use crate::{
    binary::include::*,
    compat::{common::StrBuf, queue::SimpleQueue, work_queue::queue_work},
    memory_fence::memory_fence,
    timer::yield_task,
};

#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32s3")]
use esp32s3_hal as hal;

use hal::macros::ram;

#[cfg_attr(feature = "esp32c3", path = "os_adapter_esp32c3.rs")]
#[cfg_attr(feature = "esp32s3", path = "os_adapter_esp32s3.rs")]
#[cfg_attr(feature = "esp32", path = "os_adapter_esp32.rs")]
pub(crate) mod ble_os_adapter_chip_specific;

static BT_RECEIVE_QUEUE: Mutex<RefCell<SimpleQueue<ReceivedPacket, 10>>> =
    Mutex::new(RefCell::new(SimpleQueue::new()));

#[derive(Debug, Clone, Copy)]
pub struct ReceivedPacket {
    pub len: u8,
    pub data: [u8; 256],
}

static BT_INTERNAL_QUEUE: Mutex<RefCell<SimpleQueue<[u8; 8], 10>>> =
    Mutex::new(RefCell::new(SimpleQueue::new()));

#[repr(C)]
struct vhci_host_callback_s {
    notify_host_send_available: extern "C" fn(), /* callback used to notify that the host can send packet to controller */
    notify_host_recv: extern "C" fn(*mut u8, u16) -> i32, /* callback used to notify that the controller has a packet to send to the host */
}

extern "C" {
    fn btdm_osi_funcs_register(osi_funcs: *const ()) -> i32;
    fn btdm_controller_get_compile_version() -> *const u8;

    #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
    fn btdm_controller_init(config_opts: *const esp_bt_controller_config_t) -> i32;

    #[cfg(feature = "esp32")]
    fn btdm_controller_init(
        config_mask: u32,
        config_opts: *const esp_bt_controller_config_t,
    ) -> i32;

    fn btdm_controller_enable(mode: esp_bt_mode_t);

    fn API_vhci_host_check_send_available() -> bool;
    fn API_vhci_host_send_packet(data: *const u8, len: u16);
    fn API_vhci_host_register_callback(vhci_host_callbac: *const vhci_host_callback_s) -> i32;
}

static VHCI_HOST_CALLBACK: vhci_host_callback_s = vhci_host_callback_s {
    notify_host_send_available: notify_host_send_available,
    notify_host_recv: notify_host_recv,
};

extern "C" fn notify_host_send_available() {
    trace!("notify_host_send_available");
}

extern "C" fn notify_host_recv(data: *mut u8, len: u16) -> i32 {
    trace!("notify_host_recv {:p} {}", data, len);

    unsafe {
        let mut buf = [0u8; 256];
        for i in 0..len {
            let b = data.offset(i as isize).read();
            buf[i as usize] = b;
        }

        let packet = ReceivedPacket {
            len: len as u8,
            data: buf,
        };

        critical_section::with(|cs| {
            let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);
            queue.enqueue(packet).unwrap();
        });
    }

    0
}

#[cfg(target_arch = "riscv32")]
static mut G_INTER_FLAGS: [u8; 10] = [0; 10];

#[cfg(target_arch = "xtensa")]
static mut G_INTER_FLAGS: [u32; 10] = [0; 10];

static mut INTERRUPT_DISABLE_CNT: usize = 0;

#[ram]
unsafe extern "C" fn interrupt_enable() {
    INTERRUPT_DISABLE_CNT -= 1;
    let flags = G_INTER_FLAGS[INTERRUPT_DISABLE_CNT];
    log::trace!("interrupt_enable {}", flags);
    critical_section::release(core::mem::transmute(flags));
}

#[ram]
unsafe extern "C" fn interrupt_disable() {
    log::trace!("interrupt_disable");
    let flags = core::mem::transmute(critical_section::acquire());
    G_INTER_FLAGS[INTERRUPT_DISABLE_CNT] = flags;
    INTERRUPT_DISABLE_CNT += 1;
    log::trace!("interrupt_disable {}", flags);
}

#[ram]
unsafe extern "C" fn task_yield() {
    todo!();
}

unsafe extern "C" fn task_yield_from_isr() {
    todo!();
}

unsafe extern "C" fn semphr_create(max: u32, init: u32) -> *const () {
    crate::common_adapter::semphr_create(max, init) as *const ()
}

unsafe extern "C" fn semphr_delete(sem: *const ()) {
    crate::common_adapter::semphr_delete(sem as *mut crate::binary::c_types::c_void);
}

unsafe extern "C" fn semphr_take(sem: *const (), block_time_ms: u32) -> i32 {
    crate::common_adapter::semphr_take(sem as *mut crate::binary::c_types::c_void, block_time_ms)
}

unsafe extern "C" fn semphr_give(sem: *const ()) -> i32 {
    crate::common_adapter::semphr_give(sem as *mut crate::binary::c_types::c_void)
}

unsafe extern "C" fn mutex_create() -> *const () {
    todo!();
}

unsafe extern "C" fn mutex_delete(_mutex: *const ()) {
    todo!();
}

unsafe extern "C" fn mutex_lock(_mutex: *const ()) -> i32 {
    todo!();
}

unsafe extern "C" fn mutex_unlock(_mutex: *const ()) -> i32 {
    todo!();
}

unsafe extern "C" fn queue_create(len: u32, item_size: u32) -> *const () {
    if len != 5 && item_size != 8 {
        panic!("Unexpected queue spec {} {}", len, item_size);
    }
    &BT_INTERNAL_QUEUE as *const _ as *const ()
}

unsafe extern "C" fn queue_delete(queue: *const ()) {
    trace!("Unimplemented queue_delete {:p}", queue);
}

#[ram]
unsafe extern "C" fn queue_send(queue: *const (), item: *const (), _block_time_ms: u32) -> i32 {
    if queue == &BT_INTERNAL_QUEUE as *const _ as *const () {
        critical_section::with(|_| {
            // assume the size is 8 - shouldn't rely on that
            let message = item as *const u8;
            let mut data = [0u8; 8];
            for i in 0..8 as usize {
                data[i] = *(message.offset(i as isize));
            }
            trace!("queue posting {:x?}", data);

            critical_section::with(|cs| {
                let mut queue = BT_INTERNAL_QUEUE.borrow_ref_mut(cs);
                queue.enqueue(data).unwrap();
            });
            memory_fence();
        });
    } else {
        panic!("Unknown queue");
    }
    1
}

#[ram]
unsafe extern "C" fn queue_send_from_isr(
    _queue: *const (),
    _item: *const (),
    _hptw: *const (),
) -> i32 {
    log::trace!("queue_send_from_isr {:p} {:p} {:p}", _queue, _item, _hptw);
    // Force to set the value to be false
    *(_hptw as *mut bool) = false;
    queue_send(_queue, _item, 0)
}

unsafe extern "C" fn queue_recv(queue: *const (), item: *const (), block_time_ms: u32) -> i32 {
    trace!(
        "queue_recv {:p} item {:p} block_time_tick {}",
        queue,
        item,
        block_time_ms
    );

    // is this ticks or millis?
    let end_time = crate::timer::get_systimer_count()
        + (block_time_ms as u64 * (crate::timer::TICKS_PER_SECOND / 1000));

    // handle the BT_QUEUE
    if queue == &BT_INTERNAL_QUEUE as *const _ as *const () {
        loop {
            let res = critical_section::with(|_| {
                memory_fence();

                critical_section::with(|cs| {
                    let mut queue = BT_INTERNAL_QUEUE.borrow_ref_mut(cs);
                    if let Some(message) = queue.dequeue() {
                        let item = item as *mut u8;
                        for i in 0..8 {
                            item.offset(i).write_volatile(message[i as usize]);
                        }
                        trace!("received {:x?}", message);
                        1
                    } else {
                        0
                    }
                })
            });

            if res == 1 {
                trace!("queue_recv returns");
                return res;
            }

            if block_time_ms != OSI_FUNCS_TIME_BLOCKING
                && crate::timer::get_systimer_count() > end_time
            {
                trace!("queue_recv returns with timeout");
                return -1;
            }

            yield_task();
        }
    } else {
        panic!("Unknown queue to handle in queue_recv");
    }
}

#[ram]
unsafe extern "C" fn queue_recv_from_isr(
    _queue: *const (),
    _item: *const (),
    _hptw: *const (),
) -> i32 {
    todo!();
}

unsafe extern "C" fn task_create(
    func: *mut crate::binary::c_types::c_void,
    name: *const u8,
    stack_depth: u32,
    param: *mut crate::binary::c_types::c_void,
    prio: u32,
    handle: *mut crate::binary::c_types::c_void,
    core_id: u32,
) -> i32 {
    let n = StrBuf::from(name);
    trace!(
        "recheck implementation: task_create {:p} {:p} {} {} {:p} {} {:p} {}",
        func,
        name,
        n.as_str_ref(),
        stack_depth,
        param,
        prio,
        handle,
        core_id
    );

    *(handle as *mut usize) = 0; // we will run it in task 0

    queue_work(func, name, stack_depth, param, prio, handle, core_id);
    1
}

unsafe extern "C" fn task_delete(_task: *const ()) {
    todo!();
}

#[ram]
unsafe extern "C" fn is_in_isr() -> i32 {
    0
}

#[ram]
unsafe extern "C" fn cause_sw_intr_to_core(_core: i32, _intr_no: i32) -> i32 {
    #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
    todo!("cause_sw_intr_to_core is not implemented for this target");

    #[cfg(feature = "esp32")]
    {
        log::trace!("cause_sw_intr_to_core {} {}", _core, _intr_no);
        let intr = 1 << _intr_no;
        core::arch::asm!("wsr.226  {0}", in(reg) intr, options(nostack)); // 226 = "intset"
        0
    }
}

unsafe extern "C" fn malloc(size: u32) -> *const () {
    crate::compat::malloc::malloc(size) as *const ()
}

unsafe extern "C" fn malloc_internal(size: u32) -> *const () {
    crate::compat::malloc::malloc(size) as *const ()
}

unsafe extern "C" fn free(ptr: *const ()) {
    crate::compat::malloc::free(ptr as *const u8);
}

#[allow(unused)]
#[ram]
unsafe extern "C" fn srand(seed: u32) {
    log::debug!("!!!! unimplemented srand {}", seed);
}

#[allow(unused)]
#[ram]
unsafe extern "C" fn rand() -> i32 {
    trace!("rand");
    crate::common_adapter::random() as i32
}

#[ram]
unsafe extern "C" fn btdm_lpcycles_2_hus(_cycles: u32, _error_corr: u32) -> u32 {
    todo!();
}

#[ram]
unsafe extern "C" fn btdm_hus_2_lpcycles(us: u32) -> u32 {
    const RTC_CLK_CAL_FRACT: u32 = 19;
    let g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
    let g_btdm_lpcycle_us = 2 << (g_btdm_lpcycle_us_frac);

    // Converts a duration in half us into a number of low power clock cycles.
    let cycles: u64 = (us as u64) << (g_btdm_lpcycle_us_frac as u64 / g_btdm_lpcycle_us as u64);
    log::debug!("*** NOT implemented btdm_hus_2_lpcycles {} {}", us, cycles);
    // probably not right ... NX returns half of the values we calculate here

    cycles as u32
}

unsafe extern "C" fn btdm_sleep_check_duration(_slot_cnt: i32) -> i32 {
    todo!();
}

unsafe extern "C" fn btdm_sleep_enter_phase1(_lpcycles: i32) {
    todo!();
}

unsafe extern "C" fn btdm_sleep_enter_phase2() {
    todo!();
}

unsafe extern "C" fn btdm_sleep_exit_phase1() {
    todo!();
}

unsafe extern "C" fn btdm_sleep_exit_phase2() {
    todo!();
}

unsafe extern "C" fn btdm_sleep_exit_phase3() {
    todo!();
}

unsafe extern "C" fn coex_schm_status_bit_set(_typ: i32, status: i32) {
    log::debug!("coex_schm_status_bit_set {} {}", _typ, status);
    #[cfg(coex)]
    crate::binary::include::coex_schm_status_bit_set(_typ as u32, status as u32);
}

unsafe extern "C" fn coex_schm_status_bit_clear(_typ: i32, status: i32) {
    log::debug!("coex_schm_status_bit_clear {} {}", _typ, status);
    #[cfg(coex)]
    crate::binary::include::coex_schm_status_bit_clear(_typ as u32, status as u32);
}

#[ram]
unsafe extern "C" fn read_efuse_mac(mac: *const ()) -> i32 {
    crate::common_adapter::chip_specific::read_mac(mac as *mut _, 2)
}

#[cfg(feature = "esp32")]
unsafe extern "C" fn set_isr13(n: i32, handler: unsafe extern "C" fn(), arg: *const ()) -> i32 {
    ble_os_adapter_chip_specific::set_isr(n, handler, arg)
}

#[cfg(feature = "esp32")]
unsafe extern "C" fn interrupt_l3_disable() {
    // log::info!("unimplemented interrupt_l3_disable");
}

#[cfg(feature = "esp32")]
unsafe extern "C" fn interrupt_l3_restore() {
    //  log::info!("unimplemented interrupt_l3_restore");
}

#[cfg(feature = "esp32")]
unsafe extern "C" fn custom_queue_create(
    _len: u32,
    _item_size: u32,
) -> *mut crate::binary::c_types::c_void {
    todo!();
}

pub(crate) fn ble_init() {
    unsafe {
        *(HCI_OUT_COLLECTOR.as_mut_ptr()) = HciOutCollector::new();
        // turn on logging
        #[cfg(feature = "wifi-logs")]
        {
            extern "C" {
                static mut g_bt_plf_log_level: u32;
            }

            log::debug!("g_bt_plf_log_level = {}", g_bt_plf_log_level);
            g_bt_plf_log_level = 10;
        }

        // esp32_bt_controller_init
        ble_os_adapter_chip_specific::btdm_controller_mem_init();

        let mut cfg = ble_os_adapter_chip_specific::create_ble_config();

        let res = btdm_osi_funcs_register(&G_OSI_FUNCS as *const _ as *const ());
        if res != 0 {
            panic!("btdm_osi_funcs_register returned {}", res);
        }

        #[cfg(coex)]
        {
            let res = crate::wifi::coex_init();
            if res != 0 {
                panic!("got error");
            }
        }

        let version = btdm_controller_get_compile_version();
        let version_str = StrBuf::from(version);
        log::debug!("BT controller compile version {}", version_str.as_str_ref());

        ble_os_adapter_chip_specific::bt_periph_module_enable();

        ble_os_adapter_chip_specific::disable_sleep_mode();

        #[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
        let res = btdm_controller_init(&mut cfg as *mut esp_bt_controller_config_t);

        #[cfg(feature = "esp32")]
        let res = btdm_controller_init(
            (1 << 3) | (1 << 4),
            &mut cfg as *mut esp_bt_controller_config_t,
        ); // see btdm_config_mask_load for mask

        if res != 0 {
            panic!("btdm_controller_init returned {}", res);
        }

        log::debug!("The btdm_controller_init was initialized");

        #[cfg(coex)]
        crate::binary::include::coex_enable();

        crate::common_adapter::chip_specific::phy_enable();

        #[cfg(feature = "esp32")]
        {
            extern "C" {
                fn btdm_rf_bb_init_phase2();
            }

            btdm_rf_bb_init_phase2();
            coex_bt_high_prio();
        }

        #[cfg(coex)]
        coex_enable();

        btdm_controller_enable(esp_bt_mode_t_ESP_BT_MODE_BLE);

        API_vhci_host_register_callback(&VHCI_HOST_CALLBACK);
    }
}

static mut BLE_HCI_READ_DATA: [u8; 256] = [0u8; 256];
static mut BLE_HCI_READ_DATA_INDEX: usize = 0;
static mut BLE_HCI_READ_DATA_LEN: usize = 0;

pub fn read_hci(data: &mut [u8]) -> usize {
    unsafe {
        if BLE_HCI_READ_DATA_LEN == 0 {
            critical_section::with(|cs| {
                let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);

                if let Some(packet) = queue.dequeue() {
                    for i in 0..(packet.len as usize + 0/*1*/) {
                        BLE_HCI_READ_DATA[i] = packet.data[i];
                    }

                    BLE_HCI_READ_DATA_LEN = packet.len as usize + 0 /*1*/;
                    BLE_HCI_READ_DATA_INDEX = 0;
                }
            });
        }

        if BLE_HCI_READ_DATA_LEN > 0 {
            data[0] = BLE_HCI_READ_DATA[BLE_HCI_READ_DATA_INDEX];
            BLE_HCI_READ_DATA_INDEX += 1;

            if BLE_HCI_READ_DATA_INDEX >= BLE_HCI_READ_DATA_LEN {
                BLE_HCI_READ_DATA_LEN = 0;
                BLE_HCI_READ_DATA_INDEX = 0;
            }
            return 1;
        }
    }

    0
}

pub fn send_hci(data: &[u8]) {
    let hci_out = unsafe { &mut *HCI_OUT_COLLECTOR.as_mut_ptr() };
    hci_out.push(data);

    if hci_out.is_ready() {
        let packet = hci_out.packet();

        unsafe {
            loop {
                let can_send = API_vhci_host_check_send_available();

                if !can_send {
                    log::trace!("can_send is false");
                    continue;
                }

                API_vhci_host_send_packet(packet.as_ptr() as *const u8, packet.len() as u16);
                log::trace!("sent vhci host packet");

                break;
            }
        }

        hci_out.reset();
    }
}
