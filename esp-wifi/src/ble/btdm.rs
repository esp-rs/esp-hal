use core::{cell::RefCell, ptr::addr_of};

use critical_section::Mutex;
use portable_atomic::{AtomicBool, Ordering};

use crate::{
    binary::include::*,
    ble::{
        btdm::ble_os_adapter_chip_specific::{osi_funcs_s, G_OSI_FUNCS},
        HciOutCollector,
        HCI_OUT_COLLECTOR,
    },
    compat::{common::str_from_c, queue::SimpleQueue},
    hal::macros::ram,
    memory_fence::memory_fence,
    timer::yield_task,
};

#[cfg_attr(esp32c3, path = "os_adapter_esp32c3.rs")]
#[cfg_attr(esp32s3, path = "os_adapter_esp32s3.rs")]
#[cfg_attr(esp32, path = "os_adapter_esp32.rs")]
pub(crate) mod ble_os_adapter_chip_specific;

static BT_RECEIVE_QUEUE: Mutex<RefCell<SimpleQueue<ReceivedPacket, 10>>> =
    Mutex::new(RefCell::new(SimpleQueue::new()));

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceivedPacket {
    pub len: u8,
    pub data: [u8; 256],
}

static BT_INTERNAL_QUEUE: Mutex<RefCell<SimpleQueue<[u8; 8], 10>>> =
    Mutex::new(RefCell::new(SimpleQueue::new()));

static PACKET_SENT: AtomicBool = AtomicBool::new(true);

#[repr(C)]
struct vhci_host_callback_s {
    notify_host_send_available: extern "C" fn(), /* callback used to notify that the host can
                                                  * send packet to controller */
    notify_host_recv: extern "C" fn(*mut u8, u16) -> i32, /* callback used to notify that the
                                                           * controller has a packet to send to
                                                           * the host */
}

extern "C" {
    fn btdm_osi_funcs_register(osi_funcs: *const osi_funcs_s) -> i32;
    fn btdm_controller_get_compile_version() -> *const u8;

    #[cfg(any(esp32c3, esp32s3))]
    fn btdm_controller_init(config_opts: *const esp_bt_controller_config_t) -> i32;

    #[cfg(esp32)]
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
    notify_host_send_available,
    notify_host_recv,
};

extern "C" fn notify_host_send_available() {
    trace!("notify_host_send_available");

    PACKET_SENT.store(true, Ordering::Relaxed);
}

extern "C" fn notify_host_recv(data: *mut u8, len: u16) -> i32 {
    trace!("notify_host_recv {:?} {}", data, len);

    unsafe {
        let mut buf = [0u8; 256];
        buf[..len as usize].copy_from_slice(core::slice::from_raw_parts(data, len as usize));

        let packet = ReceivedPacket {
            len: len as u8,
            data: buf,
        };

        critical_section::with(|cs| {
            let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);
            if queue.enqueue(packet).is_err() {
                warn!("Dropping BLE packet");
            }
        });

        dump_packet_info(core::slice::from_raw_parts(data as *const u8, len as usize));

        #[cfg(feature = "async")]
        crate::ble::controller::asynch::hci_read_data_available();
    }

    0
}

#[cfg(target_arch = "riscv32")]
type InterruptsFlagType = u8;

#[cfg(target_arch = "xtensa")]
type InterruptsFlagType = u32;

static mut G_INTER_FLAGS: [InterruptsFlagType; 10] = [0; 10];

static mut INTERRUPT_DISABLE_CNT: usize = 0;

#[ram]
unsafe extern "C" fn interrupt_enable() {
    INTERRUPT_DISABLE_CNT -= 1;
    let flags = G_INTER_FLAGS[INTERRUPT_DISABLE_CNT];
    trace!("interrupt_enable {}", flags);
    critical_section::release(core::mem::transmute::<
        InterruptsFlagType,
        critical_section::RestoreState,
    >(flags));
}

#[ram]
unsafe extern "C" fn interrupt_disable() {
    trace!("interrupt_disable");
    let flags = core::mem::transmute::<critical_section::RestoreState, InterruptsFlagType>(
        critical_section::acquire(),
    );
    G_INTER_FLAGS[INTERRUPT_DISABLE_CNT] = flags;
    INTERRUPT_DISABLE_CNT += 1;
    trace!("interrupt_disable {}", flags);
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
    trace!("Unimplemented queue_delete {:?}", queue);
}

#[ram]
unsafe extern "C" fn queue_send(queue: *const (), item: *const (), _block_time_ms: u32) -> i32 {
    if queue == &BT_INTERNAL_QUEUE as *const _ as *const () {
        critical_section::with(|_| {
            // assume the size is 8 - shouldn't rely on that
            let message = item as *const u8;
            let mut data = [0u8; 8];
            for (i, data) in data.iter_mut().enumerate() {
                *data = *message.add(i);
            }
            trace!("queue posting {:?}", data);

            critical_section::with(|cs| {
                let mut queue = BT_INTERNAL_QUEUE.borrow_ref_mut(cs);
                unwrap!(queue.enqueue(data));
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
    trace!("queue_send_from_isr {:?} {:?} {:?}", _queue, _item, _hptw);
    // Force to set the value to be false
    *(_hptw as *mut bool) = false;
    queue_send(_queue, _item, 0)
}

unsafe extern "C" fn queue_recv(queue: *const (), item: *const (), block_time_ms: u32) -> i32 {
    trace!(
        "queue_recv {:?} item {:?} block_time_tick {}",
        queue,
        item,
        block_time_ms
    );

    let forever = block_time_ms == crate::compat::common::OSI_FUNCS_TIME_BLOCKING;
    let start = crate::timer::get_systimer_count();
    let block_ticks = crate::timer::millis_to_ticks(block_time_ms as u64);

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
                        trace!("received {:?}", message);
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

            if !forever && crate::timer::elapsed_time_since(start) > block_ticks {
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
    let n = str_from_c(name);
    trace!(
        "task_create {:?} {:?} {} {} {:?} {} {:?} {}",
        func,
        name,
        n,
        stack_depth,
        param,
        prio,
        handle,
        core_id
    );

    let task_func = core::mem::transmute::<
        *mut crate::binary::c_types::c_void,
        extern "C" fn(*mut esp_wifi_sys::c_types::c_void),
    >(func);

    let task = crate::preempt::arch_specific::task_create(task_func, param, stack_depth as usize);
    *(handle as *mut usize) = task as usize;

    1
}

unsafe extern "C" fn task_delete(task: *const ()) {
    trace!("task delete called for {:?}", task);

    let task = if task.is_null() {
        crate::preempt::current_task()
    } else {
        task as *mut _
    };
    crate::preempt::schedule_task_deletion(task);
}

#[ram]
unsafe extern "C" fn is_in_isr() -> i32 {
    0
}

#[ram]
unsafe extern "C" fn cause_sw_intr_to_core(_core: i32, _intr_no: i32) -> i32 {
    #[cfg(any(esp32c3, esp32s3))]
    todo!("cause_sw_intr_to_core is not implemented for this target");

    #[cfg(esp32)]
    {
        trace!("cause_sw_intr_to_core {} {}", _core, _intr_no);
        let intr = 1 << _intr_no;
        core::arch::asm!("wsr.intset  {0}", in(reg) intr, options(nostack));
        0
    }
}

#[allow(unused)]
#[ram]
unsafe extern "C" fn srand(seed: u32) {
    debug!("!!!! unimplemented srand {}", seed);
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
    debug!("*** NOT implemented btdm_hus_2_lpcycles {} {}", us, cycles);
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
    debug!("coex_schm_status_bit_set {} {}", _typ, status);
    #[cfg(coex)]
    crate::binary::include::coex_schm_status_bit_set(_typ as u32, status as u32);
}

unsafe extern "C" fn coex_schm_status_bit_clear(_typ: i32, status: i32) {
    debug!("coex_schm_status_bit_clear {} {}", _typ, status);
    #[cfg(coex)]
    crate::binary::include::coex_schm_status_bit_clear(_typ as u32, status as u32);
}

#[ram]
unsafe extern "C" fn read_efuse_mac(mac: *const ()) -> i32 {
    crate::common_adapter::read_mac(mac as *mut _, 2)
}

#[cfg(esp32)]
unsafe extern "C" fn set_isr13(n: i32, handler: unsafe extern "C" fn(), arg: *const ()) -> i32 {
    ble_os_adapter_chip_specific::set_isr(n, handler, arg)
}

#[cfg(esp32)]
unsafe extern "C" fn interrupt_l3_disable() {
    // info!("unimplemented interrupt_l3_disable");
}

#[cfg(esp32)]
unsafe extern "C" fn interrupt_l3_restore() {
    //  info!("unimplemented interrupt_l3_restore");
}

#[cfg(esp32)]
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
        #[cfg(feature = "sys-logs")]
        {
            extern "C" {
                static mut g_bt_plf_log_level: u32;
            }

            debug!("g_bt_plf_log_level = {}", g_bt_plf_log_level);
            g_bt_plf_log_level = 10;
        }

        // esp32_bt_controller_init
        ble_os_adapter_chip_specific::btdm_controller_mem_init();

        let mut cfg = ble_os_adapter_chip_specific::create_ble_config();

        let res = btdm_osi_funcs_register(addr_of!(G_OSI_FUNCS));
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
        let version_str = str_from_c(version);
        debug!("BT controller compile version {}", version_str);

        ble_os_adapter_chip_specific::bt_periph_module_enable();

        ble_os_adapter_chip_specific::disable_sleep_mode();

        #[cfg(any(esp32c3, esp32s3))]
        let res = btdm_controller_init(&mut cfg as *mut esp_bt_controller_config_t);

        #[cfg(esp32)]
        let res = btdm_controller_init(
            (1 << 3) | (1 << 4),
            &mut cfg as *mut esp_bt_controller_config_t,
        ); // see btdm_config_mask_load for mask

        if res != 0 {
            panic!("btdm_controller_init returned {}", res);
        }

        debug!("The btdm_controller_init was initialized");

        #[cfg(coex)]
        crate::binary::include::coex_enable();

        crate::common_adapter::chip_specific::phy_enable();

        #[cfg(esp32)]
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

#[cfg(feature = "async")]
pub fn have_hci_read_data() -> bool {
    critical_section::with(|cs| {
        let queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);
        !queue.is_empty()
            || unsafe {
                BLE_HCI_READ_DATA_LEN > 0 && (BLE_HCI_READ_DATA_LEN >= BLE_HCI_READ_DATA_INDEX)
            }
    })
}

pub(crate) fn read_next(data: &mut [u8]) -> usize {
    critical_section::with(|cs| {
        let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);

        match queue.dequeue() {
            Some(packet) => {
                data[..packet.len as usize].copy_from_slice(&packet.data[..packet.len as usize]);
                packet.len as usize
            }
            None => 0,
        }
    })
}

pub fn read_hci(data: &mut [u8]) -> usize {
    unsafe {
        if BLE_HCI_READ_DATA_LEN == 0 {
            critical_section::with(|cs| {
                let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);

                if let Some(packet) = queue.dequeue() {
                    BLE_HCI_READ_DATA[..packet.len as usize]
                        .copy_from_slice(&packet.data[..packet.len as usize]);
                    BLE_HCI_READ_DATA_LEN = packet.len as usize;
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
                    trace!("can_send is false");
                    continue;
                }

                PACKET_SENT.store(false, Ordering::Relaxed);
                API_vhci_host_send_packet(packet.as_ptr(), packet.len() as u16);
                trace!("sent vhci host packet");

                dump_packet_info(packet);

                break;
            }

            // make sure the packet buffer doesn't get touched until sent
            while !PACKET_SENT.load(Ordering::Relaxed) {}
        }

        hci_out.reset();
    }
}

#[allow(unreachable_code, unused_variables)]
fn dump_packet_info(buffer: &[u8]) {
    #[cfg(not(feature = "dump-packets"))]
    return;

    critical_section::with(|cs| {
        info!("@HCIFRAME {:?}", buffer);
    });
}
