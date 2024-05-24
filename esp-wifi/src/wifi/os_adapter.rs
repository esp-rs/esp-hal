#[cfg_attr(esp32c3, path = "os_adapter_esp32c3.rs")]
#[cfg_attr(esp32c2, path = "os_adapter_esp32c2.rs")]
#[cfg_attr(esp32c6, path = "os_adapter_esp32c6.rs")]
#[cfg_attr(esp32, path = "os_adapter_esp32.rs")]
#[cfg_attr(esp32s3, path = "os_adapter_esp32s3.rs")]
#[cfg_attr(esp32s2, path = "os_adapter_esp32s2.rs")]
pub(crate) mod os_adapter_chip_specific;

use core::cell::RefCell;

use critical_section::Mutex;
use enumset::EnumSet;
use log::{debug, trace};

use crate::{
    binary::include::*,
    common_adapter::RADIO_CLOCKS,
    compat::{
        common::{
            create_recursive_mutex, create_wifi_queue, lock_mutex, receive_queued, send_queued,
            thread_sem_get, unlock_mutex, StrBuf,
        },
        malloc::calloc,
        work_queue::queue_work,
    },
    hal::system::RadioClockController,
    hal::system::RadioPeripherals,
    memory_fence::memory_fence,
    timer::yield_task,
};

#[cfg(target_arch = "riscv32")]
use crate::compat::common::syslog;

use super::WifiEvent;

pub(crate) static mut WIFI_STATE: i32 = -1;

// useful for waiting for events - clear and wait for the event bit to be set again
pub(crate) static WIFI_EVENTS: Mutex<RefCell<EnumSet<WifiEvent>>> =
    Mutex::new(RefCell::new(enumset::enum_set!()));

pub fn is_connected() -> bool {
    unsafe { WIFI_STATE == wifi_event_t_WIFI_EVENT_STA_CONNECTED as i32 }
}

#[derive(Debug, Clone, Copy)]
pub enum WifiState {
    WifiReady,
    StaStart,
    StaStop,
    StaConnected,
    StaDisconnected,
    ApStart,
    ApStop,
    ApStaConnected,
    ApStaDisconnected,
    Invalid,
}

#[allow(non_upper_case_globals)]
pub fn get_wifi_state() -> WifiState {
    match unsafe { WIFI_STATE as u32 } {
        wifi_event_t_WIFI_EVENT_WIFI_READY => WifiState::WifiReady,
        wifi_event_t_WIFI_EVENT_STA_START => WifiState::StaStart,
        wifi_event_t_WIFI_EVENT_STA_STOP => WifiState::StaStop,
        wifi_event_t_WIFI_EVENT_STA_CONNECTED => WifiState::StaConnected,
        wifi_event_t_WIFI_EVENT_STA_DISCONNECTED => WifiState::StaDisconnected,
        wifi_event_t_WIFI_EVENT_AP_START => WifiState::ApStart,
        wifi_event_t_WIFI_EVENT_AP_STOP => WifiState::ApStop,
        wifi_event_t_WIFI_EVENT_AP_STACONNECTED => WifiState::ApStaConnected,
        wifi_event_t_WIFI_EVENT_AP_STADISCONNECTED => WifiState::ApStaDisconnected,
        _ => WifiState::Invalid,
    }
}

/****************************************************************************
 * Name: wifi_env_is_chip
 *
 * Description:
 *   Config chip environment
 *
 * Returned Value:
 *   True if on chip or false if on FPGA.
 *
 ****************************************************************************/
pub unsafe extern "C" fn env_is_chip() -> bool {
    true
}

/****************************************************************************
 * Name: wifi_set_intr
 *
 * Description:
 *   Do nothing
 *
 * Input Parameters:
 *     cpu_no      - The CPU which the interrupt number belongs.
 *     intr_source - The interrupt hardware source number.
 *     intr_num    - The interrupt number CPU.
 *     intr_prio   - The interrupt priority.
 *
 * Returned Value:
 *     None
 *
 ****************************************************************************/
pub unsafe extern "C" fn set_intr(cpu_no: i32, intr_source: u32, intr_num: u32, intr_prio: i32) {
    trace!(
        "set_intr {} {} {} {}",
        cpu_no,
        intr_source,
        intr_num,
        intr_prio
    );

    crate::wifi::os_adapter::os_adapter_chip_specific::set_intr(
        cpu_no,
        intr_source,
        intr_num,
        intr_prio,
    );
}

/****************************************************************************
 * Name: wifi_clear_intr
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn clear_intr(_intr_source: u32, _intr_num: u32) {
    // original code does nothing here
    debug!("clear_intr called {} {}", _intr_source, _intr_num);
}

pub static mut ISR_INTERRUPT_1: (
    *mut crate::binary::c_types::c_void,
    *mut crate::binary::c_types::c_void,
) = (core::ptr::null_mut(), core::ptr::null_mut());

/****************************************************************************
 * Name: esp_set_isr
 *
 * Description:
 *   Register interrupt function
 *
 * Input Parameters:
 *   n   - Interrupt ID
 *   f   - Interrupt function
 *   arg - Function private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn set_isr(
    n: i32,
    f: *mut crate::binary::c_types::c_void,
    arg: *mut crate::binary::c_types::c_void,
) {
    trace!("set_isr - interrupt {} function {:p} arg {:p}", n, f, arg);

    match n {
        0 => {
            ISR_INTERRUPT_1 = (f, arg);
        }
        1 => {
            ISR_INTERRUPT_1 = (f, arg);
        }
        _ => panic!("set_isr - unsupported interrupt number {}", n),
    }
}

/****************************************************************************
 * Name: esp32c3_ints_on
 *
 * Description:
 *   Enable Wi-Fi interrupt
 *
 * Input Parameters:
 *   mask - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn ints_on(mask: u32) {
    trace!("ints_on {:x}", mask);

    crate::wifi::os_adapter::os_adapter_chip_specific::chip_ints_on(mask);
}

/****************************************************************************
 * Name: esp32c3_ints_off
 *
 * Description:
 *   Disable Wi-Fi interrupt
 *
 * Input Parameters:
 *   mask - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn ints_off(mask: u32) {
    trace!("ints_off - not implemented - {:x}", mask);
}

/****************************************************************************
 * Name: wifi_is_from_isr
 *
 * Description:
 *   Check current is in interrupt
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if in interrupt or false if not
 *
 ****************************************************************************/
pub unsafe extern "C" fn is_from_isr() -> bool {
    true
}

/****************************************************************************
 * Name: esp_spin_lock_create
 *
 * Description:
 *   Create spin lock in SMP mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Spin lock data pointer
 *
 ****************************************************************************/
static mut FAKE_SPIN_LOCK: u8 = 1;
pub unsafe extern "C" fn spin_lock_create() -> *mut crate::binary::c_types::c_void {
    // original: return (void *)1;
    let ptr = &mut FAKE_SPIN_LOCK as *mut _ as *mut crate::binary::c_types::c_void;
    trace!("spin_lock_create {:p}", ptr);
    ptr
}

/****************************************************************************
 * Name: esp_spin_lock_delete
 *
 * Description:
 *   Delete spin lock
 *
 * Input Parameters:
 *   lock - Spin lock data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn spin_lock_delete(lock: *mut crate::binary::c_types::c_void) {
    // original: DEBUGASSERT((int)lock == 1);
    trace!("spin_lock_delete {:p}", lock);
}

/****************************************************************************
 * Name: esp_wifi_int_disable
 *
 * Description:
 *   Enter critical section by disabling interrupts and taking the spin lock
 *   if in SMP mode.
 *
 * Input Parameters:
 *   wifi_int_mux - Spin lock data pointer
 *
 * Returned Value:
 *   CPU PS value.
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_int_disable(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
) -> u32 {
    crate::wifi::os_adapter::os_adapter_chip_specific::wifi_int_disable(wifi_int_mux)
}

/****************************************************************************
 * Name: esp_wifi_int_restore
 *
 * Description:
 *   Exit from critical section by enabling interrupts and releasing the spin
 *   lock if in SMP mode.
 *
 * Input Parameters:
 *   wifi_int_mux - Spin lock data pointer
 *   tmp          - CPU PS value.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_int_restore(
    wifi_int_mux: *mut crate::binary::c_types::c_void,
    tmp: u32,
) {
    crate::wifi::os_adapter::os_adapter_chip_specific::wifi_int_restore(wifi_int_mux, tmp)
}

/****************************************************************************
 * Name: esp_task_yield_from_isr
 *
 * Description:
 *   Do nothing in NuttX
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn task_yield_from_isr() {
    // original: /* Do nothing */
    trace!("task_yield_from_isr");
    yield_task();
}

/****************************************************************************
 * Name: esp_thread_semphr_get
 *
 * Description:
 *   Get thread self's semaphore
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Semaphore data pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_thread_semphr_get() -> *mut crate::binary::c_types::c_void {
    thread_sem_get()
}

/****************************************************************************
 * Name: esp_mutex_create
 *
 * Description:
 *   Create mutex
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Mutex data pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn mutex_create() -> *mut crate::binary::c_types::c_void {
    todo!("mutex_create")
}

/****************************************************************************
 * Name: esp_recursive_mutex_create
 *
 * Description:
 *   Create recursive mutex
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Recursive mutex data pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn recursive_mutex_create() -> *mut crate::binary::c_types::c_void {
    create_recursive_mutex()
}

/****************************************************************************
 * Name: esp_mutex_delete
 *
 * Description:
 *   Delete mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn mutex_delete(_mutex: *mut crate::binary::c_types::c_void) {
    todo!("mutex_delete")
}

/****************************************************************************
 * Name: esp_mutex_lock
 *
 * Description:
 *   Lock mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn mutex_lock(mutex: *mut crate::binary::c_types::c_void) -> i32 {
    lock_mutex(mutex)
}

/****************************************************************************
 * Name: esp_mutex_unlock
 *
 * Description:
 *   Unlock mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn mutex_unlock(mutex: *mut crate::binary::c_types::c_void) -> i32 {
    unlock_mutex(mutex)
}

/****************************************************************************
 * Name: esp_queue_create
 *
 * Description:
 *   Create message queue
 *
 * Input Parameters:
 *   queue_len - queue message number
 *   item_size - message size
 *
 * Returned Value:
 *   Message queue data pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn queue_create(
    _queue_len: u32,
    _item_size: u32,
) -> *mut crate::binary::c_types::c_void {
    todo!("queue_create")
}

/****************************************************************************
 * Name: esp_queue_delete
 *
 * Description:
 *   Delete message queue
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn queue_delete(_queue: *mut crate::binary::c_types::c_void) {
    todo!("queue_delete")
}

/****************************************************************************
 * Name: esp_queue_send
 *
 * Description:
 *   Send message of low priority to queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn queue_send(
    queue: *mut crate::binary::c_types::c_void,
    item: *mut crate::binary::c_types::c_void,
    block_time_tick: u32,
) -> i32 {
    send_queued(queue, item, block_time_tick)
}

/****************************************************************************
 * Name: esp_queue_send_from_isr
 *
 * Description:
 *   Send message of low priority to queue in ISR within
 *   a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   hptw  - No mean
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn queue_send_from_isr(
    queue: *mut crate::binary::c_types::c_void,
    item: *mut crate::binary::c_types::c_void,
    _hptw: *mut crate::binary::c_types::c_void,
) -> i32 {
    trace!("queue_send_from_isr");
    *(_hptw as *mut u32) = 1;
    queue_send(queue, item, 1000)
}

/****************************************************************************
 * Name: esp_queue_send_to_back
 *
 * Description:
 *   Send message of low priority to queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn queue_send_to_back(
    _queue: *mut crate::binary::c_types::c_void,
    _item: *mut crate::binary::c_types::c_void,
    _block_time_tick: u32,
) -> i32 {
    todo!("queue_send_to_back")
}

/****************************************************************************
 * Name: esp_queue_send_from_to_front
 *
 * Description:
 *   Send message of high priority to queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn queue_send_to_front(
    _queue: *mut crate::binary::c_types::c_void,
    _item: *mut crate::binary::c_types::c_void,
    _block_time_tick: u32,
) -> i32 {
    todo!("queue_send_to_front")
}

/****************************************************************************
 * Name: esp_queue_recv
 *
 * Description:
 *   Receive message from queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn queue_recv(
    queue: *mut crate::binary::c_types::c_void,
    item: *mut crate::binary::c_types::c_void,
    block_time_tick: u32,
) -> i32 {
    receive_queued(queue, item, block_time_tick)
}

/****************************************************************************
 * Name: esp_queue_msg_waiting
 *
 * Description:
 *   Get message number in the message queue
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *
 * Returned Value:
 *   Message number
 *
 ****************************************************************************/
pub unsafe extern "C" fn queue_msg_waiting(_queue: *mut crate::binary::c_types::c_void) -> u32 {
    todo!("queue_msg_waiting")
}

/****************************************************************************
 * Name: esp_event_group_create
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn event_group_create() -> *mut crate::binary::c_types::c_void {
    todo!("event_group_create")
}

/****************************************************************************
 * Name: esp_event_group_delete
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn event_group_delete(_event: *mut crate::binary::c_types::c_void) {
    todo!("event_group_delete")
}

/****************************************************************************
 * Name: esp_event_group_set_bits
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn event_group_set_bits(
    _event: *mut crate::binary::c_types::c_void,
    _bits: u32,
) -> u32 {
    todo!("event_group_set_bits")
}

/****************************************************************************
 * Name: esp_event_group_clear_bits
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn event_group_clear_bits(
    _event: *mut crate::binary::c_types::c_void,
    _bits: u32,
) -> u32 {
    todo!("event_group_clear_bits")
}

/****************************************************************************
 * Name: esp_event_group_wait_bits
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn event_group_wait_bits(
    _event: *mut crate::binary::c_types::c_void,
    _bits_to_wait_for: u32,
    _clear_on_exit: crate::binary::c_types::c_int,
    _wait_for_all_bits: crate::binary::c_types::c_int,
    _block_time_tick: u32,
) -> u32 {
    todo!("event_group_wait_bits")
}

/****************************************************************************
 * Name: esp_task_create_pinned_to_core
 *
 * Description:
 *   Create task and bind it to target CPU, the task will run when it
 *   is created
 *
 * Input Parameters:
 *   entry       - Task entry
 *   name        - Task name
 *   stack_depth - Task stack size
 *   param       - Task private data
 *   prio        - Task priority
 *   task_handle - Task handle pointer which is used to pause, resume
 *                 and delete the task
 *   core_id     - CPU which the task runs in
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn task_create_pinned_to_core(
    task_func: *mut crate::binary::c_types::c_void,
    name: *const crate::binary::c_types::c_char,
    stack_depth: u32,
    param: *mut crate::binary::c_types::c_void,
    prio: u32,
    task_handle: *mut crate::binary::c_types::c_void,
    core_id: u32,
) -> i32 {
    trace!("task_create_pinned_to_core task_func {:p} name {} stack_depth {} param {:p} prio {}, task_handle {:p} core_id {}",
        task_func,
        StrBuf::from(name as *const u8).as_str_ref(),
        stack_depth,
        param,
        prio,
        task_handle,
        core_id
    );

    *(task_handle as *mut usize) = 0; // we will run it in task 0

    queue_work(
        task_func,
        name,
        stack_depth,
        param,
        prio,
        task_handle,
        core_id,
    );
    1
}

/****************************************************************************
 * Name: esp_task_create
 *
 * Description:
 *   Create task and the task will run when it is created
 *
 * Input Parameters:
 *   entry       - Task entry
 *   name        - Task name
 *   stack_depth - Task stack size
 *   param       - Task private data
 *   prio        - Task priority
 *   task_handle - Task handle pointer which is used to pause, resume
 *                 and delete the task
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn task_create(
    _task_func: *mut crate::binary::c_types::c_void,
    _name: *const crate::binary::c_types::c_char,
    _stack_depth: u32,
    _param: *mut crate::binary::c_types::c_void,
    _prio: u32,
    _task_handle: *mut crate::binary::c_types::c_void,
) -> i32 {
    todo!("task_create");
}

/****************************************************************************
 * Name: esp_task_delete
 *
 * Description:
 *   Delete the target task
 *
 * Input Parameters:
 *   task_handle - Task handle pointer which is used to pause, resume
 *                 and delete the task
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn task_delete(_task_handle: *mut crate::binary::c_types::c_void) {
    todo!("task_delete")
}

/****************************************************************************
 * Name: esp_task_delay
 *
 * Description:
 *   Current task wait for some ticks
 *
 * Input Parameters:
 *   tick - Waiting ticks
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn task_delay(tick: u32) {
    log::trace!("task_delay tick {}", tick);
    let timeout = crate::timer::get_systimer_count() + tick as u64;
    loop {
        if crate::timer::get_systimer_count() >= timeout {
            break;
        }
        yield_task();
    }
}

/****************************************************************************
 * Name: esp_task_ms_to_tick
 *
 * Description:
 *   Transform from millim seconds to system ticks
 *
 * Input Parameters:
 *   ms - Millim seconds
 *
 * Returned Value:
 *   System ticks
 *
 ****************************************************************************/
pub unsafe extern "C" fn task_ms_to_tick(ms: u32) -> i32 {
    trace!("task_ms_to_tick ms {}", ms);
    (ms as u64 * crate::timer::TICKS_PER_SECOND / 1000) as i32
}

/****************************************************************************
 * Name: esp_task_get_current_task
 *
 * Description:
 *   Transform from millim seconds to system ticks
 *
 * Input Parameters:
 *   ms - Millim seconds
 *
 * Returned Value:
 *   System ticks
 *
 ****************************************************************************/
pub unsafe extern "C" fn task_get_current_task() -> *mut crate::binary::c_types::c_void {
    let res = crate::preempt::preempt::current_task() as *mut crate::binary::c_types::c_void;
    trace!("task get current task - return {:p}", res);

    res
}

/****************************************************************************
 * Name: esp_task_get_max_priority
 *
 * Description:
 *   Get OS task maximum priority
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Task maximum priority
 *
 ****************************************************************************/
pub unsafe extern "C" fn task_get_max_priority() -> i32 {
    trace!("task_get_max_priority");
    255
}

/****************************************************************************
 * Name: esp_malloc
 *
 * Description:
 *   Allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn malloc(
    size: crate::binary::c_types::c_uint,
) -> *mut crate::binary::c_types::c_void {
    crate::compat::malloc::malloc(size as u32) as *mut crate::binary::c_types::c_void
}

/****************************************************************************
 * Name: esp_free
 *
 * Description:
 *   Free a block of memory
 *
 * Input Parameters:
 *   ptr - memory block
 *
 * Returned Value:
 *   No
 *
 ****************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn free(p: *mut crate::binary::c_types::c_void) {
    crate::compat::malloc::free(p as *const _ as *const u8);
}

/****************************************************************************
 * Name: esp_event_post
 *
 * Description:
 *   Active work queue and let the work to process the cached event
 *
 * Input Parameters:
 *   event_base      - Event set name
 *   event_id        - Event ID
 *   event_data      - Event private data
 *   event_data_size - Event data size
 *   ticks           - Waiting system ticks
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn event_post(
    event_base: *const crate::binary::c_types::c_char,
    event_id: i32,
    event_data: *mut crate::binary::c_types::c_void,
    event_data_size: size_t,
    ticks_to_wait: u32,
) -> i32 {
    trace!(
        "event_post {:?} {} {:p} {} {:?}",
        event_base,
        event_id,
        event_data,
        event_data_size,
        ticks_to_wait
    );
    use num_traits::FromPrimitive;

    let event = WifiEvent::from_i32(event_id).unwrap();
    log::trace!("EVENT: {:?}", event);
    critical_section::with(|cs| WIFI_EVENTS.borrow_ref_mut(cs).insert(event));

    let take_state = match event {
        WifiEvent::StaConnected
        | WifiEvent::StaDisconnected
        | WifiEvent::StaStart
        | WifiEvent::StaStop
        | WifiEvent::WifiReady
        | WifiEvent::ScanDone
        | WifiEvent::ApStart
        | WifiEvent::ApStop
        | WifiEvent::ApStaconnected
        | WifiEvent::ApStadisconnected => true,
        other => {
            log::info!("Unhandled event: {:?}", other);
            false
        }
    };

    if take_state {
        WIFI_STATE = event_id;
    }

    #[cfg(feature = "async")]
    event.waker().wake();

    #[cfg(feature = "embassy-net")]
    if matches!(
        event,
        WifiEvent::StaConnected
            | WifiEvent::StaDisconnected
            | WifiEvent::ApStart
            | WifiEvent::ApStop
    ) {
        crate::wifi::embassy::LINK_STATE.wake();
    }

    memory_fence();

    0
}

/****************************************************************************
 * Name: esp_get_free_heap_size
 *
 * Description:
 *   Get free heap size by byte
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Free heap size
 *
 ****************************************************************************/
pub unsafe extern "C" fn get_free_heap_size() -> u32 {
    todo!("get_free_heap_size")
}

/****************************************************************************
 * Name: esp_rand
 *
 * Description:
 *   Get random data of type uint32_t
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Random data
 *
 ****************************************************************************/
pub unsafe extern "C" fn rand() -> u32 {
    crate::common_adapter::random()
}

/****************************************************************************
 * Name: esp_dport_access_stall_other_cpu_start
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn dport_access_stall_other_cpu_start_wrap() {
    trace!("dport_access_stall_other_cpu_start_wrap")
}

/****************************************************************************
 * Name: esp_dport_access_stall_other_cpu_end
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn dport_access_stall_other_cpu_end_wrap() {
    trace!("dport_access_stall_other_cpu_end_wrap")
}
/****************************************************************************
 * Name: wifi_apb80m_request
 *
 * Description:
 *   Take Wi-Fi lock in auto-sleep
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_apb80m_request() {
    trace!("wifi_apb80m_request - no-op")
}
/****************************************************************************
 * Name: wifi_apb80m_release
 *
 * Description:
 *   Release Wi-Fi lock in auto-sleep
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_apb80m_release() {
    trace!("wifi_apb80m_release - no-op")
}

/****************************************************************************
 * Name: esp32c3_phy_disable
 *
 * Description:
 *   Deinitialize PHY hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn phy_disable() {
    trace!("phy_disable");

    crate::common_adapter::chip_specific::phy_disable();
}

/****************************************************************************
 * Name: esp32c3_phy_enable
 *
 * Description:
 *   Initialize PHY hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn phy_enable() {
    // quite some code needed here
    trace!("phy_enable");

    crate::common_adapter::chip_specific::phy_enable();
}

/****************************************************************************
 * Name: wifi_phy_update_country_info
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn phy_update_country_info(
    country: *const crate::binary::c_types::c_char,
) -> crate::binary::c_types::c_int {
    // not implemented in original code
    trace!("phy_update_country_info {}", *country as u8 as char);
    -1
}

/****************************************************************************
 * Name: wifi_reset_mac
 *
 * Description:
 *   Reset Wi-Fi hardware MAC
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_reset_mac() {
    trace!("wifi_reset_mac");
    RADIO_CLOCKS.as_mut().unwrap().reset_mac();
}

/****************************************************************************
 * Name: wifi_clock_enable
 *
 * Description:
 *   Enable Wi-Fi clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_clock_enable() {
    trace!("wifi_clock_enable");
    RADIO_CLOCKS
        .as_mut()
        .unwrap()
        .enable(RadioPeripherals::Wifi);
}

/****************************************************************************
 * Name: wifi_clock_disable
 *
 * Description:
 *   Disable Wi-Fi clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_clock_disable() {
    trace!("wifi_clock_disable");
    RADIO_CLOCKS
        .as_mut()
        .unwrap()
        .disable(RadioPeripherals::Wifi);
}

/****************************************************************************
 * Name: wifi_rtc_enable_iso
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_rtc_enable_iso() {
    todo!("wifi_rtc_enable_iso")
}

/****************************************************************************
 * Name: wifi_rtc_disable_iso
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_rtc_disable_iso() {
    todo!("wifi_rtc_disable_iso")
}

/****************************************************************************
 * Name: esp_timer_get_time
 *
 * Description:
 *   Get time in microseconds since boot.
 *
 * Returned Value:
 *   System time in micros
 *
 ****************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn esp_timer_get_time() -> i64 {
    trace!("esp_timer_get_time");
    (crate::timer::get_systimer_count() / (crate::timer::TICKS_PER_SECOND / 1_000_000)) as i64
}

/****************************************************************************
 * Name: esp_nvs_set_i8
 *
 * Description:
 *   Save data of type int8_t into file system
 *
 * Input Parameters:
 *   handle - NVS handle
 *   key    - Data index
 *   value  - Stored data
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_set_i8(
    _handle: u32,
    _key: *const crate::binary::c_types::c_char,
    _value: i8,
) -> crate::binary::c_types::c_int {
    debug!("nvs_set_i8");
    -1
}

/****************************************************************************
 * Name: esp_nvs_get_i8
 *
 * Description:
 *   Read data of type int8_t from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *   out_value - Read buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_get_i8(
    _handle: u32,
    _key: *const crate::binary::c_types::c_char,
    _out_value: *mut i8,
) -> crate::binary::c_types::c_int {
    todo!("nvs_get_i8")
}

/****************************************************************************
 * Name: esp_nvs_set_u8
 *
 * Description:
 *   Save data of type uint8_t into file system
 *
 * Input Parameters:
 *   handle - NVS handle
 *   key    - Data index
 *   value  - Stored data
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_set_u8(
    _handle: u32,
    _key: *const crate::binary::c_types::c_char,
    _value: u8,
) -> crate::binary::c_types::c_int {
    todo!("nvs_set_u8")
}

/****************************************************************************
 * Name: esp_nvs_get_u8
 *
 * Description:
 *   Read data of type uint8_t from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *   out_value - Read buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_get_u8(
    _handle: u32,
    _key: *const crate::binary::c_types::c_char,
    _out_value: *mut u8,
) -> crate::binary::c_types::c_int {
    todo!("nvs_get_u8")
}

/****************************************************************************
 * Name: esp_nvs_set_u16
 *
 * Description:
 *   Save data of type uint16_t into file system
 *
 * Input Parameters:
 *   handle - NVS handle
 *   key    - Data index
 *   value  - Stored data
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_set_u16(
    _handle: u32,
    _key: *const crate::binary::c_types::c_char,
    _value: u16,
) -> crate::binary::c_types::c_int {
    todo!("nvs_set_u16")
}

/****************************************************************************
 * Name: esp_nvs_get_u16
 *
 * Description:
 *   Read data of type uint16_t from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *   out_value - Read buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_get_u16(
    _handle: u32,
    _key: *const crate::binary::c_types::c_char,
    _out_value: *mut u16,
) -> crate::binary::c_types::c_int {
    todo!("nvs_get_u16")
}

/****************************************************************************
 * Name: esp_nvs_open
 *
 * Description:
 *   Create a file system storage data object
 *
 * Input Parameters:
 *   name       - Storage index
 *   open_mode  - Storage mode
 *   out_handle - Storage handle
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_open(
    _name: *const crate::binary::c_types::c_char,
    _open_mode: u32,
    _out_handle: *mut u32,
) -> crate::binary::c_types::c_int {
    todo!("nvs_open")
}

/****************************************************************************
 * Name: esp_nvs_close
 *
 * Description:
 *   Close storage data object and free resource
 *
 * Input Parameters:
 *   handle - NVS handle
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_close(_handle: u32) {
    todo!("nvs_close")
}

/****************************************************************************
 * Name: esp_nvs_commit
 *
 * Description:
 *   This function has no practical effect
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_commit(_handle: u32) -> crate::binary::c_types::c_int {
    todo!("nvs_commit")
}

/****************************************************************************
 * Name: esp_nvs_set_blob
 *
 * Description:
 *   Save a block of data into file system
 *
 * Input Parameters:
 *   handle - NVS handle
 *   key    - Data index
 *   value  - Stored buffer pointer
 *   length - Buffer length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_set_blob(
    _handle: u32,
    _key: *const crate::binary::c_types::c_char,
    _value: *const crate::binary::c_types::c_void,
    _length: size_t,
) -> crate::binary::c_types::c_int {
    todo!("nvs_set_blob")
}

/****************************************************************************
 * Name: esp_nvs_get_blob
 *
 * Description:
 *   Read a block of data from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *   out_value - Read buffer pointer
 *   length    - Buffer length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_get_blob(
    _handle: u32,
    _key: *const crate::binary::c_types::c_char,
    _out_value: *mut crate::binary::c_types::c_void,
    _length: *mut size_t,
) -> crate::binary::c_types::c_int {
    todo!("nvs_get_blob")
}

/****************************************************************************
 * Name: esp_nvs_erase_key
 *
 * Description:
 *   Read a block of data from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn nvs_erase_key(
    _handle: u32,
    _key: *const crate::binary::c_types::c_char,
) -> crate::binary::c_types::c_int {
    todo!("nvs_erase_key")
}

/****************************************************************************
 * Name: esp_get_random
 *
 * Description:
 *   Fill random data int given buffer of given length
 *
 * Input Parameters:
 *   buf - buffer pointer
 *   len - buffer length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn get_random(buf: *mut u8, len: size_t) -> crate::binary::c_types::c_int {
    for i in 0..len as usize {
        buf.add(i)
            .write_volatile((crate::wifi::rand() & 0xff) as u8)
    }

    0
}

/****************************************************************************
 * Name: esp_get_time
 *
 * Description:
 *   Get std C time
 *
 * Input Parameters:
 *   t - buffer to store time of type timeval
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn get_time(
    _t: *mut crate::binary::c_types::c_void,
) -> crate::binary::c_types::c_int {
    todo!("get_time")
}

/****************************************************************************
 * Name: esp_log_write
 *
 * Description:
 *   Output log with by format string and its arguments
 *
 * Input Parameters:
 *   level  - log level, no mean here
 *   tag    - log TAG, no mean here
 *   format - format string
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
#[allow(unreachable_code)]
pub unsafe extern "C" fn log_write(
    _level: u32,
    _tag: *const crate::binary::c_types::c_char,
    _format: *const crate::binary::c_types::c_char,
    _args: ...
) {
    #[cfg(not(feature = "wifi-logs"))]
    return;

    #[cfg(target_arch = "riscv32")]
    syslog(_level, _format as *const u8, _args);
}

/****************************************************************************
 * Name: esp_log_writev
 *
 * Description:
 *   Output log with by format string and its arguments
 *
 * Input Parameters:
 *   level  - log level, no mean here
 *   tag    - log TAG, no mean here
 *   format - format string
 *   args   - arguments list
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
#[allow(improper_ctypes_definitions)]
pub unsafe extern "C" fn log_writev(
    _level: u32,
    _tag: *const crate::binary::c_types::c_char,
    _format: *const crate::binary::c_types::c_char,
    _args: va_list,
) {
    #[cfg(not(feature = "wifi-logs"))]
    return;

    #[cfg(target_arch = "xtensa")]
    #[allow(unreachable_code)]
    {
        let s = StrBuf::from(_format as *const u8);
        log::info!("{}", s.as_str_ref());
    }

    #[cfg(target_arch = "riscv32")]
    #[allow(unreachable_code)]
    {
        let _args = core::mem::transmute(_args);
        syslog(_level, _format as *const u8, _args);
    }
}

/****************************************************************************
 * Name: esp_log_timestamp
 *
 * Description:
 *   Get system time by millim second
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   System time
 *
 ****************************************************************************/
pub unsafe extern "C" fn log_timestamp() -> u32 {
    (crate::timer::get_systimer_count() / crate::timer::TICKS_PER_SECOND / 1_000) as u32
}

/****************************************************************************
 * Name: esp_malloc_internal
 *
 * Description:
 *   Drivers allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn malloc_internal(size: size_t) -> *mut crate::binary::c_types::c_void {
    crate::compat::malloc::malloc(size as u32) as *mut crate::binary::c_types::c_void
}

/****************************************************************************
 * Name: esp_realloc_internal
 *
 * Description:
 *   Drivers allocate a block of memory by old memory block
 *
 * Input Parameters:
 *   ptr  - old memory pointer
 *   size - memory size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn realloc_internal(
    _ptr: *mut crate::binary::c_types::c_void,
    _size: size_t,
) -> *mut crate::binary::c_types::c_void {
    todo!("realloc_internal")
}

/****************************************************************************
 * Name: esp_calloc_internal
 *
 * Description:
 *   Drivers allocate some continuous blocks of memory
 *
 * Input Parameters:
 *   n    - memory block number
 *   size - memory block size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn calloc_internal(
    n: size_t,
    size: size_t,
) -> *mut crate::binary::c_types::c_void {
    calloc(n as u32, size as u32) as *mut crate::binary::c_types::c_void
}

/****************************************************************************
 * Name: esp_zalloc_internal
 *
 * Description:
 *   Drivers allocate a block of memory and clear it with 0
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn zalloc_internal(size: size_t) -> *mut crate::binary::c_types::c_void {
    calloc(size as u32, 1u32) as *mut crate::binary::c_types::c_void
}

/****************************************************************************
 * Name: esp_wifi_malloc
 *
 * Description:
 *   Applications allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_malloc(size: size_t) -> *mut crate::binary::c_types::c_void {
    malloc(size as u32)
}

/****************************************************************************
 * Name: esp_wifi_realloc
 *
 * Description:
 *   Applications allocate a block of memory by old memory block
 *
 * Input Parameters:
 *   ptr  - old memory pointer
 *   size - memory size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_realloc(
    _ptr: *mut crate::binary::c_types::c_void,
    _size: size_t,
) -> *mut crate::binary::c_types::c_void {
    todo!("wifi_realloc")
}

/****************************************************************************
 * Name: esp_wifi_calloc
 *
 * Description:
 *   Applications allocate some continuous blocks of memory
 *
 * Input Parameters:
 *   n    - memory block number
 *   size - memory block size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_calloc(
    n: size_t,
    size: size_t,
) -> *mut crate::binary::c_types::c_void {
    trace!("wifi_calloc {} {}", n, size);
    calloc(n as u32, size as u32) as *mut crate::binary::c_types::c_void
}

/****************************************************************************
 * Name: esp_wifi_zalloc
 *
 * Description:
 *   Applications allocate a block of memory and clear it with 0
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_zalloc(size: size_t) -> *mut crate::binary::c_types::c_void {
    wifi_calloc(size, 1)
}

/****************************************************************************
 * Name: esp_wifi_create_queue
 *
 * Description:
 *   Create Wi-Fi static message queue
 *
 * Input Parameters:
 *   queue_len - queue message number
 *   item_size - message size
 *
 * Returned Value:
 *   Wi-Fi static message queue data pointer
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_create_queue(
    queue_len: crate::binary::c_types::c_int,
    item_size: crate::binary::c_types::c_int,
) -> *mut crate::binary::c_types::c_void {
    create_wifi_queue(queue_len, item_size)
}

/****************************************************************************
 * Name: esp_wifi_delete_queue
 *
 * Description:
 *   Delete Wi-Fi static message queue
 *
 * Input Parameters:
 *   queue - Wi-Fi static message queue data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn wifi_delete_queue(queue: *mut crate::binary::c_types::c_void) {
    trace!(
        "wifi_delete_queue {:p} - not implemented - doing nothing",
        queue
    );
}

/****************************************************************************
 * Name: wifi_coex_deinit
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn coex_deinit() {
    log::warn!("coex_deinit - not implemented");
}

/****************************************************************************
 * Name: wifi_coex_enable
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn coex_enable() -> crate::binary::c_types::c_int {
    log::debug!("coex_enable");

    #[cfg(coex)]
    return crate::binary::include::coex_enable();

    #[cfg(not(coex))]
    0
}

/****************************************************************************
 * Name: wifi_coex_disable
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn coex_disable() {
    log::debug!("coex_disable");

    #[cfg(coex)]
    crate::binary::include::coex_disable();
}

/****************************************************************************
 * Name: esp_coex_status_get
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
pub unsafe extern "C" fn coex_status_get() -> u32 {
    log::debug!("coex_status_get");

    #[cfg(coex)]
    return crate::binary::include::coex_status_get();

    #[cfg(not(coex))]
    0
}

/****************************************************************************
 * Name: esp_coex_condition_set
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_condition_set(type_: u32, dissatisfy: bool) {
    log::debug!("coex_condition_set");

    #[cfg(coex)]
    crate::binary::include::coex_condition_set(type_, dissatisfy);
}

/****************************************************************************
 * Name: esp_coex_wifi_request
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_wifi_request(
    event: u32,
    latency: u32,
    duration: u32,
) -> crate::binary::c_types::c_int {
    log::debug!("coex_wifi_request");

    #[cfg(coex)]
    return crate::binary::include::coex_wifi_request(event, latency, duration);

    #[cfg(not(coex))]
    0
}

/****************************************************************************
 * Name: esp_coex_wifi_release
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_wifi_release(event: u32) -> crate::binary::c_types::c_int {
    log::debug!("coex_wifi_release");

    #[cfg(coex)]
    return crate::binary::include::coex_wifi_release(event);

    #[cfg(not(coex))]
    0
}

/****************************************************************************
 * Name: wifi_coex_wifi_set_channel
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_wifi_channel_set(
    primary: u8,
    secondary: u8,
) -> crate::binary::c_types::c_int {
    log::debug!("coex_wifi_channel_set");

    #[cfg(coex)]
    return crate::binary::include::coex_wifi_channel_set(primary, secondary);

    #[cfg(not(coex))]
    0
}

/****************************************************************************
 * Name: wifi_coex_get_event_duration
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_event_duration_get(
    event: u32,
    duration: *mut u32,
) -> crate::binary::c_types::c_int {
    log::debug!("coex_event_duration_get");

    #[cfg(coex)]
    return crate::binary::include::coex_event_duration_get(event, duration);

    #[cfg(not(coex))]
    0
}

/****************************************************************************
 * Name: wifi_coex_get_pti
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[cfg(any(esp32c3, esp32c2, esp32c6, esp32s3))]
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_pti_get(event: u32, pti: *mut u8) -> crate::binary::c_types::c_int {
    log::debug!("coex_pti_get");

    #[cfg(coex)]
    return crate::binary::include::coex_pti_get(event, pti);

    #[cfg(not(coex))]
    0
}

#[cfg(any(esp32, esp32s2))]
pub unsafe extern "C" fn coex_pti_get(event: u32, pti: *mut u8) -> crate::binary::c_types::c_int {
    log::debug!("coex_pti_get {} {:p}", event, pti);
    0
}

/****************************************************************************
 * Name: wifi_coex_clear_schm_status_bit
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_status_bit_clear(type_: u32, status: u32) {
    log::debug!("coex_schm_status_bit_clear");

    #[cfg(coex)]
    crate::binary::include::coex_schm_status_bit_clear(type_, status);
}

/****************************************************************************
 * Name: wifi_coex_set_schm_status_bit
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_status_bit_set(type_: u32, status: u32) {
    log::debug!("coex_schm_status_bit_set");

    #[cfg(coex)]
    crate::binary::include::coex_schm_status_bit_set(type_, status);
}

/****************************************************************************
 * Name: wifi_coex_set_schm_interval
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_interval_set(interval: u32) -> crate::binary::c_types::c_int {
    log::debug!("coex_schm_interval_set");

    #[cfg(coex)]
    return crate::binary::include::coex_schm_interval_set(interval);

    #[cfg(not(coex))]
    0
}

/****************************************************************************
 * Name: wifi_coex_get_schm_interval
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_interval_get() -> u32 {
    log::debug!("coex_schm_interval_get");

    #[cfg(coex)]
    return crate::binary::include::coex_schm_interval_get();

    #[cfg(not(coex))]
    0
}

/****************************************************************************
 * Name: wifi_coex_get_schm_curr_period
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_curr_period_get() -> u8 {
    log::debug!("coex_schm_curr_period_get");

    #[cfg(coex)]
    return crate::binary::include::coex_schm_curr_period_get();

    #[cfg(not(coex))]
    0
}

/****************************************************************************
 * Name: wifi_coex_get_schm_curr_phase
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_curr_phase_get() -> *mut crate::binary::c_types::c_void {
    log::debug!("coex_schm_curr_phase_get");

    #[cfg(coex)]
    return crate::binary::include::coex_schm_curr_phase_get();

    #[cfg(not(coex))]
    return 0 as *mut crate::binary::c_types::c_void;
}

pub unsafe extern "C" fn coex_schm_process_restart_wrapper() -> esp_wifi_sys::c_types::c_int {
    log::debug!("coex_schm_process_restart_wrapper");

    #[cfg(not(coex))]
    return 0;

    #[cfg(coex)]
    crate::binary::include::coex_schm_process_restart()
}

#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_register_cb_wrapper(
    arg1: esp_wifi_sys::c_types::c_int,
    cb: ::core::option::Option<
        unsafe extern "C" fn(arg1: esp_wifi_sys::c_types::c_int) -> esp_wifi_sys::c_types::c_int,
    >,
) -> esp_wifi_sys::c_types::c_int {
    log::debug!("coex_schm_register_cb_wrapper {} {:?}", arg1, cb);

    #[cfg(not(coex))]
    return 0;

    #[cfg(coex)]
    crate::binary::include::coex_schm_register_callback(
        arg1 as u32,
        (cb.unwrap()) as *const esp_wifi_sys::c_types::c_void as *mut esp_wifi_sys::c_types::c_void,
    )
}

/****************************************************************************
 * Name: esp_clk_slowclk_cal_get_wrapper
 *
 * Description:
 *   Get the calibration value of RTC slow clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The calibration value obtained using rtc_clk_cal
 *
 ****************************************************************************/
pub unsafe extern "C" fn slowclk_cal_get() -> u32 {
    trace!("slowclk_cal_get");

    // TODO not hardcode this

    #[cfg(esp32s2)]
    return 44462;

    #[cfg(esp32s3)]
    return 44462;

    #[cfg(esp32c3)]
    return 28639;

    #[cfg(esp32c2)]
    return 28639;

    #[cfg(esp32c6)]
    return 0;

    #[cfg(esp32)]
    return 28639;
}
