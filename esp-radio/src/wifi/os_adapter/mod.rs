#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32c2, path = "esp32c2.rs")]
#[cfg_attr(esp32c3, path = "esp32c3.rs")]
#[cfg_attr(esp32c5, path = "esp32c5.rs")]
#[cfg_attr(esp32c6, path = "esp32c6.rs")]
#[cfg_attr(esp32h2, path = "esp32h2.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
pub(crate) mod os_adapter_chip_specific;

use core::ptr::NonNull;

use allocator_api2::boxed::Box;
use esp_phy::PhyController;
use esp_sync::RawMutex;

use super::event::WifiEvent;
use crate::{
    compat::{
        common::{str_from_c, thread_sem_get},
        malloc::{InternalMemory, calloc_internal},
    },
    hal::{clock::ModemClockController, peripherals::WIFI},
    sys::c_types::*,
    time::{blob_ticks_to_micros, millis_to_blob_ticks},
};

static WIFI_LOCK: RawMutex = RawMutex::new();

pub(crate) fn shutdown_wifi_isr() {
    os_adapter_chip_specific::shutdown_wifi_isr();
}

/// **************************************************************************
/// Name: wifi_env_is_chip
///
/// Description:
///   Config chip environment
///
/// Returned Value:
///   True if on chip or false if on FPGA.
///
/// *************************************************************************
pub unsafe extern "C" fn env_is_chip() -> bool {
    true
}

/// **************************************************************************
/// Name: wifi_set_intr
///
/// Description:
///   Do nothing
///
/// Input Parameters:
///     cpu_no      - The CPU which the interrupt number belongs.
///     intr_source - The interrupt hardware source number.
///     intr_num    - The interrupt number CPU.
///     intr_prio   - The interrupt priority.
///
/// Returned Value:
///     None
///
/// *************************************************************************
pub unsafe extern "C" fn set_intr(cpu_no: i32, intr_source: u32, intr_num: u32, intr_prio: i32) {
    trace!(
        "set_intr {} {} {} {}",
        cpu_no, intr_source, intr_num, intr_prio
    );
    unsafe {
        crate::wifi::os_adapter::os_adapter_chip_specific::set_intr(
            cpu_no,
            intr_source,
            intr_num,
            intr_prio,
        );
    }
}

/// **************************************************************************
/// Name: wifi_clear_intr
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn clear_intr(intr_source: u32, intr_num: u32) {
    // original code does nothing here
    trace!("clear_intr called {} {}", intr_source, intr_num);
}

/// **************************************************************************
/// Name: esp32c3_ints_on
///
/// Description:
///   Enable Wi-Fi interrupt
///
/// Input Parameters:
///   mask - No mean
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn ints_on(mask: u32) {
    trace!("chip_ints_on {:x}", mask);

    crate::wifi::os_adapter::os_adapter_chip_specific::chip_ints_on(mask);
}

/// **************************************************************************
/// Name: esp32c3_ints_off
///
/// Description:
///   Disable Wi-Fi interrupt
///
/// Input Parameters:
///   mask - No mean
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn ints_off(mask: u32) {
    trace!("chip_ints_off {:x}", mask);

    crate::wifi::os_adapter::os_adapter_chip_specific::chip_ints_off(mask);
}

/// **************************************************************************
/// Name: wifi_is_from_isr
///
/// Description:
///   Check current is in interrupt
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   true if in interrupt or false if not
///
/// *************************************************************************
pub unsafe extern "C" fn is_from_isr() -> bool {
    true
}

/// **************************************************************************
/// Name: esp_spin_lock_create
///
/// Description:
///   Create spin lock in SMP mode
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   Spin lock data pointer
///
/// *************************************************************************
pub unsafe extern "C" fn spin_lock_create() -> *mut c_void {
    let ptr = crate::compat::semaphore::sem_create(1, 1);

    trace!("spin_lock_create {:?}", ptr);
    ptr as *mut c_void
}

/// **************************************************************************
/// Name: esp_spin_lock_delete
///
/// Description:
///   Delete spin lock
///
/// Input Parameters:
///   lock - Spin lock data pointer
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn spin_lock_delete(lock: *mut c_void) {
    trace!("spin_lock_delete {:?}", lock);

    crate::compat::semaphore::sem_delete(lock);
}

/// **************************************************************************
/// Name: esp_wifi_int_disable
///
/// Description:
///   Enter critical section by disabling interrupts and taking the spin lock
///   if in SMP mode.
///
/// Input Parameters:
///   wifi_int_mux - Spin lock data pointer
///
/// Returned Value:
///   CPU PS value.
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_int_disable(_wifi_int_mux: *mut c_void) -> u32 {
    trace!("wifi_int_disable");
    // TODO: can we use wifi_int_mux?
    let token = unsafe { WIFI_LOCK.acquire() };
    unsafe { core::mem::transmute::<esp_sync::RestoreState, u32>(token) }
}

/// **************************************************************************
/// Name: esp_wifi_int_restore
///
/// Description:
///   Exit from critical section by enabling interrupts and releasing the spin
///   lock if in SMP mode.
///
/// Input Parameters:
///   wifi_int_mux - Spin lock data pointer
///   tmp          - CPU PS value.
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_int_restore(_wifi_int_mux: *mut c_void, tmp: u32) {
    trace!("wifi_int_restore");
    let token = unsafe { core::mem::transmute::<u32, esp_sync::RestoreState>(tmp) };
    unsafe { WIFI_LOCK.release(token) }
}

/// **************************************************************************
/// Name: esp_task_yield_from_isr
///
/// Description:
///   Do nothing in NuttX
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn task_yield_from_isr() {
    trace!("task_yield_from_isr");
    crate::preempt::yield_task_from_isr();
}

/// **************************************************************************
/// Name: esp_thread_semphr_get
///
/// Description:
///   Get thread self's semaphore
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   Semaphore data pointer
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_thread_semphr_get() -> *mut c_void {
    thread_sem_get()
}

/// **************************************************************************
/// Name: esp_mutex_create
///
/// Description:
///   Create mutex
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   Mutex data pointer
///
/// *************************************************************************
pub unsafe extern "C" fn mutex_create() -> *mut c_void {
    trace!("mutex_create");
    crate::compat::mutex::mutex_create(false)
}

/// **************************************************************************
/// Name: esp_recursive_mutex_create
///
/// Description:
///   Create recursive mutex
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   Recursive mutex data pointer
///
/// *************************************************************************
pub unsafe extern "C" fn recursive_mutex_create() -> *mut c_void {
    trace!("recursive_mutex_create");
    crate::compat::mutex::mutex_create(true)
}

/// **************************************************************************
/// Name: esp_mutex_delete
///
/// Description:
///   Delete mutex
///
/// Input Parameters:
///   mutex_data - mutex data pointer
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn mutex_delete(mutex: *mut c_void) {
    crate::compat::mutex::mutex_delete(mutex);
}

/// **************************************************************************
/// Name: esp_mutex_lock
///
/// Description:
///   Lock mutex
///
/// Input Parameters:
///   mutex_data - mutex data pointer
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
pub unsafe extern "C" fn mutex_lock(mutex: *mut c_void) -> i32 {
    crate::compat::mutex::mutex_lock(mutex)
}

/// **************************************************************************
/// Name: esp_mutex_unlock
///
/// Description:
///   Unlock mutex
///
/// Input Parameters:
///   mutex_data - mutex data pointer
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
pub unsafe extern "C" fn mutex_unlock(mutex: *mut c_void) -> i32 {
    crate::compat::mutex::mutex_unlock(mutex)
}

/// **************************************************************************
/// Name: esp_event_group_create
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn event_group_create() -> *mut c_void {
    todo!("event_group_create")
}

/// **************************************************************************
/// Name: esp_event_group_delete
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn event_group_delete(_event: *mut c_void) {
    todo!("event_group_delete")
}

/// **************************************************************************
/// Name: esp_event_group_set_bits
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn event_group_set_bits(_event: *mut c_void, _bits: u32) -> u32 {
    todo!("event_group_set_bits")
}

/// **************************************************************************
/// Name: esp_event_group_clear_bits
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn event_group_clear_bits(_event: *mut c_void, _bits: u32) -> u32 {
    todo!("event_group_clear_bits")
}

/// **************************************************************************
/// Name: esp_event_group_wait_bits
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn event_group_wait_bits(
    _event: *mut c_void,
    _bits_to_wait_for: u32,
    _clear_on_exit: c_int,
    _wait_for_all_bits: c_int,
    _block_time_tick: u32,
) -> u32 {
    todo!("event_group_wait_bits")
}

fn common_task_create(
    task_func: *mut c_void,
    name: *const c_char,
    stack_depth: u32,
    param: *mut c_void,
    prio: u32,
    task_handle: *mut c_void,
    core_id: Option<u32>,
) -> i32 {
    let task_name = unsafe { str_from_c(name as _) };
    trace!(
        "task_create task_func {:?} name {} stack_depth {} param {:?} prio {}, task_handle {:?} core_id {:?}",
        task_func, task_name, stack_depth, param, prio, task_handle, core_id
    );

    unsafe {
        let task_func = core::mem::transmute::<
            *mut c_void,
            extern "C" fn(*mut crate::sys::c_types::c_void),
        >(task_func);

        let task = crate::preempt::task_create(
            task_name,
            task_func,
            param,
            prio,
            core_id,
            stack_depth as usize,
        );
        *(task_handle as *mut usize) = task.as_ptr() as usize;

        1
    }
}

/// **************************************************************************
/// Name: esp_task_create_pinned_to_core
///
/// Description:
///   Create task and bind it to target CPU, the task will run when it
///   is created
///
/// Input Parameters:
///   entry       - Task entry
///   name        - Task name
///   stack_depth - Task stack size
///   param       - Task private data
///   prio        - Task priority
///   task_handle - Task handle pointer which is used to pause, resume
///                 and delete the task
///   core_id     - CPU which the task runs in
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
pub unsafe extern "C" fn task_create_pinned_to_core(
    task_func: *mut c_void,
    name: *const c_char,
    stack_depth: u32,
    param: *mut c_void,
    prio: u32,
    task_handle: *mut c_void,
    core_id: u32,
) -> i32 {
    common_task_create(
        task_func,
        name,
        stack_depth,
        param,
        prio,
        task_handle,
        if core_id < 2 { Some(core_id) } else { None },
    )
}

/// **************************************************************************
/// Name: esp_task_create
///
/// Description:
///   Create task and the task will run when it is created
///
/// Input Parameters:
///   entry       - Task entry
///   name        - Task name
///   stack_depth - Task stack size
///   param       - Task private data
///   prio        - Task priority
///   task_handle - Task handle pointer which is used to pause, resume
///                 and delete the task
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
pub unsafe extern "C" fn task_create(
    task_func: *mut c_void,
    name: *const c_char,
    stack_depth: u32,
    param: *mut c_void,
    prio: u32,
    task_handle: *mut c_void,
) -> i32 {
    common_task_create(task_func, name, stack_depth, param, prio, task_handle, None)
}

/// **************************************************************************
/// Name: esp_task_delete
///
/// Description:
///   Delete the target task
///
/// Input Parameters:
///   task_handle - Task handle pointer which is used to pause, resume
///                 and delete the task
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn task_delete(task_handle: *mut c_void) {
    trace!("task delete called for {:?}", task_handle);

    unsafe {
        crate::preempt::schedule_task_deletion(NonNull::new(task_handle.cast::<()>()));
    }
}

/// **************************************************************************
/// Name: esp_task_delay
///
/// Description:
///   Current task wait for some ticks
///
/// Input Parameters:
///   tick - Waiting ticks
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn task_delay(tick: u32) {
    trace!("task_delay tick {}", tick);
    crate::preempt::usleep(blob_ticks_to_micros(tick))
}

/// **************************************************************************
/// Name: esp_task_ms_to_tick
///
/// Description:
///   Transform from milliseconds to system ticks
///
/// Input Parameters:
///   ms - Milliseconds
///
/// Returned Value:
///   System ticks
///
/// *************************************************************************
pub unsafe extern "C" fn task_ms_to_tick(ms: u32) -> i32 {
    trace!("task_ms_to_tick ms {}", ms);
    millis_to_blob_ticks(ms) as i32
}

/// **************************************************************************
/// Name: esp_task_get_current_task
///
/// Description:
///   Retrieves the current task
///
/// Returned Value:
///   A pointer to the current task
///
/// *************************************************************************
pub unsafe extern "C" fn task_get_current_task() -> *mut c_void {
    let res = crate::preempt::current_task().cast::<c_void>().as_ptr();
    trace!("task get current task - return {:?}", res);

    res
}

/// **************************************************************************
/// Name: esp_task_get_max_priority
///
/// Description:
///   Get OS task maximum priority
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   Task maximum priority
///
/// *************************************************************************
pub unsafe extern "C" fn task_get_max_priority() -> i32 {
    trace!("task_get_max_priority");
    crate::preempt::max_task_priority() as i32
}

/// **************************************************************************
/// Name: esp_malloc
///
/// Description:
///   Allocate a block of memory
///
/// Input Parameters:
///   size - memory size
///
/// Returned Value:
///   Memory pointer
///
/// *************************************************************************
pub unsafe extern "C" fn malloc(size: usize) -> *mut c_void {
    unsafe { crate::compat::malloc::malloc(size).cast() }
}

/// **************************************************************************
/// Name: esp_free
///
/// Description:
///   Free a block of memory
///
/// Input Parameters:
///   ptr - memory block
///
/// Returned Value:
///   No
///
/// *************************************************************************
pub unsafe extern "C" fn free(p: *mut c_void) {
    unsafe {
        crate::compat::malloc::free(p.cast());
    }
}

/// **************************************************************************
/// Name: esp_event_post
///
/// Description:
///   Active work queue and let the work to process the cached event
///
/// Input Parameters:
///   event_base      - Event set name
///   event_id        - Event ID
///   event_data      - Event private data
///   event_data_size - Event data size
///   ticks           - Waiting system ticks
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn event_post(
    event_base: *const c_char,
    event_id: i32,
    event_data: *mut c_void,
    event_data_size: usize,
    ticks_to_wait: u32,
) -> i32 {
    trace!(
        "event_post {:?} {} {:?} {} {:?}",
        event_base, event_id, event_data, event_data_size, ticks_to_wait
    );
    use num_traits::FromPrimitive;

    if let Some(event) = super::event::WifiEvent::from_i32(event_id) {
        trace!("EVENT: {:?}", event);

        super::state::update_state(event);

        if let Some(payload) = super::event::EventInfo::from_wifi_event_raw(event, event_data)
            && let Ok(publisher) = super::event::EVENT_CHANNEL.publisher()
            && publisher.try_publish(payload).is_err()
        {
            warn!(
                "Lost event - consider increasing the capacity of the internal wifi event channel."
            );
        }

        match event {
            WifiEvent::StationConnected | WifiEvent::StationDisconnected => {
                crate::wifi::embassy::STA_LINK_STATE_WAKER.wake();
            }

            WifiEvent::AccessPointStart | WifiEvent::AccessPointStop => {
                crate::wifi::embassy::AP_LINK_STATE_WAKER.wake();
            }

            _ => {}
        }
    } else {
        warn!("Got unmapped event: {}", event_id);
    }

    0
}

/// **************************************************************************
/// Name: esp_get_free_heap_size
///
/// Description:
///   Get free heap size by byte
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   Free heap size
///
/// *************************************************************************
pub unsafe extern "C" fn get_free_heap_size() -> u32 {
    unsafe { crate::compat::malloc::get_free_internal_heap_size() as u32 }
}

/// **************************************************************************
/// Name: esp_rand
///
/// Description:
///   Get random data of type uint32_t
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   Random data
///
/// *************************************************************************
pub unsafe extern "C" fn rand() -> u32 {
    unsafe { crate::common_adapter::random() }
}

/// **************************************************************************
/// Name: esp_dport_access_stall_other_cpu_start
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn dport_access_stall_other_cpu_start_wrap() {
    trace!("dport_access_stall_other_cpu_start_wrap")
}

/// **************************************************************************
/// Name: esp_dport_access_stall_other_cpu_end
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn dport_access_stall_other_cpu_end_wrap() {
    trace!("dport_access_stall_other_cpu_end_wrap")
}
/// **************************************************************************
/// Name: wifi_apb80m_request
///
/// Description:
///   Take Wi-Fi lock in auto-sleep
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_apb80m_request() {
    trace!("wifi_apb80m_request - no-op")
}
/// **************************************************************************
/// Name: wifi_apb80m_release
///
/// Description:
///   Release Wi-Fi lock in auto-sleep
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_apb80m_release() {
    trace!("wifi_apb80m_release - no-op")
}

/// **************************************************************************
/// Name: esp32c3_phy_disable
///
/// Description:
///   Deinitialize PHY hardware
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn phy_disable() {
    trace!("phy_disable");
    unsafe { WIFI::steal() }.decrease_phy_ref_count();
}

/// **************************************************************************
/// Name: esp32c3_phy_enable
///
/// Description:
///   Initialize PHY hardware
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn phy_enable() {
    // quite some code needed here
    trace!("phy_enable");
    core::mem::forget(unsafe { WIFI::steal() }.enable_phy());
}

/// **************************************************************************
/// Name: wifi_phy_update_country_info
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[allow(clippy::unnecessary_cast)]
pub unsafe extern "C" fn phy_update_country_info(country: *const c_char) -> c_int {
    unsafe {
        // not implemented in original code
        trace!("phy_update_country_info {}", str_from_c(country.cast()));
        -1
    }
}

/// **************************************************************************
/// Name: wifi_reset_mac
///
/// Description:
///   Reset Wi-Fi hardware MAC
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_reset_mac() {
    trace!("wifi_reset_mac");
    // stealing WIFI is safe, since it is passed into the initialization function of the BLE
    // controller.
    unsafe { WIFI::steal() }.reset_wifi_mac();
}

/// **************************************************************************
/// Name: wifi_clock_enable
///
/// Description:
///   Enable Wi-Fi clock
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_clock_enable() {
    trace!("wifi_clock_enable");
    // stealing WIFI is safe, since it is passed into the initialization function of the BLE
    // controller.
    unsafe { WIFI::steal() }.enable_modem_clock(true);
}

/// **************************************************************************
/// Name: wifi_clock_disable
///
/// Description:
///   Disable Wi-Fi clock
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_clock_disable() {
    trace!("wifi_clock_disable");
    // stealing WIFI is safe, since it is passed into the initialization function of the BLE
    // controller.
    unsafe { WIFI::steal() }.enable_modem_clock(false);
}

/// **************************************************************************
/// Name: wifi_rtc_enable_iso
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_rtc_enable_iso() {
    todo!("wifi_rtc_enable_iso")
}

/// **************************************************************************
/// Name: wifi_rtc_disable_iso
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_rtc_disable_iso() {
    todo!("wifi_rtc_disable_iso")
}

/// **************************************************************************
/// Name: esp_nvs_set_i8
///
/// Description:
///   Save data of type int8_t into file system
///
/// Input Parameters:
///   handle - NVS handle
///   key    - Data index
///   value  - Stored data
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_set_i8(_handle: u32, _key: *const c_char, _value: i8) -> c_int {
    debug!("nvs_set_i8");
    -1
}

/// **************************************************************************
/// Name: esp_nvs_get_i8
///
/// Description:
///   Read data of type int8_t from file system
///
/// Input Parameters:
///   handle    - NVS handle
///   key       - Data index
///   out_value - Read buffer pointer
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_get_i8(
    _handle: u32,
    _key: *const c_char,
    _out_value: *mut i8,
) -> c_int {
    todo!("nvs_get_i8")
}

/// **************************************************************************
/// Name: esp_nvs_set_u8
///
/// Description:
///   Save data of type uint8_t into file system
///
/// Input Parameters:
///   handle - NVS handle
///   key    - Data index
///   value  - Stored data
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_set_u8(_handle: u32, _key: *const c_char, _value: u8) -> c_int {
    todo!("nvs_set_u8")
}

/// **************************************************************************
/// Name: esp_nvs_get_u8
///
/// Description:
///   Read data of type uint8_t from file system
///
/// Input Parameters:
///   handle    - NVS handle
///   key       - Data index
///   out_value - Read buffer pointer
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_get_u8(
    _handle: u32,
    _key: *const c_char,
    _out_value: *mut u8,
) -> c_int {
    todo!("nvs_get_u8")
}

/// **************************************************************************
/// Name: esp_nvs_set_u16
///
/// Description:
///   Save data of type uint16_t into file system
///
/// Input Parameters:
///   handle - NVS handle
///   key    - Data index
///   value  - Stored data
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_set_u16(_handle: u32, _key: *const c_char, _value: u16) -> c_int {
    todo!("nvs_set_u16")
}

/// **************************************************************************
/// Name: esp_nvs_get_u16
///
/// Description:
///   Read data of type uint16_t from file system
///
/// Input Parameters:
///   handle    - NVS handle
///   key       - Data index
///   out_value - Read buffer pointer
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_get_u16(
    _handle: u32,
    _key: *const c_char,
    _out_value: *mut u16,
) -> c_int {
    todo!("nvs_get_u16")
}

/// **************************************************************************
/// Name: esp_nvs_open
///
/// Description:
///   Create a file system storage data object
///
/// Input Parameters:
///   name       - Storage index
///   open_mode  - Storage mode
///   out_handle - Storage handle
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_open(
    _name: *const c_char,
    _open_mode: u32,
    _out_handle: *mut u32,
) -> c_int {
    todo!("nvs_open")
}

/// **************************************************************************
/// Name: esp_nvs_close
///
/// Description:
///   Close storage data object and free resource
///
/// Input Parameters:
///   handle - NVS handle
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_close(_handle: u32) {
    todo!("nvs_close")
}

/// **************************************************************************
/// Name: esp_nvs_commit
///
/// Description:
///   This function has no practical effect
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_commit(_handle: u32) -> c_int {
    todo!("nvs_commit")
}

/// **************************************************************************
/// Name: esp_nvs_set_blob
///
/// Description:
///   Save a block of data into file system
///
/// Input Parameters:
///   handle - NVS handle
///   key    - Data index
///   value  - Stored buffer pointer
///   length - Buffer length
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_set_blob(
    _handle: u32,
    _key: *const c_char,
    _value: *const c_void,
    _length: usize,
) -> c_int {
    todo!("nvs_set_blob")
}

/// **************************************************************************
/// Name: esp_nvs_get_blob
///
/// Description:
///   Read a block of data from file system
///
/// Input Parameters:
///   handle    - NVS handle
///   key       - Data index
///   out_value - Read buffer pointer
///   length    - Buffer length
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_get_blob(
    _handle: u32,
    _key: *const c_char,
    _out_value: *mut c_void,
    _length: *mut usize,
) -> c_int {
    todo!("nvs_get_blob")
}

/// **************************************************************************
/// Name: esp_nvs_erase_key
///
/// Description:
///   Read a block of data from file system
///
/// Input Parameters:
///   handle    - NVS handle
///   key       - Data index
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn nvs_erase_key(_handle: u32, _key: *const c_char) -> c_int {
    todo!("nvs_erase_key")
}

/// **************************************************************************
/// Name: esp_get_random
///
/// Description:
///   Fill random data int given buffer of given length
///
/// Input Parameters:
///   buf - buffer pointer
///   len - buffer length
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn get_random(buf: *mut u8, len: usize) -> c_int {
    trace!("get_random");
    unsafe {
        crate::common_adapter::__esp_radio_esp_fill_random(buf, len as u32);
    }
    0
}

/// **************************************************************************
/// Name: esp_get_time
///
/// Description:
///   Get std C time
///
/// Input Parameters:
///   t - buffer to store time of type timeval
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn get_time(_t: *mut c_void) -> c_int {
    todo!("get_time")
}

/// **************************************************************************
/// Name: esp_log_write
///
/// Description:
///   Output log with by format string and its arguments
///
/// Input Parameters:
///   level  - log level, no mean here
///   tag    - log TAG, no mean here
///   format - format string
///
/// Returned Value:
///   None
///
/// *************************************************************************
#[cfg(feature = "print-logs-from-driver")]
pub unsafe extern "C" fn log_write(
    level: u32,
    _tag: *const c_char,
    format: *const c_char,
    args: ...
) {
    unsafe {
        crate::sys::log::syslog(level, format as _, args);
    }
}

/// **************************************************************************
/// Name: esp_log_writev
///
/// Description:
///   Output log with by format string and its arguments
///
/// Input Parameters:
///   level  - log level, no mean here
///   tag    - log TAG, no mean here
///   format - format string
///   args   - arguments list
///
/// Returned Value:
///   None
///
/// *************************************************************************
#[cfg(feature = "print-logs-from-driver")]
#[allow(improper_ctypes_definitions)]
pub unsafe extern "C" fn log_writev(
    level: u32,
    _tag: *const c_char,
    format: *const c_char,
    args: crate::sys::include::va_list,
) {
    unsafe {
        crate::sys::log::syslog(
            level,
            format as _,
            core::mem::transmute::<crate::sys::include::va_list, core::ffi::VaListImpl<'_>>(args),
        );
    }
}

/// **************************************************************************
/// Name: esp_log_timestamp
///
/// Description:
///   Get system time by millim second
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   System time
///
/// *************************************************************************
pub unsafe extern "C" fn log_timestamp() -> u32 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis() as u32
}

/// **************************************************************************
/// Name: esp_malloc_internal
///
/// Description:
///   Drivers allocate a block of memory
///
/// Input Parameters:
///   size - memory size
///
/// Returned Value:
///   Memory pointer
///
/// *************************************************************************
pub unsafe extern "C" fn malloc_internal(size: usize) -> *mut c_void {
    unsafe { crate::compat::malloc::malloc_internal(size).cast() }
}

/// **************************************************************************
/// Name: esp_realloc_internal
///
/// Description:
///   Drivers allocate a block of memory by old memory block
///
/// Input Parameters:
///   ptr  - old memory pointer
///   size - memory size
///
/// Returned Value:
///   New memory pointer
///
/// *************************************************************************
pub unsafe extern "C" fn realloc_internal(ptr: *mut c_void, size: usize) -> *mut c_void {
    unsafe { crate::compat::malloc::realloc_internal(ptr.cast(), size).cast() }
}

/// **************************************************************************
/// Name: esp_calloc_internal
///
/// Description:
///   Drivers allocate some continuous blocks of memory
///
/// Input Parameters:
///   n    - memory block number
///   size - memory block size
///
/// Returned Value:
///   New memory pointer
///
/// *************************************************************************
pub unsafe extern "C" fn calloc_internal_wrapper(n: usize, size: usize) -> *mut c_void {
    unsafe { calloc_internal(n as u32, size) as *mut c_void }
}

/// **************************************************************************
/// Name: esp_zalloc_internal
///
/// Description:
///   Drivers allocate a block of memory and clear it with 0
///
/// Input Parameters:
///   size - memory size
///
/// Returned Value:
///   New memory pointer
///
/// *************************************************************************
pub unsafe extern "C" fn zalloc_internal(size: usize) -> *mut c_void {
    unsafe { calloc_internal(size as u32, 1usize) as *mut c_void }
}

/// **************************************************************************
/// Name: esp_wifi_malloc
///
/// Description:
///   Applications allocate a block of memory
///
/// Input Parameters:
///   size - memory size
///
/// Returned Value:
///   Memory pointer
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_malloc(size: usize) -> *mut c_void {
    unsafe { malloc_internal(size) }
}

/// **************************************************************************
/// Name: esp_wifi_realloc
///
/// Description:
///   Applications allocate a block of memory by old memory block
///
/// Input Parameters:
///   ptr  - old memory pointer
///   size - memory size
///
/// Returned Value:
///   New memory pointer
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_realloc(ptr: *mut c_void, size: usize) -> *mut c_void {
    unsafe { realloc_internal(ptr, size) }
}

/// **************************************************************************
/// Name: esp_wifi_calloc
///
/// Description:
///   Applications allocate some continuous blocks of memory
///
/// Input Parameters:
///   n    - memory block number
///   size - memory block size
///
/// Returned Value:
///   New memory pointer
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_calloc(n: usize, size: usize) -> *mut c_void {
    trace!("wifi_calloc {} {}", n, size);
    unsafe { calloc_internal(n as u32, size) as *mut c_void }
}

/// **************************************************************************
/// Name: esp_wifi_zalloc
///
/// Description:
///   Applications allocate a block of memory and clear it with 0
///
/// Input Parameters:
///   size - memory size
///
/// Returned Value:
///   New memory pointer
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_zalloc(size: usize) -> *mut c_void {
    unsafe { wifi_calloc(size, 1) }
}

/// **************************************************************************
/// Name: esp_wifi_create_queue
///
/// Description:
///   Create Wi-Fi static message queue
///
/// Input Parameters:
///   queue_len - queue message number
///   item_size - message size
///
/// Returned Value:
///   Wi-Fi static message queue data pointer
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_create_queue(queue_len: c_int, item_size: c_int) -> *mut c_void {
    let queue = crate::compat::queue::queue_create(queue_len, item_size);

    let queue_ptr: *mut *mut c_void = Box::leak(Box::new_in(queue, InternalMemory));

    queue_ptr.cast()
}

/// **************************************************************************
/// Name: esp_wifi_delete_queue
///
/// Description:
///   Delete Wi-Fi static message queue
///
/// Input Parameters:
///   queue - Wi-Fi static message queue data pointer
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn wifi_delete_queue(queue: *mut c_void) {
    let queue_ptr: *mut *mut c_void = queue.cast();

    let boxed = unsafe { Box::from_raw_in(queue_ptr, InternalMemory) };

    crate::compat::queue::queue_delete(*boxed)
}

/// **************************************************************************
/// Name: wifi_coex_deinit
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn coex_deinit() {
    trace!("coex_deinit");

    #[cfg(coex)]
    unsafe {
        crate::sys::include::coex_deinit()
    };
}

/// **************************************************************************
/// Name: wifi_coex_enable
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn coex_enable() -> c_int {
    trace!("coex_enable");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_enable() };

    #[cfg(not(coex))]
    0
}

/// **************************************************************************
/// Name: wifi_coex_disable
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn coex_disable() {
    trace!("coex_disable");

    #[cfg(coex)]
    unsafe {
        crate::sys::include::coex_disable()
    };
}

/// **************************************************************************
/// Name: esp_coex_status_get
///
/// Description:
///   Don't support
///
/// *************************************************************************
pub unsafe extern "C" fn coex_status_get() -> u32 {
    trace!("coex_status_get");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_status_get(0b1) }; // COEX_STATUS_GET_WIFI_BITMAP

    #[cfg(not(coex))]
    0
}

/// **************************************************************************
/// Name: esp_coex_wifi_request
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[cfg_attr(not(coex), allow(unused_variables))]
pub unsafe extern "C" fn coex_wifi_request(event: u32, latency: u32, duration: u32) -> c_int {
    trace!("coex_wifi_request");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_wifi_request(event, latency, duration) };

    #[cfg(not(coex))]
    0
}

/// **************************************************************************
/// Name: esp_coex_wifi_release
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[cfg_attr(not(coex), allow(unused_variables))]
pub unsafe extern "C" fn coex_wifi_release(event: u32) -> c_int {
    trace!("coex_wifi_release");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_wifi_release(event) };

    #[cfg(not(coex))]
    0
}

/// **************************************************************************
/// Name: wifi_coex_wifi_set_channel
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[cfg_attr(not(coex), allow(unused_variables))]
pub unsafe extern "C" fn coex_wifi_channel_set(primary: u8, secondary: u8) -> c_int {
    trace!("coex_wifi_channel_set");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_wifi_channel_set(primary, secondary) };

    #[cfg(not(coex))]
    0
}

/// **************************************************************************
/// Name: wifi_coex_get_event_duration
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[cfg_attr(not(coex), allow(unused_variables))]
pub unsafe extern "C" fn coex_event_duration_get(event: u32, duration: *mut u32) -> c_int {
    trace!("coex_event_duration_get");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_event_duration_get(event, duration) };

    #[cfg(not(coex))]
    0
}

/// **************************************************************************
/// Name: wifi_coex_get_pti
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[cfg(any(esp32c3, esp32c2, esp32c5, esp32c6, esp32s3))]
#[cfg_attr(not(coex), allow(unused_variables))]
pub unsafe extern "C" fn coex_pti_get(event: u32, pti: *mut u8) -> c_int {
    trace!("coex_pti_get");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_pti_get(event, pti) };

    #[cfg(not(coex))]
    0
}

#[cfg(any(esp32, esp32s2))]
pub unsafe extern "C" fn coex_pti_get(event: u32, pti: *mut u8) -> c_int {
    trace!("coex_pti_get {} {:?}", event, pti);
    0
}

/// **************************************************************************
/// Name: wifi_coex_clear_schm_status_bit
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_status_bit_clear(type_: u32, status: u32) {
    trace!("coex_schm_status_bit_clear");

    #[cfg(coex)]
    unsafe {
        crate::sys::include::coex_schm_status_bit_clear(type_, status)
    };
}

/// **************************************************************************
/// Name: wifi_coex_set_schm_status_bit
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_status_bit_set(type_: u32, status: u32) {
    trace!("coex_schm_status_bit_set");

    #[cfg(coex)]
    unsafe {
        crate::sys::include::coex_schm_status_bit_set(type_, status)
    };
}

/// **************************************************************************
/// Name: wifi_coex_set_schm_interval
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_interval_set(interval: u32) -> c_int {
    trace!("coex_schm_interval_set");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_schm_interval_set(interval) };

    #[cfg(not(coex))]
    0
}

/// **************************************************************************
/// Name: wifi_coex_get_schm_interval
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_interval_get() -> u32 {
    trace!("coex_schm_interval_get");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_schm_interval_get() };

    #[cfg(not(coex))]
    0
}

/// **************************************************************************
/// Name: wifi_coex_get_schm_curr_period
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_curr_period_get() -> u8 {
    trace!("coex_schm_curr_period_get");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_schm_curr_period_get() };

    #[cfg(not(coex))]
    0
}

/// **************************************************************************
/// Name: wifi_coex_get_schm_curr_phase
///
/// Description:
///   Don't support
///
/// *************************************************************************
#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_curr_phase_get() -> *mut c_void {
    trace!("coex_schm_curr_phase_get");

    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_schm_curr_phase_get() };

    #[cfg(not(coex))]
    return core::ptr::null_mut();
}

pub unsafe extern "C" fn coex_schm_process_restart_wrapper() -> crate::sys::c_types::c_int {
    trace!("coex_schm_process_restart_wrapper");

    #[cfg(not(coex))]
    return 0;

    #[cfg(coex)]
    unsafe {
        crate::sys::include::coex_schm_process_restart()
    }
}

#[allow(unused_variables)]
pub unsafe extern "C" fn coex_schm_register_cb_wrapper(
    arg1: crate::sys::c_types::c_int,
    cb: ::core::option::Option<
        unsafe extern "C" fn(arg1: crate::sys::c_types::c_int) -> crate::sys::c_types::c_int,
    >,
) -> crate::sys::c_types::c_int {
    trace!("coex_schm_register_cb_wrapper {} {:?}", arg1, cb);

    #[cfg(not(coex))]
    return 0;

    #[cfg(coex)]
    unsafe {
        crate::sys::include::coex_schm_register_callback(
            arg1 as u32,
            unwrap!(cb) as *const crate::sys::c_types::c_void as *mut crate::sys::c_types::c_void,
        )
    }
}

pub unsafe extern "C" fn coex_schm_flexible_period_set(period: u8) -> i32 {
    trace!("coex_schm_flexible_period_set {}", period);

    #[cfg(coex)]
    unsafe {
        unsafe extern "C" {
            fn coex_schm_flexible_period_set(period: u8) -> i32;
        }

        coex_schm_flexible_period_set(period)
    }

    #[cfg(not(coex))]
    0
}

pub unsafe extern "C" fn coex_schm_flexible_period_get() -> u8 {
    trace!("coex_schm_flexible_period_get");

    #[cfg(coex)]
    unsafe {
        unsafe extern "C" {
            fn coex_schm_flexible_period_get() -> u8;
        }

        coex_schm_flexible_period_get()
    }

    #[cfg(not(coex))]
    0
}

pub unsafe extern "C" fn coex_register_start_cb(
    _cb: Option<unsafe extern "C" fn() -> crate::sys::c_types::c_int>,
) -> crate::sys::c_types::c_int {
    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_register_start_cb(_cb) };

    #[cfg(not(coex))]
    0
}

pub unsafe extern "C" fn coex_schm_get_phase_by_idx(
    _phase_idx: i32,
) -> *mut crate::sys::c_types::c_void {
    #[cfg(coex)]
    return unsafe { crate::sys::include::coex_schm_get_phase_by_idx(_phase_idx) };

    #[cfg(not(coex))]
    core::ptr::null_mut()
}

/// **************************************************************************
/// Name: esp_clk_slowclk_cal_get_wrapper
///
/// Description:
///   Get the calibration value of RTC slow clock
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   The calibration value obtained using rtc_clk_cal
///
/// *************************************************************************
#[allow(unused)]
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

    #[cfg(any(esp32c6, esp32h2, esp32c5))]
    return 0;

    #[cfg(esp32)]
    return 28639;
}
