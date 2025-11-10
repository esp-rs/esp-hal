#[cfg(feature = "alloc")]
use esp_rom_sys::SYSCALL_TABLE;

#[cfg(feature = "alloc")]
extern "C" fn _getreent() -> *mut esp_rom_sys::_reent {
    use allocator_api2::boxed::Box;
    crate::task::with_current_task(|task| {
        task.thread_local
            .reent
            .get_or_insert_with(|| unsafe {
                // Safety: _reent is a C-type containing integers and pointers, it is safe to
                // zero-initialize it.
                Box::new_zeroed().assume_init()
            })
            .as_mut() as *mut _
    })
}

pub fn setup_syscalls() {
    #[cfg(feature = "alloc")]
    unsafe {
        SYSCALL_TABLE.__getreent = Some(_getreent);
    }
}
