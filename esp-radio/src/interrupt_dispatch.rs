use core::{ffi::c_void, sync::atomic::Ordering};

use portable_atomic::AtomicPtr;

pub(crate) struct Handler {
    f: AtomicPtr<c_void>,
    arg: AtomicPtr<c_void>,
}

impl Handler {
    pub const fn new() -> Self {
        Self {
            f: AtomicPtr::new(core::ptr::null_mut()),
            arg: AtomicPtr::new(core::ptr::null_mut()),
        }
    }

    pub fn set(&self, f: *const c_void, arg: *const c_void) {
        self.arg.store(arg.cast_mut(), Ordering::Relaxed);
        self.f.store(f.cast_mut(), Ordering::Release);
    }

    #[crate::hal::ram]
    pub fn dispatch(&self) {
        let f = self.f.load(Ordering::Acquire);
        if !f.is_null() {
            let func = unsafe {
                core::mem::transmute::<*const c_void, unsafe extern "C" fn(*mut c_void)>(f)
            };
            let arg = self.arg.load(Ordering::Relaxed);
            trace!("calling {:x} with {:x}", f as usize, arg as usize);
            unsafe { func(arg) };
            trace!("{:x} done", f as usize);
        }
    }
}
