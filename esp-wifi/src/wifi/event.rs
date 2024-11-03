use core::cell::RefCell;

use critical_section::Mutex;

pub(crate) mod sealed {
    use super::*;

    pub trait Sealed {
        /// Get the static reference to the handler for this event.
        fn get_handler() -> &'static Mutex<RefCell<Option<&'static mut Handler<Self>>>>;
    }
}
/// The type of handlers of events.
pub type Handler<T> = dyn FnMut(&T) + Sync + Send;

fn default_handler<Event>() -> &'static mut Handler<Event> {
    fn nop<'a, T>(_: &'a T) {}
    fn size_of_val<T>(_: &mut T) -> usize {
        core::mem::size_of::<T>()
    }

    // Be paranoid about coercing from a zero sized pointee
    let a = &mut nop::<Event>;
    debug_assert_eq!(0, size_of_val(a));
    let a: &mut Handler<Event> = a;
    // SAFETY: We're extending the lifetime of a borrow of a technically local
    // variable, but the pointee is a ZST (and the vtable is static).  If exclusive
    // references to function item types had the same magic as empty arrays and
    // slices, this wouldn't be necessary. See <https://users.rust-lang.org/t/safety-of-lifetime-extending-fnmut-created-from-fn/120626/4>
    // and <https://github.com/rust-lang/rust/issues/103821>
    unsafe { &mut *(a as *mut Handler<Event>) }
}

/// Data for a wifi event which can be handled by an event handler.
pub trait WifiEventData: sealed::Sealed + Sized + 'static {
    /// Get the handler for this event, replacing it with the default handler.
    fn take_handler() -> &'static mut Handler<Self> {
        critical_section::with(|cs| {
            Self::get_handler()
                .borrow_ref_mut(cs)
                .take()
                .unwrap_or(default_handler::<Self>())
        })
    }
    /// Set the handler for this event, returning the old handler.
    fn replace_handler(f: &'static mut Handler<Self>) -> &'static mut Handler<Self> {
        critical_section::with(|cs| {
            Self::get_handler()
                .borrow_ref_mut(cs)
                .replace(f)
                .unwrap_or(default_handler::<Self>())
        })
    }
    /// Atomic combination of `take_handler` and `replace_handler`. Use this to
    /// add a new handler while preserving the previous.
    fn update_handler(f: impl FnOnce(&'static mut Handler<Self>) -> &'static mut Handler<Self>) {
        critical_section::with(|_cs| {
            let handler = Self::take_handler();
            Self::replace_handler(f(handler));
        });
    }
}

macro_rules! impl_wifi_event_data {
    ($ty:path) => {
        pub use $ty;
        impl sealed::Sealed for $ty {
            fn get_handler() -> &'static Mutex<RefCell<Option<&'static mut Handler<Self>>>> {
                static HANDLE: Mutex<RefCell<Option<&'static mut Handler<$ty>>>> =
                    Mutex::new(RefCell::new(None));
                &HANDLE
            }
        }
        impl WifiEventData for $ty {}
    };
}

impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_probe_req_rx_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_staconnected_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_stadisconnected_t);
