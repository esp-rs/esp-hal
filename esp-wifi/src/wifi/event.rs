use core::cell::RefCell;

use critical_section::Mutex;

mod sealed {
    pub trait Sealed {}
}
/// The type of handlers of events.
pub type Handler<T> = dyn FnMut(&T) + Sync + Send;
// fn default_handler<'a, T>(_: &'a T) {}
/// Data for a wifi event which can be handled by an event handler.
pub trait WifiEventData: sealed::Sealed + Sized + 'static {
    /// Get the static reference to the handler for this event.
    fn get_handler() -> &'static Mutex<RefCell<Option<&'static mut Handler<Self>>>>;

    /// Get the handler for this event, replacing it with the default handler.
    fn take_handler() -> Option<&'static mut Handler<Self>> {
        critical_section::with(|cs| Self::get_handler().borrow_ref_mut(cs).take())
    }
    /// Set the handler for this event.
    fn set_handler(f: &'static mut Handler<Self>) {
        critical_section::with(|cs| Self::get_handler().borrow_ref_mut(cs).replace(f));
    }
    /// Atomic combination of `set_handler` and `get_handler`. Use this to add a
    /// new handler while preserving the previous.
    fn update_handler(
        f: impl FnOnce(Option<&'static mut Handler<Self>>) -> &'static mut Handler<Self>,
    ) {
        critical_section::with(|cs| {
            let handler = Self::take_handler();
            Self::get_handler().borrow_ref_mut(cs).replace(f(handler))
        });
    }
}

macro_rules! impl_wifi_event_data {
    ($ty:path) => {
        pub use $ty;
        impl sealed::Sealed for $ty {}
        impl WifiEventData for $ty {
            fn get_handler() -> &'static Mutex<RefCell<Option<&'static mut Handler<Self>>>> {
                static HANDLE: Mutex<RefCell<Option<&'static mut Handler<$ty>>>> =
                    Mutex::new(RefCell::new(None));
                &HANDLE
            }
        }
    };
}

impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_probe_req_rx_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_staconnected_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_stadisconnected_t);
