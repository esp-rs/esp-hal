//! WiFi event handling.

use alloc::boxed::Box;

pub(crate) mod internal;

type EventHandler<'a, T> = dyn FnMut(&T) + Send + Sync + 'a;
type BoxedEventHandler<'a, T> = Box<EventHandler<'a, T>>;

/// A trait that identifies a listenable WiFi event.
pub trait Event: Sized {
    /// The read-only data view provided when the event occurs.
    /// For events with no data, this will be `()`.
    type Data<'a>;

    /// Update the handler for this event, chaining it after any existing handler.
    fn update_handler(handler: impl FnMut(&Self::Data<'_>) + Send + Sync + 'static);

    /// Replace the handler for this event, returning the old one.
    fn replace_handler(
        handler: impl FnMut(&Self::Data<'_>) + Send + Sync + 'static,
    ) -> BoxedEventHandler<'static, Self::Data<'static>>;

    /// Take the handler for this event, leaving the default (no-op) handler in its place.
    fn take_handler() -> BoxedEventHandler<'static, Self::Data<'static>>;
}

// Helper macro to generate the public API for each event.
macro_rules! define_event {
    // Version for events WITH data
    (
        $(#[$outer:meta])*
        $event_token:ident,
        $event_view:ident,
        $internal_struct:ty,
        { $( $(#[$inner:meta])* pub fn $method:ident(&self) -> $ret:ty; )* }
    ) => {
        $(#[$outer])*
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $event_token;

        /// A read-only view of the data for an [`
        #[doc = stringify!($event_token)]
        /// `] event.
        ///
        /// This struct is provided as an argument to the handler closure and
        /// cannot be stored, as its lifetime is tied to the handler's execution.
        #[derive(Copy, Clone)]
        pub struct $event_view<'a>(&'a $internal_struct);

        impl<'a> $event_view<'a> {
            $(
                $(#[$inner])*
                ///
                pub fn $method(&self) -> $ret {
                    self.0.$method()
                }
            )*
        }

        impl Event for $event_token {
            type Data<'a> = $event_view<'a>;

            fn update_handler(mut handler: impl FnMut(&Self::Data<'_>) + Send + Sync + 'static) {
                use internal::EventExt as _;
                <$internal_struct>::update_handler(move |internal_data| {
                    handler(&$event_view(internal_data))
                });
            }

            fn replace_handler(
                mut handler: impl FnMut(&Self::Data<'_>) + Send + Sync + 'static,
            ) -> Box<dyn FnMut(&Self::Data<'_>) + Send + Sync> {
                use internal::EventExt as _;
                let mut prev_handler = <$internal_struct>::replace_handler(move |internal_data| {
                    handler(&$event_view(internal_data))
                });

                Box::new(move |data| prev_handler(&data.0))
            }

            fn take_handler() -> Box<dyn FnMut(&Self::Data<'_>) + Send + Sync> {
                use internal::EventExt as _;
                let mut prev_handler = <$internal_struct>::take_handler();
                Box::new(move |data| prev_handler(&data.0))
            }
        }
    };
    // Version for events WITHOUT data
    (
        $(#[$outer:meta])*
        $event_token:ident
    ) => {
        $(#[$outer])*
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $event_token;

        impl Event for $event_token {
            type Data<'a> = ();

            fn update_handler(mut handler: impl FnMut(&()) + Send + Sync + 'static) {
                use internal::EventExt as _;
                <internal::$event_token>::update_handler(move |_| handler(&()));
            }

            fn replace_handler(
                mut handler: impl FnMut(&()) + Send + Sync + 'static,
            ) -> Box<dyn FnMut(&Self::Data<'_>) + Send + Sync> {
                use internal::EventExt as _;
                let mut prev_handler = <internal::$event_token>::replace_handler(move |_| handler(&()));
                Box::new(move |_| prev_handler(&internal::$event_token))
            }

            fn take_handler() -> Box<dyn FnMut(&Self::Data<'_>) + Send + Sync> {
                use internal::EventExt as _;
                let mut prev_handler = <internal::$event_token>::take_handler();
                Box::new(move |_| prev_handler(&internal::$event_token))
            }
        }
    }
}

// Define all public events here.
define_event!(
    /// Event fired when a station connects to the AP.
    ApStaconnected,
    ApStaconnectedEvent,
    internal::ApStaconnected,
    {
        /// Get the MAC address of the connected station.
        pub fn mac(&self) -> &[u8; 6];
        /// Get the Association ID (AID) of the connected station.
        pub fn aid(&self) -> u8;
    }
);

define_event!(
    /// Event fired when a station disconnects from the AP.
    ApStadisconnected,
    ApStadisconnectedEvent,
    internal::ApStadisconnected,
    {
        /// Get the MAC address of the disconnected station.
        pub fn mac(&self) -> &[u8; 6];
        /// Get the reason code for the disconnection.
        pub fn reason(&self) -> u16;
    }
);

define_event!(
    /// Event fired when the Wi-Fi driver is ready.
    WifiReady
);
define_event!(
    /// Event fired when the station starts.
    StaStart
);
define_event!(
    /// Event fired when the access point starts.
    ApStart
);
