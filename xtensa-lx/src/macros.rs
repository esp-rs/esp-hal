/// Macro to create a mutable reference to a statically allocated value
///
/// This macro returns a value with type `Option<&'static mut $ty>`.
/// `Some($expr)` will be returned the first time the macro is executed; further
/// calls will return `None`. To avoid `unwrap`ping a `None` variant the caller
/// must ensure that the macro is called from a function that's executed at most
/// once in the whole lifetime of the program.
///
/// # Example
///
/// ``` no_run
/// use xtensa_lx::singleton;
///
/// fn main() {
///     // OK if `main` is executed only once
///     let x: &'static mut bool = singleton!(: bool = false).unwrap();
///
///     let y = alias();
///     // BAD this second call to `alias` will definitively `panic!`
///     let y_alias = alias();
/// }
///
/// fn alias() -> &'static mut bool {
///     singleton!(: bool = false).unwrap()
/// }
/// ```
#[macro_export]
macro_rules! singleton {
    ($(#[$meta:meta])* $name:ident: $ty:ty = $expr:expr_2021) => {
        $crate::_export::critical_section::with(|_| {
            // this is a tuple of a MaybeUninit and a bool because using an Option here is
            // problematic:  Due to niche-optimization, an Option could end up producing a non-zero
            // initializer value which would move the entire static from `.bss` into `.data`...
            $(#[$meta])*
            static mut $name: (::core::mem::MaybeUninit<$ty>, bool) =
                (::core::mem::MaybeUninit::uninit(), false);

            #[allow(unsafe_code)]
            let used = unsafe { $name.1 };
            if used {
                None
            } else {
                let expr = $expr;

                #[allow(unsafe_code)]
                unsafe {
                    $name.1 = true;
                    Some($name.0.write(expr))
                }
            }
        })
    };
    ($(#[$meta:meta])* : $ty:ty = $expr:expr_2021) => {
        $crate::singleton!($(#[$meta])* VAR: $ty = $expr)
    };
}
