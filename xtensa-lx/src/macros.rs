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
    (: $ty:ty = $expr:expr) => {
        $crate::interrupt::free(|_| {
            static mut VAR: Option<$ty> = None;

            #[allow(unsafe_code)]
            let used = unsafe { VAR.is_some() };
            if used {
                None
            } else {
                let expr = $expr;

                #[allow(unsafe_code)]
                unsafe {
                    VAR = Some(expr)
                }

                #[allow(unsafe_code)]
                unsafe {
                    VAR.as_mut()
                }
            }
        })
    };
}
