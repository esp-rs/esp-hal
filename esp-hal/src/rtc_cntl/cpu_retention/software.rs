//! CPU retention through a wake stub, which saves and restores the CPU itself.

pub(crate) fn installed_buffer_ptr() -> Option<*mut u8> {
    None
}
