//! Threads.
//!
//! A thread runs a function or a closure on its own stack. Threads are scheduled by priority, and
//! threads of the same priority share the CPU in a round-robin fashion.
//!
//! A thread is created by a [`ThreadSpawner`], which owns the memory the thread runs on. The
//! memory is either allocated on the heap by [`ThreadSpawner::new`], or provided by the caller as
//! a [`Stack`] via [`ThreadSpawner::from_static`].
//!
//! [`ThreadSpawner::spawn`] starts the thread and returns a [`JoinHandle`]. Use the handle to wait
//! for the thread and to read the value the thread returns. If you drop the handle, the thread is
//! detached, and it continues to run until it exits.
//!
//! A thread can not be deleted. It exits when its function returns.
//!
//! ## Example
//!
//! ```rust, no_run
#![doc = esp_hal::before_snippet!()]
//! # struct FakeHeap;
//! # unsafe impl core::alloc::GlobalAlloc for FakeHeap {
//! #     unsafe fn alloc(&self, _: core::alloc::Layout) -> *mut u8 {
//! #         unimplemented!()
//! #     }
//! #     unsafe fn dealloc(&self, _: *mut u8, _: core::alloc::Layout) {
//! #         unimplemented!()
//! #     }
//! # }
//! # #[global_allocator]
//! # static ALLOCATOR: FakeHeap = FakeHeap;
//! use esp_hal::timer::timg::TimerGroup;
//! use esp_rtos::thread::{Stack, ThreadSpawner};
//! use static_cell::ConstStaticCell;
//!
//! static STACK: ConstStaticCell<Stack<4096>> = ConstStaticCell::new(Stack::new());
//!
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
//!
//! let offset = 10;
//! let handle = ThreadSpawner::from_static(STACK.take())
//!     .with_name("worker")
//!     .with_priority(2)
//!     .spawn(move || 32 + offset);
//!
//! let (result, spawner) = handle.join();
//! assert_eq!(result, 42);
//!
//! // `spawner` owns the stack again, and can start another thread on it.
#![doc = esp_hal::after_snippet!()]
//! ```

use core::{
    alloc::Layout,
    ffi::c_void,
    marker::PhantomData,
    mem::{ManuallyDrop, MaybeUninit},
    ptr::NonNull,
};

#[cfg(feature = "alloc")]
use allocator_api2::alloc::Allocator;
use esp_hal::{
    system::Cpu,
    time::{Duration, Instant},
};

#[doc(no_inline)]
pub use crate::CurrentThreadHandle;
#[cfg(feature = "alloc")]
use crate::InternalMemory;
use crate::{
    SCHEDULER,
    run_queue::MaxPriority,
    scheduler::SchedulerState,
    task::{Task, TaskPtr},
    wait_queue::WaitQueue,
};

/// The alignment of a stack, and the granularity of stack sizes.
const STACK_ALIGN: usize = 16;

/// The number of bytes reserved below the usable stack, for the stack guard.
const GUARD_RESERVE: usize = if cfg!(any(hw_task_overflow_detection, sw_task_overflow_detection)) {
    (esp_config::esp_config_int!(usize, "ESP_HAL_CONFIG_STACK_GUARD_OFFSET") + 4)
        .next_multiple_of(STACK_ALIGN)
} else {
    0
};

/// The smallest stack a thread can run on, after the payload is placed on it.
const MIN_STACK: usize = 512;

/// The memory a thread runs on.
///
/// `SIZE` is the number of stack bytes available to the thread. The stack guard and the
/// bookkeeping of the scheduler are stored outside of it, so `Stack<4096>` occupies more than 4096
/// bytes. The closure and the return value of the thread, however, are placed on the stack, and so
/// they reduce the number of bytes the thread can use.
///
/// Pass the stack to [`ThreadSpawner::from_static`] to run a thread on it.
#[repr(C, align(16))]
pub struct Stack<const SIZE: usize> {
    // The stack grows down, so the guard must come first. The scheduler's bookkeeping comes last,
    // where an overflow cannot reach it.
    guard: MaybeUninit<[u8; GUARD_RESERVE]>,
    stack: MaybeUninit<[u8; SIZE]>,
    resources: MaybeUninit<Resources>,
}

// The stack contains uninitialized memory only, until a thread is started on it. The thread's data
// is not reachable through the `Stack`, and only the thread that owns the memory can access it.
unsafe impl<const SIZE: usize> Send for Stack<SIZE> {}
unsafe impl<const SIZE: usize> Sync for Stack<SIZE> {}

impl<const SIZE: usize> Default for Stack<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const SIZE: usize> Stack<SIZE> {
    /// Creates a new stack, uninitialized.
    pub const fn new() -> Self {
        const {
            ::core::assert!(
                SIZE.is_multiple_of(STACK_ALIGN),
                "The stack size must be a multiple of 16 bytes"
            );
            ::core::assert!(SIZE > MIN_STACK, "The stack is too small");
        }

        Self {
            resources: MaybeUninit::uninit(),
            guard: MaybeUninit::uninit(),
            stack: MaybeUninit::uninit(),
        }
    }
}

/// The scheduler's per-thread bookkeeping, at the end of the thread's memory.
#[repr(C, align(16))]
struct Resources {
    task: Task,
    inner: ThreadInner,
}

/// The state a thread shares with its [`JoinHandle`]. It is destroyed when both the scheduler and
/// the `JoinHandle` are done with the thread.
///
/// The scheduler lock protects the flags. Both the thread and its `JoinHandle` change them, and
/// the two can run on different CPUs.
struct ThreadInner {
    /// The thread wrote its return value into the payload.
    has_result: bool,
    /// The `JoinHandle` was dropped, so nobody will read the return value.
    detached: bool,
    /// The scheduler no longer uses the thread's memory.
    released: bool,
    /// Whether the memory block must be returned to the allocator.
    heap: bool,
    wait_queue: WaitQueue,
    /// The closure of the thread, replaced by its return value when the thread exits.
    payload: *mut u8,
}

/// A pointer to the bookkeeping of a thread, which holds a [`Task`] and a [`ThreadInner`], and
/// which sits at the end of the thread's memory block.
#[derive(Clone, Copy)]
pub(crate) struct ThreadPtr(NonNull<Resources>);

impl ThreadPtr {
    pub(crate) fn task(self) -> TaskPtr {
        unsafe { NonNull::new_unchecked(&raw mut (*self.0.as_ptr()).task) }
    }

    fn inner_ptr(self) -> *mut ThreadInner {
        unsafe { &raw mut (*self.0.as_ptr()).inner }
    }

    /// # Safety
    ///
    /// The thread must be prepared, and the caller must not create a second reference to the same
    /// `ThreadInner`.
    unsafe fn inner<'a>(self) -> &'a mut ThreadInner {
        unsafe { &mut *self.inner_ptr() }
    }

    fn as_param(self) -> *mut c_void {
        self.0.as_ptr().cast()
    }

    /// # Safety
    ///
    /// `param` must come from [`ThreadPtr::as_param`].
    unsafe fn from_param(param: *mut c_void) -> Self {
        Self(unwrap!(NonNull::new(param.cast::<Resources>())))
    }

    /// The start of the memory block, which is the low end of the task's stack.
    ///
    /// # Safety
    ///
    /// The task must be in place.
    unsafe fn base(self) -> NonNull<u8> {
        unsafe { NonNull::new_unchecked(self.task().as_ref().stack.cast::<u8>()) }
    }

    /// Places the payload between the stack and the bookkeeping, so that a stack overflow hits the
    /// stack guard first.
    fn reserve_payload(self, base: NonNull<u8>, payload: Layout) -> *mut u8 {
        let stack_bottom = base.as_ptr() as usize;
        let payload_end = self.0.as_ptr() as usize;

        let payload_addr = align_down(payload_end - payload.size(), payload.align().max(4));

        assert!(
            align_down(payload_addr, STACK_ALIGN) >= stack_bottom + GUARD_RESERVE + MIN_STACK,
            "The thread does not fit its stack: it needs {} bytes for the closure and the return \
            value, and at least {} bytes of stack, but the stack is {} bytes",
            payload.size(),
            MIN_STACK,
            payload_end - stack_bottom - GUARD_RESERVE
        );

        payload_addr as *mut u8
    }

    /// Creates the task of a prepared thread, which runs `entry(param)`, and hands it to the
    /// scheduler.
    ///
    /// The stack spans from the stack guard to the payload.
    ///
    /// # Safety
    ///
    /// The closure the thread runs, if any, must already be in place.
    unsafe fn launch(
        self,
        base: NonNull<u8>,
        options: &ThreadOptions,
        entry: extern "C" fn(*mut c_void),
        param: *mut c_void,
    ) {
        let stack_bottom = base.as_ptr();
        let stack_top = align_down(unsafe { self.inner().payload } as usize, STACK_ALIGN);
        let stack = core::ptr::slice_from_raw_parts_mut(
            stack_bottom.cast::<MaybeUninit<u32>>(),
            (stack_top - stack_bottom as usize) / 4,
        );

        let task = self.task();
        unsafe {
            task.as_ptr()
                .write(Task::new(options, entry, param, stack, self))
        };

        SCHEDULER.with(|scheduler| scheduler.register_task(task));
    }

    /// Destroys the bookkeeping of the thread. The scheduler leaves the [`Task`] in place when it
    /// deletes the thread, because the task locates the memory block.
    ///
    /// # Safety
    ///
    /// The thread must be prepared, and nothing may use the bookkeeping afterwards.
    unsafe fn destroy(self) {
        unsafe { core::ptr::drop_in_place(self.task().as_ptr()) };
        unsafe { core::ptr::drop_in_place(self.inner_ptr()) };
    }

    /// Returns the memory block to the allocator, if it came from there.
    ///
    /// # Safety
    ///
    /// Nothing may use the block afterwards.
    unsafe fn deallocate(self, base: NonNull<u8>, heap: bool) {
        if heap {
            cfg_select! {
                feature = "alloc" => unsafe {
                    let block = self.0.as_ptr().add(1) as usize - base.as_ptr() as usize;
                    let size = block - GUARD_RESERVE - size_of::<Resources>();
                    InternalMemory.deallocate(base, block_layout(size))
                },
                _ => {
                    _ = base;
                }
            }
        }
    }
}

/// The layout of a memory block that offers `size` bytes of stack.
#[cfg(feature = "alloc")]
fn block_layout(size: usize) -> Layout {
    unwrap!(
        Layout::from_size_align(GUARD_RESERVE + size + size_of::<Resources>(), STACK_ALIGN).ok(),
        "The stack size is too large"
    )
}

/// The properties of a thread, which a `join` returns so that the next thread can reuse them.
#[derive(Clone, Copy)]
pub(crate) struct ThreadOptions {
    pub name: Option<&'static str>,
    pub priority: usize,
    pub pinned_to: Option<Cpu>,
}

impl ThreadOptions {
    const fn new() -> Self {
        Self {
            name: None,
            priority: 0,
            pinned_to: None,
        }
    }
}

const fn align_down(addr: usize, align: usize) -> usize {
    addr & !(align - 1)
}

/// The error returned when a thread stack cannot be allocated.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OutOfMemory(usize);

impl core::fmt::Display for OutOfMemory {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "Failed to allocate {} bytes of memory for the thread",
            self.0
        )
    }
}

impl core::error::Error for OutOfMemory {}

/// Owns the memory of a thread, and starts the thread on it.
///
/// The spawner is consumed by [`ThreadSpawner::spawn`], and [`JoinHandle::join`] returns it, so
/// that the next thread can run on the same memory.
pub struct ThreadSpawner {
    /// The start of the memory block, which holds no thread yet.
    base: NonNull<u8>,
    /// The bookkeeping at the end of the block.
    thread: ThreadPtr,
    /// Whether the block must be returned to the allocator.
    heap: bool,
    options: ThreadOptions,
}

unsafe impl Send for ThreadSpawner {}

impl ThreadSpawner {
    /// Allocates a stack of `stack_size` bytes in internal memory.
    ///
    /// The closure and the return value of the thread are placed on this stack, so they reduce the
    /// number of bytes the thread can use.
    ///
    /// This function panics if the allocation fails. Use [`ThreadSpawner::try_new`] to handle the
    /// failure.
    #[cfg(feature = "alloc")]
    #[cfg_attr(docsrs, doc(cfg(feature = "alloc")))]
    pub fn new(stack_size: usize) -> Self {
        unwrap!(
            Self::try_new(stack_size).ok(),
            "Failed to allocate a thread stack"
        )
    }

    /// Allocates a stack of `stack_size` bytes in internal memory.
    ///
    /// This function returns an error if the allocation fails.
    #[cfg(feature = "alloc")]
    #[cfg_attr(docsrs, doc(cfg(feature = "alloc")))]
    pub fn try_new(stack_size: usize) -> Result<Self, OutOfMemory> {
        let size = stack_size.next_multiple_of(STACK_ALIGN);
        let base = InternalMemory
            .allocate(block_layout(size))
            .map_err(|_| OutOfMemory(stack_size))?
            .cast::<u8>();

        Ok(Self {
            base,
            thread: ThreadPtr(unsafe { base.add(GUARD_RESERVE + size) }.cast()),
            heap: true,
            options: ThreadOptions::new(),
        })
    }

    /// Runs the thread on a statically allocated stack.
    pub fn from_static<const SIZE: usize>(stack: &'static mut Stack<SIZE>) -> Self {
        Self {
            base: NonNull::from(&mut *stack).cast(),
            thread: ThreadPtr(NonNull::from(&mut stack.resources).cast()),
            heap: false,
            options: ThreadOptions::new(),
        }
    }

    /// Sets the name of the thread.
    ///
    /// The name is used by the logs and by `rtos-trace`. Threads have no name by default.
    #[must_use]
    pub fn with_name(mut self, name: &'static str) -> Self {
        self.options.name = Some(name);
        self
    }

    /// Sets the priority of the thread.
    ///
    /// Higher numbers mean higher priority. The default priority is 0, which is the lowest.
    ///
    /// This function panics if `priority` is greater than the highest supported priority.
    #[must_use]
    pub fn with_priority(mut self, priority: usize) -> Self {
        assert!(
            priority <= MaxPriority::MAX_PRIORITY,
            "The priority must not be greater than {}",
            MaxPriority::MAX_PRIORITY
        );
        self.options.priority = priority;
        self
    }

    /// Runs the thread on a single CPU.
    ///
    /// By default a thread can run on any CPU the scheduler runs on.
    #[must_use]
    pub fn with_pinned_to(mut self, cpu: Cpu) -> Self {
        self.options.pinned_to = Some(cpu);
        self
    }

    /// Starts the thread.
    ///
    /// The thread runs `f`, and exits when `f` returns. Drop the returned handle to detach the
    /// thread, or use [`JoinHandle::join`] to wait for it and to read its return value.
    ///
    /// This function panics if `f` and its return value do not fit the thread's stack, if the
    /// scheduler is not running, if it is called from an interrupt handler, or if the thread is
    /// pinned to a CPU that does not run the scheduler.
    pub fn spawn<F, T>(self, f: F) -> JoinHandle<T>
    where
        F: FnOnce() -> T + Send + 'static,
        T: Send + 'static,
    {
        crate::assert_thread_mode("ThreadSpawner::spawn");

        let payload = unwrap!(
            Layout::from_size_align(
                size_of::<F>().max(size_of::<T>()),
                align_of::<F>().max(align_of::<T>()),
            )
            .ok()
        );

        let (base, options) = (self.base, self.options);
        let thread = self.prepare(payload);

        unsafe { thread.inner().payload.cast::<F>().write(f) };
        unsafe { thread.launch(base, &options, thread_entry::<F, T>, thread.as_param()) };

        JoinHandle {
            thread,
            options,
            _result: PhantomData,
        }
    }

    /// Starts a thread that runs a C-compatible function.
    #[cfg(feature = "esp-radio")]
    pub(crate) fn spawn_extern(
        self,
        entry: extern "C" fn(*mut c_void),
        param: *mut c_void,
    ) -> TaskPtr {
        let (base, options) = (self.base, self.options);
        let thread = self.prepare(Layout::new::<()>());

        // These threads have no `JoinHandle`, so the scheduler releases their memory when they
        // exit.
        unsafe { thread.inner().detached = true };
        unsafe { thread.launch(base, &options, entry, param) };

        thread.task()
    }

    /// Reserves the payload, and initializes the state the thread shares with its `JoinHandle`.
    ///
    /// The caller must move the closure, if any, into `ThreadInner::payload`, and must then call
    /// [`ThreadPtr::launch`] to start the thread.
    fn prepare(self, payload: Layout) -> ThreadPtr {
        assert!(
            SCHEDULER.with(|scheduler| scheduler.time_driver.is_some()),
            "The scheduler is not running. Call esp_rtos::start before spawning a thread."
        );

        // The thread owns its memory from here on. `JoinHandle` or the scheduler releases it.
        let this = ManuallyDrop::new(self);

        let thread = this.thread;
        unsafe {
            thread.inner_ptr().write(ThreadInner {
                has_result: false,
                detached: false,
                released: false,
                heap: this.heap,
                wait_queue: WaitQueue::new(),
                payload: thread.reserve_payload(this.base, payload),
            })
        };

        thread
    }
}

impl Drop for ThreadSpawner {
    fn drop(&mut self) {
        unsafe { self.thread.deallocate(self.base, self.heap) };
    }
}

extern "C" fn thread_entry<F, T>(param: *mut c_void)
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    let thread = unsafe { ThreadPtr::from_param(param) };

    let payload = unsafe { thread.inner().payload };
    let f = unsafe { payload.cast::<F>().read() };

    let result = f();

    unsafe { publish(thread, result) };
}

/// Hands the return value of the thread to the `JoinHandle`, or drops it if the thread is
/// detached.
unsafe fn publish<T>(thread: ThreadPtr, result: T) {
    // The thread is still running, so nobody can release its memory while we write.
    let payload = unsafe { thread.inner().payload.cast::<T>() };
    unsafe { payload.write(result) };

    let detached = SCHEDULER.with(|_scheduler| {
        let inner = unsafe { thread.inner() };
        inner.has_result = !inner.detached;
        inner.detached
    });

    if detached {
        unsafe { payload.drop_in_place() };
    }
}

/// Releases the memory of an exited thread. The scheduler calls this after it switched off the
/// thread's stack for the last time, so the memory can be reused from this point on.
///
/// # Safety
///
/// The scheduler must no longer use the `Task` of the thread.
pub(crate) unsafe fn release(thread: ThreadPtr, scheduler: &mut SchedulerState) {
    let inner = unsafe { thread.inner() };
    inner.released = true;

    if inner.detached {
        unsafe { free(thread) };
    } else {
        inner.wait_queue.notify(scheduler);
    }
}

/// Destroys the thread's bookkeeping, and returns heap-allocated memory to the allocator.
unsafe fn free(thread: ThreadPtr) {
    let base = unsafe { thread.base() };
    let heap = unsafe { thread.inner().heap };

    unsafe { thread.destroy() };
    unsafe { thread.deallocate(base, heap) };
}

/// A handle to a running thread.
///
/// Dropping the handle detaches the thread. The thread keeps running, but its return value is
/// dropped, and the memory of a thread that runs on a heap-allocated stack is returned to the
/// allocator when the thread exits. The memory of a thread that runs on a [`Stack`] is not
/// recovered.
pub struct JoinHandle<T> {
    thread: ThreadPtr,
    /// The properties of the thread, which outlive it, so that [`JoinHandle::join`] can return a
    /// spawner that starts an identical thread.
    options: ThreadOptions,
    _result: PhantomData<T>,
}

unsafe impl<T: Send> Send for JoinHandle<T> {}

impl<T> JoinHandle<T> {
    /// Returns the name of the thread.
    pub fn name(&self) -> Option<&'static str> {
        self.options.name
    }

    /// Returns whether the thread has finished running its closure.
    pub fn is_finished(&self) -> bool {
        SCHEDULER.with(|_scheduler| unsafe { self.thread.inner().has_result })
    }

    /// Detaches the thread.
    ///
    /// The thread keeps running until it exits, but its return value is dropped, and its memory
    /// cannot be reused.
    pub fn detach(self) {}

    /// Waits for the thread to exit, and returns its return value together with a spawner that
    /// owns the memory the thread ran on. The spawner has the properties of the thread that
    /// exited.
    ///
    /// The current thread sleeps until the thread exits. This function does not change the
    /// priority of the thread it waits for, so a thread of a middle priority can delay a
    /// high-priority thread that waits for a low-priority one.
    ///
    /// This function panics if a thread tries to join itself.
    pub fn join(self) -> (T, ThreadSpawner) {
        let this = ManuallyDrop::new(self);
        let thread = this.thread;

        assert!(
            thread.task() != SCHEDULER.current_task(),
            "A thread cannot join itself"
        );

        // The memory is safe to reuse once the scheduler marks the thread released.
        loop {
            let released = SCHEDULER.with(|scheduler| {
                let inner = unsafe { thread.inner() };
                if inner.released {
                    true
                } else {
                    inner
                        .wait_queue
                        .wait_with_deadline(scheduler, Instant::EPOCH + Duration::MAX);
                    false
                }
            });

            if released {
                break;
            }
        }

        let inner = unsafe { thread.inner() };

        debug_assert!(
            inner.has_result,
            "The thread exited without publishing its return value"
        );

        let result = unsafe { inner.payload.cast::<T>().read() };
        let base = unsafe { thread.base() };
        let heap = inner.heap;

        unsafe { thread.destroy() };

        (
            result,
            ThreadSpawner {
                base,
                thread,
                heap,
                options: this.options,
            },
        )
    }
}

impl<T> Drop for JoinHandle<T> {
    fn drop(&mut self) {
        // The scheduler releases the thread's memory while it holds its lock, and it may do so as
        // soon as we mark the thread detached. Working under the same lock makes sure that exactly
        // one of the two frees the memory, and that the other one stops using it.
        SCHEDULER.with(|_scheduler| {
            let inner = unsafe { self.thread.inner() };
            inner.detached = true;

            if inner.has_result {
                unsafe { inner.payload.cast::<T>().drop_in_place() };
            }

            if inner.released {
                unsafe { free(self.thread) };
            }
        });
    }
}
