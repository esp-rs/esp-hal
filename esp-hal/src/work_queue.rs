//! A generic work queue.
//!
//! Work queues are the backbone of cryptographic drivers. They enable asynchronous and
//! blocking operations using shared peripherals, like crypto accelerators. Clients
//! post work items into the work queue, and poll for completion. The act of polling
//! processes the queue, which allows any number of clients to post work without deadlocking.
//!
//! Work queues are configured by backends. Backends register a `process` callback which is
//! called when a client posts a work item or polls for completion.
//!
//! Posting a work item into the queue returns a handle. The handle can be used to poll whether
//! the work item has been processed. Dropping the handle will cancel the work item.

use core::{future::poll_fn, marker::PhantomData};

use crate::{asynch::AtomicWaker, sync::Locked};

pub(crate) struct VTable<T: Sync> {
    /// Starts processing a new work item.
    ///
    /// The function returns whether the work item was accepted. If there is no driver currently
    /// processing the queue, this function will return false to prevent removing the work item
    /// from the queue.
    ///
    /// This function should be as short as possible.
    pub(crate) post: fn(*const (), &mut T) -> bool,

    /// Polls the status of a particular work item.
    ///
    /// Return `None` if the work item is not currently being processed.
    ///
    /// This function should be as short as possible.
    pub(crate) poll: fn(*const (), &mut T) -> Option<Poll>,

    /// Attempts to abort processing a work item.
    ///
    /// This function should be as short as possible.
    pub(crate) cancel: fn(*const (), &mut T),

    /// Called when the work queue becomes empty.
    ///
    /// This function should be as short as possible.
    pub(crate) on_empty: fn(*const ()),
}

impl<T: Sync> VTable<T> {
    pub(crate) const fn noop() -> Self {
        Self {
            post: |_, _| false,
            poll: |_, _| None,
            cancel: |_, _| (),
            on_empty: |_| (),
        }
    }
}

struct Inner<T: Sync> {
    head: *mut WorkItem<T>,
    tail: *mut WorkItem<T>,
    current: *mut WorkItem<T>,

    data: *const (),
    vtable: VTable<T>,
}

impl<T: Sync> Inner<T> {
    /// Places a work item at the end of the queue.
    fn enqueue(&mut self, ptr: *mut WorkItem<T>) {
        if self.tail.is_null() {
            // Queue is empty, set both `head` and `tail`.
            self.tail = ptr;
            self.head = ptr;
        } else {
            unsafe {
                // Safety: we just checked that `tail` is not null.
                (*self.tail).next = ptr;
            }
        }
    }

    /// Places a work item at the front of the queue.
    fn enqueue_front(&mut self, ptr: *mut WorkItem<T>) {
        unsafe { (*ptr).next = self.head };
        self.head = ptr;
        if self.tail.is_null() {
            self.tail = ptr;
        }
    }

    /// Pops and returns a work item from the start of the queue.
    fn dequeue(&mut self) -> Option<*mut WorkItem<T>> {
        let ptr = self.head;
        // If the `head` is null, the queue is empty. Return None and do nothing.
        if ptr.is_null() {
            return None;
        }

        unsafe {
            // Safety: we just checked that `ptr` is not null.
            self.head = (*ptr).next;
        }

        // If the new `head` is null, the queue is empty. Clear the `tail` pointer.
        if self.head.is_null() {
            self.tail = core::ptr::null_mut();
        }

        Some(ptr)
    }

    /// Removes the item from the queue.
    ///
    /// Returns `true` if the work item was successfully removed, `false` if the work item was not
    /// found in the queue.
    fn remove(&mut self, ptr: *mut WorkItem<T>) -> bool {
        // Walk the queue to find `ptr`.
        let mut prev = core::ptr::null_mut();
        let mut current = self.head;
        while !current.is_null() {
            let next = unsafe {
                // Safety: we've just verified that `current` is not null.
                (*current).next
            };

            if current != ptr {
                // Not what we're looking for. Move to the next element.
                prev = current;
                current = next;
                continue;
            }

            // We've found `ptr`. Remove it from the list.
            if ptr == self.head {
                self.head = next;
            } else {
                unsafe {
                    // Safety: If `ptr` is not the `head` of the queue, we must have a previous
                    // element.
                    (*prev).next = next;
                }
            }

            if ptr == self.tail {
                self.tail = prev;
            }

            return true;
        }

        // Did not find `ptr`.
        false
    }

    fn post_to_driver(&mut self, ptr: *mut WorkItem<T>) {
        // Start processing a new work item.
        if unsafe { (self.vtable.post)(self.data, &mut (*ptr).data) } {
            self.current = ptr;
        } else {
            // If the driver didn't accept the work item, place it back to the front of the queue.
            self.enqueue_front(ptr);
        }
    }

    /// Runs one processing iteration.
    ///
    /// This function enqueues a new work item or polls the status of the currently processed one.
    fn process(&mut self) {
        if self.current.is_null() {
            if let Some(ptr) = self.dequeue() {
                self.post_to_driver(ptr);
            }
            // The queue is empty, but the driver should already have been notified when the queue
            // became empty, so we don't notify it here.
        } else {
            let result = unsafe { (self.vtable.poll)(self.data, &mut (*self.current).data) };

            if let Some(Poll::Ready(status)) = result {
                unsafe { (*self.current).complete(status) };

                if let Some(ptr) = self.dequeue() {
                    self.post_to_driver(ptr);
                } else {
                    // There are no more work items. Notify the driver.
                    self.current = core::ptr::null_mut();
                    (self.vtable.on_empty)(self.data);
                }
            }
        }
    }

    /// Cancels a particular work item.
    ///
    /// If the work item is currently being processed, this function notifies the driver. Otherwise,
    /// it tries to remove the pointer from the work queue.
    fn cancel(&mut self, work_item: *const WorkItem<T>) {
        if core::ptr::eq(self.current.cast_const(), work_item) {
            // Cancelling an in-progress item is more complicated than plucking it from the
            // queue. Forward the request to the driver to (maybe) cancel the
            // operation.
            unsafe { (self.vtable.cancel)(self.data, &mut (*self.current).data) };
        } else if unsafe { (*work_item).status == Poll::Pending } {
            // The work item is not the current one, remove it from the queue. This immediately
            // cancels the work item, if it was in fact in this work queue.
            if self.remove(work_item.cast_mut()) {
                unsafe { (*work_item.cast_mut()).complete(Status::Cancelled) };
            }
        }
    }
}

/// A generic work queue.
pub(crate) struct WorkQueue<T: Sync> {
    inner: Locked<Inner<T>>,
}

impl<T: Sync> WorkQueue<T> {
    /// Creates a new `WorkQueue`.
    pub const fn new() -> Self {
        Self {
            inner: Locked::new(Inner {
                head: core::ptr::null_mut(),
                tail: core::ptr::null_mut(),
                current: core::ptr::null_mut(),

                data: core::ptr::null(),
                vtable: VTable::noop(),
            }),
        }
    }

    /// Configures the queue.
    ///
    /// # Safety
    ///
    /// The `data` pointer must be valid as long as the `WorkQueue` is configured with it.
    pub unsafe fn configure<D: Sync>(&self, data: *const D, vtable: VTable<T>) {
        self.inner.with(|inner| {
            inner.data = data.cast();
            inner.vtable = vtable;
        })
    }

    /// Enqueues a work item.
    pub fn post_work<'t>(&'t self, work_item: &'t mut WorkItem<T>) -> Handle<'t, T> {
        let ptr = unsafe {
            // Safety: `Handle` and `work_item` have lifetime 't, which ensures this call
            // can't be called on an in-flight work item. As `Handle` (and the underlying driver
            // that processed the work queue) does not use the reference, and using the
            // reference is not possible while `Handle` exists, this should be safe.
            work_item.prepare()
        };

        self.inner.with(|inner| inner.enqueue(ptr));

        Handle {
            queue: self,
            work_item: ptr.cast_const(),
            _marker: PhantomData,
        }
    }

    /// Polls the queue once.
    pub fn process(&self) {
        self.inner.with(|inner| inner.process());
    }

    /// Schedules the work item to be cancelled.
    ///
    /// The work item should not be assumed to be immediately cancelled. Polling its handle
    /// is necessary to ensure it is no longer being processed by the underlying driver.
    pub fn cancel(&self, work_item: *const WorkItem<T>) {
        self.inner.with(|inner| inner.cancel(work_item));
    }
}

/// The status of a [`WorkItem`].
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Status {
    /// The processing has completed.
    Completed,

    /// The work item has been cancelled.
    Cancelled,
}

/// A unit of work in the work queue.
pub(crate) struct WorkItem<T: Sync> {
    next: *mut WorkItem<T>,
    status: Poll,
    data: T,
    waker: AtomicWaker,
}

impl<T: Sync> WorkItem<T> {
    /// Completes the work item.
    ///
    /// This function is intended to be called from the underlying drivers.
    pub fn complete(&mut self, status: Status) {
        self.status = Poll::Ready(status);
        self.waker.wake();
    }

    /// Prepares a work item to be enqueued.
    ///
    /// # Safety:
    ///
    /// The caller must ensure the reference is not used again while the pointer returned by this
    /// function is in use.
    unsafe fn prepare(&mut self) -> *mut Self {
        self.next = core::ptr::null_mut();

        self.status = Poll::Pending;

        self as *mut _
    }
}

/// The status of a [`WorkItem`].
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Poll {
    /// The work item has not yet been fully processed.
    Pending,

    /// The work item has been processed.
    Ready(Status),
}

/// A reference to a posted [`WorkItem`].
///
/// This struct ensures that the work item is valid until the item is processed or is removed from
/// the work queue.
///
/// Dropping the handle cancels the work item, but may block for some time if the work item is
/// already being processed.
pub(crate) struct Handle<'t, T: Sync> {
    queue: &'t WorkQueue<T>,
    work_item: *const WorkItem<T>,
    // Make sure lifetime is invariant to prevent UB.
    _marker: PhantomData<&'t mut WorkItem<T>>,
}

impl<'t, T: Sync> Handle<'t, T> {
    fn status(&self) -> Poll {
        unsafe { (*self.work_item).status }
    }

    /// Returns the status of the work item.
    pub fn poll(&mut self) -> Poll {
        let status = self.status();
        if status == Poll::Pending {
            self.queue.process();
            self.status()
        } else {
            status
        }
    }

    /// Waits for the work item to be processed.
    pub fn wait(&mut self) -> impl Future<Output = Status> {
        poll_fn(|ctx| {
            unsafe { (*self.work_item).waker.register(ctx.waker()) };
            match self.poll() {
                Poll::Pending => core::task::Poll::Pending,
                Poll::Ready(status) => core::task::Poll::Ready(status),
            }
        })
    }
}

impl<'t, T: Sync> Drop for Handle<'t, T> {
    fn drop(&mut self) {
        self.queue.cancel(self.work_item);
        // We must wait for the driver to release our WorkItem.
        while self.poll() == Poll::Pending {}
    }
}

pub(crate) struct WorkQueueDriver<'t, D, T>
where
    D: Sync,
    T: Sync,
{
    queue: &'t WorkQueue<T>,
    _marker: PhantomData<&'t mut D>,
}

impl<'t, D, T> WorkQueueDriver<'t, D, T>
where
    D: Sync,
    T: Sync,
{
    pub fn new(driver: &'t mut D, vtable: VTable<T>, queue: &'t WorkQueue<T>) -> Self {
        unsafe {
            // Safety: the lifetime 't ensures the pointer remains valid for the lifetime of the
            // WorkQueueDriver. The Drop implementation (and the general "Don't forget" clause)
            // ensure the pointer is not used after the WQD has been dropped.
            queue.configure((driver as *mut D).cast_const(), vtable);
        }
        Self {
            queue,
            _marker: PhantomData,
        }
    }

    pub fn poll(&mut self) {
        self.queue.process();
    }
}

impl<D, T> Drop for WorkQueueDriver<'_, D, T>
where
    D: Sync,
    T: Sync,
{
    fn drop(&mut self) {
        unsafe {
            // Safety: the noop VTable functions don't use the pointer at all.
            self.queue.configure(core::ptr::null::<D>(), VTable::noop())
        };
    }
}

/// Used by work queue clients, allows hiding WorkItem.
pub(crate) struct WorkQueueFrontend<T: Sync> {
    work_item: WorkItem<T>,
}

impl<T: Sync> WorkQueueFrontend<T> {
    pub fn new(initial: T) -> Self {
        Self {
            work_item: WorkItem {
                next: core::ptr::null_mut(),
                status: Poll::Pending,
                data: initial,
                waker: AtomicWaker::new(),
            },
        }
    }

    pub fn data_mut(&mut self) -> &mut T {
        &mut self.work_item.data
    }

    pub fn post<'t>(&'t mut self, queue: &'t WorkQueue<T>) -> Handle<'t, T> {
        queue.post_work(&mut self.work_item)
    }
}
