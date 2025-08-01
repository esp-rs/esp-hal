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
#![cfg_attr(esp32c2, allow(unused))]

use core::{future::poll_fn, marker::PhantomData, ptr::NonNull};

use crate::{asynch::AtomicWaker, sync::Locked};

/// Queue driver operations.
///
/// Functions in this VTable are provided by drivers that consume work items.
/// These functions may be called both in the context of the queue frontends and drivers.
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

    /// Called when the driver may be stopped.
    ///
    /// This function should be as short as possible.
    pub(crate) stop: fn(*const ()),
}

impl<T: Sync> VTable<T> {
    pub(crate) const fn noop() -> Self {
        Self {
            post: |_, _| false,
            poll: |_, _| None,
            cancel: |_, _| (),
            stop: |_| (),
        }
    }
}

struct Inner<T: Sync> {
    head: Option<NonNull<WorkItem<T>>>,
    tail: Option<NonNull<WorkItem<T>>>,
    current: Option<NonNull<WorkItem<T>>>,

    // The data pointer will be passed to VTable functions, which may be called in any context.
    data: *const (),
    vtable: VTable<T>,
}

impl<T: Sync> Inner<T> {
    /// Places a work item at the end of the queue.
    fn enqueue(&mut self, ptr: NonNull<WorkItem<T>>) {
        if let Some(tail) = self.tail.as_mut() {
            // Queue contains something, append to `tail`.
            unsafe { tail.as_mut().next = Some(ptr) };
        } else {
            // Queue was empty, set `head` to the first element.
            self.head = Some(ptr);
        }

        // Move `tail` to the newly inserted item.
        self.tail = Some(ptr);
    }

    /// Places a work item at the front of the queue.
    fn enqueue_front(&mut self, mut ptr: NonNull<WorkItem<T>>) {
        // Chain the node into the list.
        unsafe { ptr.as_mut().next = self.head };

        // Adjust list `head` to point at the new-first element.
        self.head = Some(ptr);
        if self.tail.is_none() {
            // The queue was empty, we need to set `tail` to the last element.
            self.tail = Some(ptr);
        }
    }

    /// Runs one processing iteration.
    ///
    /// This function enqueues a new work item or polls the status of the currently processed one.
    fn process(&mut self) {
        if let Some(mut current) = self.current {
            let result = unsafe { (self.vtable.poll)(self.data, &mut current.as_mut().data) };

            if let Some(Poll::Ready(status)) = result {
                unsafe { current.as_mut().complete(status) };
                self.current = None;

                self.dequeue_and_post(true);
            }
        } else {
            // If the queue is empty, the driver should already have been notified when the queue
            // became empty, so we don't notify it here.
            self.dequeue_and_post(false);
        }
    }

    // Note: even if the queue itself may be implemented lock-free, dequeuing and posting to the
    // driver must be done atomically to ensure that the queue can be processed fully by any of
    // the frontends polling it.
    fn dequeue_and_post(&mut self, notify_on_empty: bool) {
        if let Some(mut ptr) = self.dequeue() {
            // Start processing a new work item.

            if unsafe { (self.vtable.post)(self.data, &mut ptr.as_mut().data) } {
                self.current = Some(ptr);
            } else {
                // If the driver didn't accept the work item, place it back to the front of the
                // queue.
                self.enqueue_front(ptr);
            }
        } else if notify_on_empty {
            // There are no more work items. Notify the driver that it can stop.
            (self.vtable.stop)(self.data);
        }
    }

    /// Pops and returns a work item from the start of the queue.
    fn dequeue(&mut self) -> Option<NonNull<WorkItem<T>>> {
        // If the `head` is None, the queue is empty. Return None and do nothing.
        let ptr = self.head?;

        unsafe { self.head = ptr.as_ref().next };

        // If the new `head` is null, the queue is empty. Clear the `tail` pointer.
        if self.head.is_none() {
            self.tail = None;
        }

        Some(ptr)
    }

    /// Cancels a particular work item.
    ///
    /// If the work item is currently being processed, this function notifies the driver. Otherwise,
    /// it tries to remove the pointer from the work queue.
    ///
    /// The function returns true when the item was immediately cancelled.
    ///
    /// This function is not `unsafe` because it only dereferences `work_item` if the function has
    /// determined that the item belongs to this queue.
    fn cancel(&mut self, mut work_item: NonNull<WorkItem<T>>) -> bool {
        if self.current == Some(work_item) {
            // Cancelling an in-progress item is more complicated than plucking it from the
            // queue. Forward the request to the driver to (maybe) cancel the
            // operation.
            (self.vtable.cancel)(self.data, unsafe {
                // This is safe to do, because the work item is currently owned by this queue.
                &mut work_item.as_mut().data
            });
            // Queue will need to be polled to query item status.
            return false;
        }

        // The work item is not the current one, remove it from the queue. This immediately
        // cancels the work item. `remove` only uses the address of the work item without
        // dereferencing it.
        if self.remove(work_item) {
            unsafe { work_item.as_mut().complete(Status::Cancelled) };
            // Cancelled immediately, no further polling necessary for this item.
            return true;
        }

        // In this case the item doesn't belong to this queue, it can be in any state. The item will
        // need to be polled, but this also means something may have gone wrong.
        false
    }

    /// Removes the item from the queue.
    ///
    /// Returns `true` if the work item was successfully removed, `false` if the work item was not
    /// found in the queue.
    ///
    /// This function is not `unsafe` because it does not dereference `ptr`, so it does not matter
    /// that `ptr` may belong to a different work queue.
    fn remove(&mut self, ptr: NonNull<WorkItem<T>>) -> bool {
        // Walk the queue to find `ptr`.
        let mut prev = None;
        let mut current = self.head;
        while let Some(current_item) = current {
            let next = unsafe { current_item.as_ref().next };

            if current_item != ptr {
                // Not what we're looking for. Move to the next element.
                prev = current;
                current = next;
                continue;
            }

            // We've found `ptr`. Remove it from the list.
            if Some(ptr) == self.head {
                self.head = next;
            } else {
                // Unwrapping is fine, because if the current pointer is not the `head`, the
                // previous pointer must be Some.
                unsafe { unwrap!(prev).as_mut().next = next };
            }

            if Some(ptr) == self.tail {
                self.tail = prev;
            }

            return true;
        }

        // Did not find `ptr`.
        false
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
                head: None,
                tail: None,
                current: None,

                data: core::ptr::null(),
                vtable: VTable::noop(),
            }),
        }
    }

    /// Configures the queue.
    ///
    /// The provided data pointer will be passed to the VTable functions.
    ///
    /// # Safety
    ///
    /// The `data` pointer must be valid as long as the `WorkQueue` is configured with it. The
    /// driver must access the data pointer appropriately (i.e. it must not move !Send data out of
    /// it).
    pub unsafe fn configure<D: Sync>(&self, data: *const D, vtable: VTable<T>) {
        self.inner.with(|inner| {
            (inner.vtable.stop)(inner.data);

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
            work_item: ptr,
            _marker: PhantomData,
        }
    }

    /// Polls the queue once.
    pub fn process(&self) {
        self.inner.with(|inner| inner.process());
    }

    /// Polls the queue once and returns the status of the given work item.
    ///
    /// ## Safety
    ///
    /// The caller must ensure that `item` belongs to the polled queue. An item belongs to the
    /// **last queue it was enqueued in**, even if the item is no longer in the queue's linked
    /// list. This relationship is broken when the Handle that owns the WorkItem is dropped.
    pub unsafe fn poll(&self, item: NonNull<WorkItem<T>>) -> Poll {
        self.inner.with(|inner| {
            let status = unsafe { (*item.as_ptr()).status };
            if status == Poll::Pending {
                inner.process();
                unsafe { (*item.as_ptr()).status }
            } else {
                status
            }
        })
    }

    /// Schedules the work item to be cancelled.
    ///
    /// The function returns true when the item was immediately cancelled. If the function returns
    /// `false`, the item will need to be polled until its status becomes [`Poll::Ready`].
    ///
    /// The work item should not be assumed to be immediately cancelled. Polling its handle
    /// is necessary to ensure it is no longer being processed by the underlying driver.
    pub fn cancel(&self, work_item: NonNull<WorkItem<T>>) -> bool {
        self.inner.with(|inner| inner.cancel(work_item))
    }
}

/// The status of a work item.
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
    next: Option<NonNull<WorkItem<T>>>,
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
    unsafe fn prepare(&mut self) -> NonNull<Self> {
        self.next = None;
        self.status = Poll::Pending;

        NonNull::from(self)
    }
}

/// The status of a work item posted to a work queue.
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
    work_item: NonNull<WorkItem<T>>,
    // Make sure lifetime is invariant to prevent UB.
    _marker: PhantomData<&'t mut WorkItem<T>>,
}

impl<'t, T: Sync> Handle<'t, T> {
    /// Returns the status of the work item.
    pub fn poll(&mut self) -> Poll {
        unsafe { self.queue.poll(self.work_item) }
    }

    /// Waits for the work item to be processed.
    pub fn wait(&mut self) -> impl Future<Output = Status> {
        poll_fn(|ctx| {
            unsafe { (*self.work_item.as_ptr()).waker.register(ctx.waker()) };
            match self.poll() {
                Poll::Pending => core::task::Poll::Pending,
                Poll::Ready(status) => core::task::Poll::Ready(status),
            }
        })
    }
}

impl<'t, T: Sync> Drop for Handle<'t, T> {
    fn drop(&mut self) {
        if self.queue.cancel(self.work_item) {
            // We must wait for the driver to release our WorkItem.
            while self.poll() == Poll::Pending {}
        }
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

    #[expect(unused)] // TODO this will be used with AesDmaBackend, for example
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
                next: None,
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
