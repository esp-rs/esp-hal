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
#![cfg_attr(not(feature = "unstable"), allow(unused))]

use core::{future::poll_fn, marker::PhantomData, ptr::NonNull, task::Context};

use embassy_sync::waitqueue::WakerRegistration;
use esp_sync::NonReentrantMutex;

/// Queue driver operations.
///
/// Functions in this VTable are provided by drivers that consume work items.
/// These functions may be called both in the context of the queue frontends and drivers.
pub(crate) struct VTable<T: Sync + Send> {
    /// Starts processing a new work item.
    ///
    /// The function returns whether the work item was accepted, and its poll status if it was
    /// accepted. If there is no driver currently processing the queue, this function will
    /// return None to prevent removing the work item from the queue.
    ///
    /// This function should be as short as possible.
    pub(crate) post: fn(NonNull<()>, &mut T) -> Option<Poll>,

    /// Polls the status of the current work item.
    ///
    /// The work queue ensures that the item passed here has been first passed to the driver by
    /// `post`.
    ///
    /// This function should be as short as possible.
    pub(crate) poll: fn(NonNull<()>, &mut T) -> Poll,

    /// Attempts to abort processing a work item.
    ///
    /// This function should be as short as possible.
    pub(crate) cancel: fn(NonNull<()>, &mut T),

    /// Called when the driver may be stopped.
    ///
    /// This function should be as short as possible.
    pub(crate) stop: fn(NonNull<()>),
}

impl<T: Sync + Send> VTable<T> {
    pub(crate) const fn noop() -> Self {
        Self {
            post: |_, _| None,
            poll: |_, _| unreachable!(),
            cancel: |_, _| (),
            stop: |_| (),
        }
    }
}

struct Inner<T: Sync + Send> {
    head: Option<NonNull<WorkItem<T>>>,
    tail: Option<NonNull<WorkItem<T>>>,
    current: Option<NonNull<WorkItem<T>>>,

    // The data pointer will be passed to VTable functions, which may be called in any context.
    data: NonNull<()>,
    vtable: VTable<T>,

    // Counts suspend requests. When this reaches 0 again, the all wakers in the queue need to be
    // waken to continue processing.
    suspend_count: usize,
    // The task waiting for the queue to be suspended. There can be multiple tasks, but that's
    // practically rare (in this setup, it needs both HMAC and DSA to want to work at the same
    // time).
    suspend_waker: WakerRegistration,
}

unsafe impl<T: Sync + Send> Send for Inner<T> {}
unsafe impl<T: Sync + Send> Sync for Inner<T> {}

impl<T: Sync + Send> Inner<T> {
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
    /// Returns whether the function should be re-called by the caller.
    fn process(&mut self) -> bool {
        if let Some(mut current) = self.current {
            let poll_result = (self.vtable.poll)(self.data, &mut unsafe { current.as_mut() }.data);

            match poll_result {
                Poll::Ready(status) => {
                    unsafe { current.as_mut() }.complete(status);
                    self.current = None;
                    if self.suspend_count > 0 {
                        // Queue suspended, stop the driver.
                        (self.vtable.stop)(self.data);
                        self.suspend_waker.wake();
                        false
                    } else {
                        self.dequeue_and_post(true)
                    }
                }
                Poll::Pending(recall) => recall,
            }
        } else {
            // If the queue is empty, the driver should already have been notified when the queue
            // became empty, so we don't notify it here.
            self.dequeue_and_post(false)
        }
    }

    /// Retrieves the next work queue item and sends it to the driver.
    ///
    /// Returns true if the queue needs to be polled again.
    // Note: even if the queue itself may be implemented lock-free, dequeuing and posting to the
    // driver must be done atomically to ensure that the queue can be processed fully by any of
    // the frontends polling it.
    fn dequeue_and_post(&mut self, notify_on_empty: bool) -> bool {
        let Some(mut ptr) = self.dequeue() else {
            if notify_on_empty {
                // There are no more work items. Notify the driver that it can stop.
                (self.vtable.stop)(self.data);
            }
            return false;
        };

        // Start processing a new work item.

        if let Some(poll_status) = (self.vtable.post)(self.data, &mut unsafe { ptr.as_mut() }.data)
        {
            match poll_status {
                Poll::Pending(recall) => {
                    unsafe { ptr.as_mut().status = Poll::Pending(recall) };
                    self.current = Some(ptr);
                    recall
                }
                Poll::Ready(status) => {
                    unsafe { ptr.as_mut() }.complete(status);
                    // The driver immediately processed the work item.
                    // Polling again needs to dequeue the next item.
                    true
                }
            }
        } else {
            // If the driver didn't accept the work item, place it back to the front of the
            // queue.
            self.enqueue_front(ptr);
            false
        }
    }

    /// Pops and returns a work item from the start of the queue.
    fn dequeue(&mut self) -> Option<NonNull<WorkItem<T>>> {
        // If the `head` is None, the queue is empty. Return None and do nothing.
        let ptr = self.head?;

        self.head = unsafe { ptr.as_ref() }.next;

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
            (self.vtable.cancel)(
                self.data,
                // This is safe to do, because the work item is currently owned by this queue.
                &mut unsafe { work_item.as_mut() }.data,
            );
            // Queue will need to be polled to query item status.
            return false;
        }

        if unsafe { work_item.as_ref() }.status.is_ready() {
            // Nothing to do.
            return true;
        }

        // The work item is not the current one, remove it from the queue. This immediately
        // cancels the work item. `remove` only uses the address of the work item without
        // dereferencing it.
        if self.remove(work_item) {
            unsafe { work_item.as_mut() }.complete(Status::Cancelled);
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
            let next = unsafe { current_item.as_ref() }.next;

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
                unsafe { unwrap!(prev).as_mut() }.next = next;
            }

            if Some(ptr) == self.tail {
                self.tail = prev;
            }

            return true;
        }

        // Did not find `ptr`.
        false
    }

    /// Increases the suspend counter, preventing new work items from starting to be processed.
    ///
    /// If the current work item finishes processing, the driver is shut down. Call `is_active` to
    /// determine when the queue enters suspended state.
    fn suspend(&mut self, ctx: Option<&Context<'_>>) {
        self.suspend_count += 1;
        if let Some(ctx) = ctx {
            if self.current.is_some() {
                self.suspend_waker.register(ctx.waker());
            } else {
                ctx.waker().wake_by_ref();
            }
        }
    }

    /// Decreases the suspend counter.
    ///
    /// When it reaches 0, this function wakes async tasks that poll the queue. They need to be
    /// waken to ensure that their items don't end up stuck. Blocking pollers will eventually end up
    /// looping when their turn comes.
    fn resume(&mut self) {
        self.suspend_count -= 1;
        if self.suspend_count == 0 {
            self.wake_polling_tasks();
        }
    }

    fn wake_polling_tasks(&mut self) {
        if self.data == NonNull::dangling() {
            // No VTable means no driver, no need to continue processing.
            return;
        }
        // Walk through the list and wake polling tasks.
        let mut current = self.head;
        while let Some(mut current_item) = current {
            let item = unsafe { current_item.as_mut() };

            item.waker.wake();

            current = item.next;
        }
    }

    fn is_active(&self) -> bool {
        self.current.is_some()
    }

    unsafe fn configure(&mut self, data: NonNull<()>, vtable: VTable<T>) {
        (self.vtable.stop)(self.data);

        self.data = data;
        self.vtable = vtable;

        if self.suspend_count == 0 {
            self.wake_polling_tasks();
        }
    }
}

/// A generic work queue.
pub(crate) struct WorkQueue<T: Sync + Send> {
    inner: NonReentrantMutex<Inner<T>>,
}

impl<T: Sync + Send> WorkQueue<T> {
    /// Creates a new `WorkQueue`.
    pub const fn new() -> Self {
        Self {
            inner: NonReentrantMutex::new(Inner {
                head: None,
                tail: None,
                current: None,

                data: NonNull::dangling(),
                vtable: VTable::noop(),

                suspend_count: 0,
                suspend_waker: WakerRegistration::new(),
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
    pub unsafe fn configure<D: Sync + Send>(&self, data: NonNull<D>, vtable: VTable<T>) {
        self.inner
            .with(|inner| unsafe { inner.configure(data.cast(), vtable) })
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
    ///
    /// Returns true if the queue needs to be polled again.
    #[allow(unused)]
    pub fn process(&self) -> bool {
        self.inner.with(|inner| inner.process())
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
            let status = unsafe { &*item.as_ptr() }.status;
            if status.is_pending() {
                inner.process();
                unsafe { &*item.as_ptr() }.status
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
pub(crate) struct WorkItem<T: Sync + Send> {
    next: Option<NonNull<WorkItem<T>>>,
    status: Poll,
    data: T,
    waker: WakerRegistration,
}

impl<T: Sync + Send + Clone> Clone for WorkItem<T> {
    fn clone(&self) -> Self {
        Self {
            // A work item can only be cloned when it's not in a queue.
            next: None,
            status: Poll::Pending(false),
            data: self.data.clone(),
            waker: WakerRegistration::new(),
        }
    }
}

impl<T: Sync + Send> WorkItem<T> {
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
        self.status = Poll::Pending(false);

        NonNull::from(self)
    }
}

/// The status of a work item posted to a work queue.
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum Poll {
    /// The work item has not yet been fully processed. Contains whether the caller should poll the
    /// queue again. This only has effect on async pollers, which will need to wake their tasks
    /// immediately.
    Pending(bool),

    /// The work item has been processed.
    Ready(Status),
}

impl Poll {
    /// Returns whether the current result is still pending.
    pub fn is_pending(self) -> bool {
        matches!(self, Self::Pending(_))
    }

    /// Returns whether the current result is ready.
    pub fn is_ready(self) -> bool {
        !self.is_pending()
    }
}

/// A reference to a posted [`WorkItem`].
///
/// This struct ensures that the work item is valid until the item is processed or is removed from
/// the work queue.
///
/// Dropping the handle cancels the work item, but may block for some time if the work item is
/// already being processed.
pub(crate) struct Handle<'t, T: Sync + Send> {
    queue: &'t WorkQueue<T>,
    work_item: NonNull<WorkItem<T>>,
    // Make sure lifetime is invariant to prevent UB.
    _marker: PhantomData<&'t mut WorkItem<T>>,
}

impl<'t, T: Sync + Send> Handle<'t, T> {
    pub(crate) fn from_completed_work_item(
        queue: &'t WorkQueue<T>,
        work_item: &'t mut WorkItem<T>,
    ) -> Self {
        // Don't use `complete` here, we don't need to wake anything, just ensure that the item will
        // not be put into the queue.
        work_item.status = Poll::Ready(Status::Completed);

        Self {
            queue,
            work_item: NonNull::from(work_item),
            _marker: PhantomData,
        }
    }

    fn poll_inner(&mut self) -> Poll {
        unsafe { self.queue.poll(self.work_item) }
    }

    /// Returns the status of the work item.
    pub fn poll(&mut self) -> bool {
        self.poll_inner().is_ready()
    }

    /// Polls the work item to completion, by busy-looping.
    ///
    /// This function returns immediately if `poll` returns `true`.
    #[inline]
    pub fn wait_blocking(mut self) -> Status {
        loop {
            if let Poll::Ready(status) = self.poll_inner() {
                return status;
            }
        }
    }

    /// Waits until the work item is completed.
    pub fn wait(&mut self) -> impl Future<Output = Status> {
        poll_fn(|ctx| {
            unsafe { self.work_item.as_mut() }
                .waker
                .register(ctx.waker());
            match self.poll_inner() {
                Poll::Pending(recall) => {
                    if recall {
                        ctx.waker().wake_by_ref();
                    }
                    core::task::Poll::Pending
                }
                Poll::Ready(status) => core::task::Poll::Ready(status),
            }
        })
    }

    /// Cancels the work item and asynchronously waits until it is removed from the work queue.
    pub async fn cancel(&mut self) {
        if !self.queue.cancel(self.work_item) {
            self.wait().await;
        }
    }
}

impl<'t, T: Sync + Send> Drop for Handle<'t, T> {
    fn drop(&mut self) {
        if !self.queue.cancel(self.work_item) {
            // We must wait for the driver to release our WorkItem.
            while self.poll_inner().is_pending() {}
        }
    }
}

pub(crate) struct WorkQueueDriver<'t, D, T>
where
    D: Sync + Send,
    T: Sync + Send,
{
    queue: &'t WorkQueue<T>,
    _marker: PhantomData<&'t mut D>,
}

impl<'t, D, T> WorkQueueDriver<'t, D, T>
where
    D: Sync + Send,
    T: Sync + Send,
{
    pub fn new(driver: &'t mut D, vtable: VTable<T>, queue: &'t WorkQueue<T>) -> Self {
        unsafe {
            // Safety: the lifetime 't ensures the pointer remains valid for the lifetime of the
            // WorkQueueDriver. The Drop implementation (and the general "Don't forget" clause)
            // ensure the pointer is not used after the WQD has been dropped.
            queue.configure(NonNull::from(driver), vtable);
        }
        Self {
            queue,
            _marker: PhantomData,
        }
    }

    /// Shuts down the driver.
    pub fn stop(self) -> impl Future<Output = ()> {
        let mut suspended = false;
        poll_fn(move |ctx| {
            self.queue.inner.with(|inner| {
                if !inner.is_active() {
                    unsafe {
                        // Safety: the noop VTable functions don't use the pointer at all.
                        self.queue
                            .configure(NonNull::<D>::dangling(), VTable::noop())
                    };
                    // Make sure the queue doesn't remain suspended when the driver is re-started.
                    if suspended {
                        inner.resume();
                    }
                    return core::task::Poll::Ready(());
                }
                // This may kick out other suspend() callers, but that should be okay. They will
                // only be able to do work if the queue is !active, for them it doesn't matter if
                // the queue is suspended or stopped completely - just that it isn't running. As for
                // the possible waker churn, we can use MultiWakerRegistration with a capacity
                // suitable for the number of possible suspenders (2-3 unless the work queue ends up
                // being used more widely), if this turns out to be a problem.
                inner.suspend_waker.register(ctx.waker());
                if !suspended {
                    inner.suspend(Some(ctx));
                    suspended = true;
                }

                core::task::Poll::Pending
            })
        })
    }
}

impl<D, T> Drop for WorkQueueDriver<'_, D, T>
where
    D: Sync + Send,
    T: Sync + Send,
{
    fn drop(&mut self) {
        let wait_for_suspended = self.queue.inner.with(|inner| {
            if inner.is_active() {
                inner.suspend(None);
                true
            } else {
                unsafe { inner.configure(NonNull::dangling(), VTable::noop()) };
                false
            }
        });

        if !wait_for_suspended {
            return;
        }

        loop {
            let done = self.queue.inner.with(|inner| {
                if inner.is_active() {
                    return false;
                }

                unsafe { inner.configure(NonNull::dangling(), VTable::noop()) };

                inner.resume();

                true
            });
            if done {
                break;
            }
        }
    }
}

/// Used by work queue clients, allows hiding WorkItem.
#[derive(Clone)]
pub(crate) struct WorkQueueFrontend<T: Sync + Send> {
    work_item: WorkItem<T>,
}

impl<T: Sync + Send> WorkQueueFrontend<T> {
    pub fn new(initial: T) -> Self {
        Self {
            work_item: WorkItem {
                next: None,
                status: Poll::Pending(false),
                data: initial,
                waker: WakerRegistration::new(),
            },
        }
    }

    pub fn data_mut(&mut self) -> &mut T {
        &mut self.work_item.data
    }

    pub fn post<'t>(&'t mut self, queue: &'t WorkQueue<T>) -> Handle<'t, T> {
        queue.post_work(&mut self.work_item)
    }

    /// Creates a Handle for a work item that does not need to be put into the queue.
    pub fn post_completed<'t>(&'t mut self, queue: &'t WorkQueue<T>) -> Handle<'t, T> {
        Handle::from_completed_work_item(queue, &mut self.work_item)
    }
}
