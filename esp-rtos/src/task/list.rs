use core::{marker::PhantomData, ptr::NonNull};

use super::{Task, TaskPtr};

/// An abstraction that allows the task to contain multiple different queue pointers.
pub(crate) trait TaskListElement: Default {
    /// Returns the pointer to the next element in the list.
    fn next(task: TaskPtr) -> Option<TaskPtr>;

    /// Sets the pointer to the next element in the list.
    fn set_next(task: TaskPtr, next: Option<TaskPtr>);

    /// Returns whether the task is in the list. If this function returns `None`, we don't know.
    fn is_in_queue(_task: TaskPtr) -> Option<bool> {
        // By default we don't store this information, so we return "Don't know".
        None
    }

    /// Marks whether the task is in the list.
    fn mark_in_queue(_task: TaskPtr, _in_queue: bool) {}
}

macro_rules! task_list_item {
    ($struct:ident, $field:ident $(, $in_queue_field:ident)?) => {
        #[derive(Default)]
        pub(crate) struct $struct;
        impl TaskListElement for $struct {
            fn next(task: TaskPtr) -> Option<TaskPtr> {
                unsafe { task.as_ref().$field }
            }

            fn set_next(mut task: TaskPtr, next: Option<TaskPtr>) {
                unsafe {
                    task.as_mut().$field = next;
                }
            }

            $(
                fn is_in_queue(task: TaskPtr) -> Option<bool> {
                    Some(unsafe { task.as_ref().$in_queue_field })
                }

                fn mark_in_queue(mut task: TaskPtr, in_queue: bool) {
                    unsafe {
                        task.as_mut().$in_queue_field = in_queue;
                    }
                }
            )?
        }
    };
}

/// A singly linked list of tasks.
///
/// Use this where you don't care about the order of list elements.
///
/// The `E` type parameter is used to access the data in the task object that belongs to this list.
#[derive(Default)]
pub(crate) struct TaskList<E> {
    head: Option<TaskPtr>,
    _item: PhantomData<E>,
}

impl<E: TaskListElement> TaskList<E> {
    pub const fn new() -> Self {
        Self {
            head: None,
            _item: PhantomData,
        }
    }

    pub fn push(&mut self, task: TaskPtr) {
        if E::is_in_queue(task) == Some(true) {
            return;
        }
        E::mark_in_queue(task, true);

        debug_assert!(E::next(task).is_none());
        E::set_next(task, self.head);
        self.head = Some(task);
    }

    pub fn pop(&mut self) -> Option<TaskPtr> {
        let popped = self.head.take();

        if let Some(task) = popped {
            self.head = E::next(task);
            E::set_next(task, None);
            E::mark_in_queue(task, false);
        }

        popped
    }

    pub fn remove(&mut self, task: TaskPtr) {
        if E::is_in_queue(task) == Some(false) {
            return;
        }

        // TODO: maybe this (and TaskQueue::remove) may prove too expensive.
        let mut list = core::mem::take(self);
        while let Some(popped) = list.pop() {
            if popped != task {
                self.push(popped);
            } else {
                E::mark_in_queue(task, false);
            }
        }
    }

    #[cfg(feature = "rtos-trace")]
    pub fn iter(&self) -> impl Iterator<Item = TaskPtr> {
        let mut current = self.head;
        core::iter::from_fn(move || {
            let task = current?;
            current = E::next(task);
            Some(task)
        })
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.head.is_none()
    }
}

/// A singly linked queue of tasks.
///
/// Use this where you care about the order of list elements. Elements are popped from the front,
/// and pushed to the back.
///
/// The `E` type parameter is used to access the data in the task object that belongs to this list.
#[derive(Default)]
pub(crate) struct TaskQueue<E> {
    head: Option<TaskPtr>,
    tail: Option<TaskPtr>,
    _item: PhantomData<E>,
}

impl<E: TaskListElement> TaskQueue<E> {
    pub const fn new() -> Self {
        Self {
            head: None,
            tail: None,
            _item: PhantomData,
        }
    }

    pub fn push(&mut self, task: TaskPtr) {
        if E::is_in_queue(task) == Some(true) {
            return;
        }
        E::mark_in_queue(task, true);

        debug_assert!(E::next(task).is_none());
        if let Some(tail) = self.tail {
            E::set_next(tail, Some(task));
        } else {
            self.head = Some(task);
        }
        self.tail = Some(task);
    }

    pub fn pop(&mut self) -> Option<TaskPtr> {
        let popped = self.head.take();

        if let Some(task) = popped {
            self.head = E::next(task);
            E::set_next(task, None);
            if self.head.is_none() {
                self.tail = None;
            }
            E::mark_in_queue(task, false);
        }

        popped
    }

    pub fn pop_if(&mut self, cond: impl Fn(&Task) -> bool) -> Option<TaskPtr> {
        let mut popped = None;

        let mut list = core::mem::take(self);
        while let Some(task) = list.pop() {
            if popped.is_none() && cond(unsafe { task.as_ref() }) {
                E::mark_in_queue(task, false);
                popped = Some(task);
            } else {
                self.push(task);
            }
        }

        popped
    }

    pub fn remove(&mut self, task: TaskPtr) {
        if E::is_in_queue(task) == Some(false) {
            return;
        }

        _ = self.pop_if(|t| NonNull::from(t) == task);
    }

    #[cfg(multi_core)]
    pub fn any(&self, cond: impl Fn(&Task) -> bool) -> bool {
        let mut current = self.head;
        while let Some(task) = current {
            if cond(unsafe { task.as_ref() }) {
                return true;
            }
            current = E::next(task);
        }
        false
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.head.is_none()
    }
}
