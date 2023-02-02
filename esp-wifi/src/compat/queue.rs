#![allow(unused)]

use core::mem::MaybeUninit;

pub struct SimpleQueue<T, const N: usize>
where
    T: Copy,
{
    data: [Option<T>; N],
    read_index: usize,
    write_index: usize,
}

impl<T, const N: usize> SimpleQueue<T, N>
where
    T: Copy,
{
    pub const fn new() -> SimpleQueue<T, N> {
        let mut queue = unsafe {
            SimpleQueue {
                data: [None; N],
                read_index: 0,
                write_index: 0,
            }
        };

        queue
    }

    pub fn enqueue(&mut self, e: T) -> bool {
        self.data[self.write_index] = Some(e);

        self.write_index += 1;
        self.write_index %= N;

        if self.write_index == self.read_index {
            return false;
        }

        true
    }

    pub fn dequeue(&mut self) -> Option<T> {
        if self.write_index == self.read_index {
            None
        } else {
            let result = self.data[self.read_index].take();
            self.read_index += 1;
            self.read_index %= N;
            result
        }
    }

    pub fn is_empty(&self) -> bool {
        self.read_index == self.write_index
    }

    pub fn is_full(&self) -> bool {
        let mut next_write = self.read_index + 1;
        next_write %= N;

        next_write == self.read_index
    }
}
