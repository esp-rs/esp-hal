//! The three cJSON functions MultiNet uses to read the layer settings that its model embeds as a
//! JSON document: `cJSON_Parse`, `cJSON_GetObjectItem` and `cJSON_Delete`.
//!
//! The nodes have the layout of cJSON 1.7, which MultiNet reads directly.

use core::{
    ffi::{CStr, c_char, c_int},
    ptr,
};

use crate::memory::{boxed, deallocate};

/// `cJSON`, one value of a document.
#[repr(C)]
pub struct Node {
    /// The next and previous value in the parent. The first value's `prev` is the last value.
    next: *mut Node,
    prev: *mut Node,
    /// The first value of an array or an object.
    child: *mut Node,
    kind: c_int,
    valuestring: *mut c_char,
    /// The number, saturated to `int`.
    valueint: c_int,
    valuedouble: f64,
    /// The key in the parent object.
    string: *mut c_char,
}

const FALSE: c_int = 1 << 0;
const TRUE: c_int = 1 << 1;
const NULL: c_int = 1 << 2;
const NUMBER: c_int = 1 << 3;
const STRING: c_int = 1 << 4;
const ARRAY: c_int = 1 << 5;
const OBJECT: c_int = 1 << 6;
const IS_REFERENCE: c_int = 1 << 8;
const STRING_IS_CONST: c_int = 1 << 9;

/// cJSON's limit of nested arrays and objects.
const NESTING_LIMIT: usize = 1000;

impl Node {
    fn new(kind: c_int) -> *mut Node {
        boxed(Node {
            next: ptr::null_mut(),
            prev: ptr::null_mut(),
            child: ptr::null_mut(),
            kind,
            valuestring: ptr::null_mut(),
            valueint: 0,
            valuedouble: 0.0,
            string: ptr::null_mut(),
        })
    }
}

/// A recursive-descent parser over the document.
struct Parser<'a> {
    text: &'a [u8],
    at: usize,
}

impl Parser<'_> {
    fn skip_whitespace(&mut self) {
        while self.text.get(self.at).is_some_and(u8::is_ascii_whitespace) {
            self.at += 1;
        }
    }

    fn eat(&mut self, literal: &[u8]) -> bool {
        let found = self.text[self.at..].starts_with(literal);
        if found {
            self.at += literal.len();
        }
        found
    }

    /// Parses a value into a new node, or returns null.
    fn value(&mut self, depth: usize) -> *mut Node {
        if depth > NESTING_LIMIT {
            return ptr::null_mut();
        }
        self.skip_whitespace();
        match self.text.get(self.at) {
            Some(b'{') => self.container(depth, OBJECT, b'}'),
            Some(b'[') => self.container(depth, ARRAY, b']'),
            Some(b'"') => {
                let node = Node::new(STRING);
                let string = self.string();
                // SAFETY: `node` is null or a new node.
                unsafe { finish(node, string, |node, string| (*node).valuestring = string) }
            }
            Some(b'-' | b'0'..=b'9') => self.number(),
            _ if self.eat(b"true") => Node::new(TRUE),
            _ if self.eat(b"false") => Node::new(FALSE),
            _ if self.eat(b"null") => Node::new(NULL),
            _ => ptr::null_mut(),
        }
    }

    fn number(&mut self) -> *mut Node {
        let (number, length) = crate::libc::parse_float(&self.text[self.at..]);
        if length == 0 {
            return ptr::null_mut();
        }
        self.at += length;
        let node = Node::new(NUMBER);
        if !node.is_null() {
            // SAFETY: `node` is a new node.
            unsafe {
                (*node).valuedouble = number;
                // `as` saturates, as cJSON does.
                (*node).valueint = number as c_int;
            }
        }
        node
    }

    /// Parses a string in quotes into a new NUL-terminated string, or returns null.
    fn string(&mut self) -> *mut c_char {
        if !self.eat(b"\"") {
            return ptr::null_mut();
        }
        let mut bytes = alloc_free::Bytes::default();
        loop {
            let Some(&byte) = self.text.get(self.at) else {
                return ptr::null_mut();
            };
            self.at += 1;
            match byte {
                b'"' => return bytes.into_c_string(),
                b'\\' => {
                    let Some(&escape) = self.text.get(self.at) else {
                        return ptr::null_mut();
                    };
                    self.at += 1;
                    let byte = match escape {
                        b'b' => 0x08,
                        b'f' => 0x0C,
                        b'n' => b'\n',
                        b'r' => b'\r',
                        b't' => b'\t',
                        b'u' => {
                            let Some(c) = self.unicode_escape() else {
                                return ptr::null_mut();
                            };
                            let mut utf8 = [0; 4];
                            if !bytes.extend(c.encode_utf8(&mut utf8).as_bytes()) {
                                return ptr::null_mut();
                            }
                            continue;
                        }
                        other => other,
                    };
                    if !bytes.extend(&[byte]) {
                        return ptr::null_mut();
                    }
                }
                byte => {
                    if !bytes.extend(&[byte]) {
                        return ptr::null_mut();
                    }
                }
            }
        }
    }

    /// Parses the hex digits of a `\u` escape, and a second one for a surrogate pair.
    fn unicode_escape(&mut self) -> Option<char> {
        let hex = |parser: &mut Self| {
            let digits = parser.text.get(parser.at..parser.at + 4)?;
            parser.at += 4;
            u32::from_str_radix(core::str::from_utf8(digits).ok()?, 16).ok()
        };
        let first = hex(self)?;
        if (0xD800..0xDC00).contains(&first) {
            if !self.eat(b"\\u") {
                return None;
            }
            let second = hex(self)?;
            if !(0xDC00..0xE000).contains(&second) {
                return None;
            }
            char::from_u32(0x10000 + ((first - 0xD800) << 10) + (second - 0xDC00))
        } else {
            char::from_u32(first)
        }
    }

    /// Parses an array or an object into a new node, or returns null.
    fn container(&mut self, depth: usize, kind: c_int, close: u8) -> *mut Node {
        self.at += 1;
        let node = Node::new(kind);
        if node.is_null() {
            return node;
        }
        let mut last: *mut Node = ptr::null_mut();
        self.skip_whitespace();
        if self.eat(&[close]) {
            return node;
        }
        loop {
            let mut key = ptr::null_mut();
            if kind == OBJECT {
                self.skip_whitespace();
                key = self.string();
                self.skip_whitespace();
                if key.is_null() || !self.eat(b":") {
                    // SAFETY: `key` is null or a new string, `node` a node of this parser.
                    unsafe {
                        deallocate(key.cast());
                        cJSON_Delete(node);
                    }
                    return ptr::null_mut();
                }
            }
            let child = self.value(depth + 1);
            // SAFETY: `child` is null or a new node, `node` and `last` nodes of this parser.
            unsafe {
                if child.is_null() {
                    deallocate(key.cast());
                    cJSON_Delete(node);
                    return ptr::null_mut();
                }
                (*child).string = key;
                if last.is_null() {
                    (*node).child = child;
                } else {
                    (*last).next = child;
                    (*child).prev = last;
                }
                (*(*node).child).prev = child;
            }
            last = child;
            self.skip_whitespace();
            if self.eat(b",") {
                continue;
            }
            if self.eat(&[close]) {
                return node;
            }
            // SAFETY: `node` is a node of this parser.
            unsafe { cJSON_Delete(node) };
            return ptr::null_mut();
        }
    }
}

/// Completes `node` with `value`, or frees both if either is missing.
///
/// # Safety
///
/// `node` and `value` must be null or new allocations.
unsafe fn finish<V>(
    node: *mut Node,
    value: *mut V,
    set: impl FnOnce(*mut Node, *mut V),
) -> *mut Node {
    unsafe {
        if node.is_null() || value.is_null() {
            deallocate(value.cast());
            if !node.is_null() {
                cJSON_Delete(node);
            }
            return ptr::null_mut();
        }
    }
    set(node, value);
    node
}

mod alloc_free {
    //! A growing byte buffer in [`crate::memory`], for strings of unknown length.

    use core::{ffi::c_char, ptr};

    use crate::memory::{allocate, allocation_size, deallocate};

    pub struct Bytes {
        start: *mut u8,
        length: usize,
    }

    impl Default for Bytes {
        fn default() -> Self {
            Self {
                start: ptr::null_mut(),
                length: 0,
            }
        }
    }

    impl Bytes {
        /// Appends `bytes`, or returns `false` without memory.
        pub fn extend(&mut self, bytes: &[u8]) -> bool {
            // One more byte for the NUL.
            let needed = self.length + bytes.len() + 1;
            // SAFETY: `start` is null or comes from `allocate`.
            let capacity = if self.start.is_null() {
                0
            } else {
                unsafe { allocation_size(self.start) }
            };
            if needed > capacity {
                let grown = allocate(needed.max(2 * capacity).max(16), 1);
                if grown.is_null() {
                    return false;
                }
                // SAFETY: Both buffers hold `length` bytes, and `start` is not used again.
                unsafe {
                    if !self.start.is_null() {
                        ptr::copy_nonoverlapping(self.start, grown, self.length);
                        deallocate(self.start);
                    }
                }
                self.start = grown;
            }
            // SAFETY: The buffer has room for the bytes.
            unsafe {
                ptr::copy_nonoverlapping(bytes.as_ptr(), self.start.add(self.length), bytes.len())
            };
            self.length += bytes.len();
            true
        }

        /// Returns the bytes as a NUL-terminated string, or null without memory.
        pub fn into_c_string(mut self) -> *mut c_char {
            if self.start.is_null() && !self.extend(&[]) {
                return ptr::null_mut();
            }
            // SAFETY: `extend` left room for the NUL.
            unsafe { self.start.add(self.length).write(0) };
            let start = self.start;
            self.start = ptr::null_mut();
            start.cast()
        }
    }

    impl Drop for Bytes {
        fn drop(&mut self) {
            // SAFETY: `start` is null or comes from `allocate`.
            unsafe { deallocate(self.start) };
        }
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn cJSON_Parse(text: *const c_char) -> *mut Node {
    if text.is_null() {
        return ptr::null_mut();
    }
    // SAFETY: C callers pass a NUL-terminated string.
    let text = unsafe { CStr::from_ptr(text) }.to_bytes();
    let mut parser = Parser { text, at: 0 };
    let node = parser.value(0);
    parser.skip_whitespace();
    if !node.is_null() && parser.at != text.len() {
        // Trailing garbage. cJSON_Parse does not require the end, but nothing passes any.
        log::debug!(
            "cJSON_Parse ignored {} trailing bytes",
            text.len() - parser.at
        );
    }
    node
}

/// Finds the value under `name` in `object`, ignoring ASCII case, as cJSON does.
#[unsafe(no_mangle)]
unsafe extern "C" fn cJSON_GetObjectItem(object: *const Node, name: *const c_char) -> *mut Node {
    if object.is_null() || name.is_null() {
        return ptr::null_mut();
    }
    // SAFETY: C callers pass a node from `cJSON_Parse` and a NUL-terminated name.
    unsafe {
        let name = CStr::from_ptr(name).to_bytes();
        let mut child = (*object).child;
        while !child.is_null() {
            let key = (*child).string;
            if !key.is_null() && CStr::from_ptr(key).to_bytes().eq_ignore_ascii_case(name) {
                return child;
            }
            child = (*child).next;
        }
    }
    ptr::null_mut()
}

/// Frees `node`, its children and the values after it, as cJSON does.
#[unsafe(no_mangle)]
unsafe extern "C" fn cJSON_Delete(mut node: *mut Node) {
    // SAFETY: C callers pass null or nodes from `cJSON_Parse`.
    unsafe {
        while !node.is_null() {
            let next = (*node).next;
            if (*node).kind & IS_REFERENCE == 0 {
                cJSON_Delete((*node).child);
                deallocate((*node).valuestring.cast());
            }
            if (*node).kind & STRING_IS_CONST == 0 {
                deallocate((*node).string.cast());
            }
            deallocate(node.cast());
            node = next;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    unsafe fn get<'a>(object: *const Node, name: &CStr) -> &'a Node {
        unsafe { &*cJSON_GetObjectItem(object, name.as_ptr()) }
    }

    #[test]
    fn layer_settings() {
        let text = c"{\"conv1d0\": {\"name\": \"conv1d0\", \"size\": 1, \"out_mbit\": 8, \"scale\": -2.5e1},
            \"list\": [1, \"t\\u00e9\\n\", true, null, {}], \"empty\": []}";
        unsafe {
            let root = cJSON_Parse(text.as_ptr());
            assert!(!root.is_null());
            let conv = get(root, c"CONV1D0");
            assert_eq!(CStr::from_ptr(get(conv, c"name").valuestring), c"conv1d0");
            assert_eq!(get(conv, c"out_mbit").valueint, 8);
            assert_eq!(get(conv, c"scale").valuedouble, -25.0);
            let list = get(root, c"list");
            let first = &*list.child;
            assert_eq!(first.valueint, 1);
            assert_eq!(
                CStr::from_ptr((*first.next).valuestring).to_bytes(),
                "té\n".as_bytes()
            );
            assert_eq!((*(*first.next).next).kind, TRUE);
            assert_eq!(first.prev, (*(*(*first.next).next).next).next);
            assert!(get(root, c"empty").child.is_null());
            assert!(cJSON_GetObjectItem(root, c"missing".as_ptr()).is_null());
            cJSON_Delete(root);
        }
    }

    #[test]
    fn invalid_documents() {
        for text in [
            c"{\"a\": 1",
            c"{\"a\" 1}",
            c"[1,]x",
            c"\"open",
            c"{1: 2}",
            c"nope",
        ] {
            let root = unsafe { cJSON_Parse(text.as_ptr()) };
            assert!(root.is_null(), "{text:?} should not parse");
        }
    }
}
