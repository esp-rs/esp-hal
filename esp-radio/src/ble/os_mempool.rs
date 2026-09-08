//! NimBLE `os_mempool` implementation.
//!
//! Port of IDF `components/bt/porting/mem/os_mempool.c` without
//! poison/guard/runtime-alloc. The BLE blob calls these unprefixed symbols.
//!
//! On C2, ROM provides `r_os_mempool_*` and `g_os_mempool_list`. This module
//! exports the unprefixed names against a private list, so ROM msys
//! (`r_os_msys_*`) and blob calls to `os_mempool_*` do not share a list.

use core::ptr;

use crate::ble::npl::OsMempool;

const OS_OK: i32 = 0;
const OS_INVALID_PARM: i32 = 3;
const OS_MEM_NOT_ALIGNED: i32 = 4;
const OS_ALIGNMENT: usize = 4;
const OS_MEMPOOL_F_EXT: u8 = 1;
const OS_MEMPOOL_F_FRAG: u8 = 8;
const OS_MEMPOOL_F_COMBINATION: u8 = 128;

type OsMempoolPutFn =
    unsafe extern "C" fn(mpe: *mut OsMempoolExt, data: *mut u8, arg: *mut u8) -> i32;
type OsMempoolGetFn = unsafe extern "C" fn(mpe: *mut OsMempoolExt, arg: *mut u8) -> *mut u8;

#[repr(C)]
pub(crate) struct OsMempoolExt {
    mp: OsMempool,
    put_cb: Option<OsMempoolPutFn>,
    put_arg: *mut u8,
    get_cb: Option<OsMempoolGetFn>,
    get_arg: *mut u8,
}

#[repr(C)]
struct OsMemblock {
    next: *mut OsMemblock,
}

static mut G_OS_MEMPOOL_LIST: *mut OsMempool = ptr::null_mut();

fn true_block_size(block_size: u32) -> usize {
    (block_size as usize + OS_ALIGNMENT - 1) & !(OS_ALIGNMENT - 1)
}

// IDF compares the name *pointer* (re-init of the same pool), not the string.
unsafe fn list_contains_name(name: *const u8) -> *mut OsMempool {
    let mut cur = unsafe { G_OS_MEMPOOL_LIST };
    while !cur.is_null() {
        if unsafe { (*cur).name } == name {
            return cur;
        }
        cur = unsafe { (*cur).next.cast_mut() };
    }
    ptr::null_mut()
}

unsafe fn list_insert_tail(mp: *mut OsMempool) {
    unsafe { (*mp).next = ptr::null() };
    let mut cur = unsafe { G_OS_MEMPOOL_LIST };
    if cur.is_null() {
        unsafe { G_OS_MEMPOOL_LIST = mp };
        return;
    }
    loop {
        let next = unsafe { (*cur).next };
        if next.is_null() {
            unsafe { (*cur).next = mp };
            break;
        }
        cur = next.cast_mut();
    }
}

unsafe fn chain_blocks(mp: *mut OsMempool, membuf: *mut u8, blocks: u16) {
    if blocks == 0 {
        unsafe { (*mp).first = ptr::null() };
        return;
    }

    let stride = true_block_size(unsafe { (*mp).mp_block_size });
    unsafe { (*mp).first = membuf.cast() };
    let mut block = membuf.cast::<OsMemblock>();
    for i in 1..blocks {
        let next = unsafe { membuf.add(i as usize * stride) }.cast::<OsMemblock>();
        unsafe { (*block).next = next };
        block = next;
    }
    unsafe { (*block).next = ptr::null_mut() };
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_mempool_init_internal(
    mp: *mut OsMempool,
    blocks: u16,
    block_size: u32,
    membuf: *mut u8,
    name: *const u8,
    flags: u8,
) -> i32 {
    if mp.is_null() || block_size == 0 {
        return OS_INVALID_PARM;
    }
    if membuf.is_null() && blocks != 0 {
        return OS_INVALID_PARM;
    }
    if !membuf.is_null() && !(membuf as usize).is_multiple_of(OS_ALIGNMENT) {
        return OS_MEM_NOT_ALIGNED;
    }

    unsafe {
        (*mp).mp_block_size = block_size;
        (*mp).mp_num_free = blocks;
        (*mp).mp_min_free = blocks;
        (*mp).mp_flags = flags;
        (*mp).mp_num_blocks = blocks;
        (*mp).mp_membuf_addr = membuf as u32;
        (*mp).name = name;
        chain_blocks(mp, membuf, blocks);

        let existing = list_contains_name(name);
        if !existing.is_null() {
            let _ = os_mempool_unregister(existing);
        }
        list_insert_tail(mp);
    }
    OS_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_mempool_init(
    mp: *mut OsMempool,
    blocks: u16,
    block_size: u32,
    membuf: *mut u8,
    name: *const u8,
) -> i32 {
    unsafe { os_mempool_init_internal(mp, blocks, block_size, membuf, name, 0) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_mempool_ext_init(
    mpe: *mut OsMempoolExt,
    blocks: u16,
    block_size: u32,
    membuf: *mut u8,
    name: *const u8,
) -> i32 {
    if mpe.is_null() {
        return OS_INVALID_PARM;
    }
    let rc = unsafe {
        os_mempool_init_internal(
            ptr::addr_of_mut!((*mpe).mp),
            blocks,
            block_size,
            membuf,
            name,
            OS_MEMPOOL_F_EXT,
        )
    };
    if rc != 0 {
        return rc;
    }
    unsafe {
        (*mpe).put_cb = None;
        (*mpe).put_arg = ptr::null_mut();
        (*mpe).get_cb = None;
        (*mpe).get_arg = ptr::null_mut();
    }
    OS_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_ext_mempool_register_cb(
    mpe: *mut OsMempoolExt,
    put_cb: Option<OsMempoolPutFn>,
    put_arg: *mut u8,
    get_cb: Option<OsMempoolGetFn>,
    get_arg: *mut u8,
) {
    if mpe.is_null() {
        return;
    }
    unsafe {
        (*mpe).put_cb = put_cb;
        (*mpe).put_arg = put_arg;
        (*mpe).get_cb = get_cb;
        (*mpe).get_arg = get_arg;
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_mempool_unregister(mp: *mut OsMempool) -> i32 {
    if mp.is_null() {
        return OS_INVALID_PARM;
    }

    let mut prev: *mut OsMempool = ptr::null_mut();
    let mut cur = unsafe { G_OS_MEMPOOL_LIST };
    while !cur.is_null() {
        if cur == mp {
            let next = unsafe { (*cur).next };
            if prev.is_null() {
                unsafe { G_OS_MEMPOOL_LIST = next.cast_mut() };
            } else {
                unsafe { (*prev).next = next };
            }
            unsafe { (*mp).next = ptr::null() };
            return OS_OK;
        }
        prev = cur;
        cur = unsafe { (*cur).next.cast_mut() };
    }
    OS_INVALID_PARM
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_mempool_clear(mp: *mut OsMempool) -> i32 {
    if mp.is_null() {
        return OS_INVALID_PARM;
    }
    unsafe {
        (*mp).mp_num_free = (*mp).mp_num_blocks;
        (*mp).mp_min_free = (*mp).mp_num_blocks;
        chain_blocks(mp, (*mp).mp_membuf_addr as *mut u8, (*mp).mp_num_blocks);
    }
    OS_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_mempool_flags_set(mp: *mut OsMempool, flags: u8) {
    if !mp.is_null() {
        unsafe { (*mp).mp_flags |= flags };
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_memblock_from(mp: *const OsMempool, block: *const u8) -> i32 {
    if mp.is_null() || block.is_null() {
        return 0;
    }
    let start = unsafe { (*mp).mp_membuf_addr } as usize;
    let stride = true_block_size(unsafe { (*mp).mp_block_size });
    let end = start + unsafe { (*mp).mp_num_blocks } as usize * stride;
    let addr = block as usize;
    if addr < start || addr >= end {
        return 0;
    }
    if unsafe { (*mp).mp_flags } & OS_MEMPOOL_F_COMBINATION == 0
        && !(addr - start).is_multiple_of(stride)
    {
        return 0;
    }
    1
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_mempool_ext_clear(mpe: *mut OsMempoolExt) -> i32 {
    if mpe.is_null() {
        return OS_INVALID_PARM;
    }
    unsafe {
        (*mpe).put_cb = None;
        (*mpe).put_arg = ptr::null_mut();
        (*mpe).get_cb = None;
        (*mpe).get_arg = ptr::null_mut();
        let rc = os_mempool_clear(ptr::addr_of_mut!((*mpe).mp));
        (*mpe).mp.mp_flags = 0;
        rc
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_mempool_is_sane(mp: *const OsMempool) -> bool {
    if mp.is_null() {
        return false;
    }
    let mut block = unsafe { (*mp).first };
    while !block.is_null() {
        if unsafe { os_memblock_from(mp, block.cast()) } == 0 {
            return false;
        }
        block = unsafe { (*block.cast::<OsMemblock>()).next.cast() };
    }
    true
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_mempool_module_init() {}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_memblock_get(mp: *mut OsMempool) -> *mut u8 {
    if mp.is_null() {
        return ptr::null_mut();
    }

    if unsafe { (*mp).mp_flags } & OS_MEMPOOL_F_EXT != 0 {
        let mpe = mp.cast::<OsMempoolExt>();
        if let Some(cb) = unsafe { (*mpe).get_cb } {
            return unsafe { cb(mpe, (*mpe).get_arg) };
        }
    }

    let token = unsafe { crate::ESP_RADIO_LOCK.acquire() };
    let block = unsafe {
        if (*mp).mp_num_free == 0 {
            ptr::null_mut()
        } else {
            let block = (*mp).first.cast_mut();
            (*mp).first = (*block.cast::<OsMemblock>()).next.cast();
            (*mp).mp_num_free -= 1;
            if (*mp).mp_min_free > (*mp).mp_num_free {
                (*mp).mp_min_free = (*mp).mp_num_free;
            }
            block.cast()
        }
    };
    unsafe { crate::ESP_RADIO_LOCK.release(token) };
    block
}

unsafe fn memblock_put_from_cb(mp: *mut OsMempool, block_addr: *mut u8) -> i32 {
    if unsafe { (*mp).mp_flags } & OS_MEMPOOL_F_FRAG == 0
        && unsafe { os_memblock_from(mp, block_addr) } == 0
    {
        return OS_INVALID_PARM;
    }

    let token = unsafe { crate::ESP_RADIO_LOCK.acquire() };
    let rc = unsafe {
        let mut cur = (*mp).first.cast::<OsMemblock>();
        let mut dup = false;
        while !cur.is_null() {
            if cur.cast::<u8>() == block_addr {
                dup = true;
                break;
            }
            cur = (*cur).next;
        }
        if dup || (*mp).mp_num_free >= (*mp).mp_num_blocks {
            OS_INVALID_PARM
        } else {
            let block = block_addr.cast::<OsMemblock>();
            (*block).next = (*mp).first.cast_mut().cast();
            (*mp).first = block.cast();
            (*mp).mp_num_free += 1;
            OS_OK
        }
    };
    unsafe { crate::ESP_RADIO_LOCK.release(token) };
    rc
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_memblock_put(mp: *mut OsMempool, block_addr: *mut u8) -> i32 {
    if mp.is_null() || block_addr.is_null() {
        return OS_INVALID_PARM;
    }

    if unsafe { (*mp).mp_flags } & OS_MEMPOOL_F_EXT != 0 {
        let mpe = mp.cast::<OsMempoolExt>();
        if let Some(cb) = unsafe { (*mpe).put_cb } {
            return unsafe { cb(mpe, block_addr, (*mpe).put_arg) };
        }
    }

    unsafe { memblock_put_from_cb(mp, block_addr) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn os_memblock_put_from_cb(mp: *mut OsMempool, block_addr: *mut u8) -> i32 {
    if mp.is_null() || block_addr.is_null() {
        return OS_INVALID_PARM;
    }
    unsafe { memblock_put_from_cb(mp, block_addr) }
}
