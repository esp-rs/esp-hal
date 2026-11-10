use std::{
    ptr,
    sync::{
        Mutex,
        atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering},
    },
    thread,
};

use super::queue::{self, *};

const PRO: u32 = 0;
const APP: u32 = 1;
const SEQ_MASK: u32 = (1 << 27) - 1;
const BUSY: Result<u32, u32> = Err(POST_BUSY);

fn decode(value: u32) -> Result<u32, u32> {
    match value & POST_RESULT_MASK {
        POST_OK => Ok(value >> SEQ_SHIFT),
        code => Err(code),
    }
}

fn post(lists: &Lists, core: u32, request: &IpcRequest, arg: usize) -> Result<u32, u32> {
    decode(unsafe { queue::post(lists, core, request, arg, || {}) })
}

fn post_raising(
    lists: &Lists,
    core: u32,
    request: &IpcRequest,
    arg: usize,
    raises: &AtomicUsize,
) -> Result<u32, u32> {
    decode(unsafe {
        queue::post(lists, core, request, arg, || {
            raises.fetch_add(1, Ordering::SeqCst);
        })
    })
}

fn poll(request: &IpcRequest, seq: u32) -> Option<u32> {
    queue::poll(request, seq << SEQ_SHIFT)
}

fn seq_reached(current: u32, target: u32) -> bool {
    queue::seq_reached(current << SEQ_SHIFT, target << SEQ_SHIFT)
}

fn drain(lists: &Lists, core: u32) {
    unsafe { queue::drain(lists, core) }
}

fn flags(request: &IpcRequest) -> u32 {
    request.word().load(Ordering::SeqCst) & (QUEUED | RUNNING | ARMED)
}

fn status_seq(request: &IpcRequest) -> u32 {
    request.status().load(Ordering::SeqCst) >> SEQ_SHIFT
}

fn to_ptr(request: &IpcRequest) -> *mut IpcRequest {
    ptr::from_ref(request).cast_mut()
}

extern "C" fn nothing(_: usize) {}

#[test]
fn second_post_with_same_arg_joins() {
    static LISTS: Lists = Lists::new();
    static RUNS: AtomicUsize = AtomicUsize::new(0);
    extern "C" fn count(_: usize) {
        RUNS.fetch_add(1, Ordering::SeqCst);
    }
    static REQUEST: IpcRequest = IpcRequest::new(count);

    let raises = AtomicUsize::new(0);
    assert_eq!(post_raising(&LISTS, PRO, &REQUEST, 5, &raises), Ok(1));
    assert_eq!(LISTS.head.load(Ordering::SeqCst), to_ptr(&REQUEST));
    assert_eq!(raises.load(Ordering::SeqCst), 1);

    assert_eq!(post_raising(&LISTS, PRO, &REQUEST, 5, &raises), Ok(1));
    assert_eq!(raises.load(Ordering::SeqCst), 1);

    assert_eq!(poll(&REQUEST, 1), None);
    drain(&LISTS, PRO);
    assert_eq!(RUNS.load(Ordering::SeqCst), 1);
    assert_eq!(poll(&REQUEST, 1), Some(DONE));
    assert_eq!(flags(&REQUEST), 0);
}

#[test]
fn post_with_different_arg_is_busy() {
    static LISTS: Lists = Lists::new();
    static ARG: AtomicUsize = AtomicUsize::new(0);
    extern "C" fn record(arg: usize) {
        ARG.store(arg, Ordering::SeqCst);
    }
    static REQUEST: IpcRequest = IpcRequest::new(record);

    assert_eq!(post(&LISTS, PRO, &REQUEST, 1), Ok(1));
    assert_eq!(post(&LISTS, PRO, &REQUEST, 2), BUSY);

    drain(&LISTS, PRO);
    assert_eq!(ARG.load(Ordering::SeqCst), 1);
}

#[test]
fn post_to_unarmed_request_is_busy() {
    let lists = Lists::new();
    let request = IpcRequest::new(nothing);

    // Another post is between its CAS and its store of `armed`.
    request
        .word()
        .store((1 << SEQ_SHIFT) | QUEUED, Ordering::SeqCst);
    assert_eq!(post(&lists, PRO, &request, 0), BUSY);
    assert!(lists.head.load(Ordering::SeqCst).is_null());
}

#[test]
fn post_to_other_cpu_while_pending_is_busy() {
    let lists = [Lists::new(), Lists::new()];
    let request = IpcRequest::new(nothing);

    assert_eq!(post(&lists[0], PRO, &request, 0), Ok(1));
    assert_eq!(post(&lists[1], APP, &request, 0), BUSY);
    assert!(lists[1].head.load(Ordering::SeqCst).is_null());
}

#[test]
fn post_to_other_cpu_while_running_is_busy() {
    static LISTS: [Lists; 2] = [Lists::new(), Lists::new()];
    static RESULT: AtomicU32 = AtomicU32::new(u32::MAX);
    extern "C" fn post_to_app(_: usize) {
        let result = unsafe { queue::post(&LISTS[1], APP, &REQUEST, 0, || {}) };
        RESULT.store(result, Ordering::SeqCst);
    }
    static REQUEST: IpcRequest = IpcRequest::new(post_to_app);

    assert_eq!(post(&LISTS[0], PRO, &REQUEST, 0), Ok(1));
    drain(&LISTS[0], PRO);
    assert_eq!(decode(RESULT.load(Ordering::SeqCst)), BUSY);
    assert!(LISTS[1].head.load(Ordering::SeqCst).is_null());
}

#[test]
fn post_to_other_cpu_after_the_run_ended_takes_the_request() {
    let lists = [Lists::new(), Lists::new()];
    let request = IpcRequest::new(nothing);

    // The PRO core stored the outcome of run 1, and did not clear `running` yet.
    request
        .word()
        .store((1 << SEQ_SHIFT) | RUNNING, Ordering::SeqCst);
    request
        .status()
        .store((1 << SEQ_SHIFT) | DONE, Ordering::SeqCst);

    assert_eq!(post(&lists[1], APP, &request, 0), Ok(2));
    assert_eq!(flags(&request), QUEUED | ARMED);

    clear_running(&request, PRO);
    assert_eq!(flags(&request), QUEUED | ARMED);

    drain(&lists[1], APP);
    assert_eq!(poll(&request, 2), Some(DONE));
    assert_eq!(flags(&request), 0);
}

#[test]
fn post_to_same_cpu_while_running_queues_a_new_run() {
    static LISTS: Lists = Lists::new();
    static RESULT: AtomicU32 = AtomicU32::new(u32::MAX);
    static FLAGS: AtomicU32 = AtomicU32::new(0);
    static RUNS: AtomicUsize = AtomicUsize::new(0);
    extern "C" fn post_again(arg: usize) {
        RUNS.fetch_add(1, Ordering::SeqCst);
        if arg == 1 {
            let result = unsafe { queue::post(&LISTS, PRO, &REQUEST, 2, || {}) };
            RESULT.store(result, Ordering::SeqCst);
            FLAGS.store(flags(&REQUEST), Ordering::SeqCst);
        }
    }
    static REQUEST: IpcRequest = IpcRequest::new(post_again);

    assert_eq!(post(&LISTS, PRO, &REQUEST, 1), Ok(1));
    drain(&LISTS, PRO);

    assert_eq!(decode(RESULT.load(Ordering::SeqCst)), Ok(2));
    assert_eq!(FLAGS.load(Ordering::SeqCst), QUEUED | RUNNING | ARMED);
    assert_eq!(RUNS.load(Ordering::SeqCst), 1);
    assert_eq!(status_seq(&REQUEST), 1);
    assert_eq!(poll(&REQUEST, 2), None);
    assert_eq!(flags(&REQUEST), QUEUED | ARMED);
    assert_eq!(LISTS.head.load(Ordering::SeqCst), to_ptr(&REQUEST));

    drain(&LISTS, PRO);
    assert_eq!(RUNS.load(Ordering::SeqCst), 2);
    assert_eq!(poll(&REQUEST, 2), Some(DONE));
    assert_eq!(flags(&REQUEST), 0);
}

#[test]
fn join_after_the_run_ended_returns_the_new_seq() {
    let lists = Lists::new();
    let request = IpcRequest::new(nothing);

    assert_eq!(post(&lists, PRO, &request, 7), Ok(1));

    // Between the joiner's first load of `word` and its read of `arg`, run 1 ends, and a new post
    // arms the request with the joiner's `arg`.
    let mut fired = false;
    let result = unsafe {
        queue::post_with_hook(
            &lists,
            PRO,
            &request,
            7,
            || {},
            || {
                if !fired {
                    fired = true;
                    drain(&lists, PRO);
                    assert_eq!(post(&lists, PRO, &request, 7), Ok(2));
                }
            },
        )
    };

    assert!(fired);
    assert_eq!(decode(result), Ok(2));
    assert_eq!(poll(&request, 1), Some(DONE));
    assert_eq!(poll(&request, 2), None);
}

#[test]
fn join_after_the_run_ended_sees_a_new_arg() {
    let lists = Lists::new();
    let request = IpcRequest::new(nothing);

    assert_eq!(post(&lists, PRO, &request, 7), Ok(1));

    let mut fired = false;
    let result = unsafe {
        queue::post_with_hook(
            &lists,
            PRO,
            &request,
            7,
            || {},
            || {
                if !fired {
                    fired = true;
                    drain(&lists, PRO);
                    assert_eq!(post(&lists, PRO, &request, 8), Ok(2));
                }
            },
        )
    };

    assert!(fired);
    assert_eq!(decode(result), BUSY);
}

#[test]
fn drain_runs_requests_in_post_order() {
    static LISTS: Lists = Lists::new();
    static ORDER: Mutex<Vec<usize>> = Mutex::new(Vec::new());
    extern "C" fn record(arg: usize) {
        ORDER.lock().unwrap().push(arg);
    }
    static REQUESTS: [IpcRequest; 3] = [const { IpcRequest::new(record) }; 3];

    for (arg, request) in REQUESTS.iter().enumerate() {
        assert_eq!(post(&LISTS, PRO, request, arg), Ok(1));
    }
    drain(&LISTS, PRO);

    assert_eq!(*ORDER.lock().unwrap(), [0, 1, 2]);
    for request in &REQUESTS {
        assert_eq!(
            request.status().load(Ordering::SeqCst),
            (1 << SEQ_SHIFT) | DONE
        );
        assert_eq!(flags(request), 0);
    }
    assert!(LISTS.head.load(Ordering::SeqCst).is_null());
    assert!(LISTS.drain.load(Ordering::SeqCst).is_null());
    assert!(LISTS.current.load(Ordering::SeqCst).is_null());
}

#[test]
fn status_never_moves_backwards() {
    static LISTS: Lists = Lists::new();
    extern "C" fn maybe_post_again(arg: usize) {
        if arg % 3 == 0 {
            let _ = unsafe { queue::post(&LISTS, PRO, &REQUEST, arg + 1, || {}) };
        }
    }
    static REQUEST: IpcRequest = IpcRequest::new(maybe_post_again);

    let mut last = status_seq(&REQUEST);
    for arg in 0..50 {
        let _ = post(&LISTS, PRO, &REQUEST, arg);
        if arg % 2 == 0 {
            let _ = post(&LISTS, PRO, &REQUEST, arg);
        }
        drain(&LISTS, PRO);

        let seq = status_seq(&REQUEST);
        assert!(seq_reached(seq, last));
        last = seq;
    }
    drain(&LISTS, PRO);
    assert!(seq_reached(status_seq(&REQUEST), last));
}

#[test]
fn seq_reached_wraps() {
    assert!(seq_reached(5, 5));
    assert!(seq_reached(6, 5));
    assert!(!seq_reached(5, 6));
    assert!(seq_reached(0, SEQ_MASK));
    assert!(!seq_reached(SEQ_MASK, 0));
    assert!(seq_reached(3, SEQ_MASK - 3));
    assert!(!seq_reached(SEQ_MASK - 3, 3));
}

#[test]
fn post_wraps_seq() {
    let lists = Lists::new();
    let request = IpcRequest::new(nothing);
    request
        .word()
        .store(SEQ_MASK << SEQ_SHIFT, Ordering::SeqCst);
    request
        .status()
        .store(SEQ_MASK << SEQ_SHIFT, Ordering::SeqCst);

    assert_eq!(post(&lists, PRO, &request, 0), Ok(0));
    assert_eq!(poll(&request, 0), None);
    assert_eq!(poll(&request, SEQ_MASK), Some(DONE));

    drain(&lists, PRO);
    assert_eq!(poll(&request, 0), Some(DONE));
    assert_eq!(post(&lists, PRO, &request, 0), Ok(1));
}

#[test]
fn fail_stranded_fails_each_list() {
    let lists = Lists::new();
    let scratch = Lists::new();
    let in_head = IpcRequest::new(nothing);
    let in_drain = IpcRequest::new(nothing);
    let in_both = IpcRequest::new(nothing);

    assert_eq!(post(&lists, APP, &in_head, 1), Ok(1));

    // The handler took `in_drain` and `in_both`, stored `in_both` in `current`, and stopped
    // before it advanced `drain`.
    assert_eq!(post(&scratch, APP, &in_drain, 2), Ok(1));
    assert_eq!(post(&scratch, APP, &in_both, 3), Ok(1));
    lists.drain.store(
        scratch.head.swap(ptr::null_mut(), Ordering::SeqCst),
        Ordering::SeqCst,
    );
    lists.current.store(to_ptr(&in_both), Ordering::SeqCst);

    unsafe { queue::fail_stranded(&lists, APP) };

    for request in [&in_head, &in_drain, &in_both] {
        assert_eq!(
            request.status().load(Ordering::SeqCst),
            (1 << SEQ_SHIFT) | OFFLINE
        );
        assert_eq!(poll(request, 1), Some(OFFLINE));
        assert_eq!(flags(request), 0);
    }
    assert_eq!(
        in_both.word().load(Ordering::SeqCst),
        in_drain.word().load(Ordering::SeqCst)
    );
    assert!(lists.head.load(Ordering::SeqCst).is_null());
    assert!(lists.drain.load(Ordering::SeqCst).is_null());
    assert!(lists.current.load(Ordering::SeqCst).is_null());

    // A failed request takes posts again.
    assert_eq!(post(&lists, APP, &in_both, 4), Ok(2));
    drain(&lists, APP);
    assert_eq!(poll(&in_both, 2), Some(DONE));
}

#[test]
fn fail_stranded_fails_the_running_request() {
    let lists = Lists::new();
    let scratch = Lists::new();
    let running = IpcRequest::new(nothing);

    // `func` runs on the APP core, and a post to the APP core queued a new run.
    assert_eq!(post(&scratch, APP, &running, 0), Ok(1));
    running.word().store(
        (2 << SEQ_SHIFT) | CORE | QUEUED | ARMED | RUNNING,
        Ordering::SeqCst,
    );
    lists.current.store(to_ptr(&running), Ordering::SeqCst);
    lists.head.store(to_ptr(&running), Ordering::SeqCst);

    unsafe { queue::fail_stranded(&lists, APP) };

    assert_eq!(poll(&running, 1), Some(OFFLINE));
    assert_eq!(poll(&running, 2), Some(OFFLINE));
    assert_eq!(flags(&running), 0);
}

#[test]
fn fail_stranded_keeps_the_outcome_of_an_ended_run() {
    let lists = Lists::new();
    let request = IpcRequest::new(nothing);

    // The APP core stored the outcome of run 1, and stopped before it cleared `running`.
    request
        .word()
        .store((1 << SEQ_SHIFT) | CORE | RUNNING, Ordering::SeqCst);
    request
        .status()
        .store((1 << SEQ_SHIFT) | DONE, Ordering::SeqCst);
    lists.current.store(to_ptr(&request), Ordering::SeqCst);

    unsafe { queue::fail_stranded(&lists, APP) };

    assert_eq!(poll(&request, 1), Some(DONE));
    assert_eq!(flags(&request), 0);
}

#[test]
fn fail_stranded_skips_a_request_of_the_other_cpu() {
    let lists = [Lists::new(), Lists::new()];
    let request = IpcRequest::new(nothing);

    // The APP core stopped in `func`. The run ended there in the model, and a post to the PRO core
    // took the request again.
    lists[1].current.store(to_ptr(&request), Ordering::SeqCst);
    assert_eq!(post(&lists[0], PRO, &request, 0), Ok(1));

    unsafe { queue::fail_stranded(&lists[1], APP) };

    assert_eq!(flags(&request), QUEUED | ARMED);
    assert_eq!(poll(&request, 1), None);
    drain(&lists[0], PRO);
    assert_eq!(poll(&request, 1), Some(DONE));
}

#[test]
fn func_that_posts_itself_runs_on_the_next_drain() {
    static LISTS: Lists = Lists::new();
    static RUNS: AtomicUsize = AtomicUsize::new(0);
    extern "C" fn repost(arg: usize) {
        if RUNS.fetch_add(1, Ordering::SeqCst) == 0 {
            let result = unsafe { queue::post(&LISTS, PRO, &REQUEST, arg, || {}) };
            assert_eq!(decode(result), Ok(2));
        }
    }
    static REQUEST: IpcRequest = IpcRequest::new(repost);

    assert_eq!(post(&LISTS, PRO, &REQUEST, 0), Ok(1));
    drain(&LISTS, PRO);
    assert_eq!(RUNS.load(Ordering::SeqCst), 1);
    assert_eq!(poll(&REQUEST, 1), Some(DONE));
    assert_eq!(poll(&REQUEST, 2), None);

    drain(&LISTS, PRO);
    assert_eq!(RUNS.load(Ordering::SeqCst), 2);
    assert_eq!(poll(&REQUEST, 2), Some(DONE));
}

#[test]
fn concurrent_posts_with_same_arg() {
    static LISTS: Lists = Lists::new();
    static RUNS: AtomicUsize = AtomicUsize::new(0);
    extern "C" fn count(_: usize) {
        RUNS.fetch_add(1, Ordering::SeqCst);
    }
    static REQUEST: IpcRequest = IpcRequest::new(count);

    let results = thread::scope(|s| {
        let threads = [(); 2].map(|_| s.spawn(|| post(&LISTS, PRO, &REQUEST, 3)));
        threads.map(|thread| thread.join().unwrap())
    });

    assert!(results.contains(&Ok(1)));
    for result in results {
        match result {
            Ok(seq) => assert_eq!(seq, 1),
            BUSY => assert_eq!(post(&LISTS, PRO, &REQUEST, 3), Ok(1)),
            Err(code) => panic!("unexpected result {code}"),
        }
    }

    drain(&LISTS, PRO);
    assert_eq!(RUNS.load(Ordering::SeqCst), 1);
    assert_eq!(poll(&REQUEST, 1), Some(DONE));
}

#[test]
fn every_ticket_completes_while_another_thread_drains() {
    const ROUNDS: usize = if cfg!(miri) { 20 } else { 10_000 };

    static LISTS: Lists = Lists::new();
    static LAST_ARG: AtomicUsize = AtomicUsize::new(usize::MAX);
    extern "C" fn record(arg: usize) {
        LAST_ARG.store(arg, Ordering::SeqCst);
    }
    static REQUEST: IpcRequest = IpcRequest::new(record);
    static PENDING: AtomicBool = AtomicBool::new(false);
    static STOP: AtomicBool = AtomicBool::new(false);

    thread::scope(|s| {
        s.spawn(|| {
            while !STOP.load(Ordering::SeqCst) {
                // The handler clears the request before it drains.
                if PENDING.swap(false, Ordering::SeqCst) {
                    drain(&LISTS, PRO);
                } else {
                    thread::yield_now();
                }
            }
        });

        for arg in 0..ROUNDS {
            let result = unsafe {
                queue::post(&LISTS, PRO, &REQUEST, arg, || {
                    PENDING.store(true, Ordering::SeqCst)
                })
            };
            let seq = decode(result).unwrap();
            while poll(&REQUEST, seq).is_none() {
                thread::yield_now();
            }
            assert_eq!(poll(&REQUEST, seq), Some(DONE));
            assert_eq!(LAST_ARG.load(Ordering::SeqCst), arg);
        }

        STOP.store(true, Ordering::SeqCst);
    });
}
