//! Error reporting that is safe on any thread.
//!
//! Godot FFI (including `godot_error!`) may only be used on the main thread;
//! calling it from a worker panics in debug gdext builds. Worker threads (the
//! sim) install a thread-local collector so [`report_error!`] queues messages
//! instead; the owner drains them and prints engine-side on the main thread.
//! Threads without a collector print directly, so editor tooling keeps its
//! immediate `godot_error!` behaviour.

use std::cell::RefCell;
use std::fmt;

thread_local! {
    static COLLECTOR: RefCell<Option<Vec<String>>> = const { RefCell::new(None) };
}

/// Queue this thread's [`report_error!`] messages instead of printing them
/// through the engine. Call at worker-thread start; pair with [`drain`].
/// Idempotent: an already-installed queue (and its messages) is kept.
pub fn install_collector() {
    COLLECTOR.with(|c| {
        let mut c = c.borrow_mut();
        if c.is_none() {
            *c = Some(Vec::new());
        }
    });
}

/// Take all queued messages (empty when none or no collector installed).
pub fn drain() -> Vec<String> {
    COLLECTOR.with(|c| {
        c.borrow_mut()
            .as_mut()
            .map(std::mem::take)
            .unwrap_or_default()
    })
}

#[doc(hidden)]
pub fn error_fmt(args: fmt::Arguments) {
    COLLECTOR.with(|c| match &mut *c.borrow_mut() {
        Some(queue) => queue.push(args.to_string()),
        None => godot::global::godot_error!("{args}"),
    });
}

/// `godot_error!`-style reporting that works from any thread (see module docs).
#[macro_export]
macro_rules! report_error {
    ($($t:tt)*) => {
        $crate::report::error_fmt(format_args!($($t)*))
    };
}
