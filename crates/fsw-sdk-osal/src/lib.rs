// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! OS abstraction layer interfaces and host backend.

use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};
use std::thread;

use fsw_sdk_core::SdkError;

/// Opaque task handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TaskHandle(pub usize);

/// Queue API used by runtime and apps.
pub trait MessageQueue<T>: Send + Sync {
    /// Sends one item to the queue.
    fn send(&self, item: T) -> Result<(), SdkError>;

    /// Receives one item from the queue.
    fn recv(&self) -> Result<T, SdkError>;
}

/// Core OS abstraction for tasking primitives.
pub trait Osal: Send + Sync {
    /// Spawns a named task.
    fn spawn_task(
        &self,
        name: &'static str,
        f: Box<dyn FnOnce() + Send + 'static>,
    ) -> Result<TaskHandle, SdkError>;
}

/// Host OSAL backend based on std thread.
#[derive(Default)]
pub struct HostOsal {
    next_id: Arc<Mutex<usize>>,
}

impl HostOsal {
    /// Creates a new host backend.
    #[must_use]
    pub fn new() -> Self {
        Self {
            next_id: Arc::new(Mutex::new(1)),
        }
    }
}

impl Osal for HostOsal {
    fn spawn_task(
        &self,
        name: &'static str,
        f: Box<dyn FnOnce() + Send + 'static>,
    ) -> Result<TaskHandle, SdkError> {
        let mut guard = self.next_id.lock().map_err(|_| SdkError::BackendFailure)?;
        let handle = TaskHandle(*guard);
        *guard += 1;

        thread::Builder::new()
            .name(name.to_string())
            .spawn(f)
            .map(|_| handle)
            .map_err(|_| SdkError::BackendFailure)
    }
}

/// In-process queue implementation for host simulation.
pub struct HostQueue<T> {
    inner: Arc<(Mutex<VecDeque<T>>, Condvar)>,
}

impl<T> HostQueue<T> {
    /// Creates an empty queue.
    #[must_use]
    pub fn new() -> Self {
        Self {
            inner: Arc::new((Mutex::new(VecDeque::new()), Condvar::new())),
        }
    }
}

impl<T> Default for HostQueue<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Send> MessageQueue<T> for HostQueue<T> {
    fn send(&self, item: T) -> Result<(), SdkError> {
        let (mutex, cv) = &*self.inner;
        let mut q = mutex.lock().map_err(|_| SdkError::BackendFailure)?;
        q.push_back(item);
        cv.notify_one();
        Ok(())
    }

    fn recv(&self) -> Result<T, SdkError> {
        let (mutex, cv) = &*self.inner;
        let mut q = mutex.lock().map_err(|_| SdkError::BackendFailure)?;

        loop {
            if let Some(item) = q.pop_front() {
                return Ok(item);
            }
            q = cv.wait(q).map_err(|_| SdkError::BackendFailure)?;
        }
    }
}
