// Copyright (c) 2026 James Pogemiller/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Error types for all SDK boundaries.

use core::fmt;

/// Common SDK error variant.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SdkError {
    /// A component has not been found in the runtime registry.
    ComponentNotFound,
    /// The component cannot transition from the current state.
    InvalidState,
    /// The requested operation exceeded configured time limits.
    Timeout,
    /// Configuration or manifest data is invalid.
    InvalidConfig,
    /// Required dependency was not found in the component graph.
    DependencyMissing,
    /// Manifest content is malformed or unsupported.
    ManifestParse,
    /// A generic implementation-specific failure.
    BackendFailure,
}

impl fmt::Display for SdkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let msg = match self {
            Self::ComponentNotFound => "component not found",
            Self::InvalidState => "invalid lifecycle state",
            Self::Timeout => "operation timed out",
            Self::InvalidConfig => "invalid configuration",
            Self::DependencyMissing => "required dependency missing",
            Self::ManifestParse => "manifest parse failure",
            Self::BackendFailure => "backend failure",
        };
        write!(f, "{msg}")
    }
}

impl std::error::Error for SdkError {}
