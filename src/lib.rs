#![allow(clippy::missing_safety_doc)]

use mimalloc::MiMalloc;

#[global_allocator]
static GLOBAL_ALLOCATOR: MiMalloc = MiMalloc;

mod abi;
mod helper;
mod rapier;

pub use rapier::ffi::*;

#[cfg(feature = "anvilkit-bridge")]
pub use rapier::ffi::AnvilKitAppHandle;
