// madcad-core: Rust implementation of pymadcad core algorithms
//
// This module is compiled to madcad/core.so and imported as:
//   from madcad import core
//   # or
//   from madcad.core import some_function

use pyo3::prelude::*;

// modules mirroring python ones
pub mod bevel;
pub mod boolean;
pub mod hashing;
pub mod math;
pub mod mesh;
pub mod triangulation;

/// Rust core module for pymadcad
#[pymodule]
#[pyo3(name = "core")]
fn madcad_core(_m: &Bound<'_, PyModule>) -> PyResult<()> {
    // TODO: Register submodules and functions here
    // Example:
    // m.add_function(wrap_pyfunction!(boolean_union, m)?)?;
    // m.add_class::<Mesh>()?;
    Ok(())
}
