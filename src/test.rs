/// Test helpers for Python <-> Rust data conversion testing.
/// Exposed as the `core.test` sub-module.

use pyo3::prelude::*;
use crate::math::*;
use crate::buffer::*;

#[pymodule]
pub mod test {
    use super::*;

    // --- passthrough functions for single vectors ---

    #[pyfunction]
    fn passthrough_vec3(v: PyVec3) -> PyVec3 { v }

    #[pyfunction]
    fn passthrough_uvec2(v: PyUVec2) -> PyUVec2 { v }

    #[pyfunction]
    fn passthrough_uvec3(v: PyUVec3) -> PyUVec3 { v }

    // --- passthrough functions for typedlists ---

    #[pyfunction]
    fn passthrough_typedlist_vec3(py: Python<'_>, buf: PyTypedList<Vec3>) -> PyResult<PyTypedList<Vec3>> {
        PyTypedList::new(py, buf.as_slice().to_vec())
    }

    #[pyfunction]
    fn passthrough_typedlist_vec2(py: Python<'_>, buf: PyTypedList<Vec2>) -> PyResult<PyTypedList<Vec2>> {
        PyTypedList::new(py, buf.as_slice().to_vec())
    }

    #[pyfunction]
    fn passthrough_typedlist_uvec2(py: Python<'_>, buf: PyTypedList<UVec2>) -> PyResult<PyTypedList<UVec2>> {
        PyTypedList::new(py, buf.as_slice().to_vec())
    }

    #[pyfunction]
    fn passthrough_typedlist_uvec3(py: Python<'_>, buf: PyTypedList<PaddedUVec3>) -> PyResult<PyTypedList<PaddedUVec3>> {
        PyTypedList::new(py, buf.as_slice().to_vec())
    }

    #[pyfunction]
    fn passthrough_typedlist_index(py: Python<'_>, buf: PyTypedList<Index>) -> PyResult<PyTypedList<Index>> {
        PyTypedList::new(py, buf.as_slice().to_vec())
    }

    #[pyfunction]
    fn passthrough_typedlist_float(py: Python<'_>, buf: PyTypedList<Float>) -> PyResult<PyTypedList<Float>> {
        PyTypedList::new(py, buf.as_slice().to_vec())
    }

    // --- sequence generators ---

    #[pyfunction]
    fn sequence_vec3(n: usize) -> PyResult<PyVec3> {
        let i = n as Float;
        Ok(PyVec3::from(Vec3::from([i, i + 0.5, i + 0.25])))
    }

    #[pyfunction]
    fn sequence_uvec3(n: usize) -> PyResult<PyUVec3> {
        let i = n as Index;
        Ok(PyUVec3::from(UVec3::from([i, i + 1, i + 2])))
    }

    #[pyfunction]
    fn sequence_uvec2(n: usize) -> PyResult<PyUVec2> {
        let i = n as Index;
        Ok(PyUVec2::from(UVec2::from([i, i + 1])))
    }

    #[pyfunction]
    fn sequence_typedlist_vec3(py: Python<'_>, n: usize) -> PyResult<PyTypedList<Vec3>> {
        let data: Vec<Vec3> = (0..n).map(|i| {
            let f = i as Float;
            Vec3::from([f, f + 0.5, f + 0.25])
        }).collect();
        PyTypedList::new(py, data)
    }

    #[pyfunction]
    fn sequence_typedlist_uvec3(py: Python<'_>, n: usize) -> PyResult<PyTypedList<PaddedUVec3>> {
        let data: Vec<PaddedUVec3> = (0..n).map(|i| {
            let idx = i as Index;
            PaddedUVec3::from(UVec3::from([idx, idx + 1, idx + 2]))
        }).collect();
        PyTypedList::new(py, data)
    }

    #[pyfunction]
    fn sequence_typedlist_uvec2(py: Python<'_>, n: usize) -> PyResult<PyTypedList<UVec2>> {
        let data: Vec<UVec2> = (0..n).map(|i| {
            let idx = i as Index;
            UVec2::from([idx, idx + 1])
        }).collect();
        PyTypedList::new(py, data)
    }

    #[pyfunction]
    fn sequence_typedlist_vec2(py: Python<'_>, n: usize) -> PyResult<PyTypedList<Vec2>> {
        let data: Vec<Vec2> = (0..n).map(|i| {
            let f = i as Float;
            Vec2::from([f, f + 0.5])
        }).collect();
        PyTypedList::new(py, data)
    }

    #[pyfunction]
    fn sequence_typedlist_index(py: Python<'_>, n: usize) -> PyResult<PyTypedList<Index>> {
        let data: Vec<Index> = (0..n).map(|i| i as Index).collect();
        PyTypedList::new(py, data)
    }

    #[pyfunction]
    fn sequence_typedlist_float(py: Python<'_>, n: usize) -> PyResult<PyTypedList<Float>> {
        let data: Vec<Float> = (0..n).map(|i| i as Float * 0.1).collect();
        PyTypedList::new(py, data)
    }
}
