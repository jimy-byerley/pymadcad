// madcad-core: Rust implementation of pymadcad core algorithms
//
// This module is compiled to madcad/core.so and imported as:
//   from madcad import core
//   # or
//   from madcad.core import some_function

// modules mirroring python ones
pub mod bevel;
pub mod boolean;
pub mod buffer;
pub mod hashing;
pub mod math;
pub mod mesh;
pub mod aabox;
pub mod triangulation;
pub mod hull;

use pyo3::prelude::*;
use pyo3::create_exception;
use pyo3::exceptions::PyException;
use pyo3::marker::Ungil;    

create_exception!(core, TriangulationError, PyException);

/// Rust core module for pymadcad
#[pymodule]
mod core {
    use super::*;
    use crate::{
        math::*,
        buffer::*,
        };

    #[pymodule_export]
    use super::buffer::Bytes;
    
    #[pymodule_export]
    use super::TriangulationError;

    #[pyfunction]
    fn triangulation_loop_d2(
        py: Python<'_>,
        points: PyTypedList<Vec2>,
        prec: Float,
    ) -> PyResult<PyTypedList<PaddedUVec3>> {
        let simplices = may_detach(py, points.len() > 1_000, ||
            super::triangulation::triangulation_loop_d2(points.as_slice(), prec))
            .map_err(|remains|  TriangulationError::new_err(format!("triangulation failed, remaining outline: {:?}", remains)))?;
        // Convert to padded format for Python compatibility
        let padded: Vec<PaddedUVec3> = simplices.into_iter().map(|v| v.into()).collect();
        PyTypedList::new(py, padded)
    }
/*
    #[pyfunction]
    fn pierce_surface_surface(py: Python<'_>, subject: PyAny, tool: PyAny) -> PyAny {
        let buffers_subject = Surface::extract(py, subject)?;
        let buffers_tool = Surface::extract(py, tool)?;
        let result = py.detach(|| boolean::pierce_surface_surface(buffers_subject, buffers_tool))?;
        madcad_surface(
            vec3_to_typedlist(result.points)?,
            uvec3_to_typedlist(result.simplices)?, 
            index_to_typedlist(result.tracks)?,
            subject.getattr("groups")? + tool.getattr("groups")?,
            )
    }
    #[pyfunction]
    fn pierce_web_surface(subject: PyAny, tool: PyAny) -> PyAny {
        let buffers_subject = Web::extract(py, subject)?;
        let buffers_tool = Surface::extract(py, tool)?;
        let result = py.detach(|| boolean::pierce_web_surface(buffers_subject, buffers_tool))?;
        madcad_web(
            vec3_to_typedlist(result.points)?,
            uvec2_to_typedlist(result.simplices)?, 
            index_to_typedlist(result.tracks)?,
            subject.getattr("groups")? + tool.getattr("groups")?,
            )
    }

    #[pyfunction]
    fn transform_points_affine(points: PyAny, transform: PyAny) -> PyAny {
        let buffer = typedlist_to_vec3(points)?;
        let transform = glm_to_affine(transform)?;
        let result = may_detach(py, buffer.len() > 5_000, || 
            buffer.iter().map(|p|  transform * p).collect::<Vec<_>>());
        vec3_to_typedlist(result)
    }

    #[pyfunction]
    fn points_box(points: PyAny) -> PyAny {
        let buffer = typedlist_to_vec3(points)?;
        let result = may_detach(py, buffer.len() > 10_000, || (
            buffer.iter().fold(Vector::max),
            buffer.iter().fold(Vector::min),
            ));
        madcad_box(result)
    }
    */
//     #[pyfunction]
//     fn surface_box(py: Python<'_>, points: PyTypedList<Vec3>, simplices: PyTypedList<UVec3>) -> PyBox3 {
//         PyBox3::from(may_detach(py, simplices.len() > 10_000, || surface.borrow().bounds()))
//     }
    /*
    #[pyfunction]
    fn web_box(surface: PyAny) -> PyAny {
        let buffer = Web::extract(surface)?;
        let result = may_detach(py, buffer.simpliced.len() > 10_000, || surface.bounds());
        madcad_box(result)
    }*/
}

/// run the given closure detached from python thread if required, otherwise run it attached
fn may_detach<F, T>(py: Python<'_>, detach: bool, f: F) -> T
where 
    F: Ungil + FnOnce() -> T,
    T: Ungil,
{
    if detach { py.detach(f) }
    else { f() }
}
