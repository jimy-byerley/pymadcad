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
pub mod triangulation;
pub mod hull;

use pyo3::prelude::*;
use pyo3::marker::Ungil;

use crate::{
    math::*,
    mesh::*,
    buffer::*,
    };

/// Rust core module for pymadcad
#[pymodule]
fn core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Register Buffer class for data ownership
    m.add_class::<Buffer>()?;
/*
    #[pyfunction]
    fn triangulation_wire(wire: PyAny, normal: PyAny, prec: Float) -> PyAny {
        let wire = Wire::extract(py, wire)?;
        let normal = glm_to_vec3(normal)?;
        let result = may_detach(py, wire.simplices.len() > 1_000, || 
            triangulation::triangulation_wire(wire, normal, prec))?;
        uvec3_to_typedlist(result.simplices)
    }

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
    #[pyfunction]
    fn surface_box(surface: PyAny) -> PyAny {
        let buffer = Surface::extract(surface)?;
        let result = may_detach(py, buffer.simplices.len() > 10_000, || surface.bounds());
        madcad_box(result)
    }
    #[pyfunction]
    fn web_box(surface: PyAny) -> PyAny {
        let buffer = Web::extract(surface)?;
        let result = may_detach(py, buffer.simpliced.len() > 10_000, || surface.bounds());
        madcad_box(result)
    }*/

    Ok(())
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
