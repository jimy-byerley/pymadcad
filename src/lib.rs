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
pub mod rasterize;
pub mod hull;
pub mod test;

use pyo3::prelude::*;
use pyo3::create_exception;
use pyo3::exceptions::{PyException, PyValueError};
use pyo3::marker::Ungil;
use pyo3::types::{PyList, PyDict};

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
    use crate::test::test;

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

    #[pyfunction]
    fn rasterize_segment(
        space: (PyVec3, PyVec3),
        cell: Float,
    ) -> PyResult<Vec<(i64, i64, i64)>> {
        super::rasterize::rasterize_segment(&[*space.0, *space.1], cell)
            .map(|keys| keys.into_iter().map(|k| (k[0], k[1], k[2])).collect())
            .map_err(|e| PyValueError::new_err(e))
    }

    #[pyfunction]
    fn rasterize_triangle(
        space: (PyVec3, PyVec3, PyVec3),
        cell: Float,
    ) -> PyResult<Vec<(i64, i64, i64)>> {
        super::rasterize::rasterize_triangle(&[*space.0, *space.1, *space.2], cell)
            .map(|keys| keys.into_iter().map(|k| (k[0], k[1], k[2])).collect())
            .map_err(|e| PyValueError::new_err(e))
    }

    #[pyfunction]
    fn intersect_triangles(
        f0: (PyVec3, PyVec3, PyVec3),
        f1: (PyVec3, PyVec3, PyVec3),
        precision: Float,
    ) -> Option<((usize, usize, PyVec3), (usize, usize, PyVec3))> {
        let fa = [*f0.0, *f0.1, *f0.2];
        let fb = [*f1.0, *f1.1, *f1.2];
        super::boolean::intersect_triangles(&fa, &fb, precision)
            .map(|(a, b)| (
                (a.face, a.edge, PyVec3::from(a.point)),
                (b.face, b.edge, PyVec3::from(b.point)),
            ))
    }

    #[pyfunction]
    fn cut_surface(
        py: Python<'_>,
        m1: PySurface,
        m2: PySurface,
        prec: Float,
    ) -> PyResult<(PySurface, PyWeb)> {
        let s1 = m1.borrow();
        let s2 = m2.borrow();
        let nfaces = s1.simplices.len();

        let (cut, frontier) = may_detach(py, nfaces > 1_000, ||
            super::boolean::cut_surface(&s1, &s2, prec));

        let cut_faces: Vec<PaddedUVec3> = cut.simplices.into_owned().into_iter().map(|v| v.into()).collect();
        let front_edges = frontier.simplices.into_owned();

        Ok((
            PySurface {
                points: PyTypedList::new(py, cut.points.into_owned())?,
                faces: PyTypedList::new(py, cut_faces)?,
                tracks: PyTypedList::new(py, cut.tracks.into_owned())?,
                groups: m1.groups.clone_ref(py),
                options: PyDict::new(py).unbind(),
            },
            PyWeb {
                points: PyTypedList::new(py, frontier.points.into_owned())?,
                faces: PyTypedList::new(py, front_edges)?,
                tracks: PyTypedList::new(py, frontier.tracks.into_owned())?,
                groups: PyList::empty(py).unbind(),
                options: PyDict::new(py).unbind(),
            },
        ))
    }

    #[pyfunction]
    fn pierce_surface(
        py: Python<'_>,
        m1: PySurface,
        m2: PySurface,
        side: bool,
        prec: Float,
        strict: bool,
    ) -> PyResult<PySurface> {
        let s1 = m1.borrow();
        let s2 = m2.borrow();
        let nfaces = s1.simplices.len();

        let result = may_detach(py, nfaces > 1_000, ||
            super::boolean::pierce_surface(&s1, &s2, side, prec, strict))
            .map_err(|e| PyValueError::new_err(e))?;

        let result_faces: Vec<PaddedUVec3> = result.simplices.into_owned().into_iter().map(|v| v.into()).collect();

        Ok(PySurface {
            points: PyTypedList::new(py, result.points.into_owned())?,
            faces: PyTypedList::new(py, result_faces)?,
            tracks: PyTypedList::new(py, result.tracks.into_owned())?,
            groups: m1.groups.clone_ref(py),
            options: PyDict::new(py).unbind(),
        })
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
