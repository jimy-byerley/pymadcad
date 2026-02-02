/*!
    Buffer: A Python object that owns Rust Vec<T> data and exposes it via the buffer protocol.

    This allows Rust functions to return owned data that Python's arrex.typedlist can view
    without copying. The typedlist's `owner` attribute will reference this Buffer.

    Usage pattern:
    1. Rust function produces Vec<T> (e.g., Vec<[f64; 3]> for points)
    2. Wrap in Buffer and expose to Python
    3. Create Python typedlist viewing the Buffer's memory
    4. typedlist.owner points to Buffer, keeping data alive
*/

use pyo3::exceptions::PyBufferError;
use pyo3::ffi;
use pyo3::prelude::*;
use std::ffi::CString;
use std::os::raw::c_int;
use std::ptr;
use std::sync::atomic::{AtomicUsize, Ordering};

use crate::math::*;


/**
    Buffer that owns Rust data and exposes it via Python's buffer protocol.

    This is designed to work with arrex.typedlist - when you create a typedlist from a Buffer, the typedlist will reference the Buffer as its owner, keeping the data alive as long as the typedlist (or any slice of it) exists.
    This allows ownership of data to be passed to python without needing to copy the buffer into python allocated memory
*/
#[pyclass]
pub struct Buffer {
    /// Raw bytes of the buffer data
    data: Vec<u8>,
}

impl<T, D> From<Vec<Vector<T, D>>> for Buffer {
    fn from(data: Vec<Vector<T, D>>) -> Self {
        unsafe { 
            let (ptr, len, cap) = data.into_raw_parts();
            let data = Vec::from_raw_parts(
                ptr as *mut u8, 
                len * core::mem::size_of::<Vector<T,D>>(),
                cap * core::mem::size_of::<Vector<T,D>>(),
                );
            Self {data}
        }
    }
}

#[pymethods]
impl Buffer {
    /// Buffer protocol implementation
    unsafe fn __getbuffer__(
        slf: Bound<'_, Self>,
        view: *mut ffi::Py_buffer,
        flags: c_int,
    ) -> PyResult<()> {
        let buffer = slf.borrow();

        // Check flags
        if flags & ffi::PyBUF_WRITABLE != 0 {
            return Err(PyBufferError::new_err("Buffer is read-only"));
        }

        // Fill in the buffer info
        (*view).buf = buffer.data.as_ptr() as *mut _;
        (*view).len = buffer.data.len() as isize;
        (*view).itemsize = 1;
        (*view).readonly = 1;
        (*view).ndim = 1;
        (*view).format = buffer.format.as_ptr() as *mut _;

        // Shape and strides for 1D buffer
        (*view).shape = ptr::null_mut();
        (*view).strides = ptr::null_mut();
        (*view).suboffsets = ptr::null_mut();
        (*view).internal = ptr::null_mut();

        // Set the object reference to keep the buffer alive
        (*view).obj = slf.as_ptr();
        ffi::Py_INCREF((*view).obj);

        Ok(())
    }

    unsafe fn __releasebuffer__(&self, _view: *mut ffi::Py_buffer) {
    }
}


impl<'py> FromPyObject<'a, 'py> for Surface<'a> {
    type Error = PyErr;
    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        Surface {
            points: typedlist_to_vec3(py, obj.getattr("points")?)?,
            simplices: typedlist_to_uvec3(py, obj.getattr("faces")?)?,
            tracks: typedlist_to_index(py, obj.getattr("tracks")?)?,
        }
    }
}
impl FromPyObject for Web {}
impl FromPyObject for Wire {}

// impl IntoPyObject for Surface {
//     type Target = PyAny;
//     type Output = Py<PyAny>;
//     type Error = PyErr;
//     fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
//         let mesh = py.import("madcad.mesh.Mesh")?;
//         mesh.call1((
//             slice_to_typedlist(self.points),
//             slice_to_typedlist(self.simplices),
//             slice_to_typedlist(self.tracks),
//             ))
//     }
// }
// impl IntoPyObject for Web {}
// impl IntoPyObject for Wire {}

fn typedlist_to_vec3<'a, 'py>(py: Python<'py>, obj: Borrowed<'a, 'py>) -> Result<&'a [Vec3], PyErr> {
    typedlist_to_slice(py, obj, "ddd")
}
fn typedlist_to_index<'a, 'py>(py: Python<'py, obj: Borrowed<'a, 'py>) -> Result<&'a [Index], PyErr> {
    typedlist_to_slice(py, obj, "I")
}
fn typedlist_to_uvec2<'a, 'py>(py: Python<'py, obj: Borrowed<'a, 'py>) -> Result<&'a [UVec2], PyErr> {
    typedlist_to_slice(py, obj, "II")
}
fn typedlist_to_uvec3<'a, 'py>(py: Python<'py, obj: Borrowed<'a, 'py>) -> Result<Vec<Vec3>, PyErr> {
    let aligned = typedlist_to_slice::<Vector<Index, 4>>(py, obj, "IIIxxxx");
    todo!("extract only 3 first components of each item")
}

fn typedlist_to_slice<'a, 'py, T>(py: Python<'py>, obj: Borrowed<'a, 'py>, layout: &str) -> Result<&'a [T], PyErr> {
    todo!("use buffer protocol to get slice, check sizes and layout")
}


fn vec3_to_typedlist<'py>(py: Python<'py>, buffer: Vec<Vec3>) -> PyResult<Bound<'py, PyAny>> {
    let vec3 = py.import("madcad.mathutils")?.getattr("vec3")?;
    let typedlist = py.import("arrex")?.getattr("typedlist")?;
    typedlist.call1((Buffer::from(buffer), vec3))
}
fn index_to_typedlist<'py>(py: Python<'py>, buffer: Vec<Index>) -> PyResult<Bound<'py, PyAny>> {
    let typedlist = py.import("arrex")?.getattr("typedlist")?;
    typedlist.call1((Buffer::from(buffer), "I"))
}

fn madcad_surface<'py>(
    py: Python<'py>,
    points: PyAny,
    simplices: PyAny,
    tracks: PyAny,
    groups: PyAny,
) -> PyResult<PyAny> {
    py.import("madcad.mesh")?.getattr("Mesh")?.call1((points, simplices, tracks, groups))
}
