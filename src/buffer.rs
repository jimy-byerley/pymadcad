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

use pyo3::prelude::*;
use pyo3::exceptions::{PyBufferError, PyTypeError};
use pyo3::ffi;
use pyo3::buffer::{PyBuffer, PyUntypedBuffer, Element};
use pyo3::marker::Ungil;
use std::ffi::{CString, CStr};
use std::os::raw::c_int;
use std::ptr;
use std::borrow::Cow;

use crate::math::*;
use crate::mesh::*;


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

impl<T: Copy> From<Vec<T>> for Buffer {
    fn from(data: Vec<T>) -> Self {
        unsafe { 
            let (ptr, len, cap) = data.into_raw_parts();
            let data = Vec::from_raw_parts(
                ptr as *mut u8, 
                len * core::mem::size_of::<T>(),
                cap * core::mem::size_of::<T>(),
                );
            Self {data}
        }
    }
}

// #[pymethods]
// impl Buffer {
//     /// Buffer protocol implementation
//     unsafe fn __getbuffer__(
//         slf: Bound<'_, Self>,
//         view: *mut ffi::Py_buffer,
//         flags: c_int,
//     ) -> PyResult<()> {
//         let buffer = slf.borrow();
// 
//         // Check flags
//         if flags & ffi::PyBUF_WRITABLE != 0 {
//             return Err(PyBufferError::new_err("Buffer is read-only"));
//         }
// 
//         // Fill in the buffer info
//         (*view).buf = buffer.data.as_ptr() as *mut _;
//         (*view).len = buffer.data.len() as isize;
//         (*view).itemsize = 1;
//         (*view).readonly = 1;
//         (*view).ndim = 1;
//         (*view).format = buffer.format.as_ptr() as *mut _;
// 
//         // Shape and strides for 1D buffer
//         (*view).shape = ptr::null_mut();
//         (*view).strides = ptr::null_mut();
//         (*view).suboffsets = ptr::null_mut();
//         (*view).internal = ptr::null_mut();
// 
//         // Set the object reference to keep the buffer alive
//         (*view).obj = slf.as_ptr();
//         ffi::Py_INCREF((*view).obj);
// 
//         Ok(())
//     }
// 
//     unsafe fn __releasebuffer__(&self, _view: *mut ffi::Py_buffer) {
//     }
// }

/*
pub trait DType: Copy {
    fn is_compatible_format(format: &CStr) -> bool;
}

macro_rules! impl_dtype {
    ($ty:ident, $format:expr) => {
        unsafe impl Element for $ty { 
            fn is_compatible_format(format: &CStr) -> bool { format == $format }
        }
    }
}
impl_dtype!(Vec3, c"ddd");
impl_dtype!(UVec4, c"IIII");
impl_dtype!(Index, c"I");

pub struct BorrowedBuffer<T> {
    buffer: PyUntypedBuffer,
    data: PhantomData<T>,
}
impl BorrowedBuffer<T: DType> {
    fn as_slice(&self) -> &[T] {
        core::slice::from_raw_parts(self.buffer.buf_ptr() as *const T, self.buffer.item_count())
    }
}
impl<'a, 'py> FromPyObject<'a, 'py> for BorrowedBuffer {
    type Error = PyErr;
    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        let buffer = PyUntypedBuffer::get(obj)?;
        if T::is_compatible_format(buffer.format())
            {return Err(PyTypeError::new_err("unexpected buffer format"))}
        if buffer.is_c_contiguous()
            {return Err(PyTypeError::new_err("buffer should be contiguous"))}
        if buffer.item_size() != size_of::<T>()
            {return Err(PyTypeError::new_err("wrong item size"))}
        Ok(Self {
            buffer,
            data: PhantomData,
        })
    }
}

#[derive(FromPyObject)]
pub struct PySurface {
    pub points: BorrowedBuffer<Vec3>,
    pub faces: BorrowedBuffer<UVec4>,
    pub tracks: BorrowedBuffer<Index>,
    pub groups: Py<PyList>,
    pub options: Py<PyDict>,
}
impl From<&PySurface> for Surface<'_> {
    fn from(surface: &PySurface) -> Surface<'_> {
        Surface {
            points: Cow::Borrowed(surface.points.as_slice()),
            simplices: Cow::Borrowed(surface.faces.as_slice()),
            tracks: Cow::Borrowed(surface.tracks.as_slice()),
            }
    }
}*/



#[repr(C)]
#[derive(Copy, Clone)]
struct PyVec3 (Vec3);
unsafe impl Element for PyVec3 {
    fn is_compatible_format(format: &CStr) -> bool  { format == c"ddd" }
}
#[repr(C)]
#[derive(Copy, Clone)]
struct PyUVec3 (UVec3);
unsafe impl Element for PyUVec3 {
    fn is_compatible_format(format: &CStr) -> bool  { format == c"ddd" }
}

struct PySurface<'py> {
    python: Python<'py>,
    points: PyBuffer<PyVec3>,
    simplices: PyBuffer<PyUVec3>,
    tracks: PyBuffer<Index>,
}
impl<'a> From<&'a PySurface<'_>> for Surface<'a> {
    fn from(surface: &'a PySurface<'_>) -> Surface<'a> {
        Surface {
            points: Cow::Borrowed(surface.points.as_slice(surface.python)),
            simplices: Cow::Borrowed(surface.simplices.as_slice(surface.python)),
            tracks: Cow::Borrowed(surface.tracks.as_slice(surface.python)),
            }
    }
}



impl<'a, 'py> FromPyObject<'a, 'py> for Surface<'a> {
    type Error = PyErr;
    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        Ok(Surface {
            points: Cow::Borrowed(typedlist_to_vec3(&obj.getattr("points")?)?),
            simplices: Cow::Owned(typedlist_to_uvec3(&obj.getattr("faces")?)?),
            tracks: Cow::Borrowed(typedlist_to_index(&obj.getattr("tracks")?)?),
        })
    }
}
// impl FromPyObject for Web {}
// impl FromPyObject for Wire {}

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

fn typedlist_to_vec3<'a>(obj: &'a Bound<'_, PyAny>) -> Result<&'a [Vec3], PyErr> {
    typedlist_to_slice(obj, c"ddd")
}
fn typedlist_to_index<'a>(obj: &'a Bound<'_, PyAny>) -> Result<&'a [Index], PyErr> {
    typedlist_to_slice(obj, c"I")
}
fn typedlist_to_uvec2<'a>(obj: &'a Bound<'_, PyAny>) -> Result<&'a [UVec2], PyErr> {
    typedlist_to_slice(obj, c"II")
}
fn typedlist_to_uvec3(obj: &Bound<'_, PyAny>) -> Result<Vec<UVec3>, PyErr> {
    let aligned = typedlist_to_slice::<Vector<Index, 4>>(obj, c"IIIxxxx");
    todo!("extract only 3 first components of each item")
}

fn typedlist_to_slice<'a, T: Copy>(obj: &'a Bound<'_, PyAny>, layout: &CStr) -> Result<&'a [T], PyErr> {
    let buffer = PyUntypedBuffer::get(obj)?;
    if buffer.format() != layout
        {return Err(PyTypeError::new_err("unexpected buffer format"))}
    if buffer.is_c_contiguous()
        {return Err(PyTypeError::new_err("buffer should be contiguous"))}
    if buffer.item_size() != size_of::<T>()
        {return Err(PyTypeError::new_err("wrong item size"))}
    Ok(unsafe {std::slice::from_raw_parts(buffer.buf_ptr() as *const T, buffer.item_count())})
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
    points: Bound<'py, PyAny>,
    simplices: Bound<'py, PyAny>,
    tracks: Bound<'py, PyAny>,
    groups: Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    py.import("madcad.mesh")?.getattr("Mesh")?.call1((points, simplices, tracks, groups))
}
