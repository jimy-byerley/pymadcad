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
use pyo3::types::{PyList, PyDict};
use pyo3::ffi::PyObject;
use pyo3::buffer::{PyBuffer, PyUntypedBuffer, Element};
use pyo3::marker::Ungil;
use std::ffi::{CString, CStr};
use std::os::raw::c_int;
use std::ops::Deref;
use std::ptr;
use std::borrow::Cow;
use std::marker::PhantomData;

use crate::math::*;
use crate::mesh::*;
use crate::aabox::*;


macro_rules! newtype {
    ($wrapper:ident, $inner:ty) => {
        struct $wrapper($inner);
        
        impl Deref for $wrapper {
            type Target = $inner;
            fn deref(&self) -> &Self::Target {&self.0}
        }
        impl From<$inner> for $wrapper {
            fn from(inner: $inner) -> Self {Self(inner)}
        }
    }
}


/// trait mirroring arrex and glm dtype definitions
pub unsafe trait DType: Copy {
    fn is_compatible_format(format: &CStr) -> bool;
}

macro_rules! impl_dtype {
    ($ty:ident, $format:expr) => {
        unsafe impl DType for $ty { 
            fn is_compatible_format(format: &CStr) -> bool { format == $format }
        }
    }
}
impl_dtype!(Float, c"d");
impl_dtype!(Vec3, c"ddd");
impl_dtype!(Index, c"I");
impl_dtype!(UVec2, c"II");
impl_dtype!(UVec3, c"III");

pub struct PyVec<T: DType, const N: usize>(Vector<T, N>);
newtype!(PyVec3_, Vector<Float,3>);

impl<T: DType, const N: usize> PyVec<T, N> {
    pub fn borrow(&self) -> &Vector<T, N> {
        &self.0
    }
}
impl<'a, 'py, T: DType, const N: usize> FromPyObject<'a, 'py> for PyVec<T, N> {
    type Error = PyErr;
    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        let buffer = PyUntypedBuffer::get(obj.as_any())?;
        if T::is_compatible_format(buffer.format())
            {return Err(PyTypeError::new_err("unexpected buffer format"))}
        if buffer.is_c_contiguous()
            {return Err(PyTypeError::new_err("buffer should be contiguous"))}
        if buffer.item_size() != size_of::<T>()
            {return Err(PyTypeError::new_err("wrong item size"))}
        if buffer.item_count() != N
            {return Err(PyTypeError::new_err("wrong vector size"))}
        Ok(Self (Vector::from_array(unsafe {*(
            buffer.buf_ptr() as *const [T;N]
            )})))
    }
}

macro_rules! impl_vec {
    ($ty:ident, $size:expr, $glm:expr) => {
        impl<'py> IntoPyObject<'py> for PyVec<$ty, $size> {
            type Target = PyAny;
            type Output = Bound<'py, Self::Target>;
            type Error = PyErr;
            
            fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
                let vec = py.import("madcad.mathutils")?.getattr($glm)?.call0()?;
                let buf = (vec.as_ptr() as usize + size_of::<PyObject>()) as *mut [$ty; $size];
                unsafe {
                    (*buf) = *self.0.as_array();
                }
                Ok(vec)
            }
        }
    }
}
impl_vec!(Float, 3, "vec3");
impl_vec!(Index, 2, "uvec2");
impl_vec!(Index, 3, "uvec3");

/// struct mirroring typedlist, but ensure buffer hook and type for rust
pub struct BorrowedBuffer<T: DType> {
    buffer: PyUntypedBuffer,
    data: PhantomData<T>,
}
impl<T:DType> BorrowedBuffer<T> {
    pub fn as_slice(&self) -> &[T] {
        unsafe {core::slice::from_raw_parts(
            self.buffer.buf_ptr() as *const T, 
            self.buffer.item_count(),
            )}
    }
    pub fn len(&self) -> usize {
        self.buffer.item_count()
    }
}
impl<'a, 'py, T:DType> FromPyObject<'a, 'py> for BorrowedBuffer<T> {
    type Error = PyErr;
    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        let buffer = PyUntypedBuffer::get(obj.as_any())?;
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

/// struct mirroring madcad.mesh.Mesh, but ensuring buffers hooks and types for rust
#[derive(FromPyObject)]
pub struct PySurface {
    pub points: BorrowedBuffer<Vec3>,
    pub faces: BorrowedBuffer<UVec3>,
    pub tracks: BorrowedBuffer<Index>,
    pub groups: Py<PyList>,
    pub options: Py<PyDict>,
}
impl PySurface {
    pub fn borrow(&self) -> Surface<'_> {
        Surface {
            points: Cow::Borrowed(self.points.as_slice()),
            simplices: Cow::Borrowed(self.faces.as_slice()),
            tracks: Cow::Borrowed(self.tracks.as_slice()),
            }
    }
}

#[derive(FromPyObject)]
pub struct PyBox<const N: usize> {
    pub min: PyVec<Float, N>,
    pub max: PyVec<Float, N>,
}
impl<const N: usize> PyBox<N> {
    pub fn from(bounds: AABox<N>) {
        Self {
            
        }
    }
    pub fn into(self) -> AABox<N> {
        AABox{
            min: self.min.borrow().clone(),
            max: self.max.borrow().clone(),
            }
    }
}
impl<'py, const N: usize> IntoPyObject<'py> for PyBox<N>
where PyVec<Float, N>: IntoPyObject<'py> 
{
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;
    
    fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
        py.import("madcad.box")?.getattr("Box")?.call1((
            self.min,
            self.max,
            ))
    }
}



/**
    Buffer that owns Rust data and exposes it via Python's buffer protocol.

    This is designed to work with arrex.typedlist - when you create a typedlist from a Buffer, the typedlist will reference the Buffer as its owner, keeping the data alive as long as the typedlist (or any slice of it) exists.
    This allows ownership of data to be passed to python without needing to copy the buffer into python allocated memory
*/
#[pyclass]
pub struct Bytes {
    /// Raw bytes of the buffer data
    data: Vec<u8>,
}

impl<T: Copy> From<Vec<T>> for Bytes {
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
// impl Bytes {
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
//             return Err(PyBufferError::new_err("Bytes is read-only"));
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
