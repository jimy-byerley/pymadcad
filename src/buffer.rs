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

/// struct owning a ref to a python buffer and allowing to view into it
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

/** struct acting like `Cow<[T]>` but with a hook on a python buffer when borrowed

    it stores either:
        - a ref on a python buffer (borrowed)
        - a rust buffer (owned)
*/
pub enum CowBuffer<T: DType> {
    Borrowed(BorrowedBuffer<T>),
    Owned(Vec<T>),
}
impl<T: DType> CowBuffer<T> {
    fn as_slice(&self) -> &[T] {
        match self {
            Self::Borrowed(buffer) => buffer.as_slice(),
            Self::Owned(buffer) => buffer.as_slice(),
        }
    }
    pub fn len(&self) -> usize {
        match self {
            Self::Borrowed(buffer) => buffer.len(),
            Self::Owned(buffer) => buffer.len(),
        }
    }
}
impl<'a, 'py, T:DType> FromPyObject<'a, 'py> for CowBuffer<T> {
    type Error = PyErr;
    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        Ok(Self::Borrowed(obj.extract()?))
    }
}
impl<'py, T:DType> IntoPyObject<'py> for CowBuffer<T> {
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;
    
    fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
        let data = match self {
            Self::Borrowed(buffer) => buffer.as_slice().to_owned(),
            Self::Owned(buffer) => buffer,
        };
        py.import("arrex")?.getattr("typedlist")?.call1((
            Bytes::from(data),
            py.import("madcad.mathutils")?.getattr("vec3")?,  // TODO allow other types
            ))
    }
}


macro_rules! newtype {
    ($wrapper:ident, $inner:ty) => {
        pub struct $wrapper($inner);
        
        impl Deref for $wrapper {
            type Target = $inner;
            fn deref(&self) -> &Self::Target {&self.0}
        }
        impl From<$inner> for $wrapper {
            fn from(inner: $inner) -> Self {Self(inner)}
        }
    }
}

macro_rules! newtype_vec {
    ($name:ident, $dtype:ident, $size:expr, $glm:expr) => {
        newtype!($name, Vector<$dtype, $size>);
        
        impl<'a, 'py> FromPyObject<'a, 'py> for $name {
            type Error = PyErr;
            fn extract(obj: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
                let vec = obj.py().import("madcad.mathutils")?.getattr($glm)?;
                if ! obj.is_instance(&vec)?
                    {return Err(PyTypeError::new_err("wrong vector type"))}
                let buf = (obj.as_ptr() as usize + size_of::<PyObject>()) as *mut [$dtype; $size];
                Ok(Self(unsafe {
                    Vector::<$dtype, $size>::from(*buf)
                    }))
            }
        }
        impl<'py> IntoPyObject<'py> for $name {
            type Target = PyAny;
            type Output = Bound<'py, Self::Target>;
            type Error = PyErr;
            
            fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
                let vec = py.import("madcad.mathutils")?.getattr($glm)?;
                let obj = vec.call0()?;
                let buf = (obj.as_ptr() as usize + size_of::<PyObject>()) as *mut [$dtype; $size];
                unsafe {
                    (*buf) = *self.0.as_array();
                }
                Ok(vec)
            }
        }
    }
}
newtype_vec!(PyVec3, Float, 3, "vec3");
newtype_vec!(PyUVec2, Index, 2, "uvec2");
newtype_vec!(PyUVec3, Index, 3, "uvec3");

newtype!(PyBox3, AABox<3>);

impl<'a, 'py> FromPyObject<'a, 'py> for PyBox3 {
    type Error = PyErr;
    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        Ok(Self(AABox{
            min: *obj.getattr("min")?.extract::<PyVec3>()?,
            max: *obj.getattr("max")?.extract::<PyVec3>()?,
        }))
    }
}
impl<'py> IntoPyObject<'py> for PyBox3 {
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;
    
    fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
        py.import("madcad.box")?.getattr("Box")?.call1((
            PyVec3::from(self.min),
            PyVec3::from(self.max),
            ))
    }
}

/// struct mirroring madcad.mesh.Mesh, but ensuring buffers hooks and types for rust
#[derive(FromPyObject)]
pub struct PySurface {
    pub points: CowBuffer<Vec3>,
    pub faces: CowBuffer<UVec3>,
    pub tracks: CowBuffer<Index>,
    pub groups: Py<PyList>,
    pub options: Py<PyDict>,
}
impl PySurface {
    pub fn borrow(&self) -> Surface<'_> {
        Surface {
            points: self.points.as_slice().into(),
            simplices: self.faces.as_slice().into(),
            tracks: self.tracks.as_slice().into(),
            }
    }
}
impl<'py> IntoPyObject<'py> for PySurface {
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;
    
    fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
        py.import("madcad.mesh")?.getattr("Mesh")?.call1((
            self.points, self.faces, self.tracks, self.groups, self.options))
    }
}

/// struct mirroring madcad.mesh.Mesh, but ensuring buffers hooks and types for rust
#[derive(FromPyObject)]
pub struct PyWeb {
    pub points: CowBuffer<Vec3>,
    pub faces: CowBuffer<UVec2>,
    pub tracks: CowBuffer<Index>,
    pub groups: Py<PyList>,
    pub options: Py<PyDict>,
}
impl PyWeb {
    pub fn borrow(&self) -> Web<'_> {
        Web {
            points: self.points.as_slice().into(),
            simplices: self.faces.as_slice().into(),
            tracks: self.tracks.as_slice().into(),
            }
    }
}
impl<'py> IntoPyObject<'py> for PyWeb {
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;
    
    fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
        py.import("madcad.mesh")?.getattr("Web")?.call1((
            self.points, self.faces, self.tracks, self.groups, self.options))
    }
}

/// struct mirroring madcad.mesh.Mesh, but ensuring buffers hooks and types for rust
#[derive(FromPyObject)]
pub struct PyWire {
    pub points: CowBuffer<Vec3>,
    pub indices: CowBuffer<Index>,
    pub tracks: CowBuffer<Index>,
    pub groups: Py<PyList>,
    pub options: Py<PyDict>,
}
// impl PyWire {
//     pub fn borrow(&self) -> Wire<'_> {
//         Wire {
//             points: self.points.as_slice().into(),
//             simplices: self.indices.as_slice().into(),
//             tracks: self.tracks.as_slice().into(),
//             }
//     }
// }
impl<'py> IntoPyObject<'py> for PyWire {
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;
    
    fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
        py.import("madcad.mesh")?.getattr("Wire")?.call1((
            self.points, self.indices, self.tracks, self.groups, self.options))
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
