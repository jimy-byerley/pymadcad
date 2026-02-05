use pyo3::prelude::*;
use pyo3::PyTypeInfo;
use pyo3::exceptions::{PyBufferError, PyTypeError};
use pyo3::types::{PyList, PyDict, PyString, PyFloat};
use pyo3::ffi::PyObject;
use pyo3::buffer::PyUntypedBuffer;
use std::ffi::CStr;
use std::ops::Deref;
use std::ptr;
use std::marker::PhantomData;

use crate::math::*;
use crate::mesh::*;
use crate::aabox::*;


/// trait mirroring arrex and glm dtype definitions
pub unsafe trait DType: Copy {
    fn py_format() -> &'static CStr;
    fn py_dtype(py: Python<'_>) -> Bound<'_, PyAny>;
}

unsafe impl DType for Float { 
    fn py_format() -> &'static CStr   {c"d"}
    fn py_dtype(py: Python<'_>) -> Bound<'_, PyAny>  {
        PyFloat::type_object(py).into_any()
    }
}
unsafe impl DType for Index { 
    fn py_format() -> &'static CStr   {c"I"}
    fn py_dtype(py: Python<'_>) -> Bound<'_, PyAny>  {
        PyString::new(py, "I").into_any()
    }
}
unsafe impl DType for Vec2 { 
    fn py_format() -> &'static CStr   {c"dd"}
    fn py_dtype(py: Python<'_>) -> Bound<'_, PyAny>  {
        py.import("madcad.mathutils").unwrap().getattr("vec2").unwrap()
    }
}
unsafe impl DType for Vec3 { 
    fn py_format() -> &'static CStr   {c"ddd"}
    fn py_dtype(py: Python<'_>) -> Bound<'_, PyAny>  {
        py.import("madcad.mathutils").unwrap().getattr("vec3").unwrap()
    }
}
unsafe impl DType for UVec2 { 
    fn py_format() -> &'static CStr   {c"II"}
    fn py_dtype(py: Python<'_>) -> Bound<'_, PyAny>  {
        py.import("madcad.mathutils").unwrap().getattr("uvec2").unwrap()
    }
}
/// Padded UVec3 for Python buffer compatibility (uvec3 has 4 bytes padding to 16 bytes)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PaddedUVec3 {
    pub data: [Index; 3],
    _padding: Index,
}
impl From<UVec3> for PaddedUVec3 {
    fn from(v: UVec3) -> Self {
        Self { data: *v.as_array(), _padding: 0 }
    }
}
impl From<PaddedUVec3> for UVec3 {
    fn from(v: PaddedUVec3) -> Self {
        Vector::from(v.data)
    }
}
unsafe impl DType for PaddedUVec3 {
    fn py_format() -> &'static CStr   {c"IIIxxxx"}
    fn py_dtype(py: Python<'_>) -> Bound<'_, PyAny>  {
        py.import("madcad.mathutils").unwrap().getattr("uvec3").unwrap()
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
impl Bytes {
    pub fn new<T: DType>(data: Vec<T>) -> Self {
        let itemsize = core::mem::size_of::<T>();
        unsafe {
            let (ptr, len, cap) = data.into_raw_parts();
            let data = Vec::from_raw_parts(
                ptr as *mut u8,
                len * itemsize,
                cap * itemsize,
                );
            Self { data }
        }
    }
}
#[pymethods]
impl Bytes {
    unsafe fn __getbuffer__(
        slf: Bound<'_, Self>,
        view: *mut pyo3::ffi::Py_buffer,
        flags: std::os::raw::c_int,
    ) -> PyResult<()> {
        let buffer = slf.borrow();
        let format = c"B";

        // Check flags - we are read-only
        if flags & pyo3::ffi::PyBUF_WRITABLE != 0 {
            return Err(PyBufferError::new_err("Bytes is read-only"));
        }
        
        // Fill in the buffer info
        (*view).buf = buffer.data.as_ptr() as *mut _;
        (*view).len = buffer.data.len() as isize;
        (*view).itemsize = 1;
        (*view).readonly = 1;
        (*view).ndim = 1;
        (*view).format = format.as_ptr() as *mut _;

        // Shape and strides for 1D buffer
        (*view).shape = ptr::null_mut();
        (*view).strides = ptr::null_mut();
        (*view).suboffsets = ptr::null_mut();
        (*view).internal = ptr::null_mut();

        // Set the object reference to keep the buffer alive
        (*view).obj = slf.as_ptr();
        pyo3::ffi::Py_INCREF((*view).obj);

        Ok(())
    }

    unsafe fn __releasebuffer__(&self, _view: *mut pyo3::ffi::Py_buffer) {
        // Nothing to do - data is owned by self
    }
}

pub struct PyTypedList<T: DType + 'static> {
    source: Py<PyAny>,
    buffer: PyUntypedBuffer,
    data: PhantomData<T>,
}
impl<T:DType> PyTypedList<T> {
    pub fn as_slice(&self) -> &[T] {
        unsafe {core::slice::from_raw_parts(
            self.buffer.buf_ptr() as *const T, 
            self.buffer.item_count(),
            )}
    }
    pub fn len(&self) -> usize {
        self.buffer.item_count()
    }
    pub fn new(py: Python<'_>, owned: Vec<T>) -> PyResult<Self> {
        let owner = Bytes::new(owned); // give memory to a python object, without copy
        let dtype = T::py_dtype(py); // get the dtype python must see
        let new = py.import("arrex")?.getattr("typedlist")?.call1((owner, dtype))?;
        Self::extract(new.as_borrowed())
    }
}
impl<'a, 'py, T:DType> FromPyObject<'a, 'py> for PyTypedList<T> {
    type Error = PyErr;
    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        let buffer = PyUntypedBuffer::get(obj.as_any())?;
        if T::py_format() != buffer.format()
            {return Err(PyTypeError::new_err("unexpected buffer format"))}
        if !buffer.is_c_contiguous()
            {return Err(PyTypeError::new_err("buffer should be contiguous"))}
        if buffer.item_size() != size_of::<T>()
            {return Err(PyTypeError::new_err("wrong item size"))}
        Ok(Self {
            buffer,
            source: obj.to_owned().unbind(),
            data: PhantomData,
        })
    }
}
impl<'py, T:DType> IntoPyObject<'py> for PyTypedList<T> {
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;
    
    fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
        Ok(self.source.into_bound(py))
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
                Ok(obj)
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
    pub points: PyTypedList<Vec3>,
    pub faces: PyTypedList<PaddedUVec3>,
    pub tracks: PyTypedList<Index>,
    pub groups: Py<PyList>,
    pub options: Py<PyDict>,
}
impl PySurface {
    /// Convert to a borrowed Surface (requires copying faces due to padding)
    pub fn borrow(&self) -> Surface<'_> {
        Surface {
            points: self.points.as_slice().into(),
            simplices: self.faces.as_slice().iter().map(|&f| f.into()).collect::<Vec<_>>().into(),
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
    pub points: PyTypedList<Vec3>,
    pub faces: PyTypedList<UVec2>,
    pub tracks: PyTypedList<Index>,
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
    pub points: PyTypedList<Vec3>,
    pub indices: PyTypedList<Index>,
    pub tracks: PyTypedList<Index>,
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


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uvec3_padded_roundtrip() {
        let original = UVec3::from([10, 20, 30]);
        let padded: PaddedUVec3 = original.into();
        let recovered: UVec3 = padded.into();
        assert_eq!(*original.as_array(), *recovered.as_array());
    }

    #[test]
    fn test_vec_uvec3_to_vec_padded_roundtrip() {
        let originals = vec![
            UVec3::from([1, 2, 3]),
            UVec3::from([4, 5, 6]),
            UVec3::from([7, 8, 9]),
        ];
        let padded: Vec<PaddedUVec3> = originals.iter().map(|&v| v.into()).collect();
        let recovered: Vec<UVec3> = padded.into_iter().map(|p| p.into()).collect();

        for (orig, rec) in originals.iter().zip(recovered.iter()) {
            assert_eq!(orig.as_array(), rec.as_array());
        }
    }
}


// Python interop tests are in tests/test_buffer.py since cdylib + auto-initialize
// causes linker issues. Run with: pytest tests/test_buffer.py
