cimport mpi4py.MPI as MPI
from mpi4py.mpi_c cimport *

cdef extern from "music/pymusic_c.h":
    object PyUnicodeString_FromString(const char*)

from cpython.ref cimport PyObject

# Python 3 buffer interface (PEP 3118)
cdef extern from "Python.h":
    ctypedef struct Py_buffer:
        void* buf
        PyObject* obj        # owned reference 
        Py_ssize_t len
        Py_ssize_t itemsize  # This is Py_ssize_t so it can be
                              # pointed to by strides in simple case.*/
        bint readonly
        int ndim
        char* format
        Py_ssize_t *shape
        Py_ssize_t *strides
        Py_ssize_t *suboffsets
        Py_ssize_t smalltable[2]  # static store for shape and strides of
                                  # mono-dimensional buffers.
        void *internal

    cdef enum:
        PyBUF_SIMPLE
        PyBUF_WRITABLE
        PyBUF_FORMAT
        PyBUF_ANY_CONTIGUOUS
        PyBUF_ND
        PyBUF_STRIDES

    int  PyObject_CheckBuffer(object)
    int  PyObject_GetBuffer(object, Py_buffer *, int) except -1
    void PyBuffer_Release(Py_buffer *)
    int  PyBuffer_FillInfo(Py_buffer*, object, void*, Py_ssize_t, bint, int) except -1

cdef class Buffer(object):
    cdef Py_buffer pybuf
    cdef MPI.Datatype dtype
    cdef Py_ssize_t items
