#
# distutils: language = c++
#

# Compatible with Python >= 2.6, Cython
# Not PYPY apparently

from libc.stdlib cimport malloc, free

from mpi4py import MPI

###########################################################

cdef dict TypeDict = {
    "?" : MPI.C_BOOL,
    "c" : MPI.CHAR,
    "b" : MPI.SIGNED_CHAR,
    "h" : MPI.SHORT,
    "i" : MPI.INT,
    "l" : MPI.LONG,
    "q" : MPI.LONG_LONG,
    "p" : MPI.AINT,
    "B" : MPI.UNSIGNED_CHAR,
    "H" : MPI.UNSIGNED_SHORT,
    "I" : MPI.UNSIGNED,
    "L" : MPI.UNSIGNED_LONG,
    "Q" : MPI.UNSIGNED_LONG_LONG,
    "f" : MPI.FLOAT,
    "d" : MPI.DOUBLE,
    "g" : MPI.LONG_DOUBLE,
    "Zf": MPI.C_FLOAT_COMPLEX,
    "Zd": MPI.C_DOUBLE_COMPLEX,
    "Zg": MPI.C_LONG_DOUBLE_COMPLEX,
    "F" : MPI.C_FLOAT_COMPLEX,
    "D" : MPI.C_DOUBLE_COMPLEX,
    "G" : MPI.C_LONG_DOUBLE_COMPLEX,
}

cdef object getformat(Py_buffer* pybuf):
    cdef object obj = <object> pybuf.obj

    # numpy.ndarray
    try: return obj.dtype.char
    except (AttributeError, TypeError): pass

    # array.array
    try: return obj.typecode
    except (AttributeError, TypeError): pass

    if pybuf.format is NULL: return "B"
    return PyUnicodeString_FromString(pybuf.format)

cdef int bufflags = PyBUF_ANY_CONTIGUOUS | PyBUF_WRITABLE | PyBUF_FORMAT

cdef class Buffer(object):
    def __cinit__(self, object data):
        if not PyObject_CheckBuffer(data):
            raise TypeError("object does not present buffer interface")

        cdef Py_buffer* pybuf = &self.pybuf
        PyObject_GetBuffer(data, pybuf, bufflags)
        self.dtype = TypeDict[getformat(pybuf)]
        self.items = pybuf.len / pybuf.itemsize

    def __dealloc__(self):
        PyBuffer_Release(&self.pybuf)
