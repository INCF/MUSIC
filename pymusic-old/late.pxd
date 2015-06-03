from mpi4py.libmpi cimport *
from mpi4py.MPI cimport *

ctypedef object wrapIntracomm_func (MPI_Comm comm)

cdef extern from "late.h":
    cdef wrapIntracomm_func* wrapIntracomm_ptr

# Local Variables:
# mode: python
# End:
