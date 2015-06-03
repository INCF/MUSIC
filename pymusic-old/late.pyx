from late cimport *

cdef wrapIntracomm (MPI_Comm comm):
    cdef Intracomm icomm = Intracomm ()
    icomm.ob_mpi = comm
    return icomm

wrapIntracomm_ptr = wrapIntracomm

# Local Variables:
# mode: python
# End:
