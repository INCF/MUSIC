#include <mpi.h>

#include "Python.h"

extern PyObject* (*wrapIntracomm_ptr) (MPI_Comm comm);

struct _mpi_message_t;
typedef _mpi_message_t* MPI_Message; /* ??? */
