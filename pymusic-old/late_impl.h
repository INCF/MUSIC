#include <mpi.h>

#include "Python.h"

PyObject* (*wrapIntracomm_ptr) (MPI_Comm comm) = 0;

PyObject* wrapIntracomm (MPI_Comm comm)
{
  return (*wrapIntracomm_ptr) (comm);
}
