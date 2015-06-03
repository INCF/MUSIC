
#if MUSIC_USE_MPI
#include <mpi.h>
#include "music-c.h"

/* Communicators */

MPI_Comm MUSIC_setupCommunicatorGlue (MUSIC_Setup *setup);

MPI_Comm
MUSIC_setupCommunicator (MUSIC_Setup *setup)
{
  return (MPI_Comm) MUSIC_setupCommunicatorGlue (setup);
}
#endif
