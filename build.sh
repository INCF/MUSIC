#!/bin/sh

set -e
set -x

./autogen.sh

if [ "$xMPI" = "1" ] ; then
    #openmpi
   export LD_LIBRARY_PATH="/usr/lib/openmpi/lib:$LD_LIBRARY_PATH"
   export CPATH="/usr/lib/openmpi/include:$CPATH"
   export PATH="/usr/include/mpi:$PATH"
   CONFIGURE_MPI=""
else
    CONFIGURE_MPI="--disable-mpi"
fi

if [ "$xInSource" = "InSource+" ] ; then
    CONFIGURE="./configure"
else
    MUSIC_VPATH=$WORKSPACE/build
    mkdir "$MUSIC_VPATH"
    cd "$MUSIC_VPATH"
    CONFIGURE="../configure"
fi

MUSIC_RESULT=$WORKSPACE/result
mkdir "$MUSIC_RESULT"

$CONFIGURE \
    --prefix="$MUSIC_RESULT" \
    $CONFIGURE_MPI

make
make install
make installcheck
