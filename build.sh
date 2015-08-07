#!/bin/sh

set -e
set -x

./autogen.sh

#openmpi
export LD_LIBRARY_PATH="/usr/lib/openmpi/lib:$LD_LIBRARY_PATH"
export CPATH="/usr/lib/openmpi/include:$CPATH"
export PATH="/usr/include/mpi:$PATH"
if [ "$xMPI" = "1" ] ; then
    
   CONFIGURE_MPI=""
else
    CONFIGURE_MPI="--disable-mpi"
fi

#if [ "$xInSource" = "InSource+" ] ; then
 
 #   CONFIGURE="./configure"
#else
MUSIC_VPATH=build
mkdir "$MUSIC_VPATH"
cd "$MUSIC_VPATH"
CONFIGURE="../configure"
#fi

MUSIC_RESULT=result
mkdir "$MUSIC_RESULT"

MUSIC_RESULT=$(readlink -f $MUSIC_RESULT)
$CONFIGURE \
    --prefix="$MUSIC_RESULT" \
    $CONFIGURE_MPI

make
make install
make installcheck
