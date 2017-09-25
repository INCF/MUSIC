#!/bin/zsh
export LD_LIBRARY_PATH=/media/storage/HBP/install/lib:$LD_LIBRARY_PATH
export PATH=/media/storage/HBP/install/bin:$PATH
gdb -command=`pwd`/gdb.gdb --args music test.music


