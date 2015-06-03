############ 
#https://svn.boost.org/trac/boost/ticket/6580
#https://github.com/salilab/imp/issues/732
# this should be in the __init__.py
#
# How to check for openmpi?
#
import sys
def setrc():
    import mpi4py.rc
    mpi4py.rc.initialize = False
    mpi4py.rc.finalize = False

if sys.platform == 'linux2':
    import DLFCN as dl
    flags = sys.getdlopenflags()
    sys.setdlopenflags(dl.RTLD_NOW|dl.RTLD_GLOBAL)

    setrc()
    from pymusic import *

    sys.setdlopenflags(flags)
else:
    setrc()
    from pymusic import *

#import DLFCN as dl
#import sys
#sys.setdlopenflags(dl.RTLD_NOW|dl.RTLD_GLOBAL)
