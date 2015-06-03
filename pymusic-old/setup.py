from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

ext = Extension (
    "music",                     # name of extension
    ["music.pyx"],           # filename of our Cython source
    language="c++",              # this causes Cython to create C++ source
    include_dirs=['/usr/lib/openmpi/include',
                  '/usr/local/lib/python/mpi4py/include',
                  '/usr/local/include'],
    library_dirs = ['/usr/lib/openmpi/lib',
                    '/usr/local/lib'],
    libraries=['music', 'mpi_cxx', 'mpi', 'open-rte', 'open-pal',
               'dl', 'nsl', 'util', 'm'],
    #extra_link_args=[...],       # if needed
    )

ext2 = Extension (
    "music_late",
    ["late.pyx"],
    language = "c++",
    include_dirs = ['/usr/lib/openmpi/include',
                    '/usr/local/lib/python/mpi4py/include'],
    extra_link_args = ['-Wl,-rpath=/home/mdj/sans/music/trunk/pymusic',
                       'music.so']
    )

setup (name = "music",
       version = "1.0",
       description = 'Multi-Simulation Coordinator',
       ext_modules = [ext, ext2],
       cmdclass = {'build_ext': build_ext}
       )
