
import ctypes
import sys

def c_argc_argv (argv):
    """ convert python argv into ctypes (int argc, char**argv) """

    from ctypes import c_char_p, c_int

    argc = len (argv)

    # char*[] type
    t_argv = c_char_p*argc

    # construct argv for C
    c_argv = t_argv (*tuple([c_char_p(arg) for arg in argv]))
    c_argc = c_int (argc)

    return (c_argc, c_argv)


libmusic = ctypes.CDLL ("/opt/music/lib/libmusic-c.so")


x = libmusic.MUSIC_predictRank (*c_argc_argv (sys.argv))
    
print x


