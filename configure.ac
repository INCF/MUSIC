dnl Process this file with autoconf to produce configure.
AC_INIT(music, 1.1.15)
AM_INIT_AUTOMAKE
AM_CONFIG_HEADER([config.h])
AM_MAINTAINER_MODE

if test "$MPI_CXX" != ""; then
  CXX="$MPI_CXX"
fi

AC_PROG_CXX([mpiCC mpixlcxx mpixlcxx_r mpicxx CC])
MPI_CXX="$CXX"

AC_MSG_CHECKING([which MPI system we think we are using])
case "$MPI_CXX" in
  mpiCC)
    changequote({, })
    MPI_C=mpicc
    if $MPI_CXX -compile_info >/dev/null 2>/dev/null; then
      SYSGUESS=mpich
      CXX="`$MPI_CXX -compile_info | sed -e 's/^\([^ ]*\).*/\1/'`"
    else
      #	mpiCC can be a C compiler under OpenMPI
      if mpicxx -showme >/dev/null 2>/dev/null; then
        SYSGUESS=openmpi
        CXX="`mpicxx -showme | sed -e 's/^\([^ ]*\).*/\1/'`"
      else
        changequote([, ])      
        AC_MSG_ERROR([
Confused when trying to determine MPI_CXX, MPI_CXXFLAGS etc.
Please set them manually.])
        changequote({, })
      fi
    fi
    LAUNCHSTYLE=_mpirun
    TESTDIR=testsuite
    EXTRAS=extras
    changequote([, ])
    ;;
  mpixlcxx_r)
    MPI_C=mpixlc_r
    SYSGUESS=bgp
    CXX="$MPI_CXX"
    #CC="`echo $MPI_CXX | sed -e 's/xlcxx$/xlc/;s/cxx$/cc/'`"
    CC="$MPI_C"
    ;;
  mpixlcxx)
    MPI_C=mpixlc
    SYSGUESS=bgq
    CXX="$MPI_CXX"
    #CC="`echo $MPI_CXX | sed -e 's/xlcxx$/xlc/;s/cxx$/cc/'`"
    CC="$MPI_C"
    ;;
  mpicxx)
    MPI_C=mpicc
    CC="$MPI_C"
    CXX="$MPI_CXX"
    if mpich2version >/dev/null 2>/dev/null; then
       SYSGUESS=mpich2
       AC_DEFINE(HAVE_MPICH2, 1, [Define to 1 if you have MPICH2.])
    else
       SYSGUESS=bgl
    fi
    ;;
  CC)
    # We should be checking MPI version but for the Cray we do it differently
    if uname -r | grep -q cray_gem_s; then
      SYSGUESS=cray_xe6
      AC_DEFINE(CRAY_XE6, 1, [Define if this is a Cray XE6 system.])
      CC=cc
      CXX=CC
    else
      SYSGUESS=unknown
    fi
    ;;
  xlC)
    MPI_C=xlc
    SYSGUESS=bgq_seq
    CXX="$MPI_CXX"
    CC="$MPI_C"
    ;;

  *)
    SYSGUESS=unknown
    ;;
esac
echo "SYSGUESS=$SYSGUESS"

AC_MSG_CHECKING(MPI_CXXFLAGS)
if test "$MPI_CXXFLAGS" != ""; then
  echo "$MPI_CXXFLAGS"
else
  case "$SYSGUESS" in
    mpich|mpich2)
      changequote(<, >)
      MPI_CXXFLAGS="`$MPI_CXX -compile_info | sed -e 's/^[^ ]* //;s/ -c / /'`"
      changequote([, ])
      ;;
    openmpi)
      MPI_CXXFLAGS="`mpicxx -showme:compile`"
      ;;
    bgp)
      MPI_CXXFLAGS="-qarch=450 -qtune=450 -qhot -qnostrict"
      enable_shared=no
      ;;
    bgq)
      MPI_CXXFLAGS="-qarch=qp -qtune=qp -qhot -qnostrict"
      enable_shared=no
      ;;
    cray_xe6)
      MPI_CXXFLAGS=""
      ;;
    bgq_seq)
      MPI_CXXFLAGS=""
      enable_shared=no
      ;;
    *)
      AC_MSG_ERROR([
Could not determine proper value for MPI_CXXFLAGS.  Please see README.])
      ;;
  esac
  echo "$MPI_CXXFLAGS"
fi

AC_MSG_CHECKING(MPI_CFLAGS)
if test "$MPI_CFLAGS" != ""; then
  echo "$MPI_CFLAGS"
else
  case "$SYSGUESS" in
    mpich|mpich2)
      changequote(<, >)
      MPI_CFLAGS="`$MPI_C -compile_info | sed -e 's/^[^ ]* //;s/ -c / /'`"
      changequote([, ])
      ;;
    openmpi)
      MPI_CFLAGS="`mpicc -showme:compile`"
      ;;
    bgp)
      MPI_CFLAGS="-qarch=450 -qtune=450 -qhot -qnostrict"
      enable_shared=no
      ;;
    bgq)
      MPI_CFLAGS="-qarch=qp -qtune=qp -qhot -qnostrict"
      enable_shared=no
      ;;
    cray_xe6)
      MPI_CFLAGS=""
      ;;
    bgq_seq)
      MPI_CFLAGS=""
      enable_shared=no
      ;;
    *)
      AC_MSG_ERROR([
Could not determine proper value for MPI_CFLAGS.  Please see README.])
      ;;
  esac
  echo "$MPI_CFLAGS"
fi

AC_MSG_CHECKING(MPI_LDFLAGS)
if test "$MPI_LDFLAGS" != ""; then
  echo "$MPI_LDFLAGS"
else
  case "$SYSGUESS" in
    mpich|mpich2)
      changequote(<, >)
      MPI_LDFLAGS="`$MPI_CXX -link_info | sed -e 's/^[^ ]* //;s/ -c / /'`"
      changequote([, ])
      ;;
    openmpi)
      MPI_LDFLAGS="`mpicxx -showme:link`"
      ;;
    bgp|bgq|cray_xe6|bgq_seq)
      MPI_LDFLAGS=""
      ;;
    *)
      AC_MSG_ERROR([
Could not determine proper value for MPI_LDFLAGS.  Please see README.])
      ;;
  esac
  echo "$MPI_LDFLAGS"
fi

CXXFLAGS="-g -O3"

#if test "$ac_test_CXXFLAGS" != set; then
#  if test "$CXXFLAGS" = "-O2"; then
#    CXXFLAGS="-O3"
#  else
  #  CXXFLAGS="-O2"
#    CXXFLAGS="-g -O0"
if test $CXX = g++; then
	CXXFLAGS="-pedantic -Wall -Wno-long-long $CXXFLAGS"
fi
#  fi
#fi

AM_PATH_PYTHON
AM_CONDITIONAL([HAVE_PYTHON], [test "$PYTHON" != :])

AC_PROG_CC
AC_PROG_LIBTOOL

dnl Checks for rudeconfig:
AC_HEADER_STDC
AC_C_CONST
AC_C_INLINE
AC_TYPE_SIZE_T
AC_CHECK_TYPES([long long])

AC_CHECK_FUNCS([strrchr mallinfo])

dnl Other checks
OPTIONAL_UTILS=""
AC_CHECK_HEADER([GL/glut.h], OPTIONAL_UTILS+="viewevents ")

AC_LANG_PUSH(C++)
save_CXX="$CXX"
save_LIBS="$LIBS"
dnl Must use $CXX, not $MPI_CXX, here

CXX="$CXX $MPI_CXXFLAGS"
LIBS="$LIBS $MPI_LDFLAGS"
AC_CHECK_FUNCS([rts_get_personality ompi_comm_free])

AC_MSG_CHECKING([for MPI::Init_thread method])
ac_have_cxx_mpi_init_thread=no
AC_COMPILE_IFELSE(
  [AC_LANG_SOURCE([
#include <mpi.h>
int main (int argc, char **argv)
{
  int res = MPI::Init_thread (argc, argv, 0);
}
])],
AC_DEFINE(HAVE_CXX_MPI_INIT_THREAD, 1, [Define to 1 if you have the MPI::Init_thread method])
ac_have_cxx_mpi_init_thread=yes
)
AC_MSG_RESULT($ac_have_cxx_mpi_init_thread)

AC_ARG_ENABLE(mpi,
              [  --disable-mpi           disable mpi support],
              MUSIC_USE_MPI="$enableval", MUSIC_USE_MPI="yes")
if test $MUSIC_USE_MPI = yes ; then 
  MUSIC_USE_MPI=1
  MPI_UTILS="eventsource eventsink eventselect eventgenerator eventcounter eventlogger contsink messagesource"
  EXAMPLESDIR="examples"
else
  MUSIC_USE_MPI=0
  MPI_UTILS=""
  OPTIONAL_UTILS=""
  EXAMPLESDIR=""
fi
AC_ARG_ENABLE(isend,
              [  --disable-isend          disable isend],
              MUSIC_ISENDWAITALL="$enableval", MUSIC_ISENDWAITALL="yes")
if test $MUSIC_ISENDWAITALL = yes ; then 
  MUSIC_ISENDWAITALL=1
else
  MUSIC_ISENDWAITALL=0
fi
AC_ARG_ENABLE(anysource,
              [  --disable-anysource      disable any source receive],
              MUSIC_ANYSOURCE="$enableval", MUSIC_ANYSOURCE="yes")
if test $MUSIC_ANYSOURCE = yes ; then 
  MUSIC_ANYSOURCE=1
else
  MUSIC_ANYSOURCE=0
fi

LIBS="$save_LIBS"
CXX="$save_CXX"
AC_LANG_POP(C++)

if test $ac_cv_type_size_t = yes; then
  MUSIC_HAVE_SIZE_T=1
else
  MUSIC_HAVE_SIZE_T=0
fi

if test $ac_cv_type_long_long = yes; then
  MUSIC_HAVE_LONG_LONG=1
else
  MUSIC_HAVE_LONG_LONG=0
fi

if test "$prefix" = NONE; then
   PKGDATADIR=`eval echo $ac_default_prefix/share/$PACKAGE-$PACKAGE_VERSION`
else
   PKGDATADIR=`eval echo $prefix/share/$PACKAGE-$PACKAGE_VERSION`
fi

if test "$prefix" = NONE; then
   PKGEXTRABINDIR=`eval echo $ac_default_prefix/lib/$PACKAGE-$PACKAGE_VERSION`
else
   PKGEXTRABINDIR=`eval echo $prefix/lib/$PACKAGE-$PACKAGE_VERSION`
fi

#TEST_SH_LIST=${srcdir}/*.test

AC_ARG_WITH([python],
    [AS_HELP_STRING([--without-python], [ignore the presence of Python and disable PyMUSIC])])

AS_IF([test "x$with_python" != "xno"],
      [AM_PATH_PYTHON([2.6], [have_python=yes], [have_python=no])],
      [have_python=no])

AS_IF([test "x$have_python" = "xyes"], [
    PYTHON_INC=`$PYTHON -c 'import sys; from distutils import sysconfig; sys.stdout.write(sysconfig.get_python_inc())'`
    AC_CHECK_FILE(["${PYTHON_INC}/Python.h"], 
    		  [PYMUSIC_CPPFLAGS="-I${PYTHON_INC}"; PYBUFFER_CPPFLAGS="-I${PYTHON_INC}"],
                  [have_python=no])
])

AC_MSG_CHECKING([whether to build PyMUSIC])
AS_IF([test "x$have_python" = "xyes"], [],
      [AS_IF([test "x$with_python" = "xyes"],
             [AC_MSG_ERROR([PyMUSIC requested, but Python not found])])])
AC_MSG_RESULT([$have_python])

AX_CHECK_COMPILE_FLAG([-fno-strict-aliasing],
                      [PYMUSIC_CXXFLAGS="-fno-strict-aliasing"; PYBUFFER_CXXFLAGS="-fno-strict-aliasing"],
		      [])

# FIXME: this means that --without-python make will not recurse into pymusic subdir
if test "x$have_python" = "xyes"; then
    PYMUSIC_SUBDIR="pymusic"
else
    PYMUSIC_SUBDIR=""
fi

AM_CONDITIONAL([HAVE_PYTHON], [test "x$have_python" = "xyes"])

# if Python is found, configure and compile PyMUSIC
if test "x$have_python" = "xyes" ; then
 AC_CONFIG_FILES(pymusic/Makefile)
 AC_CONFIG_FILES(pymusic/setup.py)
fi


AC_SUBST([HAVE_PYTHON])
AC_SUBST([PYMUSIC_SUBDIR])
AC_SUBST([PYEXECDIR], ${pyexecdir})

AC_SUBST([PYTHON])
AC_SUBST([PYTHON_VERSION])

AC_SUBST([PYMUSIC_CPPFLAGS])
AC_SUBST([PYMUSIC_CXXFLAGS])
AC_SUBST([PYBUFFER_CPPFLAGS])
AC_SUBST([PYBUFFER_CXXFLAGS])

AC_SUBST(MPI_CXXFLAGS)
AC_SUBST(MPI_CFLAGS)
AC_SUBST(MPI_LDFLAGS)
AC_SUBST(MUSIC_HAVE_SIZE_T)
AC_SUBST(MUSIC_HAVE_LONG_LONG)
AC_SUBST(MUSIC_USE_MPI)
AC_SUBST(MUSIC_ISENDWAITALL)
AC_SUBST(MUSIC_ANYSOURCE)
AC_SUBST(MPI_UTILS)
AC_SUBST(OPTIONAL_UTILS)
AC_SUBST(TESTDIR)
AC_SUBST(EXAMPLESDIR)
AC_SUBST(EXTRAS)
AC_SUBST(PKGDATADIR)
AC_SUBST(PKGEXTRABINDIR)
AC_SUBST(LAUNCHSTYLE)

AM_CONDITIONAL([LAUNCHSTYLE_SET], [test ! -z "$LAUNCHSTYLE"])
#AC_SUBST([TEST_SH_LIST])

AC_CONFIG_FILES([
  Makefile
  mpidep/Makefile
  src/Makefile
  src/music/music-config.hh
  src/music/version.hh
  rudeconfig/Makefile
  utils/Makefile
  examples/Makefile
  testsuite/Makefile
  testsuite/music_tests.sh
  testsuite/unittests/catch/Makefile
  testsuite/sanitytests/Makefile
  music/Makefile
  music/predict_rank.py
  doc/Makefile
  extras/Makefile
])

AC_OUTPUT

if test "$ac_cv_header_GL_glut_h" = "no"; then
   AC_MSG_NOTICE([
NOTE: viewevents will not be built since the GL/glut.h header was not
      found on this system.  This is no problem unless you plan to
      view spiking activity graphically.])
fi

dnl Local Variables:
dnl comment-start: "dnl "
dnl comment-end: ""
dnl comment-start-skip: "\\bdnl\\b\\s *"
dnl End:
