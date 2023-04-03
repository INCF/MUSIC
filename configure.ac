dnl Process this file with autoconf to produce configure.
AC_INIT(music, 1.2.1)
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

CXXFLAGS="-g -O3 -std=c++11"

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
AC_CHECK_FUNCS([rts_get_personality ompi_comm_free MPI_Comm_create_errhandler MPI_Init_thread])

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
AM_CONDITIONAL([COND_USE_MPI], [test x$MUSIC_USE_MPI = x1])

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

if test $ac_cv_func_MPI_Comm_create_errhandler = yes; then
  MUSIC_HAVE_MPI_COMM_CREATE_ERRHANDLER=1
else
  MUSIC_HAVE_MPI_COMM_CREATE_ERRHANDLER=0
fi

# Pre-evaluating some autoconf variables
if test "x$prefix" = xNONE; then
  PREFIX=$ac_default_prefix
  prefix=$PREFIX
else
  PREFIX="$prefix"
fi
if test "x$exec_prefix" = xNONE; then
  EXEC_PREFIX="$PREFIX"
  exec_prefix=$EXEC_PREFIX
else
  EXEC_PREFIX="`eval echo $exec_prefix`"
fi
if test "x$bindir" = 'x${exec_prefix}/bin'; then
  BINDIR="$EXEC_PREFIX/bin"
else
  BINDIR="`eval echo $bindir`"
fi
if test "x$libdir" = 'x${exec_prefix}/lib'; then
  LIBDIR="$EXEC_PREFIX/lib"
else
  LIBDIR="`eval echo $libdir`"
fi
if test "x$datarootdir" = 'x${prefix}/share'; then
  DATAROOTDIR="$PREFIX/share"
else
  DATAROOTDIR="`eval echo $datarootdir`"
fi
if test "x$datadir" = 'x${datarootdir}'; then
  DATADIR="$DATAROOTDIR"
else
  DATADIR="`eval echo $datadir`"
fi

PKGDATADIR="$DATADIR/$PACKAGE-$PACKAGE_VERSION"
PKGEXTRABINDIR="$LIBDIR/$PACKAGE-$PACKAGE_VERSION"

#TEST_SH_LIST=${srcdir}/*.test

AC_ARG_WITH([python],
    [AS_HELP_STRING([--without-python], [ignore the presence of Python and disable PyMUSIC])])

AS_IF([test "x$with_python" != "xno"],
      [AS_IF([test "x$with_python" != "xyes"],
             [PYTHON="$with_python"])
       AM_PATH_PYTHON([2.6], [have_python=yes], [have_python=no])],
      [have_python=no])

AS_IF([test "x$have_python" = "xyes"], [
    PYTHON_INC=`$PYTHON -c 'import sys; import sysconfig; sys.stdout.write(sysconfig.get_path("include"))'`
    AC_CHECK_FILE(["${PYTHON_INC}/Python.h"],
    		  [PYMUSIC_CPPFLAGS="-I${PYTHON_INC}"; PYBUFFER_CPPFLAGS="-I${PYTHON_INC}"],
                  [have_python=no])
    # This replaces pyexecdir since Automake 1.16 has a bug which concatenates
    # the prefix (which might be local) and the platlib (might might start with local)
    PYTHON_EXECDIR=`$PYTHON -c 'import sys; import sysconfig; sys.stdout.write(sysconfig.get_path("platlib"))'`
])

AC_MSG_CHECKING([whether to build PyMUSIC])
AS_IF([test "x$have_python" = "xyes"], [],
      [AS_IF([test "x$with_python" = "xyes"],
             [AC_MSG_ERROR([PyMUSIC requested, but Python not found])])])
AC_MSG_RESULT([$have_python])

AX_CHECK_COMPILE_FLAG([-fno-strict-aliasing],
                      [PYMUSIC_CXXFLAGS="-fno-strict-aliasing"; PYBUFFER_CXXFLAGS="-fno-strict-aliasing"],
		      [])

PYMUSIC_SUBDIR=""
if test -z "${COND_USE_MPI_TRUE}"; then

# FIXME: this means that --without-python make will not recurse into pymusic subdir
if test "x$have_python" = "xyes"; then
    MUSICCONFIGDIR="music-config"
    PYMUSIC_SUBDIR="pymusic"
fi

AM_CONDITIONAL([HAVE_PYTHON], [test "x$have_python" = "xyes"])

AC_ARG_ENABLE([deb],
              [  --enable-deb            use deb package layout for install],
              [case "${enableval}" in
	         yes) deb_package=true ;;
		 no) deb_package=false ;;
		 *) deb_package=true ;;
	       esac],[deb_package=false])
AM_CONDITIONAL([DEB_PACKAGE], [test x$deb_package = xtrue])
if test x$deb_package = xtrue; then
  PYTHON_EXECDIR="\${exec_prefix}/lib/python3/dist-packages"
fi
# Have files install in the same location as was retried from Python:
# (Standard place is site-packages but Debian uses dist-packages.)
pythondir=${PYTHON_EXECDIR}
pyexecdir=${PYTHON_EXECDIR}

# if Python is found, configure and compile PyMUSIC
if test "x$have_python" = "xyes" ; then
 AC_CONFIG_FILES(pymusic/Makefile)
 AC_CONFIG_FILES(pymusic/examples/Makefile)
 AC_CONFIG_FILES(pymusic/setup.py)
fi

else # not USE_MPI

# The Python portion of MUSIC as well as the Debian package generation
# are MPI dependent

AM_CONDITIONAL([HAVE_PYTHON], [false])
AM_CONDITIONAL([DEB_PACKAGE], [false])

fi

AC_MSG_CHECKING([whether to build ROS Toolchain])
AC_ARG_WITH([ros])
ROS_SUBDIR=""
AS_IF([test "x$with_ros" = "xyes"], [ROS_SUBDIR="ros"], [with_ros=no])
AC_MSG_RESULT([$with_ros])

# if ROS is to be compiled
if test "x$with_ros" = "xyes" ; then
 AC_CONFIG_FILES(ros/Makefile)
 AC_CONFIG_FILES(ros/adapters/Makefile)
 AC_CONFIG_FILES(ros/encoder/Makefile)
 AC_CONFIG_FILES(ros/decoder/Makefile)
fi

AC_SUBST([HAVE_PYTHON])
AC_SUBST([MUSICCONFIGDIR])
AC_SUBST([PYMUSIC_SUBDIR])
AC_SUBST([ROS_SUBDIR])
AC_SUBST([PYTHON_EXECDIR])

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
AC_SUBST(MUSIC_HAVE_MPI_COMM_CREATE_ERRHANDLER)
AC_SUBST(MUSIC_USE_MPI)
AC_SUBST(MUSIC_ISENDWAITALL)
AC_SUBST(MUSIC_ANYSOURCE)
AC_SUBST(MPI_UTILS)
AC_SUBST(OPTIONAL_UTILS)
AC_SUBST(TESTDIR)
AC_SUBST(EXAMPLESDIR)
AC_SUBST(EXTRAS)
AC_SUBST(BINDIR)
AC_SUBST(LIBDIR)
AC_SUBST(PKGDATADIR)
AC_SUBST(PKGEXTRABINDIR)
AC_SUBST(LAUNCHSTYLE)

AM_CONDITIONAL([LAUNCHSTYLE_SET], [test ! -z "$LAUNCHSTYLE"])
#AC_SUBST([TEST_SH_LIST])

AM_CONDITIONAL([DARWIN], [test $(uname) = Darwin])

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
  testsuite/music_tests
  testsuite/unittests/catch/Makefile
  testsuite/sanitytests/Makefile
  music-config/Makefile
  music-config/predict_rank.py
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
