## Process this file with Automake to create Makefile.in

ACLOCAL = $(top_srcdir)/aclocal.sh

testlibdir =  $(libdir)/@PACKAGE@-@PACKAGE_VERSION@
testlib_PROGRAMS =  test_ag #test_tng 

MUSIC_INCLUDE = -I$(top_srcdir)/src -I$(top_builddir)/src

test_ag_SOURCES = test_ag.cc
test_ag_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
test_ag_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

#test_tng_SOURCES = tc_temporal_negotiator_graph.cc
#test_tng_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
#test_tng_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@


MKDEP = gcc -M $(DEFS) $(INCLUDES) $(CPPFLAGS) $(CFLAGS)
