## Process this file with Automake to create Makefile.in

ACLOCAL = $(top_srcdir)/aclocal.sh

exbindir = $(libdir)/@PACKAGE@-@PACKAGE_VERSION@
exbin_PROGRAMS = waveproducer waveconsumer testproducer testconsumer

exdatadir = $(datadir)/@PACKAGE@-@PACKAGE_VERSION@/examples
dist_exdata_DATA = messages.music wavetest.music test.music \
			viewevents.music demo.music demolarge.music \
			neuronGrid.data neuronGridLARGE.data \
			spikes0.dat spikes1.dat README

MUSIC_INCLUDE = -I$(top_srcdir)/src -I$(top_builddir)/src

waveproducer_SOURCES = waveproducer.cc
waveproducer_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
waveproducer_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

waveconsumer_SOURCES = waveconsumer.cc
waveconsumer_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
waveconsumer_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

testproducer_SOURCES = testproducer.cc
testproducer_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
testproducer_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

testconsumer_SOURCES = testconsumer.cc
testconsumer_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
testconsumer_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

MKDEP = gcc -M $(DEFS) $(INCLUDES) $(CPPFLAGS) $(CFLAGS)
