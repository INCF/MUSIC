## Process this file with Automake to create Makefile.in

ACLOCAL = $(top_srcdir)/aclocal.sh

testbindir = $(libdir)/@PACKAGE@-@PACKAGE_VERSION@
testbin_PROGRAMS = clocksource constsource contdelay eventdelay \
		multiport launchtest

#testshdir = $(datadir)/@PACKAGE@-@PACKAGE_VERSION@/tests
#dist_testsh_DATA = events.test chain.test run_test \
			divergence.test fork.test loop.test \
			cloop.test const.test contclock.test \
			launchtest.test messages.test \
			multicomm.test multiport.test

MUSIC_INCLUDE = -I$(top_srcdir)/src -I$(top_builddir)/src

clocksource_SOURCES = clocksource.cc
clocksource_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
clocksource_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

constsource_SOURCES = constsource.cc
constsource_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
constsource_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

contdelay_SOURCES = contdelay.cc
contdelay_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
contdelay_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

eventdelay_SOURCES = eventdelay.cc
eventdelay_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
eventdelay_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

multiport_SOURCES = multiport.cc
multiport_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
multiport_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

launchtest_SOURCES = launchtest.cc
launchtest_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
launchtest_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@


MKDEP = gcc -M $(DEFS) $(INCLUDES) $(CPPFLAGS) $(CFLAGS)

EXTRA_DIST = events.test chain.test run_test \
			divergence.test fork.test loop.test \
			cloop.test const.test contclock.test \
			launchtest.test messages.test \
			multicomm.test multiport.test

install-data-hook:
	mkdir -p $(DESTDIR)@PKGDATADIR@/tests
	cd $(srcdir);\
	@INSTALL_PROGRAM@ events.test chain.test run_test \
			divergence.test fork.test loop.test \
			cloop.test const.test contclock.test \
			launchtest.test messages.test \
			multicomm.test multiport.test \
			  $(DESTDIR)@PKGDATADIR@/tests