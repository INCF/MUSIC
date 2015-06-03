## Process this file with Automake to create Makefile.in

ACLOCAL = $(top_srcdir)/aclocal.sh

bin_PROGRAMS = music @MPI_UTILS@ @OPTIONAL_UTILS@

EXTRA_PROGRAMS = eventsource eventsink eventselect \
				eventgenerator eventcounter eventlogger \
				contsink messagesource viewevents

MUSIC_INCLUDE = -I$(top_srcdir)/src -I$(top_builddir)/src

 music_SOURCES = music.cc
 music_CXXFLAGS = $(MUSIC_INCLUDE) -I$(top_srcdir) @MPI_CXXFLAGS@
 music_LDADD = $(top_builddir)/src/libmusic.la $(top_builddir)/mpidep/libmpidep.la @MPI_LDFLAGS@

 eventsource_SOURCES = eventsource.cc datafile.h datafile.cc
 eventsource_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
 eventsource_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

 eventsink_SOURCES = eventsink.cc
 eventsink_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
 eventsink_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

 eventselect_SOURCES = eventselect.cc
 eventselect_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
 eventselect_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

 eventgenerator_SOURCES = eventgenerator.cc
 eventgenerator_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
 eventgenerator_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

 eventcounter_SOURCES = eventcounter.cc
 eventcounter_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
 eventcounter_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

 eventlogger_SOURCES = eventlogger.cc
 eventlogger_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
 eventlogger_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

 contsink_SOURCES = contsink.cc
 contsink_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
 contsink_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

 messagesource_SOURCES = messagesource.cc
 messagesource_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
 messagesource_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@

 viewevents_SOURCES = viewevents.cpp VisualiseNeurons.cpp VisualiseNeurons.h
 viewevents_CXXFLAGS = $(MUSIC_INCLUDE) @MPI_CXXFLAGS@
 viewevents_LDADD = $(top_builddir)/src/libmusic.la @MPI_LDFLAGS@  -lglut -lGL -lGLU 


MKDEP = gcc -M $(DEFS) $(INCLUDES) $(CPPFLAGS) $(CFLAGS)
