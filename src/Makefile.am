## Process this file with Automake to create Makefile.in

ACLOCAL = $(top_srcdir)/aclocal.sh

lib_LTLIBRARIES = libmusic.la libmusic-c.la

libmusic_la_SOURCES = \
	mpi_utils.cc music/mpi_utils.hh \
	runtime.cc music/runtime.hh \
	setup.cc music/setup.hh \
	error.cc music/error.hh music/debug.hh \
	linear_index.cc music/linear_index.hh \
	index_map.cc music/index_map.hh \
	music/data_map.hh \
	array_data.cc music/array_data.hh \
	configuration.cc music/configuration.hh \
	application_map.cc music/application_map.hh \
	application_mapper.cc music/application_mapper.hh \
	ioutils.cc music/ioutils.hh \
	connectivity.cc music/connectivity.hh \
	spatial.cc music/spatial.hh \
	music/application_graph.hh \
	temporal.cc music/temporal.hh \
	parse.cc music/parse.hh \
	port.cc music/port.hh \
	sampler.cc music/sampler.hh \
	event_router.cc music/event_router.hh \
	event_routing_map.cc music/event_routing_map.hh \
	distributor.cc music/distributor.hh \
	collector.cc music/collector.hh \
	clock.cc music/clock.hh \
	subconnector.cc music/subconnector.hh \
	connector.cc music/connector.hh \
	multibuffer.cc music/multibuffer.hh \
	connection.cc music/connection.hh \
	permutation_index.cc music/permutation_index.hh \
	index_map_factory.cc music/index_map_factory.hh \
	scheduler_agent.cc music/scheduler_agent.hh \
	scheduler.cc music/scheduler.hh \
	BIFO.cc music/BIFO.hh \
	FIBO.cc music/FIBO.hh music/message.hh \
	interval.cc music/interval.hh music/interval_tree.hh \
	music/interval_table.hh music/ordered_ilist.hh \
	music/communication.hh music/memory.hh memory.cc \
	music/predict_rank.hh predict_rank.cc \
	music/version.hh version.cc 

libmusic_la_HEADERS = music.hh
libmusic_la_CXXFLAGS = -I$(top_srcdir) @MPI_CXXFLAGS@
libmusic_la_LDFLAGS = $(top_builddir)/mpidep/libmpidep.la $(top_builddir)/rudeconfig/librudeconfig.la \
	-version-info 1:0:0 -export-dynamic @MPI_LDFLAGS@
libmusic_ladir = $(includedir)

libmusic_c_la_SOURCES = \
	music-c.cc music-c-c.c music-c.h \
	predict_rank-c.cc

libmusic_c_la_HEADERS = music-c.h  music/predict_rank-c.h
libmusic_c_la_CXXFLAGS = -DBUILDING_MUSIC_LIBRARY @MPI_CXXFLAGS@
libmusic_c_la_CFLAGS = -DBUILDING_MUSIC_LIBRARY @MPI_CFLAGS@
libmusic_c_la_LDFLAGS = \
	-version-info 1:2:0 -export-dynamic \
	$(top_builddir)/src/libmusic.la @MPI_LDFLAGS@
libmusic_c_ladir = $(includedir)
libmusic_c_la_DEPENDENCIES = libmusic.la

musicincludedir = $(includedir)/music
musicinclude_HEADERS = music/runtime.hh music/setup.hh \
		       music/interval.hh music/interval_tree.hh \
		       music/interval_table.hh music/ordered_ilist.hh \
		       music/index_map.hh music/data_map.hh \
		       music/linear_index.hh music/array_data.hh \
		       music/configuration.hh music/connectivity.hh \
		       music/application_map.hh music/ioutils.hh \
		       music/application_mapper.hh \
		       music/application_graph.hh \
		       music/spatial.hh music/temporal.hh music/error.hh \
		       music/debug.hh music/port.hh music/clock.hh \
		       music/connector.hh music/subconnector.hh \
		       music/connection.hh music/multibuffer.hh \
		       music/permutation_index.hh \
		       music/scheduler_agent.hh music/scheduler.hh \
		       music/index_map_factory.hh \
		       music/sampler.hh music/BIFO.hh \
		       music/FIBO.hh music/event_router.hh \
		       music/event_routing_map.hh \
		       music/collector.hh music/distributor.hh \
		       music/cont_data.hh music/event.hh \
		       music/message.hh music/music-config.hh \
		       music/predict_rank.hh  music/predict_rank-c.h \
		       music/communication.hh music/version.hh \
		       music/memory.hh music/mpi_utils.hh

MKDEP = gcc -M $(DEFS) $(INCLUDES) $(CPPFLAGS) $(CFLAGS)
