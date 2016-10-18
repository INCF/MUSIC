bin_PROGRAMS = linear_readout_decoder 

AM_CXXFLAGS = -I$(top_srcdir) -I$(top_srcdir)/src -I$(top_builddir)/src  @MPI_CXXFLAGS@

AM_LDFLAGS = $(top_builddir)/mpidep/libmpidep.la @MPI_LDFLAGS@ \
						$(top_builddir)/src/libmusic.la \
						$(top_builddir)/src/libmusic-c.la 
						

linear_readout_decoder_SOURCES = linear_readout.h linear_readout.cpp 
linear_readout_decoder_LDADD =  -lgsl -lgslcblas -ljsoncpp


