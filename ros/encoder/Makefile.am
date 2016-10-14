bin_PROGRAMS = rate_encoder poisson_encoder nef_encoder 

AM_CXXFLAGS = -I$(top_srcdir) -I$(top_srcdir)/src -I$(top_builddir)/src  @MPI_CXXFLAGS@

AM_LDFLAGS = $(top_builddir)/mpidep/libmpidep.la @MPI_LDFLAGS@ \
						$(top_builddir)/src/libmusic.la \
						$(top_builddir)/src/libmusic-c.la 
						

rate_encoder_SOURCES = rate_encoder.h rate_encoder.cpp 
rate_encoder_LDADD = 

poisson_encoder_SOURCES = poisson_encoder.h poisson_encoder.cpp 
poisson_encoder_LDADD = 

nef_encoder_SOURCES = nef_encoder.h nef_encoder.cpp iaf_neuron.h 
nef_encoder_LDADD = 


