
EXTRA_DIST = setup.py.in pymusic.pyx pymusic.pxd music_c.h pybuffer.pyx pybuffer.pxd

BUILT_SOURCES = pymusic.cpp pybuffer.cpp

pybuffer.cpp: pybuffer.pyx pybuffer.pxd
	cd $(top_srcdir)/pymusic; $(PYTHON) -c \
	"from Cython.Build import cythonize; \
	cythonize('pybuffer.pyx', verbose=1)"

pymusic.cpp: pymusic.pyx pymusic.pxd
	cd $(top_srcdir)/pymusic; $(PYTHON) -c \
	"from Cython.Build import cythonize; \
	cythonize('pymusic.pyx', verbose=1)"

pkgpyexec_LTLIBRARIES = pymusic.la pybuffer.la
pymusic_la_SOURCES= pymusic.cpp # main sources to link to
pymusic_la_LDFLAGS = -shared -module -avoid-version -export-dynamic $(MPI_LDFLAGS)
pybuffer_la_SOURCES= pybuffer.cpp # main sources to link to
pybuffer_la_LDFLAGS = -shared -module -avoid-version -export-dynamic $(MPI_LDFLAGS)

pymusic_la_CPPFLAGS = \
	$(MPI_CPPFLAGS) \
	$(PYMUSIC_CPPFLAGS) \
	-I$(top_srcdir)/src

pymusic_la_CXXFLAGS = \
	$(PYMUSIC_CXXFLAGS) \
	$(MPI_CXXFLAGS)

pymusic_la_LIBADD = \
	$(top_builddir)/src/libmusic.la \
	$(top_builddir)/src/libmusic-c.la

pybuffer_la_CPPFLAGS = \
	$(MPI_CPPFLAGS) \
	$(PYBUFFER_CPPFLAGS) \
	-I$(top_srcdir)/src

pybuffer_la_CXXFLAGS = \
	$(PYBUFFER_CXXFLAGS) \
	$(MPI_CXXFLAGS)

install-exec-hook:
	$(PYTHON) setup.py build \
	  --build-base=$(abs_builddir)/build install \
	  --prefix=$(DESTDIR)$(prefix) \
	  --install-lib=$(DESTDIR)$(pyexecdir) \
	  --install-scripts=$(DESTDIR)$(bindir) \
	  --install-data=$(DESTDIR)$(pkgdatadir)

clean-local:
	-rm -rf $(abs_builddir)/build
