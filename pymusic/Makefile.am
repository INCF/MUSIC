## Process this file with Automake to create Makefile.in

SUBDIRS = examples

EXTRA_DIST = setup.py.in tests.py pymusic.pyx pymusic.pxd pybuffer.pyx pybuffer.pxd \
             music/__init__.py music/music_c.h music/pybuffer.pxd music/pymusic.pxd \
             music/pymusic_c.h mpi_compat.h \
             looseversion/__init__.py looseversion/LICENSE

BUILT_SOURCES = pymusic.cpp pybuffer.cpp


pyconfig.pxi: tests.py
	cd $(top_srcdir)/pymusic; $(PYTHON) tests.py

pybuffer.cpp: pyconfig.pxi pybuffer.pyx pybuffer.pxd 
	cd $(top_srcdir)/pymusic; $(PYTHON) -c \
	"from Cython.Build import cythonize; \
	cythonize('pybuffer.pyx', verbose=1)"

pymusic.cpp: pyconfig.pxi pymusic.pyx pymusic.pxd 
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
	-I$(top_srcdir)/src -I$(top_builddir)/src

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

if DEB_PACKAGE
install-exec-hook:
	$(PYTHON) setup.py build \
	  --build-base=$(abs_builddir)/build install \
	  --old-and-unmanageable \
	  --install-layout=deb \
	  --prefix=$(DESTDIR)$(prefix)
else
install-exec-hook:
	$(PYTHON) setup.py build \
	  --build-base=$(abs_builddir)/build install \
	  --old-and-unmanageable \
	  --prefix=$(DESTDIR)$(prefix) \
	  --install-lib=$(DESTDIR)$(PYTHON_EXECDIR) \
	  --install-scripts=$(DESTDIR)$(bindir) \
	  --install-data=$(DESTDIR)$(pkgdatadir)
endif

clean-local:
	-rm -rf $(abs_builddir)/build
