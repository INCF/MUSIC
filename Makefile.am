SUBDIRS = mpidep rudeconfig src utils @TESTDIR@ @EXTRAS@ doc @EXAMPLESDIR@ @MUSICCONFIGDIR@ @PYMUSIC_SUBDIR@ @ROS_SUBDIR@

debdir=../@PACKAGE_NAME@-@PACKAGE_VERSION@

debian-source: dist
	mv @PACKAGE_NAME@-@PACKAGE_VERSION@.tar.gz ../@PACKAGE_NAME@_@PACKAGE_VERSION@.orig.tar.gz
	@test ! -e $(debdir) || ( echo "*** Remove directory ../@PACKAGE_NAME@-@PACKAGE_VERSION@" && exit 1 )
	( cd ..; tar zxf @PACKAGE_NAME@_@PACKAGE_VERSION@.orig.tar.gz )
	mkdir $(debdir)/debian
	cp -p debian/* $(debdir)/debian
	cp -p doc/music-manual.tex doc/music-rfc.bib doc/music-c-int.h $(debdir)/doc
	mkdir $(debdir)/doc/figures
	cp -p doc/figures/*.asy $(debdir)/doc/figures

EXTRA_DIST = BUGS

#dist-hook:
#	rm $(distdir)/TODO
installcheck-local:
if LAUNCHSTYLE_SET
	/bin/sh $(DESTDIR)$(bindir)/music_tests.sh
else
	@echo "checkinstall target is supported only when compiled with openMPI or MPICH libraries."
	@exit 1
endif



distuninstallcheck:
        @:

#uninstall:
#	@echo "make: uninstall target is not supported."
