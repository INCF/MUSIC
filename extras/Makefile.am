# Define binary programs to be installed
bin_PROGRAMS = music

# Define scripts to be installed
bin_SCRIPTS = music_mpirun

# Include files for distribution
EXTRA_DIST = music_mpirun

# Custom installation hook 
install-data-hook:
    @INSTALL_PROGRAM@ $(srcdir)/music_mpirun $(DESTDIR)$(bindir)/music_mpirun
