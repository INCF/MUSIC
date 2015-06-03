## Process this file with Automake to create Makefile.in

dist_man1_MANS = music.1 eventcounter.1 eventgenerator.1 eventlogger.1 \
		 eventselect.1 eventsink.1 eventsource.1 viewevents.1

EXTRA_DIST = music-manual.pdf

figures = figures/multisim.pdf \
	  figures/datamapping.pdf \
	  figures/ticklogic.pdf \
	  figures/ticklogic2.pdf \
	  figures/timeline.pdf \
	  figures/remapping.pdf \
	  figures/remapping2.pdf

CLEANFILES = music-manual.aux \
	     music-manual.bbl \
	     music-manual.blg \
	     music-manual.idx \
	     music-manual.ilg \
	     music-manual.ind \
	     music-manual.lof \
	     music-manual.log \
	     music-manual.toc \
	     music-manual.stamp1 \
	     $(figures)

linked_mans = eventgenerator.1 eventlogger.1 eventselect.1 eventsink.1 \
	      eventsource.1 viewevents.1

music-manual.pdf: music-manual.bbl music-manual.ind
	pdflatex music-manual
	pdflatex music-manual

music-manual.bbl: music-manual.stamp1
	bibtex music-manual

music-manual.ind: music-manual.stamp1
	makeindex music-manual

music-manual.stamp1: music-manual.tex music-rfc.bib music-c-int.h wavetest.music waveproducer.cc waveconsumer.cc $(figures)
	test -f music-manual.tex || ln -s $(srcdir)/music-manual.tex
	test -f music-rfc.bib || ln -s $(srcdir)/music-rfc.bib
	test -f music-c-int.h || ln -s $(srcdir)/music-c-int.h
	pdflatex music-manual
	touch music-manual.stamp1

wavetest.music:
	ln -s $(srcdir)/../examples/wavetest.music

waveproducer.cc:
	ln -s $(srcdir)/../examples/waveproducer.cc

waveconsumer.cc:
	ln -s $(srcdir)/../examples/waveconsumer.cc

$(figures): figures/%.pdf : figures/%.asy
	test -d figures || mkdir figures
	asy -f pdf -o $@ $<

$(linked_mans):
	cd $(srcdir) &&	ln eventcounter.1 $@
