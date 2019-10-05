noinst_DATA = html

EXTRA_DIST = html Doxyfile

if HAVE_DOXYGEN
html-local: Doxyfile
	$(DOXYGEN) $(DOXYFLAGS) $<
endif