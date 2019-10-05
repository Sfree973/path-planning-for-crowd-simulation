AM_CPPFLAGS = -I@top_srcdir@/RVOLIB $(OPENMP_CXXFLAGS)

dist_noinst_DATA = example.sln example.vcproj
noinst_PROGRAMS = example

example_SOURCES = \
	example.cpp

example_LDADD = @top_builddir@/RVOLIB/librvo.a $(OPENMP_CXXFLAGS)