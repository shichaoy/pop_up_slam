# iSAM Makefile
# providing shortcuts to cmake for building outside the source tree
# Michael Kaess, 2010

# default parameters, including multi-core building
make = make -j 4 -C build --no-print-directory

# creates isam libraries and isam binary
all: build build/Makefile
	@$(make)

# generate documentation (requires doxygen and graphviz)
.PHONY: doc
doc:
	@doxygen doc/doxygen/isam.dox

# remove all generated files and directories
.PHONY: distclean
distclean:
	@rm -rf build doc/html lib bin release
	@find . -name CMakeFiles |xargs rm -rf # clean up in case "cmake ." was called
	@find . -name cmake_install.cmake -delete
	@find . -name CMakeCache.txt -delete

# internal target: the actual build directory
build:
	@mkdir -p build bin lib include

# internal target: populate the build directory
build/Makefile:
	cd build && cmake ..


# create all executables in the examples/ directory
.PHONY: examples
examples:
	@$(make) examples

# create all executables in the misc/ directory
.PHONY: misc
misc:
	@$(make) misc

# default target: any target such as "clean", "example"...
# is simply passed on to the cmake-generated Makefile 
%::
	@$(make) $@
