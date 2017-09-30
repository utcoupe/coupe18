ARCH=$(uname -m)

# Check that the library has been compiled
if [ ! -f lib/libdxl.a ]; then
	echo "library does not exists";
	pushd .
	cd lib/src
	make
	echo "library compiled";
	popd
fi

# Compile the program in a build directory
mkdir -p build
cd build
cmake  ../prgm_ax12 -DCMAKE_BUILD_TYPE=Release
if [ "$ARCH" = "x86_64" ]; then
	make -j4
else
	make
fi

