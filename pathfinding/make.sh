DIR=build
mkdir -p $DIR
cd build
#rm -f CMakeCache.txt
#rm -rf CMakeFiles
cmake  ../ -DCMAKE_BUILD_TYPE=Release
if [ "$ARCH" = "x86_64" ]; then
	make -j4
else
	make
fi

