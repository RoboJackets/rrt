all:
	mkdir -p build
	cd build && cmake .. -DCMAKE_INSTALL_PREFIX:PATH="" -GNinja && ninja

install: all
	cd build && ninja install

run: all
	build/rrt-viewer

debug: all
	gdb build/rrt-viewer

tests: test-cpp
	build/test-cpp

test-cpp:
	mkdir -p build
	cd build && cmake --target test-cpp .. -GNinja && ninja test-cpp

clean:
	rm -rf build
