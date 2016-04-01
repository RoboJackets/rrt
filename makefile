all:
	mkdir -p build
	cd build && cmake .. -GNinja && ninja

run: all
	build/rrt-viewer

debug: all
	gdb build/rrt-viewer

tests: test-cpp

test-cpp:
	mkdir -p build
	cd build && cmake --target test-cpp .. -GNinja && ninja test-cpp

clean:
	rm -rf build
