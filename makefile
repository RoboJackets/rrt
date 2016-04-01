MAKE_FLAGS=--no-print-directory

all:
	mkdir -p build
	cd build && cmake .. -DCMAKE_INSTALL_PREFIX:PATH="" && make $(MAKE_FLAGS)

install: all
	cd build && make $(MAKE_FLAGS) install

run: all
	build/rrt-viewer

debug: all
	gdb build/rrt-viewer

tests: test-cpp

test-cpp:
	mkdir -p build
	cd build && cmake --target test-cpp .. && make test-cpp $(MAKE_FLAGS) && cd .. && build/test-cpp

clean:
	rm -rf build
