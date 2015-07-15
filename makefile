DEBUGGER=lldb

all:
	mkdir -p build
	cd build && cmake .. && make

run: all
	build/rrt-viewer

debug: all
	$(DEBUGGER) build/rrt-viewer -ex run

debug: all
	$(DEBUGGER) build/rrt-viewer

tests: test-cpp

test-cpp:
	mkdir -p build
	cd build && cmake --target test-cpp .. && make test-cpp && cd .. && build/test-cpp

clean:
	rm -rf build
