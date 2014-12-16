
all:
	mkdir -p build
	cd build && cmake .. && make

run: all
	build/rrt-viewer

debug: all
	gdb build/rrt-viewer

tests: all
	build/test-runner

clean:
	rm -rf build

