
all:
	mkdir -p build
	cd build && cmake .. && make

run: all
	build/rrt-viewer

debug: all
	gdb build/rrt-viewer

clean:
	rm -rf build

