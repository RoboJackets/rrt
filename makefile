
# circleci has 2 cores, but advertises 32, which causes OOMs
ifeq ($(CIRCLECI), true)
	NINJA_FLAGS=-j2
endif


all:
	mkdir -p build
	cd build && cmake .. -DCMAKE_INSTALL_PREFIX:PATH="" -GNinja && ninja $(NINJA_FLAGS)

install: all
	cd build && ninja $(NINJA_FLAGS) install

run: all
	build/rrt-viewer

debug: all
	gdb build/rrt-viewer

tests: test-cpp
	build/test-cpp

test-cpp:
	mkdir -p build
	cd build && cmake --target test-cpp .. -GNinja && ninja $(NINJA_FLAGS) test-cpp

clean:
	rm -rf build

STYLIZE_DIFFBASE ?= master
STYLE_EXCLUDE_DIRS=build third_party
# automatically format code according to our style config defined in .clang-format
pretty:
	@stylize --diffbase=$(STYLIZE_DIFFBASE) --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS)
# check if everything in our codebase is in accordance with the style config defined in .clang-format
# a nonzero exit code indicates that there's a formatting error somewhere
checkstyle:
	@printf "Run this command to reformat code if needed:\n\ngit apply <(curl file://clean.patch)\n\n"
	@stylize --diffbase=$(STYLIZE_DIFFBASE) --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS) --check --output_patch_file="/tmp/clean.patch"
