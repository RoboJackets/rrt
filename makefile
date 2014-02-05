
CXX=clang++
CXX_FLAGS=-std=c++11 -Igtest/include -Isrc

SRC=$(shell find src -type f -name '*.cpp')
OBJ=$(patsubst src/%.cpp, build/%.o, $(SRC))

TEST_SRC=$(wildcard test/*.cpp)
TEST_OBJ=$(patsubst %.cpp, build/%.o, $(TEST_SRC))


all: $(OBJ)

gtest/make/gtest_main.a:
	cd gtest/make && make

build/%.o: src/%.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXX_FLAGS) -o $@ -c $^

test: $(OBJ) $(TEST_OBJ) gtest/make/gtest_main.a
	$(CXX) $(CXX_FLAGS) $(TEST_OBJ) $(OBJ) gtest/make/gtest_main.a -lpthread -o build/test_prg
	build/test_prg

clean:
	rm -rf build
