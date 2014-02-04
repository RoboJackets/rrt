
CXX=clang++
CXX_FLAGS=-Igtest/include -std=c++11

SRC=$(wildcard *.cpp)
OBJ=$(patsubst %.cpp, build/%.o, $(SRC))

TEST_SRC=$(wildcard test/*.cpp)
TEST_OBJ=$(patsubst %.cpp, build/%.o, $(TEST_SRC))


all: $(OBJ)

gtest/make/gtest_main.a:
	cd gtest/make && make

build/%.o: %.cpp
	mkdir -p build
	mkdir -p build/test
	$(CXX) $(CXX_FLAGS) -o $@ -c $^

test: $(OBJ) $(TEST_OBJ) gtest/make/gtest_main.a
	$(CXX) $(CXX_FLAGS) $(TEST_OBJ) $(OBJ) gtest/make/gtest_main.a -o build/test_prg
	build/test_prg

clean:
	rm -rf build
