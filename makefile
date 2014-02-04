
CXX=clang++
CXX_FLAGS="-Igtest/include"

SRC=$(wildcard *.cpp)
OBJ=$(patsubst %.cpp, build/%.o, $(SRC))

TEST_SRC=$(wildcard test/*.cpp)
TEST_OBJ=$(patsubst %.cpp, build/%.o, $(TEST_SRC))


all: $(OBJ)

gtest: gtest/make/gtest_main.o
	cd gtest/make && make

build/%.o: %.cpp
	$(CXX) $(CXX_FLAGS) -o $@ -c $^

test: $(OBJ) $(TEST_OBJ) gtest
	$(CXX) $(CXX_FLAGS) $(TEST_OBJ) $(OBJ) gtest/make/gtest_main.o -o build/test
	build/test

clean:
	rm -rf build/*
