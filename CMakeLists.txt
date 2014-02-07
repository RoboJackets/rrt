CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(rrt)

ADD_DEFINITIONS(-std=c++11)

#include
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(/usr/local/include/eigen3)

# Qt
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})


# rrt-viewer
################################################################################

# rrt-viewer srcs
FILE(GLOB rrt_viewer_SRC
	"src/2dplane/*.cpp"
	"src/rrt-viewer/*.cpp"
)

# qt-compiled rrt-viewer headers
FILE(GLOB rrt_viewer_HEADERS "src/rrt-viewer/*.hpp")
QT4_WRAP_CPP(rrt_viewer_headers_MOC ${rrt_viewer_HEADERS})

# rrt-viewer
ADD_EXECUTABLE("rrt-viewer" ${rrt_viewer_SRC} ${rrt_viewer_headers_MOC})
TARGET_LINK_LIBRARIES("rrt-viewer" ${QT_LIBRARIES})


# test-runner
################################################################################

# test srcs
FILE(GLOB tests_SRC
	"test/*.cpp"
)

# stup gtest
ADD_SUBDIRECTORY(gtest)
ENABLE_TESTING()
INCLUDE_DIRECTORIES(gtest/include)

# test-runner
ADD_EXECUTABLE("test-runner" ${tests_SRC})
TARGET_LINK_LIBRARIES("test-runner" gtest gtest_main)
