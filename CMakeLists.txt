CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)
PROJECT(rrt)

# c++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

# includes
INCLUDE_DIRECTORIES(src)


# Eigen3
INCLUDE_DIRECTORIES("/usr/include/eigen3")
# FIND_PACKAGE(Eigen)


# Qt
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
FIND_PACKAGE(Qt5Widgets REQUIRED)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5_Widgets_EXECUTABLE_COMPILE_FLAGS}")


# rrt-viewer
################################################################################

# rrt-viewer srcs
FILE(GLOB rrt_viewer_SRC
	"src/2dplane/*.cpp"
	"src/rrt-viewer/*.cpp"
)

# rrt-viewer
ADD_EXECUTABLE("rrt-viewer" ${rrt_viewer_SRC})
QT5_USE_MODULES("rrt-viewer" Widgets)


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
