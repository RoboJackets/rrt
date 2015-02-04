CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)
PROJECT(rrt)

# c++11
# note: the -fPIC flag is for building with Qt...
#       it complains about position-independent code otherwise
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

# setup ccache to speed up recompiles.  It's especially useful when switching back and forth
# between branches where you'd normally have to recompile things many times.
# see http://stackoverflow.com/questions/1815688/how-to-use-ccache-with-cmake
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# includes
INCLUDE_DIRECTORIES(src)


# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

# debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


# Qt
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
FIND_PACKAGE(Qt5Widgets REQUIRED)


# rrt-viewer
################################################################################

# rrt-viewer srcs
FILE(GLOB rrt_viewer_SRC
    "src/*.cpp"
    "src/planning/*.cpp"
    "src/2dplane/*.cpp"
    "src/rrt-viewer/*.cpp"
)

# rrt-viewer
ADD_EXECUTABLE("rrt-viewer" ${rrt_viewer_SRC})
QT5_USE_MODULES("rrt-viewer" Widgets)


# gtest
################################################################################

include(ExternalProject)

ExternalProject_Add(googletest
    GIT_REPOSITORY https://git.chromium.org/git/external/googletest.git
    CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
               -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
    # disable install step
    INSTALL_COMMAND ""
)
set_target_properties(googletest PROPERTIES EXCLUDE_FROM_ALL TRUE)

# specify include dir
ExternalProject_Get_Property(googletest source_dir)
set(GTEST_INCLUDE_DIRS ${source_dir}/include)

# specify link libraries
ExternalProject_Get_Property(googletest binary_dir)
set(GTEST_LIBS_DIR ${binary_dir})



# test-cpp
################################################################################

# test srcs
file(GLOB tests_SRC
    "test/*.cpp"
    "src/*.cpp"
    "src/planning/*.cpp"
    "src/2dplane/*.cpp"
)

# stup gtest
enable_testing()
include_directories(${GTEST_INCLUDE_DIRS})

# test-runner
add_executable(test-cpp ${tests_SRC})
target_link_libraries(test-cpp
    ${GTEST_LIBS_DIR}/libgtest.a
    ${GTEST_LIBS_DIR}/libgtest_main.a
    pthread
)
set_target_properties(test-cpp PROPERTIES EXCLUDE_FROM_ALL TRUE)
add_dependencies(test-cpp googletest)

add_test(test1 test-cpp)
