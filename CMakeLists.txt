cmake_minimum_required(VERSION 3.2.0)
project(rrt)

# note: the -fPIC flag is for building with Qt...
#       it complains about position-independent code otherwise
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# setup ccache to speed up recompiles.  It's especially useful when switching back and forth
# between branches where you'd normally have to recompile things many times.
# see http://stackoverflow.com/questions/1815688/how-to-use-ccache-with-cmake
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# includes
# flann library for KD-Tree data structure
find_package(Flann REQUIRED)
add_library(Flann STATIC IMPORTED)

set(RRT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE PATH "Directory containing includes for the RRT")
include_directories(${RRT_INCLUDE_DIR})

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

# debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


# Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
if(APPLE)
    include_directories(SYSTEM /opt/X11/include/)

    # look for the homebrew-installed version of Qt5
    find_package(Qt5Widgets
        REQUIRED
        PATHS /usr/local/Cellar/qt5/*)
else()
    find_package(Qt5Widgets REQUIRED)
endif()
message(STATUS "Found Qt5: ${Qt5Widgets_DIR}")


# libRRT
################################################################################

set(lib_SRC
    "src/rrt/2dplane/2dplane.cpp"
    "src/rrt/2dplane/GridStateSpace.cpp"
    "src/rrt/2dplane/ObstacleGrid.cpp"
)

# add_library("RRT" STATIC ${lib_SRC})
add_library("RRT" ${lib_SRC})
add_dependencies(RRT Flann)
# Fix 'LZ4 not found' issue on some systems
target_link_libraries(RRT ${FLANN_LIBRARIES})

# Find LZ4 if it exists. On some platforms this is required to make Flann work,
# while on others it appears to work fine without linking it in (Flann may have
# previously distributed LZ4 symbols statically, but stopped in later versions?
find_library(LZ4_LIBRARY NAMES lz4)
if(LZ4_LIBRARY)
    target_link_libraries(RRT ${LZ4_LIBRARY})
endif()

# copy headers to install directory
set(lib_HEADERS
    "2dplane/2dplane.hpp"
    "2dplane/GridStateSpace.hpp"
    "2dplane/ObstacleGrid.hpp"
    "2dplane/PlaneStateSpace.hpp"
    "BiRRT.hpp"
    "planning/Path.hpp"
    "StateSpace.hpp"
    "Tree.hpp"
    "util.hpp"
)
foreach(file ${lib_HEADERS})
    get_filename_component(dir ${file} DIRECTORY)
    install(FILES src/rrt/${file} DESTINATION include/rrt/${dir})
endforeach()

# copy library to install directory
install(TARGETS RRT DESTINATION "lib")

# rrt-viewer
################################################################################

set(ui_SRC
    "src/rrt-viewer/main.cpp"
    "src/rrt-viewer/RRTWidget.cpp"
)

qt5_add_resources(RESOURCES src/rrt-viewer/main.qrc)
add_executable("rrt-viewer" ${ui_SRC} ${RESOURCES})
qt5_use_modules("rrt-viewer" Widgets Qml Quick)
target_link_libraries("rrt-viewer" RRT)


# gtest
################################################################################

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    enable_testing()
    include(SetupGTest)

    # test srcs
    set(tests_SRC
        "src/rrt/BiRRTTest.cpp"
        "src/rrt/TreeTest.cpp"
        )

    # test-cpp
    add_executable(test-cpp ${tests_SRC})
    target_link_libraries(test-cpp RRT)
    target_link_libraries(test-cpp ${GTEST_LIBRARIES})
    set_target_properties(test-cpp PROPERTIES EXCLUDE_FROM_ALL TRUE)
    add_dependencies(test-cpp googletest)
endif()
