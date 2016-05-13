cmake_minimum_required(VERSION 2.8.9)
project(rrt)

# c++11
# note: the -fPIC flag is for building with Qt...
#       it complains about position-independent code otherwise
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

# setup ccache to speed up recompiles.  It's especially useful when switching back and forth
# between branches where you'd normally have to recompile things many times.
# see http://stackoverflow.com/questions/1815688/how-to-use-ccache-with-cmake
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# includes
include_directories(src)

include_directories(flann/src/cpp)

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

# debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


# Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
if(APPLE)
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
    "src/2dplane/2dplane.cpp"
    "src/2dplane/GridStateSpace.cpp"
    "src/2dplane/ObstacleGrid.cpp"
    "src/2dplane/PlaneStateSpace.cpp"
)

add_library("RRT" ${lib_SRC})


# rrt-viewer
################################################################################

set(ui_SRC
    "src/rrt-viewer/main.cpp"
    "src/rrt-viewer/MainWindow.cpp"
    "src/rrt-viewer/RRTWidget.cpp"
)

add_executable("rrt-viewer" ${ui_SRC})
QT5_USE_MODULES("rrt-viewer" Widgets)
target_link_libraries("rrt-viewer" RRT)


# gtest
################################################################################

enable_testing()
include(SetupGTest)

# test srcs
set(tests_SRC
    "src/BiRRTTest.cpp"
    "src/TreeTest.cpp"
)

# test-cpp
add_executable(test-cpp ${tests_SRC})
target_link_libraries(test-cpp RRT)
target_link_libraries(test-cpp ${GTEST_BOTH_LIBRARIES})
target_link_libraries(test-cpp pthread)
set_target_properties(test-cpp PROPERTIES EXCLUDE_FROM_ALL TRUE)
add_dependencies(test-cpp googletest)
