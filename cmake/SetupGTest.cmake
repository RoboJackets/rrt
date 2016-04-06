
include(ExternalProject)

set(lib_dir ${CMAKE_CURRENT_BINARY_DIR}/googletest-prefix/src/googletest-build)
ExternalProject_Add(googletest
    URL ${PROJECT_SOURCE_DIR}/third_party/googletest/googletest
    CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
               -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
    # disable install step
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${lib_dir}/libgtest.a ${lib_dir}/libgtest_main.a
)
set_target_properties(googletest PROPERTIES EXCLUDE_FROM_ALL TRUE)

# specify include dir
ExternalProject_Get_Property(googletest source_dir)
include_directories(${source_dir}/include)

add_library(gtest STATIC IMPORTED)
set_property(TARGET gtest PROPERTY IMPORTED_LOCATION ${lib_dir}/libgtest.a)

add_library(gtest_main STATIC IMPORTED)
set_property(TARGET gtest_main PROPERTY IMPORTED_LOCATION ${lib_dir}/libgtest_main.a)

# specify libraries
set(GTEST_LIBRARIES
    gtest
    gtest_main
    pthread
)
