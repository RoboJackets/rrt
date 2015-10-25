
include(ExternalProject)

ExternalProject_Add(googletest
    URL ${PROJECT_SOURCE_DIR}/third_party/googletest
    CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
               -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
    # disable install step
    INSTALL_COMMAND ""
)

# specify include dir
ExternalProject_Get_Property(googletest source_dir)
include_directories(${source_dir}/include)

# specify link libraries
ExternalProject_Get_Property(googletest binary_dir)
set(GTEST_BOTH_LIBRARIES
    ${binary_dir}/libgtest.a
    ${binary_dir}/libgtest_main.a
    pthread
)
