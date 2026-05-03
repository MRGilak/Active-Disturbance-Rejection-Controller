cmake_minimum_required(VERSION 3.15)
project(adrc VERSION 1.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create library
add_library(adrc
    src/ADRC.cpp
    src/TD.cpp
)

# Include directories for the library itself
target_include_directories(adrc
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_executable(demo_cpp demo.cpp)
target_link_libraries(demo_cpp PRIVATE adrc)

# Install rules
install(TARGETS adrc
    EXPORT adrcTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)