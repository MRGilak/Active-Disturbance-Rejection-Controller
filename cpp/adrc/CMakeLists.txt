cmake_minimum_required(VERSION 3.15)
project(adrc VERSION 1.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing
enable_testing()

# Create library
add_library(adrc
    src/ADRC.cpp
    src/TD.cpp
)

# Warnings for the library (non-fatal by default)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(adrc PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
    target_compile_options(adrc PRIVATE /W4)
endif()

# Include directories for the library itself
target_include_directories(adrc
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_executable(demo_cpp demo.cpp)
target_link_libraries(demo_cpp PRIVATE adrc)

# Unit tests (registered with CTest)
add_executable(adrc_tests tests/test_main.cpp)
target_link_libraries(adrc_tests PRIVATE adrc)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(adrc_tests PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
    target_compile_options(adrc_tests PRIVATE /W4)
endif()

add_test(NAME adrc_tests COMMAND adrc_tests)

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