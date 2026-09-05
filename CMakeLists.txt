cmake_minimum_required(VERSION 3.16)
project(NRSA
    VERSION 0.1.0
    DESCRIPTION "NRSA — open structural analysis & RCC design engine (BNBC 2020 / ACI 318)"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -Wall -Wextra")

option(NRSA_BUILD_TESTS "Build the unit test suite" ON)
option(NRSA_BUILD_EXAMPLES "Build example programs" ON)

# ---------------------------------------------------------------------------
# Core library — src/core, src/fem, src/analysis, src/design, src/modeler
# grow into this target module-by-module; each subdirectory below only
# needs to exist and contain sources once that module is implemented.
# ---------------------------------------------------------------------------
file(GLOB_RECURSE NRSA_CORE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/core/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/fem/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/analysis/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/design/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/modeler/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/bim/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/boq/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/bbs/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/drawing/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/reports/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/validation/*.cpp"
)

add_library(nrsa_core STATIC ${NRSA_CORE_SOURCES})
target_include_directories(nrsa_core PUBLIC
    "${CMAKE_SOURCE_DIR}/src"
)

# ---------------------------------------------------------------------------
# API (headers-only for now until src/api gains sources of its own)
# ---------------------------------------------------------------------------
file(GLOB NRSA_API_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/api/*.cpp")
if(NRSA_API_SOURCES)
    add_library(nrsa_api STATIC ${NRSA_API_SOURCES})
    target_link_libraries(nrsa_api PUBLIC nrsa_core)
endif()

# ---------------------------------------------------------------------------
# Tests (Catch2-style single-header test runner to start — swap for a
# fetched Catch2/GoogleTest dependency once the test suite grows past what
# a header-only runner comfortably handles)
# ---------------------------------------------------------------------------
if(NRSA_BUILD_TESTS)
    enable_testing()
    file(GLOB NRSA_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/tests/*.cpp")
    foreach(test_src ${NRSA_TEST_SOURCES})
        get_filename_component(test_name ${test_src} NAME_WE)
        add_executable(${test_name} ${test_src})
        target_link_libraries(${test_name} PRIVATE nrsa_core)
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------
if(NRSA_BUILD_EXAMPLES)
    file(GLOB NRSA_EXAMPLE_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/examples/*.cpp")
    foreach(ex_src ${NRSA_EXAMPLE_SOURCES})
        get_filename_component(ex_name ${ex_src} NAME_WE)
        add_executable(${ex_name} ${ex_src})
        target_link_libraries(${ex_name} PRIVATE nrsa_core)
    endforeach()
endif()
