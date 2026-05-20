cmake_minimum_required(VERSION 3.16)

project(mppi_cpp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MPPI_BUILD_TESTS "Build tests" ON)

# ------------------------------------------------------------------------------
# macOS OpenMP setup (Homebrew libomp)
# ------------------------------------------------------------------------------

if(APPLE)
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY "/opt/homebrew/opt/libomp/lib/libomp.dylib")
    include_directories("/opt/homebrew/opt/libomp/include")
endif()

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)

# ------------------------------------------------------------------------------
# Library
# ------------------------------------------------------------------------------

add_library(mppi
        src/MPPIController.cpp
        src/SMPPIController.cpp
)

target_include_directories(mppi
        PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(mppi
        PUBLIC
        OpenMP::OpenMP_CXX
        Eigen3::Eigen
)

# ------------------------------------------------------------------------------
# Python bindings
# ------------------------------------------------------------------------------

pybind11_add_module(mppi_cpp
        bindings/python_bindings.cpp
)

target_include_directories(mppi_cpp
        PRIVATE ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(mppi_cpp
        PRIVATE mppi
)

# ------------------------------------------------------------------------------
# Examples / Demos
# ------------------------------------------------------------------------------

add_executable(mppi_demo
        src/main.cpp
)

target_link_libraries(mppi_demo
        PRIVATE mppi
)

add_executable(pendulum_example
        examples/pendulum_example.cpp
)

target_link_libraries(pendulum_example
        PRIVATE mppi
)

add_executable(smppi_pendulum_example
        examples/smppi_pendulum_example.cpp
)

target_link_libraries(smppi_pendulum_example
        PRIVATE mppi
)

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------

if(MPPI_BUILD_TESTS)

    enable_testing()

    function(add_mppi_test test_name test_source)
        add_executable(${test_name} ${test_source})
        target_link_libraries(${test_name} PRIVATE mppi)
        add_test(NAME ${test_name} COMMAND ${test_name})
    endfunction()

    add_mppi_test(smoke_test              tests/test_smoke.cpp)
    add_mppi_test(test_mppi_basic         tests/test_mppi_basic.cpp)
    add_mppi_test(test_mppi_api           tests/test_mppi_api.cpp)
    add_mppi_test(test_mppi_shift         tests/test_mppi_shift.cpp)
    add_mppi_test(test_mppi_reset         tests/test_mppi_reset.cpp)
    add_mppi_test(test_mppi_seed          tests/test_mppi_seed.cpp)
    add_mppi_test(test_mppi_bounds        tests/test_mppi_bounds.cpp)
    add_mppi_test(test_mppi_1d            tests/test_mppi_1d.cpp)
    add_mppi_test(test_mppi_terminal      tests/test_mppi_terminal.cpp)

    add_mppi_test(test_smppi_basic        tests/test_smppi_basic.cpp)
    add_mppi_test(test_smppi_smoothness   tests/test_smppi_smoothness.cpp)
    add_mppi_test(benchmark_cpp_native tests/benchmark_cpp_native.cpp)

endif()
