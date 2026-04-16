cmake_minimum_required(VERSION 3.16)

project(mppi_cpp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MPPI_BUILD_TESTS "Build tests" ON)

# ---- OpenMP fix for macOS ----
if(APPLE)
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY /opt/homebrew/opt/libomp/lib/libomp.dylib)
    include_directories(/opt/homebrew/opt/libomp/include)
endif()

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- Library ----
add_library(mppi
        src/dummy.cpp
        src/MPPIController.cpp
)

target_include_directories(mppi
        PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(mppi
        PUBLIC OpenMP::OpenMP_CXX Eigen3::Eigen
)

# ---- Demo executable ----
add_executable(mppi_demo
        src/main.cpp
)

target_link_libraries(mppi_demo
        PRIVATE mppi

)

# ---- Pendulum ----

add_executable(pendulum_example
        examples/pendulum_example.cpp
)

target_link_libraries(pendulum_example
        PRIVATE mppi
)

# ---- Tests ----
if(MPPI_BUILD_TESTS)
    enable_testing()

    add_executable(mppi_tests
            tests/test_smoke.cpp
    )

    target_link_libraries(mppi_tests
            PRIVATE mppi
    )

    add_test(NAME smoke_test COMMAND mppi_tests)

    add_executable(test_mppi_basic
            tests/test_mppi_basic.cpp
    )

    target_link_libraries(test_mppi_basic
            PRIVATE mppi
    )

    add_test(NAME test_mppi_basic COMMAND test_mppi_basic)

    add_executable(test_mppi_api
            tests/test_mppi_api.cpp
    )

    target_link_libraries(test_mppi_api
            PRIVATE mppi
    )

    add_test(NAME test_mppi_api COMMAND test_mppi_api)

    add_executable(test_mppi_shift
            tests/test_mppi_shift.cpp
    )

    target_link_libraries(test_mppi_shift
            PRIVATE mppi
    )
    add_test(NAME test_mppi_shift COMMAND test_mppi_shift)

    add_executable(test_mppi_reset
            tests/test_mppi_reset.cpp
    )

    target_link_libraries(test_mppi_reset
            PRIVATE mppi
    )

    add_test(NAME test_mppi_reset COMMAND test_mppi_reset)
endif()