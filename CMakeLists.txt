cmake_minimum_required(VERSION 3.10)
project(EKF_State_Estimation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Find Eigen
find_package(Eigen3 REQUIRED)

# Executable
add_executable(ekf_estimation
    src/main.cpp
    src/ekf.cpp
)

# Include dirs
target_include_directories(ekf_estimation PRIVATE
    include/
    ${EIGEN3_INCLUDE_DIR}
)
