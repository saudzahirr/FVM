cmake_minimum_required(VERSION 3.10)

project(FiniteVolumeMethod)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add subdirectory for third-party libraries
add_subdirectory(thirdParty)

# Include directories for third-party libraries
include_directories(thirdParty/Eigen_3.4.0)
include_directories(thirdParty/fmt-10.2.1/include)

# Add the executable
add_executable(${PROJECT_NAME} main.cpp)

# Link third-party libraries
target_link_libraries(${PROJECT_NAME} fmt)
