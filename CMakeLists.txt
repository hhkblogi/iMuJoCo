cmake_minimum_required(VERSION 3.20)
project(imujoco_workspace VERSION 0.1.0 LANGUAGES CXX)

# Enable testing at top level for ctest discovery
enable_testing()

# Add the driver subdirectory
add_subdirectory(driver)
