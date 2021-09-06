
cmake_minimum_required(VERSION 3.5)

project(cartesio_example_test)
find_package(catkin REQUIRED COMPONENTS cartesian_interface)

add_compile_options(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(exampleros exampleros.cpp)
target_link_libraries(exampleros ${catkin_LIBRARIES})

find_package(cartesian_interface REQUIRED)

include_directories(${cartesian_interface_INCLUDE_DIRS})

add_executable(cartesio_solver cartesio_solver.cpp)

target_link_libraries(cartesio_solver ${cartesian_interface_LIBRARIES})

add_subdirectory(lib)



