cmake_minimum_required(VERSION 3.26)
project(point_cloud_utilities)

set(CMAKE_CXX_STANDARD 17)

add_executable(point_cloud_utilities main.cpp)


find_package(argparse)
target_link_libraries(point_cloud_utilities argparse::argparse)


find_package(PCL)
target_link_libraries(point_cloud_utilities PCL::PCL)


find_package(indicators)
target_link_libraries(point_cloud_utilities indicators::indicators)



find_package(easyloggingpp)
target_link_libraries(point_cloud_utilities easyloggingpp::easyloggingpp)