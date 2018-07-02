cmake_minimum_required(VERSION 2.6)
project(hello)

set(PCL_DIR /home/hongjun/hjwang/pcl/dist/share/pcl-1.8/)
find_package(PCL 1.8 REQUIRED)
message(STATUS "PCL library status:")
message(STATUS "    version: ${PCL_VERSION}")
message(STATUS "    libraries: ${PCL_LIBRARIES}")
message(STATUS "    include path: ${PCL_INCLUDE_DIRS}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(/home/hongjun/workspace/aiethan/build/include)
include_directories(/home/hongjun/hjwang/pcl/dist/include/pcl-1.8)
include_directories(/usr/include/eigen3)
include_directories(/home/hongjun/hjwang/jsoncpp1.8.3/include)
include_directories(/home/hongjun/hjwang/openmesh/dist/include)
include_directories(/home/hongjun/hjwang/aiethan/tooldist/include)
include_directories(/home/hongjun/hjwang/meshlab-2016.12/vcglib)


add_executable(visdetectpoisson visdetectpoisson.cpp)
target_link_libraries(visdetectpoisson /home/hongjun/hjwang/openmesh/dist/lib/libOpenMeshCored.so)
target_link_libraries(visdetectpoisson /home/hongjun/workspace/aiethan/build/lib/libaiethan.so)
target_link_libraries(visdetectpoisson /home/hongjun/hjwang/jsoncpp1.8.3/lib/libjsoncpp.so)
target_link_libraries(visdetectpoisson ${PCL_LIBRARIES})
target_link_libraries(visdetectpoisson /usr/lib/x86_64-linux-gnu/libboost_system.so)
target_link_libraries(visdetectpoisson /home/hongjun/workspace/triangle/build/libtriangle_shared.so)
target_link_libraries(visdetectpoisson /usr/lib/x86_64-linux-gnu/libm.so)
target_link_libraries(visdetectpoisson /home/hongjun/hjwang/aiethan/tooldist/lib/libaiethantool.so)
target_link_libraries(visdetectpoisson /usr/lib/x86_64-linux-gnu/libgomp.so.1)
target_link_libraries(visdetectpoisson /home/hongjun/hjwang/poissonrecon/PoissonRecon/Bin/Linux/libPoissonReconSelf.a)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

install(TARGETS visdetectpoisson RUNTIME DESTINATION bin)
