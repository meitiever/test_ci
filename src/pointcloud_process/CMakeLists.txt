cmake_minimum_required(VERSION 3.0)
add_compile_options(-std=c++11)
project(point_cloud_process)


find_package(catkin REQUIRED COMPONENTS 
             roscpp std_msgs 
             sensor_msgs geometry_msgs
             message_generation nav_msgs
             cv_bridge image_transport
             )

catkin_package(
   
)

find_package(Eigen3 REQUIRED)
find_package(PCL QUIET)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(Boost COMPONENTS system)
find_package(OpenCV REQUIRED)

include_directories(include 
${catkin_INCLUDE_DIRS} 
${PCL_INCLUDE_DIRS} 
${Eigen3_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} src/PointCloudProcess.cpp)
target_link_libraries(${PROJECT_NAME} 
${catkin_LIBRARIES} 
${PCL_LIBRARIES}
${Boost_LIBRARIES}
${OpenCV_LIBS}
)
