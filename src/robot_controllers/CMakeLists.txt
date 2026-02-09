cmake_minimum_required(VERSION 3.0.2)
project(robot_controllers)#功能包名称

###############
## Configure ##
###############

#寻找系统/本地库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  robot_description
  custom_msgs #自定义消息包
)
find_package(Eigen3 REQUIRED)#eigen3库
#pinocchio
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)
#打印调试记录
# message( "PINOCCHIO_FOUND: ${PINOCCHIO_FOUND}")  # 是否找到Pinocchio
# message( "PINOCCHIO_INCLUDE_DIRS: ${PINOCCHIO_INCLUDE_DIRS}")  # 头文件路径
# message( "PINOCCHIO_LIBRARY_DIRS: ${PINOCCHIO_LIBRARY_DIRS}")  # 库文件路径
# message( "PINOCCHIO_LIBRARIES: ${PINOCCHIO_LIBRARIES}")  # 库名称
# message( "PINOCCHIO_CFLAGS_OTHER: ${PINOCCHIO_CFLAGS_OTHER}")  # 其他编译选项

#找到工作空间路径 存在WORKSPACE_PATH
get_filename_component(LEVEL1 ${CATKIN_DEVEL_PREFIX} DIRECTORY)
get_filename_component(LEVEL2 ${LEVEL1} DIRECTORY)
get_filename_component(WORKSPACE_PATH ${LEVEL2} DIRECTORY)
#链接ros特有的catkin库
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs custom_msgs
)
#找到robot_description功能包路径​
execute_process(
  COMMAND rospack find robot_description
  OUTPUT_VARIABLE robot_description_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE  # 去除换行符​
)

###########
## Build ##
###########

#链接头文件
include_directories(
  include #当前文件夹下的include
  ${PINOCCHIO_INCLUDE_DIRS} #pinocchio
  ${catkin_INCLUDE_DIRS}#链接包的include
  ${EIGEN3_INCLUDE_DIR}#eigen3
)
#指定pinocchio库文件的路径
link_directories(${PINOCCHIO_LIBRARY_DIRS})

#添加可执行程序
# add_executable(test_gait test/test_gait.cpp )
# add_dependencies(test_gait ${catkin_EXPORTED_TARGETS}) 
# target_link_libraries(test_gait ${catkin_LIBRARIES})

add_executable(main src/main.cpp src/Utils.cpp src/Dynamics.cpp src/Debug.cpp src/Gait.cpp src/Manager.cpp src/Swing.cpp src/Rviz_vision.cpp src/Estimate.cpp)
add_dependencies(main ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(main ${PINOCCHIO_LIBRARIES})

add_executable(joy_key_node src/joy_key_node.cpp)
target_link_libraries(joy_key_node ${catkin_LIBRARIES})
add_dependencies(joy_key_node ${catkin_EXPORTED_TARGETS})
