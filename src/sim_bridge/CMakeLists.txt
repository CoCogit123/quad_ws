cmake_minimum_required(VERSION 3.0.2)
project(sim_bridge)#功能包名称

###############
## Configure ##
###############

#寻找系统/本地库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  robot_description
  custom_msgs #自定义消息包
  controller_manager
	joint_state_controller
	robot_state_publisher
)
find_package(Eigen3 REQUIRED)
#找到工作空间路径 存在WORKSPACE_PATH
get_filename_component(LEVEL1 ${CATKIN_DEVEL_PREFIX} DIRECTORY)
get_filename_component(LEVEL2 ${LEVEL1} DIRECTORY)
get_filename_component(WORKSPACE_PATH ${LEVEL2} DIRECTORY)
#链接ros特有的catkin库
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs custom_msgs controller_manager joint_state_controller robot_state_publisher
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
  ${catkin_INCLUDE_DIRS}#链接包的include
  ${EIGEN3_INCLUDE_DIR}#eigen3
  ${WORKSPACE_PATH}/third_party/mujoco/glfw3/include/ #mujoco/glfw3头文件
  ${WORKSPACE_PATH}/third_party/mujoco/mujoco_3_1/include/ #mujoco/glfw3头文件
  ${robot_description_PATH}/config #gazebo仿真要用的config
)

#添加可执行程序
##############sim_mujoco程序##############
add_executable(sim_mujoco src/sim_mujoco.cpp src/MJ_interface.cpp src/UI_interface.cpp )
#链接库文件
add_dependencies(sim_mujoco ${catkin_EXPORTED_TARGETS})
#使用glfw3 mujoco
target_link_libraries(sim_mujoco 
  ${catkin_LIBRARIES}
  ${WORKSPACE_PATH}/third_party/mujoco/glfw3/lib/libglfw3.a
  ${WORKSPACE_PATH}/third_party/mujoco/mujoco_3_1/lib/libmujoco.so
  -lX11 -lXrandr -lXi -lXxf86vm -lXinerama -lpthread -lrt -ldl -lXcursor)
##############sim_gazebo程序##############
add_executable(sim_gazebo src/sim_gazebo.cpp)
add_dependencies(sim_gazebo ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_gazebo
  ${catkin_LIBRARIES}
)


