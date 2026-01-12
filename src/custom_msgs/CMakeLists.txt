cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

###############
## Configure ##
###############

#寻找系统/本地库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
  std_msgs 
  geometry_msgs
)

# ========== 声明自定义.msg文件 ==========​
add_message_files(
  FILES
  Motor_state.msg 
  Motor_control.msg 
  Sim_info.msg
  Joy_control.msg
)
# ========== 生成消息（依赖std_msgs） ==========​
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

#链接ros特有的catkin库
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime message_generation
)

###########
## Build ##
###########

#链接头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



