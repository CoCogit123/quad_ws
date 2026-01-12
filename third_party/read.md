#~/.bashrc环境配置
#mujoco
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/dog_ws/third_party/mujoco/mujoco-3.1.1
/bin

# dog
source ~/dog_ws/devel/setup.bash

#mujoco需要 
sudo apt-get install libx11-dev libxrandr-dev libxi-dev libxxf86vm-dev libxinerama-dev
sudo apt-get install libxcursor-dev


#lcm
./lcm-gen --cpp --cpp-std c++11 --cpp-hpath ~/dog_ws/devel/include/ ~/dog_ws/src/dog_control/msg/motor_control.lcm
