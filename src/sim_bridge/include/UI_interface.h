#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"
#include <string>          // C++字符串处理
#include <memory>          // 智能指针相关
#include <vector>          // 向量容器
#include <fstream>         // 文件流，用于文件操作

class UIctr {
public:
    GLFWwindow* window;     // GLFW窗口指针，用于显示图形界面

    struct ButtonState {    // 定义结构体存储键盘按键状态
        bool key_w{false};
        bool key_s{false};
        bool key_a{false};
        bool key_d{false};
        bool key_h{false};
        bool key_j{false};
        bool key_space{false};
    } buttonRead;

    // mouse interaction
    bool button_left{false};    // 鼠标左键状态
    bool button_middle{false};  // 鼠标中键状态
    bool button_right{false};   // 鼠标右键状态

    bool runSim{true};          // 是否运行仿真
    bool isContinuous{true};    // 仿真是否连续
    double lastx{0};            // 上次鼠标x坐标
    double lasty{0};            // 上次鼠标y坐标
    mjModel* mj_model;          // MuJoCo模型指针
    mjData* mj_data;            // MuJoCo数据指针

    UIctr(mjModel* modelIn, mjData* dataIn);// 构造函数声明
    void iniGLFW();// 初始化GLFW库
    void enableTracking();// 启用跟踪相机
    void createWindow(const char* windowTitle, bool saveVideo); // 创建窗口
    void updateScene();// 更新场景
    void Close();// 关闭窗口和释放资源
    

    // 键盘回调函数声明
    void Keyboard(int key, int scancode, int act, int mods);
    // 鼠标按键回调声明
    void Mouse_button(int button, int act, int mods);
    // 鼠标移动回调声明
    void Mouse_move(double xpos, double ypos);
    // 滚动回调声明
    void Scroll(double xoffset, double yoffset);
    // 获取按键状态
    ButtonState getButtonState();
    

private:
    unsigned char* image_rgb_; // 存储RGB图像数据
    float* image_depth_;// 存储深度图像数据
    FILE* file;// 文件流，用于保存视频等

    int width{1200};        // 窗口宽度
    int height{800};        // 窗口高度
    bool save_video{false}; // 是否保存视频
    bool isTrack{false};    // 是否跟踪相机
    // UI handler
    mjvCamera cam;                      //MuJoCo视图相机
    mjvOption opt;                      //可视化选项
    mjvScene scn;                       //可视化场景
    mjrContext con;                     //渲染上下文
};    


#endif