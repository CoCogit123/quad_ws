#Motor_control.msg 无法设置长度 但都要按照12的长度进行接收和输入 电机控制消息包
int16[] id #可用于校验消息包的准确性
float64[] torque
float64[] kp
float64[] kd
float64[] pos
float64[] vel