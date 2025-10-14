#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/can/can.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"
#include "tools/linear_interp/linear_interp.hpp"
#include "usart.h"
#include "can.h"
#include <stdio.h>
#include <cmath>

// 全局对象定义
sp::DBus remote(&huart3);  // DBus遥控器，使用UART3
sp::CAN can2(&hcan2);     // CAN2用于电机通讯
sp::Plotter plotter(&huart1, true);
sp::SuperCap super_cap(sp::SuperCapMode::AUTOMODE);   // 超级电容对象
sp::PM02 pm02(&huart6);   // 裁判系统模块，使用UART6
// sp::Plotter plotter(&huart1);  // Plotter数据可视化，使用UART1

// 四个RM3508电机，ID分别为1,2,3,4
sp::RM_Motor motor_lf(2, sp::RM_Motors::M3508, 14.9f);  // 左前轮
sp::RM_Motor motor_lr(3, sp::RM_Motors::M3508, 14.9f);  // 左后轮  
sp::RM_Motor motor_rf(1, sp::RM_Motors::M3508, 14.9f);  // 右前轮
sp::RM_Motor motor_rr(4, sp::RM_Motors::M3508, 14.9f);  // 右后轮

// 麦轮运动学模型 (轮子半径0.076m, 车体长0.33m, 宽0.37m)
sp::Mecanum mecanum(0.077f, 0.165f, 0.185f, false, false, true, true);

// 四个电机的PID控制器 (控制周期0.01s, Kp=2.0, Ki=0.1, Kd=0.05)
sp::PID pid_lf(0.01f, 0.25f, 0.f, 0.00f, 20.0f, 2.0f);
sp::PID pid_lr(0.01f, 0.25f, 0.f, 0.00f, 20.0f, 2.0f);
sp::PID pid_rf(0.01f, 0.25f, 0.f, 0.00f, 20.0f, 2.0f);
sp::PID pid_rr(0.01f, 0.25f, 0.f, 0.00f, 20.0f, 2.0f);
const float kV=0.0f;
// const float kS=0.;

// 功率计算参数
const float k_omega_square = 0.017f;  // omega^2项的系数
const float k_torque_square = 2.8f;  // 扭矩^2项的系数
const float kS = 5;  

// 功率限制参数
float max_power_limit = 80.0f;  // 最大功率上限 (W)
const float base_power_limit = 80.0f;  // 基础功率限制

const float cap_voltage_table[] = {4.f,14.0f,  28.0f};
const float extra_power_table[] = {0,20.0f, 75.0f};
// 3.0V: 无额外功率 (即将耗尽)
// 4.0V: +25W 额外功率
// 5.0V: +70W 额外功率
// 6.0V: +130W 额外功率 (满电)

sp::LinearInterp cap_power_interp(cap_voltage_table, extra_power_table, 3);

// 控制状态
bool system_enabled = false;
bool power_limit_disabled = false;  // 功率限制解除状态
float max_speed = 1.5f;  // 最大速度 m/s
float max_angular_speed = 10.f;  // 最大角速度 rad/s
float deadband = 0.f;  // 摇杆死区，小于此值时输出0
  
extern "C" void control_task(void const * argument)
{
  // 初始化CAN通讯
  can2.config();
  can2.start();
  
  // 初始化遥控器
  remote.request();
  
  // 初始化裁判系统
  pm02.request();
  
  // 功率限制相关变量
  static float last_power_scale = 1.0f;  // 保存功率缩放系数
  
  while (true) {
    uint32_t current_time = osKernelSysTick();
    
    // 检查遥控器连接状态
    if (!remote.is_alive(current_time)) {
      // 遥控器断开，停止所有电机
      motor_lf.cmd(0.0f);
      motor_lr.cmd(0.0f);
      motor_rf.cmd(0.0f);
      motor_rr.cmd(0.0f);
      system_enabled = false;
    } else {
      // 遥控器连接正常
      // 右开关控制enable/disable
      if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
        // DOWN: disable
        system_enabled = false;
        power_limit_disabled = false;  // 禁用时也关闭功率限制解除
      } else if (remote.sw_r == sp::DBusSwitchMode::MID) {
        // MID: 标准模式，使用基础功率限制
        system_enabled = true;
        power_limit_disabled = false;
        max_power_limit = base_power_limit;
      } else if (remote.sw_r == sp::DBusSwitchMode::UP) {
        // UP: 爆发模式，根据超级电容电压动态计算额外功率
        system_enabled = true;
        power_limit_disabled = false;
        
        // 使用线性插值根据电容电压计算额外功率
        float extra_power = cap_power_interp.calc(super_cap.voltage);
        max_power_limit = base_power_limit + extra_power;
      }
      
      if (system_enabled) {
        // 系统启用，读取遥控器输入
        float vx_raw = remote.ch_lv;  // 左摇杆垂直：前后
        float vy_raw = -remote.ch_lh;  // 左摇杆水平：左右
        float wz_raw = -remote.ch_rh;  // 右摇杆水平：转向
        
        // 检查是否在deadband范围内
        bool in_deadband = (std::abs(vx_raw) < deadband) && 
                          (std::abs(vy_raw) < deadband) && 
                          (std::abs(wz_raw) < deadband);
        
        if (in_deadband) {
          // 在deadband范围内，直接输出0
          motor_lf.cmd(0.0f);
          motor_lr.cmd(0.0f);
          motor_rf.cmd(0.0f);
          motor_rr.cmd(0.0f);
        } else {
          // 不在deadband范围内，正常控制
          float vx = vx_raw * max_speed;
          float vy = vy_raw * max_speed;
          float wz = wz_raw * max_angular_speed;
          
          // 麦轮运动学计算：将底盘速度转换为各轮速度
          mecanum.calc(vx, vy, wz);
          
          // PID控制各轮转速
          pid_lf.calc(mecanum.speed_lf, motor_lf.speed);
          pid_lr.calc(mecanum.speed_lr, motor_lr.speed);
          pid_rf.calc(mecanum.speed_rf, motor_rf.speed);
          pid_rr.calc(mecanum.speed_rr, motor_rr.speed);
          
          // 计算原始扭矩命令
          float torque_lf = pid_lf.out + kV * mecanum.speed_lf;
          float torque_lr = pid_lr.out + kV * mecanum.speed_lr;
          float torque_rf = pid_rf.out + kV * mecanum.speed_rf;
          float torque_rr = pid_rr.out + kV * mecanum.speed_rr;
          
          // 计算预测功率
          float total_omega_square = 0.0f;
          float total_torque_square = 0.0f;
          
          total_omega_square += motor_lf.speed * motor_lf.speed;
          total_omega_square += motor_lr.speed * motor_lr.speed;
          total_omega_square += motor_rf.speed * motor_rf.speed;
          total_omega_square += motor_rr.speed * motor_rr.speed;
          
          total_torque_square += torque_lf * torque_lf;
          total_torque_square += torque_lr * torque_lr;
          total_torque_square += torque_rf * torque_rf;
          total_torque_square += torque_rr * torque_rr;
          
          float output_power = torque_lf * motor_lf.speed + torque_lr * motor_lr.speed + 
                              torque_rf * motor_rf.speed + torque_rr * motor_rr.speed;
          float predicted_power = output_power + k_omega_square * total_omega_square + 
                                 k_torque_square * total_torque_square + kS;
          
          // 计算功率限制的缩放系数
          // 预测功率公式: P = Σ(τ*ω) + k_omega*Σ(ω²) + k_torque*Σ(τ²) + kS
          // 当扭矩缩放为 scale 时: P = scale*Σ(τ*ω) + k_omega*Σ(ω²) + k_torque*scale²*Σ(τ²) + kS
          // 设 P = max_power_limit，求解 scale
          
          float power_scale = 1.0f;
          if (!power_limit_disabled && predicted_power > max_power_limit && predicted_power > 0.0f) {
            // 重新计算各项系数
            float linear_term = output_power;  // Σ(τ*ω)
            float quadratic_term = k_torque_square * total_torque_square;  // k_torque*Σ(τ²)
            float constant_term = k_omega_square * total_omega_square + kS;  // k_omega*Σ(ω²) + kS
            
            // 求解二次方程: quadratic_term*scale² + linear_term*scale + constant_term = max_power_limit
            // 即: quadratic_term*scale² + linear_term*scale + (constant_term - max_power_limit) = 0
            
            float a = quadratic_term;
            float b = linear_term;
            float c = constant_term - max_power_limit;
            
            if (std::abs(a) > 1e-6f) {
              // 二次方程求解: scale = (-b ± sqrt(b² - 4ac)) / (2a)
              float discriminant = b * b - 4 * a * c;
              if (discriminant >= 0) {
                float scale1 = (-b + std::sqrt(discriminant)) / (2 * a);
                float scale2 = (-b - std::sqrt(discriminant)) / (2 * a);
                
                // 选择在[0,1]范围内的正解
                if (scale1 >= 0 && scale1 <= 1) {
                  power_scale = scale1;
                } else if (scale2 >= 0 && scale2 <= 1) {
                  power_scale = scale2;
                } else {
                  power_scale = 0.0f;  // 无解，完全停止
                }
              } else {
                power_scale = 0.0f;  // 无实数解，完全停止
              }
            } else {
              // 线性情况: b*scale + c = 0
              if (std::abs(b) > 1e-6f) {
                power_scale = -c / b;
                power_scale = std::max(0.0f, std::min(1.0f, power_scale));
              } else {
                power_scale = 0.0f;
              }
            }
          }
          
        // 保存功率缩放系数用于后续使用
        last_power_scale = power_scale;
        // power_scale = 1.0f;
          
        // 应用功率限制到扭矩命令
        motor_lf.cmd(torque_lf * power_scale);
        motor_lr.cmd(torque_lr * power_scale);
        motor_rf.cmd(torque_rf * power_scale);
        motor_rr.cmd(torque_rr * power_scale);
        
        }
      } else {
        // 系统禁用，停止所有电机
        motor_lf.cmd(0.0f);
        motor_lr.cmd(0.0f);
        motor_rf.cmd(0.0f);
        motor_rr.cmd(0.0f);
      }
    }
    
    
    
    // 发送电机控制命令（所有电机数据打包在一个CAN消息中）
    motor_lf.write(can2.tx_data);  // 电机1数据写入字节0-1
    motor_lr.write(can2.tx_data);  // 电机2数据写入字节2-3
    motor_rf.write(can2.tx_data);  // 电机3数据写入字节4-5
    motor_rr.write(can2.tx_data);  // 电机4数据写入字节6-7
    can2.send(motor_lf.tx_id);     // 只发送一次，包含所有电机数据
    
      super_cap.write(
        can2.tx_data, pm02.robot_status.chassis_power_limit, pm02.power_heat.buffer_energy,
        pm02.robot_status.power_management_chassis_output);    // chassis_output: 预测的底盘输出功率
      can2.send(super_cap.tx_id);
    
    // 发送plotter数据 - 20ms发送一次
    static uint32_t plot_counter = 0;
    
    if (++plot_counter >= 2) {  // 2 * 10ms = 20ms
      plot_counter = 0;
      
      // 计算输出功率（使用扭矩*速度估算）
      float output_power = 0.0f;
      output_power += motor_lf.torque * motor_lf.speed;
      output_power += motor_lr.torque * motor_lr.speed;
      output_power += motor_rf.torque * motor_rf.speed;
      output_power += motor_rr.torque * motor_rr.speed;
      
      // 计算预测功率：消耗功率 = 输出功率 + omega^2*定值 + 扭矩^2*定值
      float total_omega_square = 0.0f;
      float total_torque_square = 0.0f;
      
      // 计算所有电机的omega^2和扭矩^2
      total_omega_square += motor_lf.speed * motor_lf.speed;
      total_omega_square += motor_lr.speed * motor_lr.speed;
      total_omega_square += motor_rf.speed * motor_rf.speed;
      total_omega_square += motor_rr.speed * motor_rr.speed;
      
      total_torque_square += motor_lf.torque * motor_lf.torque;
      total_torque_square += motor_lr.torque * motor_lr.torque;
      total_torque_square += motor_rf.torque * motor_rf.torque;
      total_torque_square += motor_rr.torque * motor_rr.torque;
      
      // 预测功率计算：消耗功率 = 输出功率 + omega^2*定值 + 扭矩^2*定值
      float predicted_power = output_power + k_omega_square * total_omega_square + k_torque_square * total_torque_square + kS;
      
      
       
       // 发送功率数据
       plotter.plot(
         super_cap.voltage,  // 电压
         predicted_power,
         
         
         super_cap.power_in-super_cap.power_out,
         super_cap.power_in,
         super_cap.power_out,
         mecanum.speed_lf
         
       );
    }
    
    
    // 10ms控制周期
    osDelay(10);
  }
}

// CAN接收中断回调函数
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN2) {
    can2.recv(CAN_RX_FIFO0);
    
     // 调试输出：CAN接收中断被调用
     printf("CAN2 RX: ID=0x%03lX\r\n", can2.rx_id);
    
    // 处理电机反馈数据
    if (can2.rx_id == motor_lf.rx_id) {
      motor_lf.read(can2.rx_data, osKernelSysTick());
    } else if (can2.rx_id == motor_lr.rx_id) {
      motor_lr.read(can2.rx_data, osKernelSysTick());
    } else if (can2.rx_id == motor_rf.rx_id) {
      motor_rf.read(can2.rx_data, osKernelSysTick());
    } else if (can2.rx_id == motor_rr.rx_id) {
      motor_rr.read(can2.rx_data, osKernelSysTick());
     } else if (can2.rx_id == super_cap.rx_id) {
       // 处理超级电容反馈数据
       super_cap.read(can2.rx_data, osKernelSysTick());
       // 调试输出：超级电容数据更新
       // printf("SuperCap updated: voltage=%.2f, power_in=%.2f, power_out=%.2f\r\n", 
       //        super_cap.voltage, super_cap.power_in, super_cap.power_out);
     }
  }
}

// UART接收中断回调函数
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3) {
    remote.update(Size, osKernelSysTick());
    remote.request();
  } else if (huart == &huart6) {
    pm02.update(Size);
    pm02.request();
  }
}

// UART错误回调函数
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3) {
    remote.request();
  } else if (huart == &huart6) {
    pm02.request();
  }
}