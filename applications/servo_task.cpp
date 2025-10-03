#include "cmsis_os.h"
#include "io/servo/servo.hpp"
#include "tim.h"

// 舵机对象，使用TIM1的CHANNEL_1，最大角度180度
sp::Servo servo(&htim1, TIM_CHANNEL_1,  168e6f, 360.0f);

extern "C" void servo_task()
{
  // 启动舵机PWM
  servo.start();
  
  for(int i=0;i<=180;++i)
  {
    servo.set(i);
    osDelay(1000./45.);

  }
  osDelay(1000.);
  servo.set(0.);
  while(true)
  {
    osDelay(1000.0);
  }
}
