#include "cmsis_os.h"
#include "io/led/led.hpp"
#include "io/plotter/plotter.hpp"
#include "tim.h"
#include "usart.h"

// LED对象，使用TIM5
sp::LED led(&htim5);
// extern sp::Plotter plotter;  // 使用control_task中定义的plotter
extern "C" void led_task()
{
  // 启动LED PWM
  led.start();
  
  // 流水灯效果参数
  const int total_steps = 100;  // 总步数
  const int delay_ms = 50;      // 每步延迟50ms
  
  while (true) {
    // 流水灯效果：红->绿->蓝->红...
    for (int step = 0; step < total_steps; step++) {
      float r, g, b;
      
      // 计算当前步的颜色
      if (step < total_steps / 3) {
        // 红色到绿色 (0-33步)
        float progress = (float)step / (total_steps / 3);
        r = 1.0f - progress;
        g = progress;
        b = 0.0f;
      } else if (step < 2 * total_steps / 3) {
        // 绿色到蓝色 (34-66步)
        float progress = (float)(step - total_steps / 3) / (total_steps / 3);
        r = 0.0f;
        g = 1.0f - progress;
        b = progress;
      } else {
        // 蓝色到红色 (67-99步)
        float progress = (float)(step - 2 * total_steps / 3) / (total_steps / 3);
        r = progress;
        g = 0.0f;
        b = 1.0f - progress;
      }
      
      // 发送plotter数据
      // plotter.plot(r, g, b);
      
      // 设置LED颜色
      led.set(r, g, b);
      
      // 延迟
      osDelay(delay_ms);
    }
  }
}
