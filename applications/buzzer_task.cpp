#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

// C板
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

// 达妙
// sp::Buzzer buzzer(&htim12, TIM_CHANNEL_2, 240e6);

extern "C" void buzzer_task()
{
  // Windows XP 真正的开机旋律: C#-D#-G#-F#-C#
  // 音符频率定义 (Hz)
  const float C5_sharp = 554.37f;  // C#5
  const float D5_sharp = 622.25f;  // D#5  
  const float G5_sharp = 830.61f;  // G#5
  const float F5_sharp = 739.99f;  // F#5
  const float C6_sharp = 1108.73f; // C#6
  
  // Windows XP 真正的开机旋律音符序列
  float melody[] = {
    C5_sharp, D5_sharp, G5_sharp, F5_sharp, C6_sharp
  };
  
  // 每个音符的持续时间 (ms) - 根据原版调整
  int durations[] = {
    300, 200, 400, 300, 500  // 最后一个音符稍长
  };
  
  // 播放旋律
  for (int i = 0; i < 5; i++) {
    buzzer.set(melody[i], 0.007);  // 30%占空比
    buzzer.start();
    osDelay(durations[i]);
    buzzer.stop();
    osDelay(50);  // 音符间的小间隔
  }

  while (true) {
    // 主循环：蜂鸣器保持关闭状态
    osDelay(10);
    
    // plotter.plot(r,g,b
    // );
  }
}