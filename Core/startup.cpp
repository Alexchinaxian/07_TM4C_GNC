#include "main.h"
#include "stdint.h"
#include "stdbool.h"
// #include "mcu.h"

using namespace TM4C_App;

// 应用程序入口点
// extern int main(void);

// 从链接器脚本中提取的地址
extern uint32_t _text_end;
extern uint32_t _data_start;
extern uint32_t _data_end;
extern uint32_t _bss_start;
extern uint32_t _bss_end;

/**
 * @brief 复位中断处理程序
 * 这个方法在软复位和硬复位（如启动）时被MCU调用
 */

extern "C" void ResetISR(void)
{
  uint8_t *src, *dst, *end;
  // 将数据段初始化器从闪存中复制到SRAM中
  src = (uint8_t *)&_text_end;
}
