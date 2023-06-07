#include "stdint.h"
#include "stdbool.h"

#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/fpu.h"

#include "inc/hw_memmap.h"
#include "main.h"

#include "bsp/inc/bsp_MoudBus.h"
/**
 * main.c
 */
int main(void)
{
    // 初始化浮点单元的自动和懒惰状态保存
    FPULazyStackingEnable();
    // 启用浮点单元使用的协处理器。
    FPUEnable();
    // 启用浮点单元的自动保存状态，并禁用懒惰状态
    FPUStackingEnable();
    // 设置时钟
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_320),
                                            80000000);
    // 软件复位UART0
    SysCtlPeripheralReset(SYSCTL_PERIPH_UART0);
    // 软件复位DMA
    SysCtlPeripheralReset(SYSCTL_PERIPH_UDMA);
    // 初始化串口
    ConfigurePort();
    while(1)
    {

    }
}
