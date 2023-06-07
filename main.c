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
    // ��ʼ�����㵥Ԫ���Զ�������״̬����
    FPULazyStackingEnable();
    // ���ø��㵥Ԫʹ�õ�Э��������
    FPUEnable();
    // ���ø��㵥Ԫ���Զ�����״̬������������״̬
    FPUStackingEnable();
    // ����ʱ��
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_320),
                                            80000000);
    // �����λUART0
    SysCtlPeripheralReset(SYSCTL_PERIPH_UART0);
    // �����λDMA
    SysCtlPeripheralReset(SYSCTL_PERIPH_UDMA);
    // ��ʼ������
    ConfigurePort();
    while(1)
    {

    }
}
