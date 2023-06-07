#include "bsp/inc/bsp_MoudBus.h"
//*****************************************************************************
//
// Funcition code support
//
//*****************************************************************************
uint16_t g_ui16DataFieldEnable;
uint16_t g_ui16InputCoil;
uint16_t g_ui16StatusCoil;
uint16_t g_ui16InputReg;
uint16_t g_ui16HoldingReg;
/*串口号*/
uint32_t g_ui32UARTPort;
tRingBufObject g_rbRemoTIRxRingBuf;
tRingBufObject g_rbRemoTITxRingBuf;

tDataField g_tDataField;

//*****************************************************************************
//
// Ringbuffer size
//
//*****************************************************************************
#define REMOTI_UART_RX_BUF_SIZE 255
#define REMOTI_UART_TX_BUF_SIZE 255
//*****************************************************************************
//
// Ringbuffer size
//
//*****************************************************************************
uint8_t g_pui8RxBuf[REMOTI_UART_RX_BUF_SIZE];
uint8_t g_pui8TxBuf[REMOTI_UART_TX_BUF_SIZE];
//*****************************************************************************
//
// Modbus Reveice buffer
//
//*****************************************************************************
uint8_t g_pui8MbReqPDU[255];
uint8_t g_pui8MBRsp[255];
void ConfigurePort(void)
{
  g_tSerialConfig.ui32PortName = UART0;
  g_tSerialConfig.ui32SysClock = g_ui32SysClock;
  g_tSerialConfig.ui32Baud = 115200;
  g_tSerialConfig.ui8DataBit = 8;
  g_tSerialConfig.ui8StopBit = Stop_1;
  g_tSerialConfig.ui8Parity = NONE;
  g_tSerialConfig.ui16ReponseTime = 0;
  ERR = ConfigSerial(&g_tSerialConfig);
}

void Fcode06(uint8_t *pui8MBReadBuff, uint16_t *pui16DataBuff)
{
  int i = 0;
  // Start address
  uint16_t ui16RegisterAdd;

  // No of data to be read
  uint16_t ui16ValueOfRegister;

  uint16_t *p;
  p = pui16DataBuff;
  ui16ValueOfRegister = (uint16_t)(pui8MBReadBuff[4]);
  ui16ValueOfRegister = ui16ValueOfRegister << 8;
  ui16ValueOfRegister = ui16ValueOfRegister | ((uint16_t)(pui8MBReadBuff[5]));
  if (((0 == ui16ValueOfRegister) || (ui16ValueOfRegister > 0)) && (ui16ValueOfRegister <= 0xFFFF))
  {
    ui16RegisterAdd = (uint16_t)(pui8MBReadBuff[2]);
    ui16RegisterAdd = ui16RegisterAdd << 8;
    ui16RegisterAdd = ui16RegisterAdd | ((uint16_t)(pui8MBReadBuff[3]));
  }

  //
  // End of funcition
  //
}
//*****************************************************************************
//
// Timer0A interrupts
// Use for enable TX/RX pin
//
//*****************************************************************************
void Timer0AIntHandler(void)
{
}
//*****************************************************************************
//
// Timer1A interrupts
// Use for delay reponse time
//
//*****************************************************************************
void Timer1AIntHandler(void)
{
}
void UARTIntHandler(void)
{
  // 功能性要求
  uint16_t ui16Fcode = 0;
  uint16_t ui16ByteCount = 0;
  static uint16_t ui16ReadPos = 0;
  static uint8_t ui8TxByte = 0;
  /*如果帧超过8个字节，中断就会回调，所以所有的变量都必须是静态变量 在帧接收中读取占有率。*/

  /*检查是否有一个接收中断正在等待*/
  do
  {
    if (UARTIntStatus(g_ui32UARTPort, 1) & UART_INT_RX | UART_INT_RT)
    {
      UARTIntClear(g_ui32UARTPort, UART_INT_RX | UART_INT_RT);
      while (UARTCharsAvail(g_ui32UARTPort))
      {
        RingBufWriteOne(&g_rbRemoTIRxRingBuf, UARTCharGetNonBlocking(g_ui32UARTPort));
        g_pui8MbReqPDU[ui16ReadPos] = RingBufReadOne(&g_rbRemoTIRxRingBuf);
        ui16ReadPos++;
        if (ui16ReadPos > 254)
        {
          ui16ReadPos = 0;
        }
        else
        {
          // none
        }
        // 确定框架的末端。
        ui16Fcode = (uint16_t)g_pui8MbReqPDU[1];
        if (((ui16Fcode == 1) && (g_ui16StatusCoil == STATUS_COIL)) || ((ui16Fcode == 2) && (g_ui16InputCoil == INPUT_COIL)) || ((ui16Fcode == 3) && (g_ui16HoldingReg == HOLDING_REG)) || ((ui16Fcode == 4) && (g_ui16InputReg == INPUT_REG)) || ((ui16Fcode == 5) && (g_ui16StatusCoil == STATUS_COIL)) || ((ui16Fcode == 6) && (g_ui16HoldingReg == HOLDING_REG)))
        {
        }
        else if (((16 == ui16Fcode) && (g_ui16HoldingReg == HOLDING_REG)) || ((15 == ui16Fcode) && (g_ui16StatusCoil == STATUS_COIL)))
        {
          switch (ui16Fcode)
          {

          case 6: // Write single register
            Fcode06(g_pui8MbReqPDU, g_tDataField.pui16HoldingRegister);
            break;
          }
        }
        else
        {
        }
      }
    }
  } while (UARTIntStatus(g_ui32UARTPort, 1) & (UART_INT_RT | UART_INT_RX | UART_INT_TX));
}

/**
 * @brief  UART配置端口
 *
 * @param SerialConfig
 * @return 无
 */
unsigned int ConfigSerial(tSerial *SerialConfig)
{
  uint32_t ui32Config = 0;
  if ((((*SerialConfig).ui32PortName == UART0) || ((*SerialConfig).ui32PortName == UART1) || ((*SerialConfig).ui32PortName == UART2) || ((*SerialConfig).ui32PortName == UART3) || ((*SerialConfig).ui32PortName == UART4) || ((*SerialConfig).ui32PortName == UART5) || ((*SerialConfig).ui32PortName == UART6) || ((*SerialConfig).ui32PortName == UART7)) && ((*SerialConfig).ui32SysClock != 0) && ((*SerialConfig).ui32Baud != 0))
  {
    g_ui32UARTPort = (*SerialConfig).ui32PortName;
    switch ((*SerialConfig).ui32PortName)
    {
    case UART0:
      SysCtlPeripheralReset(SYSCTL_PERIPH_UART0);
      SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
      GPIOPinConfigure(GPIO_PA0_U0RX);
      GPIOPinConfigure(GPIO_PA1_U0TX);
      GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
      break;
    case UART1:
      SysCtlPeripheralReset(SYSCTL_PERIPH_UART1);
      SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOB);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
      GPIOPinConfigure(GPIO_PB0_U1RX);
      GPIOPinConfigure(GPIO_PB1_U1TX);
      GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
      break;

    case UART2:

      SysCtlPeripheralReset(SYSCTL_PERIPH_UART2);
      SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
      GPIOPinConfigure(GPIO_PA6_U2RX);
      GPIOPinConfigure(GPIO_PA7_U2TX);
      GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
      break;

    case UART3:

      SysCtlPeripheralReset(SYSCTL_PERIPH_UART3);
      SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
      GPIOPinConfigure(GPIO_PA4_U3RX);
      GPIOPinConfigure(GPIO_PA5_U3TX);
      GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);
      break;

    case UART4:

      SysCtlPeripheralReset(SYSCTL_PERIPH_UART4);
      SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
      GPIOPinConfigure(GPIO_PA2_U4RX);
      GPIOPinConfigure(GPIO_PA3_U4TX);
      GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
      break;

    case UART5:

      SysCtlPeripheralReset(SYSCTL_PERIPH_UART5);
      SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOC);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
      GPIOPinConfigure(GPIO_PC6_U5RX);
      GPIOPinConfigure(GPIO_PC7_U5TX);
      GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
      break;

    case UART6:

      SysCtlPeripheralReset(SYSCTL_PERIPH_UART6);
      SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOP);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
      GPIOPinConfigure(GPIO_PP0_U6RX);
      GPIOPinConfigure(GPIO_PP1_U6TX);
      GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);
      break;

    case UART7:

      SysCtlPeripheralReset(SYSCTL_PERIPH_UART7);
      SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOC);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
      GPIOPinConfigure(GPIO_PC4_U7RX);
      GPIOPinConfigure(GPIO_PC5_U7TX);
      GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
      break;
    }
    // 配置数据位
    switch ((*SerialConfig).ui8DataBit)
    {
    case 7:

      ui32Config |= UART_CONFIG_WLEN_7;
      break;

    case 8:

      ui32Config |= UART_CONFIG_WLEN_8;
      break;

    default:
      ui32Config |= UART_CONFIG_WLEN_8;
    }

    /*配置奇偶性*/
    switch ((*SerialConfig).ui8Parity)
    {
    case NONE:

      ui32Config |= NONE;
      break;
    case EVEN:

      ui32Config |= EVEN;
      break;

    case ODD:

      ui32Config |= ODD;
      break;

    case ONE:

      ui32Config |= ONE;
      break;

    case ZERO:

      ui32Config |= ZERO;
      break;
    }
    /*配置停止位*/
    switch ((*SerialConfig).ui8StopBit)
    {
    case Stop_1:

      ui32Config |= Stop_1;
      break;

    case Stop_2:

      ui32Config |= Stop_2;
      break;
    }
    /*配置端口*/
    UARTConfigSetExpClk((*SerialConfig).ui32PortName, (*SerialConfig).ui32SysClock, (*SerialConfig).ui32Baud, ui32Config);
    UARTFIFOLevelSet((*SerialConfig).ui32PortName, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTEnable((*SerialConfig).ui32PortName);
    UARTFIFODisable((*SerialConfig).ui32PortName);
    UARTIntRegister((*SerialConfig).ui32PortName, &UARTIntHandler);
    UARTIntEnable((*SerialConfig).ui32PortName, UART_INT_RX | UART_INT_RT);
    RingBufInit(&g_rbRemoTIRxRingBuf, g_pui8RxBuf, REMOTI_UART_RX_BUF_SIZE);
    RingBufInit(&g_rbRemoTITxRingBuf, g_pui8TxBuf, REMOTI_UART_TX_BUF_SIZE);

    /* 用于启用TX/RX引脚的配置定时器*/
    SysCtlPeripheralReset(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet() / 10000));
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    /* 配置延迟响应时间的计时器*/
    SysCtlPeripheralReset(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, 50000 * ((*SerialConfig).ui16ReponseTime));
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1AIntHandler);
    //
    //
    //
    IntMasterEnable();
  }
  //
  // Return err code
  //
  else
    return 1;
  return 0;
}
