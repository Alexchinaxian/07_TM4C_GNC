/*
 * bsp_MoudBus.h
 *
 *  Created on: 2023年6月6日
 *      Author: Xiao.Hexin.Alex
 */
#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include "ringbuf.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "inc/hw_memmap.h"

#ifndef BSP_MOUDBUS_H_
#define BSP_MOUDBUS_H_
#ifdef __cplusplus
extern "C"
{
#endif
// UART name
#define UART0 0x4000C000 // UART0
#define UART1 0x4000D000 // UART1
#define UART2 0x4000E000 // UART2
#define UART3 0x4000F000 // UART3
#define UART4 0x40010000 // UART4
#define UART5 0x40011000 // UART5
#define UART6 0x40012000 // UART6
#define UART7 0x40013000 // UART7
// Config
#define Stop_1 0x00000000 // One stop bit
#define Stop_2 0x00000008 // Two stop bits
//
#define NONE 0x00000000 // No parity
#define EVEN 0x00000006 // Even parity
#define ODD 0x00000002  // Odd parity
#define ONE 0x00000082  // Parity bit is one
#define ZERO 0x00000086 // Parity bit is zero
//
#define STATUS_COIL 0x01
#define INPUT_COIL 0x02
#define INPUT_REG 0x08
#define HOLDING_REG 0x10

#define COILSTATUS_LENGTH 125 // 250 byte = 2000 coil
#define INPUTSTATUS_LENGTH 125
#define HOLDINGREGISTER_LENGTH 200
#define INPUTREGISTER_LENGTH 200

  uint16_t ERR;

  typedef struct
  {
    /* 串口号*/
    uint32_t ui32PortName;

    /* 系统时钟 */
    uint32_t ui32SysClock;

    /* 波特率 */
    uint32_t ui32Baud;

    /* 数据位 */
    uint8_t ui8DataBit;

    /* 偶数 */
    uint8_t ui8Parity;

    /* 停止位 */
    uint8_t ui8StopBit;

    /* 回复时间 */
    uint16_t ui16ReponseTime;
  } tSerial;
  typedef struct
  {
    //
    // Coil status
    //
    uint16_t pui16CoilStatus[COILSTATUS_LENGTH];

    //
    // Input coil
    //
    uint16_t pui16InputCoil[INPUTSTATUS_LENGTH];

    //
    // Holding register
    //
    uint16_t pui16HoldingRegister[HOLDINGREGISTER_LENGTH];

    //
    // Input Register
    //
    uint16_t pui16InputRegister[INPUTREGISTER_LENGTH];

  } tDataField;
  tSerial g_tSerialConfig;

  void ConfigurePort(void);
  unsigned int ConfigSerial(tSerial *SerialConfig);
//*****************************************************************************
//
// Make the end of the C building for C++ compilers
//
//*****************************************************************************
#ifdef __cplusplus
  extern "C"
}
#endif
#endif /* BSP_MOUDBUS_H_ */
