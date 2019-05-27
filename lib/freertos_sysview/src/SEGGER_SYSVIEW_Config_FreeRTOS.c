/*********************************************************************
*            (c) 1995 - 2018 SEGGER Microcontroller GmbH             *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Config_FreeRTOS.c
Purpose : Sample setup configuration of SystemView with FreeRTOS.
Revision: $Rev: 7745 $
*/
#include "FreeRTOS.h"

#if configUSE_SYSVIEW == 1
#include "SEGGER_SYSVIEW.h"

extern const SEGGER_SYSVIEW_OS_API SYSVIEW_X_OS_TraceAPI;

/*********************************************************************
 *
 *       Defines, configurable
 *
 **********************************************************************
 */
// The application name to be displayed in SystemViewer
#define SYSVIEW_APP_NAME "c't-Bot Teensy framework"

// The target device name
#define SYSVIEW_DEVICE_NAME "Teensy 3.6"

// Frequency of the timestamp. Must match SEGGER_SYSVIEW_GET_TIMESTAMP in SEGGER_SYSVIEW_Conf.h
#define SYSVIEW_TIMESTAMP_FREQ (configCPU_CLOCK_HZ)

// System Frequency. SystemcoreClock is used in most CMSIS compatible projects.
#define SYSVIEW_CPU_FREQ configCPU_CLOCK_HZ

// The lowest RAM address used for IDs (pointers)
#define SYSVIEW_RAM_BASE (0x10000000)

/*********************************************************************
 *
 *       _cbSendSystemDesc()
 *
 *  Function description
 *    Sends SystemView description strings.
 */
static void _cbSendSystemDesc(void) {
    SEGGER_SYSVIEW_SendSysDesc("N=" SYSVIEW_APP_NAME ",D=" SYSVIEW_DEVICE_NAME ",O=FreeRTOS");
    SEGGER_SYSVIEW_SendSysDesc("I#15=SysTick");
    SEGGER_SYSVIEW_SendSysDesc("I#75=PORTA");
    SEGGER_SYSVIEW_SendSysDesc("I#76=PORTB");
    SEGGER_SYSVIEW_SendSysDesc("I#77=PORTC");
    SEGGER_SYSVIEW_SendSysDesc("I#78=PORTD");
    SEGGER_SYSVIEW_SendSysDesc("I#79=PORTE");
    SEGGER_SYSVIEW_SendSysDesc("I#80=SOFT");
}

/*********************************************************************
 *
 *       Global functions
 *
 **********************************************************************
 */
void SEGGER_SYSVIEW_Conf(void) {
    SEGGER_SYSVIEW_Init(SYSVIEW_TIMESTAMP_FREQ, SYSVIEW_CPU_FREQ, &SYSVIEW_X_OS_TraceAPI, _cbSendSystemDesc);
    SEGGER_SYSVIEW_SetRAMBase(SYSVIEW_RAM_BASE);
}

#endif // configUSE_SYSVIEW

/*************************** End of file ****************************/
