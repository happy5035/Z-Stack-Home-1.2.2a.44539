#ifndef __LOGIC_IC_H__
#define __LOGIC_IC_H__


/******************************************************************************
* If building with a C++ compiler, make all of the definitions in this header
* have a C binding.
******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"
#include "gpio.h"               // Access to driverlib gpio functions


/******************************************************************************
* DEFINES
*/


/******************************************************************************
* FUNCTION PROTOTYPES
*/
extern void logicIcInit(void);
extern void Hc595Write(void);
extern uint16_t Hc165Read(void);
extern uint16_t HC595_Val;
/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* #ifndef __LOGIC_IC_H__ */

