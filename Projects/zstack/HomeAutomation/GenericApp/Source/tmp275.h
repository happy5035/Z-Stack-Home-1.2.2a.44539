#ifndef TMP275_H
#define TMP275_H

#include "ZComDef.h"
uint8 TMP275_startMeasure(void);
int16 TMP275_ReadTemp(void);
#endif