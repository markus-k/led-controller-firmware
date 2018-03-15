#ifndef _NM_BSP_LOCM3_H_
#define _NM_BSP_LOCM3_H_

#include "../../../../src/debug.h"

#ifdef DEBUG
#define CONF_WINC_DEBUG        1
#endif

#define CONF_WINC_PRINTF(...)  do{DBG_NLF(__VA_ARGS__);}while(0)

#define CONF_WINC_USE_SPI      1
#define NM_EDGE_INTERRUPT      1

#endif
