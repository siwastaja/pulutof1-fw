#ifndef TOF_TABLE_H
#define TOF_TABLE_H

#include <stdint.h>

#define TOF_TBL_LEN 512

#define TOF_TBL_PERIOD 7494

#define TOF_TBL_HALF_PERIOD 3747

#define TOF_TBL_QUART_PERIOD 1873

extern const int16_t tof_tbl[TOF_TBL_LEN] __attribute__((section(".dtcm_data"))) __attribute__((aligned(4)));

#endif
