#ifndef sd_driver_header

#define sd_driver_header

#include "nrf.h"
#include "bsp.h"
#include "..\..\..\..\..\..\external\fatfs\src\ff.h"
#include "..\..\..\..\..\..\external\fatfs\port\diskio_blkdev.h"
#include "..\..\..\..\..\..\components\libraries\block_dev\sdc\nrf_block_dev_sdc.h"
#include "..\..\..\..\..\..\components\drivers_nrf\gpiote\nrf_drv_gpiote.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

void sd_init(void);
void sd_log(uint8_t * log_data);

#endif