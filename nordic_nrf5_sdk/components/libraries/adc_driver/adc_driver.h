#ifndef ADCdriver

#define ADCdriver

#include <stdint.h>
#include <string.h>
#include <nrf_saadc.h>
#include <nrf_drv_saadc.h>
#include <nrf_drv_ppi.h>
#include <nrf_drv_timer.h>
#include <math.h>
#include "nrf_gpio.h"
#include "..\..\..\..\..\..\components\libraries\sdcard\app_sdcard.h"
#include "..\..\..\..\..\..\components\libraries\sd_driver\sd_driver.h"

void timer_handler(nrf_timer_event_t event_type, void * p_context);

void saadc_sampling_event_init(void);

void saadc_sampling_event_enable(void);

void saadc_callback(nrf_drv_saadc_evt_t const * p_event);

void saadc_init(void);

uint8_t * get_adc_string(void);

int get_adc_flag(void);
void set_adc_flag(int f);

#endif