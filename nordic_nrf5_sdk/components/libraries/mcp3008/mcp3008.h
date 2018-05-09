#ifndef MCP3008driver

#define MCP3008driver

#include <nrf_gpio.h>
#include <nrf_delay.h>
#include <nrf_drv_spi.h>
#include <stdint.h>
#include <string.h>
#include <app_util_platform.h>
#include <math.h>
#include "..\..\..\..\..\..\components\libraries\sdcard\app_sdcard.h"
//#include <ble_nus.h>

#define MCP3008_MISO_PIN	10
#define MCP3008_MOSI_PIN	11
#define MCP3008_SCK_PIN		9
#define MCP3008_CS_PIN		7

#define SINGLE_TX		0x01
#define DIFF_TX                 0x00

// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3470

uint8_t * mcp3008_sample(void);
void mcp3008_spi_init(void);
void mcp3008_spi_uninit(void);
void spi_inst_init(nrf_drv_spi_t t_spi);

#endif
