#include "mcp3008.h"

static const nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(APP_SDCARD_SPI_INSTANCE);  /**< SPI instance. */
uint8_t mcp3008_buf[32];

void mcp3008_spi_init(void)
{
  const nrf_drv_spi_config_t spi_cfg = {
                            .sck_pin      = MCP3008_SCK_PIN,
                            .mosi_pin     = MCP3008_MOSI_PIN,
                            .miso_pin     = MCP3008_MISO_PIN,
                            .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
                            .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
                            .orc          = 0xFF,
                            .frequency    = NRF_DRV_SPI_FREQ_4M,
                            .mode         = NRF_DRV_SPI_MODE_0,
                            .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
                        };
    nrf_drv_spi_init(&m_spi, &spi_cfg, NULL, NULL);

}

void mcp3008_spi_uninit(void)
{
  nrf_drv_spi_uninit(&m_spi);
}

uint8_t * mcp3008_sample(void)
{
    mcp3008_spi_init();

    uint8_t tx[3];
    uint8_t rx[3];
    int result;
    
    //for(int channel = 0; channel < 8; channel++)
    //{
        nrf_gpio_pin_clear(7);

        tx[0] = ((0x01 << 7) |                // start bit
             (SINGLE_TX << 6) |               // single or differential
             ((0x01 & 0x07) << 3) );       // channel number
        tx[1] = 0;
        tx[2] = 0;
        nrf_drv_spi_transfer(&m_spi, tx, 3, rx, 3);

        nrf_gpio_pin_set(7);

       result = 0x3FF & ((rx[0] & 0x01) << 9 |
                          (rx[1] & 0xFF) << 1 |
                          (rx[2] & 0x80) >> 7 );
    //}
    float uhADCxConvertedValue = (1023.0 / result) - 1;
    uhADCxConvertedValue = 10000.0 / uhADCxConvertedValue;
		
    float steinhart;
    steinhart = uhADCxConvertedValue / 10000.0;
    steinhart = log(steinhart);
    steinhart /= BCOEFFICIENT;
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;

    snprintf(mcp3008_buf, sizeof(mcp3008_buf), "1: %.3f\r\n", steinhart);
    return mcp3008_buf;

     mcp3008_spi_uninit();
}
