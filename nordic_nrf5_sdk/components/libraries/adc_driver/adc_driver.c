#include "adc_driver.h"

#define SAADC_SAMPLES_IN_BUFFER         8
#define TOTAL_ADC_VALUES                16
#define SAADC_SAMPLE_RATE               1000 /**< SAADC sample rate in ms. */ 

// which analog pin to connect
//#define THERMISTORPIN A0         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3470
// the value of the 'other' resistor

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static int ble_tx_count = 0;

uint8_t sd_print_string[125];
float temp_data_storage[TOTAL_ADC_VALUES];
int adc_flag = 0;

void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            sd_log(sd_print_string);
            ble_tx_count++;
            if(ble_tx_count == 60)
            {
                adc_flag = 1;
                ble_tx_count = 0;
            }
            //set_adc_flag(1);
            break;

        default:
            //Do nothing.
            break;
    }
}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_config.frequency = NRF_TIMER_FREQ_31250Hz;
    err_code = nrf_drv_timer_init(&m_timer, &timer_config, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer,SAADC_SAMPLE_RATE);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        for (int i = 0; i < SAADC_SAMPLES_IN_BUFFER; i++)
        {
            float uhADCxConvertedValue = (1023.0 / p_event->data.done.p_buffer[i]) - 1;
            uhADCxConvertedValue = 10000.0 / uhADCxConvertedValue;
		
            float steinhart;
            steinhart = uhADCxConvertedValue / 10000.0;
            steinhart = log(steinhart);
            steinhart /= BCOEFFICIENT;
            steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
            steinhart = 1.0 / steinhart;
            steinhart -= 273.15;

            temp_data_storage[i] = steinhart;
        }
        
        snprintf(sd_print_string, sizeof(sd_print_string), "1)%.1f\r\n2)%.1f\r\n3)%.1f\r\n4)%.1f\r\n5)%.1f\r\n6)%.1f\r\n7)%.1f\r\n8)%.1f\r\n9)%.1f\r\n10)%.1f\r\n11)%.1f\r\n12)%.1f\r\n13)%.1f\r\n14)%.1f\r\n15)%.1f\r\n16)%.1f\r\n\n", temp_data_storage[0],
                                                                                                                                                                                                                                        temp_data_storage[1],
                                                                                                                                                                                                                                        temp_data_storage[2],
                                                                                                                                                                                                                                        temp_data_storage[3],
                                                                                                                                                                                                                                        temp_data_storage[4],
                                                                                                                                                                                                                                        temp_data_storage[5],
                                                                                                                                                                                                                                        temp_data_storage[6],
                                                                                                                                                                                                                                        temp_data_storage[7],
                                                                                                                                                                                                                                        temp_data_storage[8],
                                                                                                                                                                                                                                        temp_data_storage[9],
                                                                                                                                                                                                                                        temp_data_storage[10],
                                                                                                                                                                                                                                        temp_data_storage[11],
                                                                                                                                                                                                                                        temp_data_storage[12],
                                                                                                                                                                                                                                        temp_data_storage[13],
                                                                                                                                                                                                                                        temp_data_storage[14],
                                                                                                                                                                                                                                        temp_data_storage[15]);
        
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
    }
}

uint8_t * get_adc_string(void)
{
  return sd_print_string;
}

int get_adc_flag(void)
{
  return adc_flag;
}

void set_adc_flag(int f)
{
  adc_flag = f;
}

void saadc_init(void)
{
    ret_code_t err_code;
	
    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT;
	
    nrf_saadc_channel_config_t channel_0_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    //channel_0_config.gain = NRF_SAADC_GAIN4;
    channel_0_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_1_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    //channel_1_config.gain = NRF_SAADC_GAIN4;
    channel_1_config.reference = NRF_SAADC_REFERENCE_VDD4;
		
    nrf_saadc_channel_config_t channel_2_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    //channel_2_config.gain = NRF_SAADC_GAIN4;
    channel_2_config.reference = NRF_SAADC_REFERENCE_VDD4;
		
    nrf_saadc_channel_config_t channel_3_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
    //channel_3_config.gain = NRF_SAADC_GAIN4;
    channel_3_config.reference = NRF_SAADC_REFERENCE_VDD4;
		
    nrf_saadc_channel_config_t channel_4_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
    //channel_4_config.gain = NRF_SAADC_GAIN4;
    channel_4_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_5_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
    //channel_5_config.gain = NRF_SAADC_GAIN4;
    channel_5_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_6_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
    //channel_6_config.gain = NRF_SAADC_GAIN4;
    channel_6_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_7_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);
    //channel_7_config.gain = NRF_SAADC_GAIN4;
    channel_7_config.reference = NRF_SAADC_REFERENCE_VDD4;				
	
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
    APP_ERROR_CHECK(err_code);	
    err_code = nrf_drv_saadc_channel_init(4, &channel_4_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(5, &channel_5_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(6, &channel_6_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(7, &channel_7_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code); 

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}
