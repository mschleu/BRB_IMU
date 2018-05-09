#include "mpu9250.h"

int imu_pinstate = 1;

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR  
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

char IMU_ADD = 0x68;
int16_t accH;
uint8_t accL;
int16_t myAcc[3];
float tru_vals[3];

#define IMU_TX_LEN 					12
#define IMU_RX_LEN 					12
uint8_t p_tx[IMU_TX_LEN];
uint8_t p_rx[IMU_RX_LEN];
uint8_t buf[35];

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;

float pitch, yaw, roll;
float deltat = 0.0f;                             // integration interval for both filter schemes
int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval                               // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};  // vector to hold integral error for Mahony method

unsigned char imu_buffer[20];

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;
/* Indicates if operation on TWI has ended. */
static volatile bool xfer_completed = false;

void getMres(void) 
{
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
          break;
  }
}


void getGres(void) 
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}


void getAres(void) 
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

__STATIC_INLINE void data_handler(uint8_t temp)
{
	for(int i = 0; i < sizeof(p_rx); i++)
	{
		printf("IMU return val: %d\r\n", p_rx[i]);
	}
}

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            xfer_completed = true;
            break;
        default:
            break;
    }
}

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_mpu9250_config = {
       .scl                = IMU_SCL_PIN,
       .sda                = IMU_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_mpu9250_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void imu_init(void)
{
        twi_init();
	p_tx[0] = 0x75;
	nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(IMU_ADD, p_tx, 1, p_rx, 1);
	ret_code_t ret = nrf_drv_twi_xfer(&m_twi, &xfer, 0);
	
	if(ret == NRF_SUCCESS)
	{
		while(xfer_completed == false){}
	}
	
	char whoami = p_rx[0];
	if(whoami == 0x71)
	{
		resetMPU9250();
		calibrateMPU9250();
                nrf_gpio_pin_clear(23);
		nrf_delay_ms(200);
		initMPU9250();
	}
	getAres();
}

void resetMPU9250(void) 
{
  // reset device
	p_tx[0] = PWR_MGMT_1;
	p_tx[1] = 0x80;
	tx_imu_data(2);
}

void calibrateMPU9250(void)
{
	//uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
	// reset device, reset all registers, clear gyro and accelerometer bias registers
  p_tx[0] = PWR_MGMT_1;
	p_tx[1] = 0x80;
  tx_imu_data(2); // Write a one to bit 7 reset bit; toggle reset device
  nrf_delay_ms(100);  
   
	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	p_tx[0] = PWR_MGMT_1;
	p_tx[1] = 0x01;
  tx_imu_data(2);
	
	p_tx[0] = PWR_MGMT_2;
	p_tx[1] = 0x00;
  tx_imu_data(2);
  nrf_delay_ms(200); 
  
	// Configure device for bias calculation
	p_tx[0] = INT_ENABLE;
	p_tx[1] = 0x00;
  tx_imu_data(2);	// Disable all interrupts
	
	p_tx[0] = FIFO_EN;
	p_tx[1] = 0x00;
  tx_imu_data(2); // Disable FIFO
	
	p_tx[0] = PWR_MGMT_1;
	p_tx[1] = 0x00;
  tx_imu_data(2);    // Turn on internal clock source
	
	p_tx[0] = I2C_MST_CTRL;
	p_tx[1] = 0x00;
  tx_imu_data(2);	  // Disable I2C master
	
	p_tx[0] = USER_CTRL;
	p_tx[1] = 0x00;
  tx_imu_data(2);    // Disable FIFO and I2C master modes
	p_tx[1] = 0x0C;
  tx_imu_data(2);    // Reset FIFO and DMP
  nrf_delay_ms(15); 
  
	// Configure MPU9250 gyro and accelerometer for bias calculation
	p_tx[0] = CONFIG_REG;
	p_tx[1] = 0x01;
  tx_imu_data(2);      // Set low-pass filter to 188 Hz
	
	p_tx[0] = SMPLRT_DIV;
	p_tx[1] = 0x00;
  tx_imu_data(2);		  // Set sample rate to 1 kHz
	
	p_tx[0] = GYRO_CONFIG;
	p_tx[1] = 0x00;
  tx_imu_data(2);		  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	
	p_tx[0] = ACCEL_CONFIG;
	p_tx[1] = 0x00;
  tx_imu_data(2);		  // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384; //16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	p_tx[0] = USER_CTRL;
	p_tx[1] = 0x40;
  tx_imu_data(2);	   // Enable FIFO  
	
	p_tx[0] = FIFO_EN;
	p_tx[1] = 0x78;
  tx_imu_data(2);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  nrf_delay_ms(40);  // accumulate 40 samples in 80 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	p_tx[0] = FIFO_EN;
	p_tx[1] = 0x00;
  tx_imu_data(2);	    // Disable gyro and accelerometer sensors for FIFO
	
	p_tx[0] = FIFO_COUNTH;
  tx_imu_data(1);
  rx_imu_data(2);           // read FIFO sample count
  fifo_count = ((uint16_t)p_rx[0] << 8) | p_rx[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) 
  {
      int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		
      p_tx[0] = FIFO_R_W;
      tx_imu_data(1);
      rx_imu_data(12);	 // read data for averaging
      accel_temp[0] = (int16_t) (((int16_t)p_rx[0] << 8) | p_rx[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
      accel_temp[1] = (int16_t) (((int16_t)p_rx[2] << 8) | p_rx[3]  ) ;
      accel_temp[2] = (int16_t) (((int16_t)p_rx[4] << 8) | p_rx[5]  ) ;    
      gyro_temp[0]  = (int16_t) (((int16_t)p_rx[6] << 8) | p_rx[7]  ) ;
      gyro_temp[1]  = (int16_t) (((int16_t)p_rx[8] << 8) | p_rx[9]  ) ;
      gyro_temp[2]  = (int16_t) (((int16_t)p_rx[10] << 8) | p_rx[11]) ;
    
      accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
      accel_bias[1] += (int32_t) accel_temp[1];
      accel_bias[2] += (int32_t) accel_temp[2];
      gyro_bias[0]  += (int32_t) gyro_temp[0];
      gyro_bias[1]  += (int32_t) gyro_temp[1];
      gyro_bias[2]  += (int32_t) gyro_temp[2];
            
  }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  p_rx[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  p_rx[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  p_rx[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  p_rx[3] = (-gyro_bias[1]/4)       & 0xFF;
  p_rx[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  p_rx[5] = (-gyro_bias[2]/4)       & 0xFF;

	/// Push gyro biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
		writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
		writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
		writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
		writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
		writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
	*/

        p_tx[0] = XG_OFFSET_H;
	p_tx[1] = p_rx[0];
  tx_imu_data(2); 
	
	p_tx[0] = XG_OFFSET_L;
	p_tx[1] = p_rx[1];
  tx_imu_data(2); 
	
	p_tx[0] = YG_OFFSET_H;
	p_tx[1] = p_rx[2];
  tx_imu_data(2); 
	
	p_tx[0] = YG_OFFSET_L;
	p_tx[1] = p_rx[3];
  tx_imu_data(2);  
	
	p_tx[0] = ZG_OFFSET_H;
	p_tx[1] = p_rx[4];
  tx_imu_data(2);  
	
	p_tx[0] = ZG_OFFSET_L;
	p_tx[1] = p_rx[5];
  tx_imu_data(2); 

  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	p_tx[0] = XA_OFFSET_H;
	tx_imu_data(1);
	rx_imu_data(2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) ((int16_t)p_rx[0] << 8) | p_rx[1];
	p_tx[0] = YA_OFFSET_H;
	tx_imu_data(1);
	rx_imu_data(2);
  accel_bias_reg[1] = (int32_t) ((int16_t)p_rx[0] << 8) | p_rx[1];
	p_tx[0] = ZA_OFFSET_H;
	tx_imu_data(1);
	rx_imu_data(2);
  accel_bias_reg[2] = (int32_t) ((int16_t)p_rx[0] << 8) | p_rx[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  p_rx[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  p_rx[1] = (accel_bias_reg[0])      & 0xFF;
  p_rx[1] = p_rx[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  p_rx[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  p_rx[3] = (accel_bias_reg[1])      & 0xFF;
  p_rx[3] = p_rx[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  p_rx[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  p_rx[5] = (accel_bias_reg[2])      & 0xFF;
  p_rx[5] = p_rx[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
		writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
		writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
		writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
		writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
		writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	*/
	p_tx[0] = XA_OFFSET_H;
	p_tx[1] = p_rx[0];
  tx_imu_data(2); 
	
	p_tx[0] = XA_OFFSET_L;
	p_tx[1] = p_rx[1];
  tx_imu_data(2); 
	
	p_tx[0] = YA_OFFSET_H;
	p_tx[1] = p_rx[2];
  tx_imu_data(2); 
	
	p_tx[0] = YA_OFFSET_L;
	p_tx[1] = p_rx[3];
  tx_imu_data(2);  
	
	p_tx[0] = ZA_OFFSET_H;
	p_tx[1] = p_rx[4];
  tx_imu_data(2);  
	
	p_tx[0] = ZA_OFFSET_L;
	p_tx[1] = p_rx[5];
  tx_imu_data(2); 
	
	// Output scaled accelerometer biases for manual subtraction in the main program
  accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity; 
  accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
  accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void initMPU9250(void)
{
	// Initialize MPU9250 device
	// wake up device
        p_tx[0] = PWR_MGMT_1;
	p_tx[1] = 0;
        tx_imu_data(2);  // Clear sleep mode bit (6), enable all sensors 
        nrf_delay_ms(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

	// get stable time source
	p_tx[1] = 1;
        tx_imu_data(200);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

       // Configure Gyro and Accelerometer
       // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
       // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
       // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
        p_tx[0] = CONFIG_REG;
	p_tx[1] = 0x03;
        tx_imu_data(2);
 
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        p_tx[0] = SMPLRT_DIV;
	p_tx[1] = 0x04;
        tx_imu_data(2);  // Use a 200 Hz rate; the same rate set in CONFIG above
 
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	p_tx[0] = GYRO_CONFIG;
        tx_imu_data(1);
        rx_imu_data(1); 
        uint8_t c = p_rx[0]; // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5] 
        c = c & ~0x02; // Clear Fchoice bits [1:0] 
        c = c & ~0x18; // Clear AFS bits [4:3]
        c = c | Gscale << 3; // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	p_tx[0] = GYRO_CONFIG;
        p_tx[1] = c;
        tx_imu_data(2); // Write new GYRO_CONFIG value to register
  
	// Set accelerometer full-scale range configuration
	p_tx[0] = ACCEL_CONFIG;
        tx_imu_data(1);
        rx_imu_data(1); 
        c = p_rx[0]; // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5] 
        c = c & ~0x18;  // Clear AFS bits [4:3]
        c = c | Ascale << 3; // Set full scale range for the accelerometer 
        p_tx[0] = ACCEL_CONFIG;
        p_tx[1] = c;
        tx_imu_data(2); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        p_tx[0] = ACCEL_CONFIG2;
        tx_imu_data(1);
        rx_imu_data(1);
        c = p_rx[0];// get current ACCEL_CONFIG2 register value
        c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
        c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        p_tx[0] = ACCEL_CONFIG2;
        p_tx[1] = c;
        tx_imu_data(2); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
        p_tx[0] = INT_PIN_CFG;
        p_tx[1] = 0x22;
        tx_imu_data(2);
  
        p_tx[0] = INT_ENABLE;
        p_tx[1] = 0x01;
        tx_imu_data(2); // Enable data ready (bit 0) interrupt
}

void rx_imu_data(int num_xfer)
{
	xfer_completed = false;
	ret_code_t ret = nrf_drv_twi_rx(&m_twi, IMU_ADD, p_rx, num_xfer);
  if(ret == NRF_SUCCESS){	while(xfer_completed == false){}	}
}

void tx_imu_data(int num_xfer)
{
	xfer_completed = false;
	ret_code_t ret = nrf_drv_twi_tx(&m_twi, IMU_ADD, p_tx, num_xfer, false);
  if(ret == NRF_SUCCESS){	while(xfer_completed == false){}	}
}

uint8_t * imu_loop(void)
{
	
		p_tx[0] = INT_STATUS;
		tx_imu_data(1);
		rx_imu_data(1);
		
		if(p_rx[0] & 1)
		{
			p_tx[0] = ACCEL_XOUT_H;
			tx_imu_data(1);
			rx_imu_data(3);
			accH = p_rx[0];
			
			p_tx[0] = ACCEL_XOUT_L;
			tx_imu_data(1);
			rx_imu_data(3);
			accL = p_rx[0];
			
			myAcc[0] = accH << 8 | accL;
			tru_vals[0] = (float)myAcc[0] * aRes;// - accelBias[0];
			
			p_tx[0] = ACCEL_YOUT_H;
			tx_imu_data(1);
			rx_imu_data(3);
			accH = p_rx[0];
			
			p_tx[0] = ACCEL_YOUT_L;
			tx_imu_data(1);
			rx_imu_data(3);
			accL = p_rx[0];
			
			myAcc[1] = accH << 8 | accL;
			tru_vals[1] = (float)myAcc[1] * aRes;// - accelBias[1];
			
			p_tx[0] = ACCEL_ZOUT_H;
			tx_imu_data(1);
			rx_imu_data(3);
			accH = p_rx[0];
			
			p_tx[0] = ACCEL_ZOUT_L;
			tx_imu_data(1);
			rx_imu_data(3);
			accL = p_rx[0];
			
			myAcc[2] = accH << 8 | accL;
			tru_vals[2] = (float)myAcc[2] * aRes;// - accelBias[2];
			
			//printf("x: %.5f y: %.5f z: %.5f\r\n", myAcc[0], myAcc[1], myAcc[2]);
			//nrf_delay_ms(200);
			//snprintf((char *)buf, 10, "\nx:%.3f", myAcc[0]);
			snprintf(buf, sizeof(buf), "%.5f,%.5f,%.5f\r\n", tru_vals[0], tru_vals[1], tru_vals[2]);
			
			//uart_tx(buf, sizeof(buf));
			//printf(buf);
			return buf;
		}
}
