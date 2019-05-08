#include "mpu9250.h"
#include <stdint.h>
#include "main.h"


uint8_t mpuAdress = 0x68 << 1;
uint8_t magAdress = 0x0C << 1;
uint8_t i2cBuf[15];
uint32_t samples = 0;	// Used for calibration

// offsets calculated at power up
int16_t gyro_x_offset = 0;
int16_t gyro_y_offset = 0;
int16_t gyro_z_offset = 0;
int mpuReady =0;

int16_t acc_x_raw, acc_y_raw, acc_z_raw, temp_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw;

float xAcc, yAcc, zAcc; 
float xGyro, yGyro, zGyro;
float temp;
float tempOffset = 21; // Don't really get how to calibrate the offset value.

// Perhaps you should send the pointer to where to store the data here once and not in the readData funciton
void mpuInit(void){
	// Reset
	i2cBuf[0] = 0x6B;
	i2cBuf[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, mpuAdress, i2cBuf, 2, 10);
	HAL_Delay(100);

	while( HAL_I2C_IsDeviceReady(&hi2c1, mpuAdress, 1, 10) != HAL_OK);
	//The following does nothing if we don't find the mpu. Just used the while loop for always waiting until we found the MPU.
	if(HAL_I2C_IsDeviceReady(&hi2c1, mpuAdress, 1, 10) == HAL_OK){
		HAL_GPIO_TogglePin(LED_PCB_GPIO_Port, LED_PCB_Pin);
		printf("\r\n Found MPU, starting initialisation\r\n");
		mpuReady = 1;
	}else{
		printf("\r\n Did not find MPU on adress: %02X\r\n", mpuAdress);
	}
	
	// Set Gyro sensitivity - +-500 dps
	i2cBuf[0] = 0x1B;
	i2cBuf[1] = 0x08;
	HAL_I2C_Master_Transmit(&hi2c1, mpuAdress, i2cBuf, 2, 10);
	
	// Set acceleration sensitivity - +-4g
	i2cBuf[0] = 0x1C;
	i2cBuf[1] = 0x08;
	HAL_I2C_Master_Transmit(&hi2c1, mpuAdress, i2cBuf, 2, 10);
	
	//Configure the Magetometer - AK8963C
	i2cBuf[0] = 0x37; // I2C_SLV0_ADDR2
	i2cBuf[1] = 0x02;
	HAL_I2C_Master_Transmit(&hi2c1, mpuAdress, i2cBuf, 2, 10);
	
	i2cBuf[0] = 0x0A;
	i2cBuf[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, magAdress, i2cBuf, 2, 10);
	
	
}

void readData(float *p){
	//Read 7 parameters (Acceleration xyz, Temp, Gyroscope xyz) => 14 byte since they are 2 bytes each. (int16_t)
	i2cBuf[0] = 0x3B;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, i2cBuf, 1, 10);
	i2cBuf[1] = 0x00;
	HAL_I2C_Master_Receive(&hi2c1, 0xD0, &i2cBuf[1], 14, 10);
	
	// Acceleration X, Y, Z
	acc_x_raw = i2cBuf[1] << 8 | i2cBuf[2];
	acc_y_raw = i2cBuf[3] << 8 | i2cBuf[4];
	acc_z_raw = i2cBuf[5] << 8 | i2cBuf[6];
	// Temperature
	temp_raw = i2cBuf[7] << 8 | i2cBuf[8];
	// Gyroscope X, Y, Z
	gyro_x_raw = i2cBuf[9] << 8 | i2cBuf[10];
	gyro_y_raw = i2cBuf[11] << 8 | i2cBuf[12];
	gyro_z_raw = i2cBuf[13] << 8 | i2cBuf[14]; 
	
	// calculate the offsets at power up, ignore first 1s, then calibrate under 1s.
	
	if(samples < 100) {
		samples++;
		return;
	} else if(samples < 500) {
		gyro_x_offset += gyro_x_raw;
		gyro_y_offset += gyro_y_raw;
		gyro_z_offset += gyro_z_raw;
		samples++;
		return;
	} else if(samples == 500) {
		gyro_x_offset /= 400.0;
		gyro_y_offset /= 400.0;
		gyro_z_offset /= 400.0;
		samples++;
	} else {
		gyro_x_raw -= gyro_x_offset;
		gyro_y_raw -= gyro_y_offset;
		if(gyro_z_raw > gyro_z_offset || gyro_z_raw < gyro_z_offset) // ignore reading if it is below the drift
			gyro_z_raw -= gyro_z_offset;
		else 
			gyro_z_raw = 0.0f;
	}
	
	
	// Compensate the sensitivity
	p[0] = acc_x_raw/8192.0;
	p[1] = acc_y_raw/8192.0;
	p[2] = acc_z_raw/8192.0;

	// Calculate real temp
	p[3] = (temp_raw - tempOffset)/ 333.87f + 21.0f; // Needed the "f" since it wanted to convert those numbers into doubles
	
	// Compensate the sensitivity and convert to radians/s
	p[4] = gyro_x_raw / 65.5 / 180.0 * 3.141592;
	p[5] = gyro_y_raw / 65.5 / 180.0 * 3.141592;
	p[6] = gyro_z_raw / 65.5 / 180.0 * 3.141592;
	
}
