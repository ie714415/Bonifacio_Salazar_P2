#include "BMI160.h"

#define I2C_SCL		2U
#define I2C_SDA		3U
#define BAUD_RATE	115200
#define I2C_DELAY	100//(ms)

/*Global declaratios*/
static freertos_i2c_config_t bmi160_i2c_config;

freertos_i2c_flag_t bmi160_init(void)
{
	freertos_i2c_flag_t bmi160_sucess = freertos_i2c_fail;
	/*Start port and freertos i2c inicialization */
	bmi160_i2c_config.baudrate = BAUD_RATE;
	bmi160_i2c_config.i2c_number = 	freertos_i2c0;
	bmi160_i2c_config.port = freertos_i2c_portB;
	bmi160_i2c_config.scl_pin = I2C_SCL;
	bmi160_i2c_config.sda_pin = I2C_SDA;
	bmi160_i2c_config.pin_mux = kPORT_MuxAlt2;

	/*Init configuration for freertos i2c*/
	bmi160_sucess = freertos_i2c_init(bmi160_i2c_config);
	vTaskDelay(pdMS_TO_TICKS(10));

	if(freertos_i2c_sucess == bmi160_sucess)
	{
		bmi160_sucess = freertos_i2c_fail;
		/*Initial configuration of bmi160*/
		uint8_t acc_normal[2] = {BMI160_CMD_REGISTER, BMI160_ACC_NORMAL_MODE};
		uint8_t gyro_normal[2] = {BMI160_CMD_REGISTER, BMI160_GYRO_NORMAL_MODE};;
		/*Acc config*/
		bmi160_sucess = freertos_i2c_transmit(bmi160_i2c_config.i2c_number, &acc_normal , 2, BMI160_SLAVE_ADDRESS);
		vTaskDelay(pdMS_TO_TICKS(I2C_DELAY));
		/*Gyro config*/
		if(freertos_i2c_fail != bmi160_sucess)
		{
			bmi160_sucess = freertos_i2c_transmit(bmi160_i2c_config.i2c_number, &gyro_normal , 2, BMI160_SLAVE_ADDRESS);
		}
		vTaskDelay(pdMS_TO_TICKS(I2C_DELAY));
	}
	return bmi160_sucess;
}


bmi160_raw_data_t bmi160_get_data_accel(void)
{
	bmi160_raw_data_t acc_data;
	uint8_t aux_LSB_data = 0;
	uint8_t aux_MSB_data = 0;

	/*LSB and then MSB for X,Y,Z*/
	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_LSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_ACC_DATA_X_LSB);
	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_MSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_ACC_DATA_X_MSB);
	acc_data.x = (aux_MSB_data << 8) | aux_LSB_data;

	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_LSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_ACC_DATA_Y_LSB);
	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_MSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_ACC_DATA_Y_MSB);
	acc_data.y = (aux_MSB_data << 8) | aux_LSB_data;

	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_LSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_ACC_DATA_Z_LSB);
	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_MSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_ACC_DATA_Z_MSB);
	acc_data.z = (aux_MSB_data << 8) | aux_LSB_data;

	return acc_data;
}

bmi160_raw_data_t bmi160_get_data_gyro(void)
{
	bmi160_raw_data_t gyro_data;
	uint8_t aux_LSB_data = 0;
	uint8_t aux_MSB_data = 0;

	/*LSB and then MSB for X,Y,Z*/
	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_LSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_GRYO_DATA_X_LSB);
	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_MSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_GYRO_DATA_X_MSB);
	gyro_data.x = (aux_MSB_data << 8) | aux_LSB_data;

	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_LSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_GYRO_DATA_Y_LSB);
	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_MSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_GYRO_DATA_Y_MSB);
	gyro_data.y = (aux_MSB_data << 8) | aux_LSB_data;

	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_LSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_GYRO_DATA_Z_LSB);
	freertos_i2c_receive(bmi160_i2c_config.i2c_number, &aux_MSB_data, 1, BMI160_SLAVE_ADDRESS , BMI160_GYRO_DATA_Z_MSB);
	gyro_data.z = (aux_MSB_data << 8) | aux_LSB_data;

	return gyro_data;
}


