/*
 * BMI160.h
 *
 */

#ifndef BMI160_H_
#define BMI160_H_

#include "freertos_i2c.h"
#include "fsl_clock.h"
#include "fsl_port.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/*bmi160 registers*/
#define BMI160_SLAVE_ADDRESS		0x68
#define BMI160_CMD_REGISTER  		0x7E // all operations are made on this register

#define BMI160_ACC_NORMAL_MODE 		0x11
#define BMI160_GYRO_NORMAL_MODE 	0x15

#define BMI160_GRYO_DATA_X_LSB		0x0C
#define BMI160_GYRO_DATA_X_MSB		0x0D
#define BMI160_GYRO_DATA_Y_LSB		0x0E
#define BMI160_GYRO_DATA_Y_MSB      0x0F
#define BMI160_GYRO_DATA_Z_LSB      0x10
#define BMI160_GYRO_DATA_Z_MSB      0x11

#define BMI160_ACC_DATA_X_LSB		0x12
#define BMI160_ACC_DATA_X_MSB		0x13
#define BMI160_ACC_DATA_Y_LSB		0x14
#define BMI160_ACC_DATA_Y_MSB		0x15
#define BMI160_ACC_DATA_Z_LSB		0x16
#define BMI160_ACC_DATA_Z_MSB		0x17

#define BMI160_ACC_CONF				0x40
#define BMI160_ACC_800Hz			0x0B
#define BMI160_ACC_100HZ			0x08
#define BMI160_ACC_400HZ			0x0A

#define BMI160_GYRO_CONF			0x42
#define BMI160_GYRO_800Hz			0x0B
#define BMI160_GYRO_100HZ			0x08
#define BMI160_GYRO_400HZ			0x0A


typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
}bmi160_raw_data_t;

freertos_i2c_flag_t bmi160_init(void);
bmi160_raw_data_t bmi160_get_data_accel(void);
bmi160_raw_data_t bmi160_get_data_gyro(void);


#endif /* BMI160_H_ */
