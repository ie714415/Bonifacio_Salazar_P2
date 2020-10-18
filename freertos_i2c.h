#ifndef FREERTOS_I2C_H_
#define FREERTOS_I2C_H_

#include "stdint.h"

typedef enum
{
	freertos_i2c0,
	freertos_i2c1,
	freertos_i2c2,
	freertos_i2c3
} freertos_i2c_number_t;

typedef enum
{
	freertos_i2c_portA,
	freertos_i2c_portB,
	freertos_i2c_portC,
	freertos_i2c_portD,
	freertos_i2c_portE
} freertos_i2c_port_t;

typedef enum
{
	freertos_i2c_sucess,
	freertos_i2c_fail
} freertos_i2c_flag_t;

typedef struct
{
	uint32_t baudrate;
	freertos_i2c_number_t i2c_number;
	freertos_i2c_port_t port;
	uint8_t scl_pin;
	uint8_t sda_pin;
	uint8_t pin_mux;
} freertos_i2c_config_t;

freertos_i2c_flag_t freertos_i2c_init(freertos_i2c_config_t config);
freertos_i2c_flag_t freertos_i2c_transmit(freertos_i2c_number_t i2c_number, uint8_t * buffer, uint8_t lenght, uint8_t slaveaddr);
freertos_i2c_flag_t freertos_i2c_receive(freertos_i2c_number_t i2c_number, uint8_t * buffer, uint8_t lenght, uint8_t slaveaddr, uint8_t subaddr);

#endif /* FREERTOS_I2C_H_ */
