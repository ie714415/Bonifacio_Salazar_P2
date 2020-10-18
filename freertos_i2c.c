#include "freertos_i2c.h"
#include "fsl_i2c.h"
#include "fsl_clock.h"
#include "fsl_port.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define NUMBER_OF_SERIAL_PORTS (4)

typedef struct
{
	uint8_t is_init;
	i2c_master_handle_t fsl_i2c_master_handle;
	 SemaphoreHandle_t mutex_tx_rx;
	 SemaphoreHandle_t tx_rx_sem;
} freertos_i2c_hanlde_t;

static freertos_i2c_hanlde_t freertos_i2c_handles[NUMBER_OF_SERIAL_PORTS] = {0};

static inline void freertos_i2c_enable_port_clock(freertos_i2c_port_t port);

static inline void freertos_i2c_enable_clock(freertos_i2c_number_t i2c_number);

static inline PORT_Type * freertos_i2c_get_port_base(freertos_i2c_port_t port);

static inline I2C_Type * freertos_i2c_get_i2c_base(freertos_i2c_number_t i2c_number);

static void fsl_i2c_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData);

freertos_i2c_flag_t freertos_i2c_init(freertos_i2c_config_t config)
{
	freertos_i2c_flag_t retval = freertos_i2c_fail;
	i2c_master_config_t fsl_i2c_master_config;

	if(config.i2c_number < NUMBER_OF_SERIAL_PORTS)
	{
		if(!freertos_i2c_handles[config.i2c_number].is_init)
		{
			freertos_i2c_handles[config.i2c_number].mutex_tx_rx = xSemaphoreCreateMutex();

			freertos_i2c_handles[config.i2c_number].tx_rx_sem = xSemaphoreCreateBinary();

			/**Enables I2C port*/
			freertos_i2c_enable_port_clock(config.port);
			freertos_i2c_enable_clock(config.i2c_number);
			/**Port Config*/
			PORT_SetPinMux(freertos_i2c_get_port_base(config.port), config.scl_pin, config.pin_mux);
			PORT_SetPinMux(freertos_i2c_get_port_base(config.port), config.sda_pin, config.pin_mux);

			I2C_MasterGetDefaultConfig(&fsl_i2c_master_config);
			fsl_i2c_master_config.baudRate_Bps = config.baudrate;

			switch(config.i2c_number)
			{
			case freertos_i2c0:
				I2C_MasterInit(freertos_i2c_get_i2c_base(freertos_i2c0), &fsl_i2c_master_config, CLOCK_GetFreq(I2C0_CLK_SRC));
				NVIC_SetPriority(I2C0_IRQn, 5);
				break;
			case freertos_i2c1:
				I2C_MasterInit(freertos_i2c_get_i2c_base(freertos_i2c1), &fsl_i2c_master_config, CLOCK_GetFreq(I2C1_CLK_SRC));
				NVIC_SetPriority(I2C1_IRQn, 5);
				break;
			case freertos_i2c2:
				I2C_MasterInit(freertos_i2c_get_i2c_base(freertos_i2c2), &fsl_i2c_master_config, CLOCK_GetFreq(I2C2_CLK_SRC));
				NVIC_SetPriority(I2C2_IRQn, 5);
				break;
			case freertos_i2c3:
				I2C_MasterInit(freertos_i2c_get_i2c_base(freertos_i2c3), &fsl_i2c_master_config, CLOCK_GetFreq(I2C3_CLK_SRC));
				NVIC_SetPriority(I2C3_IRQn, 5);
				break;
			}

			I2C_MasterTransferCreateHandle(freertos_i2c_get_i2c_base(config.i2c_number),
					&freertos_i2c_handles[config.i2c_number].fsl_i2c_master_handle, fsl_i2c_callback, NULL);

			freertos_i2c_handles[config.i2c_number].is_init = 1;

			retval = freertos_i2c_sucess;
		}
	}
	return retval;
}

freertos_i2c_flag_t freertos_i2c_transmit(freertos_i2c_number_t i2c_number, uint8_t * buffer, uint8_t lenght, uint8_t slaveaddr)
{
	freertos_i2c_flag_t flag = freertos_i2c_fail;
	i2c_master_transfer_t xfer;

	if(freertos_i2c_handles[i2c_number].is_init)
	{
		xfer.data = buffer;
		xfer.dataSize = lenght;
		xfer.direction = kI2C_Write;
		xfer.flags = kI2C_TransferDefaultFlag;
		xfer.slaveAddress = slaveaddr;
		xfer.subaddressSize = 0;

		xSemaphoreTake(freertos_i2c_handles[i2c_number].mutex_tx_rx, portMAX_DELAY);

		I2C_MasterTransferNonBlocking(freertos_i2c_get_i2c_base(i2c_number), &freertos_i2c_handles[i2c_number].fsl_i2c_master_handle, &xfer);

		xSemaphoreTake(freertos_i2c_handles[i2c_number].tx_rx_sem, portMAX_DELAY);

		xSemaphoreGive(freertos_i2c_handles[i2c_number].mutex_tx_rx);

		flag = freertos_i2c_sucess;
	}
	return flag;
}

freertos_i2c_flag_t freertos_i2c_receive(freertos_i2c_number_t i2c_number, uint8_t * buffer, uint8_t lenght, uint8_t slaveaddr, uint8_t subaddr)
{
	freertos_i2c_flag_t flag = freertos_i2c_fail;
	i2c_master_transfer_t xfer;

	if(freertos_i2c_handles[i2c_number].is_init)
	{
		xfer.data = &subaddr;
		xfer.dataSize = lenght;
		xfer.direction = kI2C_Write;
		xfer.flags = kI2C_TransferDefaultFlag;
		xfer.slaveAddress = slaveaddr;
		xfer.subaddressSize = 0;

		xSemaphoreTake(freertos_i2c_handles[i2c_number].mutex_tx_rx, portMAX_DELAY);

		I2C_MasterTransferNonBlocking(freertos_i2c_get_i2c_base(i2c_number), &freertos_i2c_handles[i2c_number].fsl_i2c_master_handle, &xfer);

		xSemaphoreTake(freertos_i2c_handles[i2c_number].tx_rx_sem, portMAX_DELAY);

		xfer.data = buffer;
		xfer.dataSize = lenght;
		xfer.direction = kI2C_Read;
		xfer.flags = kI2C_TransferDefaultFlag;
		xfer.slaveAddress = slaveaddr;
		xfer.subaddressSize = 0;

		I2C_MasterTransferNonBlocking(freertos_i2c_get_i2c_base(i2c_number), &freertos_i2c_handles[i2c_number].fsl_i2c_master_handle, &xfer);

		xSemaphoreTake(freertos_i2c_handles[i2c_number].tx_rx_sem, portMAX_DELAY);

		xSemaphoreGive(freertos_i2c_handles[i2c_number].mutex_tx_rx);

		flag = freertos_i2c_sucess;
	}
	return flag;
}

static inline void freertos_i2c_enable_port_clock(freertos_i2c_port_t port)
{
	switch(port)
	{
	case freertos_i2c_portA:
		CLOCK_EnableClock(kCLOCK_PortA);
		break;
	case freertos_i2c_portB:
		CLOCK_EnableClock(kCLOCK_PortB);
		break;
	case freertos_i2c_portC:
		CLOCK_EnableClock(kCLOCK_PortC);
		break;
	case freertos_i2c_portD:
		CLOCK_EnableClock(kCLOCK_PortD);
		break;
	case freertos_i2c_portE:
		CLOCK_EnableClock(kCLOCK_PortE);
		break;
	}
}

static inline void freertos_i2c_enable_clock(freertos_i2c_number_t i2c_number)
{
	switch(i2c_number)
	{
	case freertos_i2c0:
		CLOCK_EnableClock(kCLOCK_I2c0);
		break;
	case freertos_i2c1:
		CLOCK_EnableClock(kCLOCK_I2c1);
		break;
	case freertos_i2c2:
		CLOCK_EnableClock(kCLOCK_I2c2);
		break;
	case freertos_i2c3:
		CLOCK_EnableClock(kCLOCK_I2c3);
		break;
	}
}

static inline PORT_Type * freertos_i2c_get_port_base(freertos_i2c_port_t port)
{
	PORT_Type * port_base = PORTA;

	switch(port)
	{
	case freertos_i2c_portA:
		port_base = PORTA;
		break;
	case freertos_i2c_portB:
		port_base = PORTB;
		break;
	case freertos_i2c_portC:
		port_base = PORTC;
		break;
	case freertos_i2c_portD:
		port_base = PORTD;
		break;
	case freertos_i2c_portE:
		port_base = PORTE;
		break;
	}
	return port_base;
}

static inline I2C_Type * freertos_i2c_get_i2c_base(freertos_i2c_number_t i2c_number)
{
	I2C_Type * retval = I2C0;

	switch(i2c_number)
	{
	case freertos_i2c0:
		retval = I2C0;
		break;
	case freertos_i2c1:
		retval = I2C1;
		break;
	case freertos_i2c2:
		retval = I2C2;
		break;
	case freertos_i2c3:
		retval = I2C3;
		break;
	}
	return retval;
}

static void fsl_i2c_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(kStatus_Success == status)
	{
		if(I2C0 == base)
		{
			xSemaphoreGiveFromISR(freertos_i2c_handles[freertos_i2c0].tx_rx_sem, &xHigherPriorityTaskWoken);
		}
		else if(I2C1 == base)
		{
			xSemaphoreGiveFromISR(freertos_i2c_handles[freertos_i2c1].tx_rx_sem, &xHigherPriorityTaskWoken);
		}
		else if(I2C2 == base)
		{
			xSemaphoreGiveFromISR(freertos_i2c_handles[freertos_i2c2].tx_rx_sem, &xHigherPriorityTaskWoken);
		}
		else
		{
			xSemaphoreGiveFromISR(freertos_i2c_handles[freertos_i2c3].tx_rx_sem, &xHigherPriorityTaskWoken);
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
