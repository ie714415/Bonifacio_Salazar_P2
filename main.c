#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "BMI160.h"
#include "mahony.h"
#include "freertos_uart.h"
#include "freertos_i2c.h"
#include "task.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
/* TODO: insert other definitions and declarations here. */
#define BAUDRATE 		115200
#define RX 				16
#define TX 				17
#define HEADER_UART		0xAAAAAAAA

typedef struct
{
	uint32_t header;
	float x;
	float y;
	float z;
} comm_msg_t;

SemaphoreHandle_t tasks_sem;
/*
 * @brief   Application entry point.
 */
void init_sistem(void *parameters);
void read_data(void *parameters);

int main(void)
{
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    tasks_sem = xSemaphoreCreateBinary();
    xTaskCreate(init_sistem, "init_sistem", 110, NULL, 2, NULL);
    xTaskCreate(read_data, "read_data", 110, NULL, 1, NULL);

    vTaskStartScheduler();
    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}

void init_sistem(void *parameters)
{
	freertos_uart_flag_t uart_succes = freertos_uart_fail;
	freertos_i2c_flag_t bmi160_sucess = freertos_i2c_fail;
	uint8_t msg[] = "UART and BMI160 configured\n\r";
	freertos_uart_config_t config;

	config.baudrate = BAUDRATE;
	config.pin_mux = kPORT_MuxAlt3;
	config.port = freertos_uart_portB;
	config.rx_pin = RX;
	config.tx_pin = TX;
	config.uart_number = freertos_uart0;

	uart_succes = freertos_uart_init(config);
	bmi160_sucess = bmi160_init();
	if((freertos_uart_sucess == uart_succes) && (freertos_i2c_sucess == bmi160_sucess))
	{
		freertos_uart_send(freertos_uart0, &msg[0], sizeof(msg));
	}
	xSemaphoreGive(tasks_sem);

	vTaskSuspend(NULL);
}

void read_data(void *parameters)
{
	xSemaphoreTake(tasks_sem, portMAX_DELAY);

	bmi160_raw_data_t acc_data;
	bmi160_raw_data_t gyro_data;
	MahonyAHRSEuler_t mahony_data;
	uint8_t * pUART_data;
	comm_msg_t data;

	data.header = HEADER_UART;
	pUART_data = (uint8_t *) &data;

	for(;;)
	{
		acc_data = bmi160_get_data_accel();
		gyro_data = bmi160_get_data_gyro();

		mahony_data = MahonyAHRSupdateIMU((float)gyro_data.x, (float)gyro_data.y, (float)gyro_data.z, (float)acc_data.x, (float)acc_data.y, (float)acc_data.z);
		data.x = mahony_data.roll;
		data.y = mahony_data.pitch;
		data.z = mahony_data.yaw;

		//Enviar los datos a la interfaz
		freertos_uart_send(freertos_uart0, pUART_data, sizeof(data));

		vTaskDelay(pdMS_TO_TICKS(300));
	}
}
