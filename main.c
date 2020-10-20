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
/* TODO: insert other definitions and declarations here. */
#define BAUDRATE 		115200
#define RX 				16
#define TX 				17
#define HEADER_UART		0xAAAAAAAA
#define SAMPLES 		100
#define SAMPLING_TIME	5	// 400Hz from bmi160 algorithm
#define SENDING_TIME	20	//30 Hz

typedef struct
{
	uint32_t header;
	float x;
	float y;
	float z;
} comm_msg_t;

MahonyAHRSEuler_t g_mahony_data;
comm_msg_t g_data;
bmi160_raw_data_t g_calibration_acc;
bmi160_raw_data_t g_calibration_gyro;
bmi160_raw_data_t g_average_acc;
bmi160_raw_data_t g_average_gyro;
uint8_t g_send_f;
/*
 * @brief   Application entry point.
 */
void init_sistem(void *parameters);
void read_data(void *parameters);
void send_data(void *parameters);
void calibrate_data(void *parameters);

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

    g_send_f = 0;
    g_data.header = HEADER_UART;
    while(!xTaskCreate(init_sistem, "init_sistem", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 1, NULL));;

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
		xTaskCreate(calibrate_data, "calibrate_data",configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 1, NULL);
	}
	vTaskSuspend(NULL);
}

void calibrate_data(void *parameters)
{
	TickType_t xLastWakeTime;
	TickType_t xfactor = pdMS_TO_TICKS(SAMPLING_TIME);
	bmi160_raw_data_t acc_data;
	bmi160_raw_data_t gyro_data;
	uint16_t idx;

	xLastWakeTime = xTaskGetTickCount();
	/*Initialize all data on cero*/
	g_calibration_acc.x = 0;
	g_calibration_acc.y = 0;
	g_calibration_acc.z = 0;

	g_calibration_gyro.x = 0;
	g_calibration_gyro.y = 0;
	g_calibration_gyro.z = 0;

	for(idx = 0; SAMPLES > idx; idx++)
	{
		acc_data = bmi160_get_data_accel();
		gyro_data = bmi160_get_data_gyro();
		/*We make an average of the BMI data when it is on idle state*/
		/*Necessary to wait until the calibration method is executed*/
		g_calibration_acc.x += acc_data.x;
		g_calibration_acc.y += acc_data.y;
		g_calibration_acc.z += acc_data.z;
		g_calibration_gyro.x += gyro_data.x;
		g_calibration_gyro.y += gyro_data.y;
		g_calibration_gyro.z += gyro_data.z;

		vTaskDelayUntil(&xLastWakeTime, xfactor);
	}
	/*Ending average process*/
	g_calibration_acc.x /= SAMPLES;
	g_calibration_acc.y /= SAMPLES;
	g_calibration_acc.z /= SAMPLES;
	g_calibration_gyro.x /= SAMPLES;
	g_calibration_gyro.y /= SAMPLES;
	g_calibration_gyro.z /= SAMPLES;
	/*Once calibration is made then the new tasks for sending and treathing data are created*/
	while(!xTaskCreate(read_data, "read_data", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 1, NULL));;
	while(!xTaskCreate(send_data, "send_data", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 2, NULL));;
	vTaskSuspend(NULL);
}

void read_data(void *parameters)
{
	TickType_t xLastWakeTime;
	TickType_t xfactor = pdMS_TO_TICKS(SAMPLING_TIME);
	bmi160_raw_data_t acc_data;
	bmi160_raw_data_t gyro_data;

	xLastWakeTime = xTaskGetTickCount();
	/*Initialize all data on zero*/
	g_average_acc.x = 0;
	g_average_acc.y = 0;
	g_average_acc.z = 0;
	g_average_gyro.x = 0;
	g_average_gyro.y = 0;
	g_average_gyro.z = 0;

	for(;;)
	{
		acc_data = bmi160_get_data_accel();
		gyro_data = bmi160_get_data_gyro();
		/*Take off offset from new data*/
		acc_data.x -= g_calibration_acc.x;
		acc_data.y -= g_calibration_acc.y;
		acc_data.z -= g_calibration_acc.z;
		gyro_data.x -= g_calibration_gyro.x;
		gyro_data.y -= g_calibration_gyro.y;
		gyro_data.z -= g_calibration_gyro.z;
		/*Take average to send to Mahony*/
		g_average_acc.x += acc_data.x;
		g_average_acc.y += acc_data.y;
		g_average_acc.z += acc_data.z;
		g_average_gyro.x += gyro_data.x;
		g_average_gyro.y += gyro_data.y;
		g_average_gyro.z += gyro_data.z;
		/*Increment flag*/
		g_send_f++;

		vTaskDelayUntil(&xLastWakeTime, xfactor);
	}
}

void send_data(void *parameters)
{
	TickType_t xLastWakeTime;
	TickType_t xfactor = pdMS_TO_TICKS(SAMPLING_TIME);
	uint8_t * pUART_data;

	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		/*Finalize average process*/
		g_average_acc.x /= g_send_f;
		g_average_acc.y /= g_send_f;
		g_average_acc.z /= g_send_f;
		g_average_gyro.x /= g_send_f;
		g_average_gyro.y /= g_send_f;
		g_average_gyro.z /= g_send_f;
		/*Sends the normalize data to mahony*/
		g_mahony_data = MahonyAHRSupdateIMU((float)g_average_gyro.x, (float)g_average_gyro.y, (float)g_average_gyro.z, (float)g_average_acc.x, (float)g_average_acc.y, (float)g_average_acc.z);
		g_data.x = g_mahony_data.roll;
		g_data.y = g_mahony_data.pitch;
		g_data.z = g_mahony_data.yaw;
		/*Sen data to the application*/
		pUART_data = (uint8_t *) &g_data;
		freertos_uart_send(freertos_uart0, pUART_data, sizeof(g_data));
		/*Re init all data*/
		g_average_acc.x  = 0 ;
		g_average_acc.y = 0 ;
		g_average_acc.z = 0 ;
		g_average_gyro.x = 0 ;
		g_average_gyro.y = 0 ;
		g_average_gyro.z = 0 ;
		g_send_f = 0;

		vTaskDelayUntil(&xLastWakeTime, xfactor);
	}
}
