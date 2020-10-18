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
#define BAUDRATE 115200
#define RX 16
#define TX 17

typedef struct
{
	uint32_t header;
	float x;
	float y;
	float z;
} comm_msg_t;
/*
 * @brief   Application entry point.
 */
void init_sistem(void *parameters);

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

    xTaskCreate(init_sistem, "init_sistem", 110, NULL, 1, NULL);

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
	uint8_t msg[] = "UART and BMI160 configured\n";
	uint8_t * data = msg[0];

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
		while('\0' != &data)
		{
			freertos_uart_send(freertos_uart0, &data, 1);
			data++;
		}
	}
	vTaskSuspend(NULL);
}
