extern "C"
{
#include "hw.h"
#include "commands.h"
#include "utils/datatypes.h"
#include "comm_uart.h"
#include "comm_can.h"
#include "bms.h"
#include "utils/mempools.h"
#include "utils/packet.h"
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

void vesc_init() 
{
    mempools_init();
    commands_init();

    #ifdef CAN_TX_GPIO_NUM
    comm_can_start(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM);
    #else
    #ifdef HW_UART_COMM
    comm_uart_init(UART_TX, UART_RX, UART_NUM, UART_BAUDRATE);
    #endif
    #endif

    #ifdef CAN_TX_GPIO_NUM
    vTaskDelay(100);
    comm_can_get_mcconf_temp(VESC_ID);
    #else
    #ifdef HW_UART_COMM
    vTaskDelay(100);
    comm_uart_get_mcconf_temp(UART_NUM);
    #endif
    #endif
}

