#ifndef HW_H_
#define HW_H_

#include "driver/gpio.h"

#define FW_VERSION_MAJOR			6
#define FW_VERSION_MINOR			02
#define HW_NAME "VESC"
#define FW_NAME "V1"

#define STATUS_RATE 10
#define CONTROLLER_ID 50
#define VESC_ID 20

///////////////////////////////////////////////////////////////////////////////

// #define HW_UART_COMM
// #define UART_NUM					2
// #define UART_BAUDRATE				115200
// #define UART_TX						17
// #define UART_RX						16

///////////////////////////////////////////////////////////////////////////////

#define CAN_TX_GPIO_NUM GPIO_NUM_17
#define CAN_RX_GPIO_NUM GPIO_NUM_18

///////////////////////////////////////////////////////////////////////////////


#endif /* HW_H_ */
