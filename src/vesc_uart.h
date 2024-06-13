extern "C"{
#include "commands.h"
#include "comm_uart.h"
#include "utils/mempools.h"
#include "utils/packet.h"
}

void vesc_uart_init(int pin_tx, int pin_rx, int uart_num, int baudrate) 
{
    mempools_init();
    commands_init();
    comm_uart_init(pin_tx, pin_rx, uart_num, baudrate);
}
