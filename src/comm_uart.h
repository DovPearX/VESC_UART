/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef MAIN_COMM_UART_H_
#define MAIN_COMM_UART_H_

bool comm_uart_init(int pin_tx, int pin_rx, int uart_num, int baudrate);
void comm_uart_stop(int uart_num);
void comm_uart_send_packet(unsigned char *data, unsigned int len, int uart_num);

void comm_uart_get_vesc_values(int uart_num);
void comm_uart_get_bms_values(int uart_num);
void comm_uart_get_mcconf_temp(int uart_num);
void comm_uart_set_mcconf_temp(int store, int forward, int reply, int divide, int uart_num);

void comm_uart_set_duty(int uart_num, float duty);
void comm_uart_set_current(int uart_num, float current);
void comm_uart_set_current_brake(int uart_num, float current);
void comm_uart_set_rpm(int uart_num, float rpm);
void comm_uart_set_pos(int uart_num, float pos);
void comm_uart_set_handbrake(int uart_num, float current);
void comm_uart_set_current_rel(int uart_num, float current_rel);


#endif /* MAIN_COMM_UART_H_ */
