/*
	Copyright 2022 Benjamin Vedder      benjamin@vedder.se
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com

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

#ifndef MAIN_COMMANDS_H_
#define MAIN_COMMANDS_H_

#include <stdint.h>
#include <stdbool.h>
#include "utils/datatypes.h"

// Private variables
bms_values bms;
mc_values values;
mc_configuration mcconf;

typedef void (*send_func_t)(unsigned char *, unsigned int);

// Functions
void commands_init(void);
void commands_process_packet(
	unsigned char *data, unsigned int len, send_func_t reply_func
);
void commands_send_packet(unsigned char *data, unsigned int len);
void commands_send_packet_can_last(unsigned char *data, unsigned int len);
send_func_t commands_get_send_func(void);
void commands_set_send_func(send_func_t func);

void commands_init_plot(char *namex, char *namey);
void commands_plot_add_graph(char *name);
void commands_plot_set_graph(int graph);
void commands_send_plot_points(float x, float y);
void commands_send_app_data(unsigned char *data, unsigned int len);

void commands_get_vesc_values(int uart_num);
void commands_get_bms_values(int uart_num);
void commands_get_mcconf_temp(int uart_num);
void commands_set_mcconf_temp(int store, int forward, int reply, int uart_num);

#endif /* MAIN_COMMANDS_H_ */
