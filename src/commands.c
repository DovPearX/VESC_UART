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

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <dirent.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "commands.h"
#include "comm_uart.h"
#include "utils/datatypes.h"
#include "utils/mempools.h"
#include "utils/utils.h"

#include "utils/packet.h"
#include "utils/buffer.h"
#include "utils/crc.h"

extern bms_values bms;
extern mc_values values;

// For double precision literals
#define D(x) 						((double)x##L)

// Function pointers
static send_func_t send_func = 0;
static send_func_t send_func_can_fwd = 0;
static send_func_t send_func_blocking = 0;

// Blocking thread
static SemaphoreHandle_t block_sem;
static uint8_t blocking_thread_cmd_buffer[PACKET_MAX_PL_LEN];
static volatile unsigned int blocking_thread_cmd_len = 0;
static volatile bool is_blocking = false;

static void block_task(void *arg) {
	for (;;) {
		is_blocking = false;

		xSemaphoreTake(block_sem, portMAX_DELAY);

		uint8_t *data = blocking_thread_cmd_buffer;
		unsigned int len = blocking_thread_cmd_len;

		COMM_PACKET_ID packet_id;
		static uint8_t send_buffer[512];

		packet_id = data[0];
		data++;
		len--;

		switch (packet_id) {
		case COMM_PING_CAN: {
			int32_t ind = 0;
			send_buffer[ind++] = COMM_PING_CAN;

			for (uint8_t i = 0;i < 255;i++) {
				// HW_TYPE hw_type;
				// if (comm_can_ping(i, &hw_type)) {
				// 	send_buffer[ind++] = i;
				// }
			}

			if (send_func_blocking) {
				send_func_blocking(send_buffer, ind);
			}
		} break;

		case COMM_TERMINAL_CMD:
			data[len] = '\0';
			//terminal_process_string((char*)data);
			break;

		default:
			break;
		}

	}

	vTaskDelete(NULL);
}

void commands_init(void) {
	block_sem = xSemaphoreCreateBinary();
	xTaskCreatePinnedToCore(block_task, "comm_block", 2500, NULL, 7, NULL, tskNO_AFFINITY);
}

void commands_process_packet(unsigned char *data, unsigned int len,
		send_func_t reply_func) {
	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	if (!send_func_can_fwd) {
		send_func_can_fwd = reply_func;
	}

	switch (packet_id) {
	case COMM_BMS_GET_VALUES: {
			int32_t ind = 0;

			bms.v_tot = buffer_get_float32(data, 1000000.0, &ind);
			bms.v_charge = buffer_get_float32(data, 1000000.0, &ind);
			bms.i_in = buffer_get_float32(data, 1000000.0, &ind);
			bms.i_in_ic = buffer_get_float32(data, 1000000.0, &ind);
			bms.ah_cnt = buffer_get_float32(data, 1000.0, &ind);
			bms.wh_cnt = buffer_get_float32(data, 1000.0, &ind);

			for (int i = 0;i < bms.cell_num;i++) {
			bms.v_cell[i] = buffer_get_float16(data, 1000, &ind);
			}

			for(int i = 0;i < bms.cell_num;i++) {
				bms.bal_state[i] = (data[ind++] != 0);
			}

			bms.temp_adc_num = buffer_get_float16(data, 1000, &ind);

			for(int i = 0;i < bms.temp_adc_num;i++) {
				bms.temps_adc[i] = buffer_get_float16(data, 100, &ind);
			}

			bms.temp_ic = buffer_get_float16(data, 100, &ind);

			bms.temp_hum = buffer_get_float16(data, 100, &ind);
			bms.hum = buffer_get_float16(data, 100, &ind);

			bms.temp_max_cell = buffer_get_float16(data, 100, &ind);

			bms.soc = buffer_get_float16(data, 1000, &ind);
			bms.soh = buffer_get_float16(data, 1000, &ind);

			bms.can_id = data[ind++];
			
			bms.ah_cnt_chg_total = buffer_get_float32_auto(data, &ind);
			bms.wh_cnt_chg_total = buffer_get_float32_auto(data, &ind);
			bms.ah_cnt_dis_total = buffer_get_float32_auto(data, &ind);
			bms.wh_cnt_dis_total = buffer_get_float32_auto(data, &ind);

			bms.pressure = buffer_get_float16(data, 0.1, &ind);

	} break;

	case COMM_GET_VALUES: {
			int32_t ind = 0;

			values.temp_fet = buffer_get_float16(data, 1e1, &ind);
			values.temp_motor = buffer_get_float16(data, 1e1, &ind);
			values.current_motor = buffer_get_float32(data, 1e2, &ind);
			values.current_in = buffer_get_float32(data, 1e2, &ind);
			values.id = buffer_get_float32(data, 1e2, &ind);
			values.iq = buffer_get_float32(data, 1e2, &ind);
			values.duty_now = buffer_get_float16(data, 1e3, &ind);
			values.rpm = buffer_get_float32(data, 1e0, &ind);
			values.v_in = buffer_get_float16(data, 1e1, &ind);
			values.amp_hours = buffer_get_float32(data, 1e4, &ind);
			values.amp_hours_charged = buffer_get_float32(data, 1e4, &ind);
			values.watt_hours = buffer_get_float32(data, 1e4, &ind);
			values.watt_hours_charged = buffer_get_float32(data, 1e4, &ind);
			values.tachometer = buffer_get_int32(data, &ind);
			values.tachometer_abs = buffer_get_int32(data, &ind);
			values.fault_code = data[ind++];
			values.position = buffer_get_float32(data, 1e6, &ind);
			values.vesc_id = data[ind++];
			ind += 4;
			values.vd = buffer_get_float32(data, 1e3, &ind);
			values.vq = buffer_get_float32(data, 1e3, &ind);
			values.killswitch = data[ind++];

	} break;


	// Blocking commands
	case COMM_TERMINAL_CMD:
	case COMM_PING_CAN:
		if (!is_blocking) {
			memcpy(blocking_thread_cmd_buffer, data - 1, len + 1);
			blocking_thread_cmd_len = len + 1;
			is_blocking = true;
			send_func_blocking = reply_func;
			xSemaphoreGive(block_sem);
		}
		break;

	default:
		break;
	}
}

/**
 * Send a packet using the last can fwd function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet_can_last(unsigned char *data, unsigned int len) {
	if (send_func_can_fwd) {
		send_func_can_fwd(data, len);
	}
}

void commands_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

send_func_t commands_get_send_func(void) {
	return send_func;
}

void commands_set_send_func(send_func_t func) {

	send_func = func;
}

void commands_init_plot(char *namex, char *namey) {
	int ind = 0;
	uint8_t *send_buffer_global = mempools_get_packet_buffer();
	send_buffer_global[ind++] = COMM_PLOT_INIT;
	memcpy(send_buffer_global + ind, namex, strlen(namex));
	ind += strlen(namex);
	send_buffer_global[ind++] = '\0';
	memcpy(send_buffer_global + ind, namey, strlen(namey));
	ind += strlen(namey);
	send_buffer_global[ind++] = '\0';
	commands_send_packet(send_buffer_global, ind);
	mempools_free_packet_buffer(send_buffer_global);
}

void commands_plot_add_graph(char *name) {
	int ind = 0;
	uint8_t *send_buffer_global = mempools_get_packet_buffer();
	send_buffer_global[ind++] = COMM_PLOT_ADD_GRAPH;
	memcpy(send_buffer_global + ind, name, strlen(name));
	ind += strlen(name);
	send_buffer_global[ind++] = '\0';
	commands_send_packet(send_buffer_global, ind);
	mempools_free_packet_buffer(send_buffer_global);
}

void commands_plot_set_graph(int graph) {
	int ind = 0;
	uint8_t buffer[2];
	buffer[ind++] = COMM_PLOT_SET_GRAPH;
	buffer[ind++] = graph;
	commands_send_packet(buffer, ind);
}

void commands_send_plot_points(float x, float y) {
	int32_t ind = 0;
	uint8_t buffer[10];
	buffer[ind++] = COMM_PLOT_DATA;
	buffer_append_float32_auto(buffer, x, &ind);
	buffer_append_float32_auto(buffer, y, &ind);
	commands_send_packet(buffer, ind);
}

void commands_send_app_data(unsigned char *data, unsigned int len) {
	int32_t index = 0;
	uint8_t *send_buffer_global = mempools_get_packet_buffer();
	send_buffer_global[index++] = COMM_CUSTOM_APP_DATA;
	memcpy(send_buffer_global + index, data, len);
	index += len;
	commands_send_packet(send_buffer_global, index);
	mempools_free_packet_buffer(send_buffer_global);
}

void commands_get_vesc_values(int uart_num) {
	int ind = 0;
	uint8_t buffer[2];
	buffer[ind++] = COMM_GET_VALUES;
	buffer[ind++] = 0;
	//commands_send_packet(buffer, ind);
	comm_uart_send_packet(buffer, ind, uart_num);
}

void commands_get_bms_values(int uart_num) {
	int ind = 0;
	uint8_t buffer[2];
	buffer[ind++] = COMM_BMS_GET_VALUES;
	buffer[ind++] = 0;
	//commands_send_packet(buffer, ind, uart_num);
	comm_uart_send_packet(buffer, ind, uart_num);
}
