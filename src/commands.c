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
#include "utils/datatypes.h"
#include "utils/mempools.h"
#include "utils/utils.h"
#include "comm_uart.h"
#include "comm_can.h"
#include "config/confgenerator.h"
#include "bms.h"
#include "hw.h"

#include "utils/packet.h"
#include "utils/buffer.h"
#include "utils/crc.h"

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
				HW_TYPE hw_type;
				if (comm_can_ping(i, &hw_type)) {
					send_buffer[ind++] = i;
				}
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
	case COMM_FW_VERSION: {
		int32_t ind = 0;
		uint8_t send_buffer[65];
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;

		strcpy((char*)(send_buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;

	    ind += 6;
		memset(send_buffer + ind, 0, 6);
		ind += 6;

		send_buffer[ind++] = 0;
		send_buffer[ind++] = 0;

		send_buffer[ind++] = HW_TYPE_CUSTOM_MODULE;
		send_buffer[ind++] = 0; // One custom config

		send_buffer[ind++] = 0; // No phase filters
		send_buffer[ind++] = 0; // No HW QML

	    send_buffer[ind++] = 0;

		send_buffer[ind++] = 0; // No NRF flags

		strcpy((char*)(send_buffer + ind), FW_NAME);
		ind += strlen(FW_NAME) + 1;

		send_buffer[ind++] = 0;

		reply_func(send_buffer, ind);
	} break;

	case COMM_FORWARD_CAN:
		send_func_can_fwd = reply_func;
		comm_can_send_buffer(data[0], data + 1, len - 1, 0);
		break;

	case COMM_CAN_FWD_FRAME: {
		int32_t ind = 0;
		uint32_t id = buffer_get_uint32(data, &ind);
		bool is_ext = data[ind++];

		if (is_ext) {
			comm_can_transmit_eid(id, data + ind, len - ind);
		} else {
			comm_can_transmit_sid(id, data + ind, len - ind);
		}
		} break;

	case COMM_IO_BOARD_GET_ALL: {
		int32_t ind = 0;
		int id = buffer_get_int16(data, &ind);

		io_board_adc_values *adc_1_4 = comm_can_get_io_board_adc_1_4_id(id);
		io_board_adc_values *adc_5_8 = comm_can_get_io_board_adc_5_8_id(id);
		io_board_digial_inputs *digital_in = comm_can_get_io_board_digital_in_id(id);

		if (!adc_1_4 && !adc_5_8 && !digital_in) {
			break;
		}

		uint8_t send_buffer[70];
		ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int16(send_buffer, id, &ind);

		if (adc_1_4) {
			send_buffer[ind++] = 1;
			buffer_append_float32_auto(send_buffer, UTILS_AGE_S(adc_1_4->rx_time), &ind);
			buffer_append_float16(send_buffer, adc_1_4->adc_voltages[0], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_1_4->adc_voltages[1], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_1_4->adc_voltages[2], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_1_4->adc_voltages[3], 1e2, &ind);
		}

		if (adc_5_8) {
			send_buffer[ind++] = 2;
			buffer_append_float32_auto(send_buffer, UTILS_AGE_S(adc_1_4->rx_time), &ind);
			buffer_append_float16(send_buffer, adc_5_8->adc_voltages[0], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_5_8->adc_voltages[1], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_5_8->adc_voltages[2], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_5_8->adc_voltages[3], 1e2, &ind);
		}

		if (digital_in) {
			send_buffer[ind++] = 3;
			buffer_append_float32_auto(send_buffer, UTILS_AGE_S(adc_1_4->rx_time), &ind);
			buffer_append_uint32(send_buffer, (digital_in->inputs >> 32) & 0xFFFFFFFF, &ind);
			buffer_append_uint32(send_buffer, (digital_in->inputs >> 0) & 0xFFFFFFFF, &ind);
		}

		reply_func(send_buffer, ind);
	} break;

	case COMM_IO_BOARD_SET_PWM: {
		int32_t ind = 0;
		int id = buffer_get_int16(data, &ind);
		int channel = buffer_get_int16(data, &ind);
		float duty = buffer_get_float32_auto(data, &ind);
		comm_can_io_board_set_output_pwm(id, channel, duty);
	} break;

	case COMM_IO_BOARD_SET_DIGITAL: {
		int32_t ind = 0;
		int id = buffer_get_int16(data, &ind);
		int channel = buffer_get_int16(data, &ind);
		bool on = data[ind++];
		comm_can_io_board_set_output_digital(id, channel, on);
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

	case COMM_BMS_GET_VALUES:
	case COMM_BMS_SET_CHARGE_ALLOWED:
	case COMM_BMS_SET_BALANCE_OVERRIDE:
	case COMM_BMS_RESET_COUNTERS:
	case COMM_BMS_FORCE_BALANCE:
	case COMM_BMS_ZERO_CURRENT_OFFSET: {
		bms_process_cmd(data - 1, len + 1, reply_func);
		break;
	}

	case COMM_GET_MCCONF_TEMP: {
			int32_t ind = 0;

			mcconf.l_current_min_scale  = buffer_get_float32_auto(data, &ind); 	// mcconf->l_current_min_scale
			mcconf.l_current_max_scale 	= buffer_get_float32_auto(data, &ind); 	// mcconf->l_current_max_scale
			mcconf.l_min_erpm 		    = buffer_get_float32_auto(data, &ind); 	// mcconf->l_min_erpm
			mcconf.l_max_erpm 	    	= buffer_get_float32_auto(data, &ind); 	// mcconf->l_max_erpm
			mcconf.l_min_duty 	    	= buffer_get_float32_auto(data, &ind); 	// mcconf->l_min_duty
			mcconf.l_max_duty 	    	= buffer_get_float32_auto(data, &ind); 	// mcconf->l_max_duty
			mcconf.l_watt_min 	    	= buffer_get_float32_auto(data, &ind); 	// mcconf->l_watt_min
			mcconf.l_watt_max 	    	= buffer_get_float32_auto(data, &ind); 	// mcconf->l_watt_max
			mcconf.l_in_current_min 	= buffer_get_float32_auto(data, &ind); 	// mcconf->l_in_current_min
			mcconf.l_in_current_max 	= buffer_get_float32_auto(data, &ind); 	// mcconf->l_in_current_max
			mcconf.si_motor_poles       = (uint8_t)data[ind++];                   // mcconf->si_motor_poles
			mcconf.si_gear_ratio 	    = buffer_get_float32_auto(data, &ind); 	// mcconf->si_gear_ratio
			mcconf.si_wheel_diameter 	= buffer_get_float32_auto(data, &ind); 	// mcconf->si_wheel_diameter

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

void commands_send_mcconf(COMM_PACKET_ID packet_id, mc_configuration* mcconf, void(*reply_func)(unsigned char* data, unsigned int len)) {
	uint8_t *send_buffer_global = mempools_get_packet_buffer();
	send_buffer_global[0] = packet_id;
	int32_t len = confgenerator_serialize_mcconf(send_buffer_global + 1, mcconf);
	if (reply_func) {
		reply_func(send_buffer_global, len + 1);
	} else {
		commands_send_packet(send_buffer_global, len + 1);
	}
	mempools_free_packet_buffer(send_buffer_global);
}

void commands_send_appconf(COMM_PACKET_ID packet_id, app_configuration *appconf, void(*reply_func)(unsigned char* data, unsigned int len)) {
	uint8_t *send_buffer_global = mempools_get_packet_buffer();
	send_buffer_global[0] = packet_id;
	int32_t len = confgenerator_serialize_appconf(send_buffer_global + 1, appconf);
	if (reply_func) {
		reply_func(send_buffer_global, len + 1);
	} else {
		commands_send_packet(send_buffer_global, len + 1);
	}
	mempools_free_packet_buffer(send_buffer_global);
}
