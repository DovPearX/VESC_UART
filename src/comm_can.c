/*
	Copyright 2022 - 2024 Benjamin Vedder	benjamin@vedder.se

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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "utils/datatypes.h"
#include "utils/buffer.h"
#include "driver/twai.h"
#include "comm_can.h"
#include "utils/crc.h"
#include "utils/packet.h"
#include "commands.h"
#include "bms.h"
#include "utils/utils.h"
#include "soc/gpio_sig_map.h"
#include "hw.h"

#include <string.h>

// Status messages
static can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_2 stat_msgs_2[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_3 stat_msgs_3[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_4 stat_msgs_4[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_5 stat_msgs_5[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_6 stat_msgs_6[CAN_STATUS_MSGS_TO_STORE];
static io_board_adc_values io_board_adc_1_4[CAN_STATUS_MSGS_TO_STORE];
static io_board_adc_values io_board_adc_5_8[CAN_STATUS_MSGS_TO_STORE];
static io_board_digial_inputs io_board_digital_in[CAN_STATUS_MSGS_TO_STORE];
static psw_status psw_stat[CAN_STATUS_MSGS_TO_STORE];

#define RX_BUFFER_NUM				2
#define RX_BUFFER_SIZE				PACKET_MAX_PL_LEN
#define RXBUF_LEN					50

static twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(0, 0, TWAI_MODE_NORMAL);

static volatile bool init_done = false;
static volatile bool sem_init_done = false;
static volatile bool stop_threads = false;
static volatile bool stop_rx = false;
static volatile bool status_running = false;
static volatile bool rx_running = false;

static SemaphoreHandle_t ping_sem;
static SemaphoreHandle_t proc_sem;
static SemaphoreHandle_t status_sem;
static SemaphoreHandle_t send_mutex;
static volatile HW_TYPE ping_hw_last = HW_TYPE_VESC;
static uint8_t rx_buffer[RX_BUFFER_NUM][RX_BUFFER_SIZE];
static int rx_buffer_offset[RX_BUFFER_NUM];
static volatile unsigned int rx_buffer_last_id;
static volatile unsigned int rx_buffer_response_type = 1;

static twai_message_t rx_buf[RXBUF_LEN];
static volatile int rx_write = 0;
static volatile int rx_read = 0;
static volatile bool use_vesc_decoder = true;

static volatile int rx_recovery_cnt = 0;

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	comm_can_send_buffer(rx_buffer_last_id, data, len, rx_buffer_response_type);
}

static void decode_msg(uint32_t eid, uint8_t *data8, int len, bool is_replaced) {
	int32_t ind = 0;
	uint8_t crc_low;
	uint8_t crc_high;
	uint8_t commands_send;

	uint8_t id = eid & 0xFF;
	CAN_PACKET_ID cmd = eid >> 8;

	if (id == 255 || id == CONTROLLER_ID) {
		switch (cmd) {
		case CAN_PACKET_FILL_RX_BUFFER: {
			int buf_ind = 0;
			int offset = data8[0];
			data8++;
			len--;

			for (int i = 0; i < RX_BUFFER_NUM;i++) {
				if ((rx_buffer_offset[i]) == offset ) {
					buf_ind = i;
					break;
				}
			}

			memcpy(rx_buffer[buf_ind] + offset, data8, len);
			rx_buffer_offset[buf_ind] += len;
		} break;

		case CAN_PACKET_FILL_RX_BUFFER_LONG: {
			int buf_ind = 0;
			int offset = (int)data8[0] << 8;
			offset |= data8[1];
			data8 += 2;
			len -= 2;

			for (int i = 0; i < RX_BUFFER_NUM;i++) {
				if ((rx_buffer_offset[i]) == offset ) {
					buf_ind = i;
					break;
				}
			}

			if ((offset + len) <= RX_BUFFER_SIZE) {
				memcpy(rx_buffer[buf_ind] + offset, data8, len);
				rx_buffer_offset[buf_ind] += len;
			}
		} break;

		case CAN_PACKET_PROCESS_RX_BUFFER: {
			ind = 0;
			unsigned int last_id = data8[ind++];
			commands_send = data8[ind++];

			if (commands_send == 0 || commands_send == 3) {
				rx_buffer_last_id = last_id;
			}

			if (commands_send == 3) {
				rx_buffer_response_type = 0;
			} else {
				rx_buffer_response_type = 1;
			}

			int rxbuf_len = (int)data8[ind++] << 8;
			rxbuf_len |= (int)data8[ind++];

			if (rxbuf_len > RX_BUFFER_SIZE) {
				break;
			}

			int buf_ind = -1;
			for (int i = 0; i < RX_BUFFER_NUM;i++) {
				if ((rx_buffer_offset[i]) == rxbuf_len ) {
					buf_ind = i;
					break;
				}
			}

			// Something is wrong, reset all buffers
			if (buf_ind < 0) {
				for (int i = 0; i < RX_BUFFER_NUM;i++) {
					rx_buffer_offset[i] = 0;
				}
				break;
			}

			rx_buffer_offset[buf_ind] = 0;

			crc_high = data8[ind++];
			crc_low = data8[ind++];

			if (crc16(rx_buffer[buf_ind], rxbuf_len)
					== ((unsigned short) crc_high << 8
							| (unsigned short) crc_low)) {

				if (is_replaced) {
					if (rx_buffer[buf_ind][0] == COMM_JUMP_TO_BOOTLOADER ||
							rx_buffer[buf_ind][0] == COMM_ERASE_NEW_APP ||
							rx_buffer[buf_ind][0] == COMM_WRITE_NEW_APP_DATA ||
							rx_buffer[buf_ind][0] == COMM_WRITE_NEW_APP_DATA_LZO ||
							rx_buffer[buf_ind][0] == COMM_ERASE_BOOTLOADER) {
						break;
					}
				}

				switch (commands_send) {
				case 0:
				case 3:
					commands_process_packet(rx_buffer[buf_ind], rxbuf_len, send_packet_wrapper);
					break;
				case 1:
					commands_send_packet_can_last(rx_buffer[buf_ind], rxbuf_len);
					break;
				case 2:
					commands_process_packet(rx_buffer[buf_ind], rxbuf_len, 0);
					break;
				default:
					break;
				}
			}
		} break;

		case CAN_PACKET_PROCESS_SHORT_BUFFER:
			ind = 0;
			unsigned int last_id = data8[ind++];
			commands_send = data8[ind++];

			if (commands_send == 0 || commands_send == 3) {
				rx_buffer_last_id = last_id;
			}

			if (commands_send == 3) {
				rx_buffer_response_type = 0;
			} else {
				rx_buffer_response_type = 1;
			}

			if (is_replaced) {
				if (data8[ind] == COMM_JUMP_TO_BOOTLOADER ||
						data8[ind] == COMM_ERASE_NEW_APP ||
						data8[ind] == COMM_WRITE_NEW_APP_DATA ||
						data8[ind] == COMM_WRITE_NEW_APP_DATA_LZO ||
						data8[ind] == COMM_ERASE_BOOTLOADER) {
					break;
				}
			}

			switch (commands_send) {
			case 0:
			case 3:
				commands_process_packet(data8 + ind, len - ind, send_packet_wrapper);
				break;
			case 1:
				commands_send_packet_can_last(data8 + ind, len - ind);
				break;
			case 2:
				commands_process_packet(data8 + ind, len - ind, 0);
				break;
			default:
				break;
			}
			break;

			case CAN_PACKET_PING: {
				uint8_t buffer[2];
				buffer[0] = CONTROLLER_ID;
				buffer[1] = HW_TYPE_CUSTOM_MODULE;
				comm_can_transmit_eid(data8[0] | ((uint32_t)CAN_PACKET_PONG << 8), buffer, 2);
			} break;

			case CAN_PACKET_PONG:
				// data8[0]; // Sender ID
				xSemaphoreGive(ping_sem);
				if (len >= 2) {
					ping_hw_last = data8[1];
				} else {
					ping_hw_last = HW_TYPE_VESC_BMS;
				}
				break;

			default:
				break;
		}
	}

	// The packets below are addressed to all devices, mainly containing status information.

	switch (cmd) {
	case CAN_PACKET_STATUS:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *stat_tmp = &stat_msgs[i];
			if (stat_tmp->id == id || stat_tmp->id == -1) {
				ind = 0;
				stat_tmp->id = id;
				stat_tmp->rx_time = xTaskGetTickCount();
				stat_tmp->rpm = (float)buffer_get_int32(data8, &ind);
				stat_tmp->current = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp->duty = (float)buffer_get_int16(data8, &ind) / 1000.0;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_2:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_2 *stat_tmp_2 = &stat_msgs_2[i];
			if (stat_tmp_2->id == id || stat_tmp_2->id == -1) {
				ind = 0;
				stat_tmp_2->id = id;
				stat_tmp_2->rx_time = xTaskGetTickCount();
				stat_tmp_2->amp_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
				stat_tmp_2->amp_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_3:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_3 *stat_tmp_3 = &stat_msgs_3[i];
			if (stat_tmp_3->id == id || stat_tmp_3->id == -1) {
				ind = 0;
				stat_tmp_3->id = id;
				stat_tmp_3->rx_time = xTaskGetTickCount();
				stat_tmp_3->watt_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
				stat_tmp_3->watt_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_4:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_4 *stat_tmp_4 = &stat_msgs_4[i];
			if (stat_tmp_4->id == id || stat_tmp_4->id == -1) {
				ind = 0;
				stat_tmp_4->id = id;
				stat_tmp_4->rx_time = xTaskGetTickCount();
				stat_tmp_4->temp_fet = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->temp_motor = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->current_in = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->pid_pos_now = (float)buffer_get_int16(data8, &ind) / 50.0;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_5:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_5 *stat_tmp_5 = &stat_msgs_5[i];
			if (stat_tmp_5->id == id || stat_tmp_5->id == -1) {
				ind = 0;
				stat_tmp_5->id = id;
				stat_tmp_5->rx_time = xTaskGetTickCount();
				stat_tmp_5->tacho_value = buffer_get_int32(data8, &ind);
				stat_tmp_5->v_in = (float)buffer_get_int16(data8, &ind) / 1e1;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_6:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_6 *stat_tmp_6 = &stat_msgs_6[i];
			if (stat_tmp_6->id == id || stat_tmp_6->id == -1) {
				ind = 0;
				stat_tmp_6->id = id;
				stat_tmp_6->rx_time = xTaskGetTickCount();
				stat_tmp_6->adc_1 = buffer_get_float16(data8, 1e3, &ind);
				stat_tmp_6->adc_2 = buffer_get_float16(data8, 1e3, &ind);
				stat_tmp_6->adc_3 = buffer_get_float16(data8, 1e3, &ind);
				stat_tmp_6->ppm = buffer_get_float16(data8, 1e3, &ind);
				break;
			}
		}
		break;

	case CAN_PACKET_IO_BOARD_ADC_1_TO_4:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			io_board_adc_values *msg = &io_board_adc_1_4[i];
			if (msg->id == id || msg->id == -1) {
				ind = 0;
				msg->id = id;
				msg->rx_time = xTaskGetTickCount();
				ind = 0;
				int j = 0;
				while (ind < len) {
					msg->adc_voltages[j++] = buffer_get_float16(data8, 1e2, &ind);
				}
				break;
			}
		}
		break;

	case CAN_PACKET_IO_BOARD_ADC_5_TO_8:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			io_board_adc_values *msg = &io_board_adc_5_8[i];
			if (msg->id == id || msg->id == -1) {
				ind = 0;
				msg->id = id;
				msg->rx_time = xTaskGetTickCount();
				ind = 0;
				int j = 0;
				while (ind < len) {
					msg->adc_voltages[j++] = buffer_get_float16(data8, 1e2, &ind);
				}
				break;
			}
		}
		break;

	case CAN_PACKET_IO_BOARD_DIGITAL_IN:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			io_board_digial_inputs *msg = &io_board_digital_in[i];
			if (msg->id == id || msg->id == -1) {
				ind = 0;
				msg->id = id;
				msg->rx_time = xTaskGetTickCount();
				msg->inputs = 0;
				ind = 0;
				while (ind < len) {
					msg->inputs |= (uint64_t)data8[ind] << (ind * 8);
					ind++;
				}
				break;
			}
		}
		break;

	case CAN_PACKET_PSW_STAT: {
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			psw_status *msg = &psw_stat[i];
			if (msg->id == id || msg->id == -1) {
				ind = 0;
				msg->id = id;
				msg->rx_time = xTaskGetTickCount();

				msg->v_in = buffer_get_float16(data8, 10.0, &ind);
				msg->v_out = buffer_get_float16(data8, 10.0, &ind);
				msg->temp = buffer_get_float16(data8, 10.0, &ind);
				msg->is_out_on = (data8[ind] >> 0) & 1;
				msg->is_pch_on = (data8[ind] >> 1) & 1;
				msg->is_dsc_on = (data8[ind] >> 2) & 1;
				ind++;
				break;
			}
		}
	} break;

	default:
		break;
	}
}



static void rx_task(void *arg) {
	twai_message_t rx_message;

	twai_reconfigure_alerts(TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF, NULL);

	while (!stop_threads && !stop_rx) {
		esp_err_t res = twai_receive(&rx_message, 2);

		if (res == ESP_OK) {
			rx_buf[rx_write] = rx_message;
			rx_write++;
			if (rx_write >= RXBUF_LEN) {
				rx_write = 0;
			}

			xSemaphoreGive(proc_sem);
		}

		uint32_t alerts;
		res = twai_read_alerts(&alerts, 0);
		if (res == ESP_OK) {
			if ((alerts & TWAI_ALERT_BUS_OFF) || (alerts & TWAI_ALERT_ERR_PASS)) {
				twai_initiate_recovery(); // Needs 128 occurrences of bus free signal
				rx_recovery_cnt++;
			}
		}
	}

	rx_running = false;
	vTaskDelete(NULL);
}

static void process_task(void *arg) {
	for (;;) {
		xSemaphoreTake(proc_sem, 10 / portTICK_PERIOD_MS);

		while (rx_read != rx_write) {
			twai_message_t *msg = &rx_buf[rx_read];
			rx_read++;
			if (rx_read >= RXBUF_LEN) {
				rx_read = 0;
			}

			if (use_vesc_decoder) {
				if (!bms_process_can_frame(msg->identifier, msg->data, msg->data_length_code, msg->extd)) {
					if (msg->extd) {
						decode_msg(msg->identifier, msg->data, msg->data_length_code, false);
					}
				}
			}
		}
	}

	vTaskDelete(NULL);
}

static void status_task(void *arg) {

	while (!stop_threads) {
		int rate = STATUS_RATE;

		if (rate < 1) {
			xSemaphoreTake(status_sem, 10);
			continue;
		}

		xSemaphoreTake(status_sem, configTICK_RATE_HZ / rate);
	}

	status_running = false;
	vTaskDelete(NULL);
}

static void start_rx_thd(void) {
	if (rx_running) {
		return;
	}

	stop_rx = false;
	rx_running = true;
	xTaskCreatePinnedToCore(rx_task, "can_rx", 1024, NULL, configMAX_PRIORITIES - 1, NULL, tskNO_AFFINITY);
}

static void stop_rx_thd(void) {
	stop_rx = true;
	while (rx_running) {
		vTaskDelay(1);
	}
}

void comm_can_start(int pin_tx, int pin_rx) {
	if (init_done) {
		return;
	}

	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		stat_msgs[i].id = -1;
		stat_msgs_2[i].id = -1;
		stat_msgs_3[i].id = -1;
		stat_msgs_4[i].id = -1;
		stat_msgs_5[i].id = -1;
		stat_msgs_6[i].id = -1;

		io_board_adc_1_4[i].id = -1;
		io_board_adc_5_8[i].id = -1;
		io_board_digital_in[i].id = -1;

		psw_stat[i].id = -1;
	}

	if (!sem_init_done) {
		ping_sem = xSemaphoreCreateBinary();
		proc_sem = xSemaphoreCreateBinary();
		status_sem = xSemaphoreCreateBinary();
		send_mutex = xSemaphoreCreateMutex();

		// The process-task is left running after the first init in case comm_can_stop
		// is called from it.
		xTaskCreatePinnedToCore(process_task, "can_proc", 3072, NULL, 8, NULL, tskNO_AFFINITY);

		sem_init_done = true;
	}

	g_config.rx_queue_len = 20;
	g_config.tx_io = pin_tx;
	g_config.rx_io = pin_rx;

	twai_driver_install(&g_config, &t_config, &f_config);
	twai_start();

	stop_threads = false;
	status_running = true;

	xTaskCreatePinnedToCore(status_task, "can_status", 1024, NULL, 7, NULL, tskNO_AFFINITY);
	start_rx_thd();

	init_done = true;
}

void comm_can_stop(void) {
	if (!init_done) {
		return;
	}

	xSemaphoreTake(send_mutex, portMAX_DELAY);
	init_done = false;
	xSemaphoreGive(send_mutex);

	stop_threads = true;
	xSemaphoreGive(status_sem);
	while (status_running || rx_running) {
		vTaskDelay(2);
	}

	twai_stop();
	twai_driver_uninstall();
}

int comm_can_get_rx_recovery_cnt(void) {
	return rx_recovery_cnt;
}

void comm_can_use_vesc_decoder(bool use_vesc_dec) {
	use_vesc_decoder = use_vesc_dec;
}

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (!init_done) {
		return;
	}

	if (len > 8) {
		len = 8;
	}

	twai_message_t tx_msg = {0};
	tx_msg.extd = 1;
	tx_msg.identifier = id;

	memcpy(tx_msg.data, data, len);
	tx_msg.data_length_code = len;

	xSemaphoreTake(send_mutex, portMAX_DELAY);

	if (!init_done) {
		xSemaphoreGive(send_mutex);
		return;
	}

	if (twai_transmit(&tx_msg, 5 / portTICK_PERIOD_MS) != ESP_OK) {
		stop_rx_thd();
		twai_stop();
		twai_initiate_recovery();
		twai_start();
		start_rx_thd();
	}
	xSemaphoreGive(send_mutex);
}

void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (!init_done) {
		return;
	}

	if (len > 8) {
		len = 8;
	}

	twai_message_t tx_msg = {0};
	tx_msg.extd = 0;
	tx_msg.identifier = id;

	memcpy(tx_msg.data, data, len);
	tx_msg.data_length_code = len;

	xSemaphoreTake(send_mutex, portMAX_DELAY);

	if (!init_done) {
		xSemaphoreGive(send_mutex);
		return;
	}

	if (twai_transmit(&tx_msg, 5 / portTICK_PERIOD_MS) != ESP_OK) {
		stop_rx_thd();
		twai_stop();
		twai_initiate_recovery();
		twai_start();
		start_rx_thd();
	}
	xSemaphoreGive(send_mutex);
}

/**
 * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
 * it will be sent in a single CAN frame, otherwise it will be split into
 * several frames.
 *
 * @param controller_id
 * The controller id to send to.
 *
 * @param data
 * The payload.
 *
 * @param len
 * The payload length.
 *
 * @param send
 * 0: Packet goes to commands_process_packet of receiver
 * 1: Packet goes to commands_send_packet of receiver
 * 2: Packet goes to commands_process and send function is set to null
 *    so that no reply is sent back.
 * 3: Same as 0, but the reply is processed locally and not sent out on the last interface.
 */
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = CONTROLLER_ID;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		comm_can_transmit_eid(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = CONTROLLER_ID;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

		comm_can_transmit_eid(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
	}
}

/**
 * Check if a VESC on the CAN-bus responds.
 *
 * @param controller_id
 * The ID of the VESC.
 *
 * @param hw_type
 * The hardware type of the CAN device.
 *
 * @return
 * True for success, false otherwise.
 */
bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type) {
	if (!init_done) {
		return false;
	}

	uint8_t buffer[1];
	buffer[0] = CONTROLLER_ID;
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_PING << 8), buffer, 1);

	bool ret = xSemaphoreTake(ping_sem, 10 / portTICK_PERIOD_MS) == pdTRUE;

	if (ret) {
		if (hw_type) {
			*hw_type = ping_hw_last;
		}
	}

	return ret;
}

void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay) {
	int32_t send_index = 0;
	uint8_t buffer[6];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	buffer_append_float16(buffer, off_delay, 1e3, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_current_brake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

void comm_can_set_current_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
}

void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay) {
	int32_t send_index = 0;
	uint8_t buffer[6];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	buffer_append_float16(buffer, off_delay, 1e3, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
}

void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
}

void comm_can_set_handbrake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current, 1e3, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
}

void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index);
}

can_status_msg *comm_can_get_status_msg_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs[index];
	} else {
		return 0;
	}
}

can_status_msg *comm_can_get_status_msg_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs[i].id == id) {
			return &stat_msgs[i];
		}
	}

	return 0;
}

can_status_msg_2 *comm_can_get_status_msg_2_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_2[index];
	} else {
		return 0;
	}
}

can_status_msg_2 *comm_can_get_status_msg_2_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_2[i].id == id) {
			return &stat_msgs_2[i];
		}
	}

	return 0;
}

can_status_msg_3 *comm_can_get_status_msg_3_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_3[index];
	} else {
		return 0;
	}
}

can_status_msg_3 *comm_can_get_status_msg_3_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_3[i].id == id) {
			return &stat_msgs_3[i];
		}
	}

	return 0;
}

can_status_msg_4 *comm_can_get_status_msg_4_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_4[index];
	} else {
		return 0;
	}
}

can_status_msg_4 *comm_can_get_status_msg_4_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_4[i].id == id) {
			return &stat_msgs_4[i];
		}
	}

	return 0;
}

can_status_msg_5 *comm_can_get_status_msg_5_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_5[index];
	} else {
		return 0;
	}
}

can_status_msg_5 *comm_can_get_status_msg_5_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_5[i].id == id) {
			return &stat_msgs_5[i];
		}
	}

	return 0;
}

can_status_msg_6 *comm_can_get_status_msg_6_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_6[index];
	} else {
		return 0;
	}
}

can_status_msg_6 *comm_can_get_status_msg_6_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_6[i].id == id) {
			return &stat_msgs_6[i];
		}
	}

	return 0;
}

io_board_adc_values *comm_can_get_io_board_adc_1_4_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE && io_board_adc_1_4[index].id >= 0) {
		return &io_board_adc_1_4[index];
	} else {
		return 0;
	}
}

io_board_adc_values *comm_can_get_io_board_adc_1_4_id(int id) {
	if (id == 255 && io_board_adc_1_4[0].id >= 0) {
		return &io_board_adc_1_4[0];
	}

	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (io_board_adc_1_4[i].id == id) {
			return &io_board_adc_1_4[i];
		}
	}

	return 0;
}

io_board_adc_values *comm_can_get_io_board_adc_5_8_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE && io_board_adc_5_8[index].id >= 0) {
		return &io_board_adc_5_8[index];
	} else {
		return 0;
	}
}

io_board_adc_values *comm_can_get_io_board_adc_5_8_id(int id) {
	if (id == 255 && io_board_adc_5_8[0].id >= 0) {
		return &io_board_adc_5_8[0];
	}

	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (io_board_adc_5_8[i].id == id) {
			return &io_board_adc_5_8[i];
		}
	}

	return 0;
}

io_board_digial_inputs *comm_can_get_io_board_digital_in_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &io_board_digital_in[index];
	} else {
		return 0;
	}
}

io_board_digial_inputs *comm_can_get_io_board_digital_in_id(int id) {
	if (id == 255 && io_board_digital_in[0].id >= 0) {
		return &io_board_digital_in[0];
	}

	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (io_board_digital_in[i].id == id) {
			return &io_board_digital_in[i];
		}
	}

	return 0;
}

void comm_can_io_board_set_output_digital(int id, int channel, bool on) {
	int32_t send_index = 0;
	uint8_t buffer[8];

	buffer[send_index++] = channel;
	buffer[send_index++] = 1;
	buffer[send_index++] = on ? 1 : 0;

	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL << 8), buffer, send_index);
}

void comm_can_io_board_set_output_pwm(int id, int channel, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[8];

	buffer[send_index++] = channel;
	buffer_append_float16(buffer, duty, 1e3, &send_index);

	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM << 8), buffer, send_index);
}

psw_status *comm_can_get_psw_status_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &psw_stat[index];
	} else {
		return 0;
	}
}

psw_status *comm_can_get_psw_status_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (psw_stat[i].id == id) {
			return &psw_stat[i];
		}
	}
	return 0;
}

void comm_can_psw_switch(int id, bool is_on, bool plot) {
	int32_t send_index = 0;
	uint8_t buffer[8];

	buffer[send_index++] = is_on ? 1 : 0;
	buffer[send_index++] = plot ? 1 : 0;

	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_PSW_SWITCH << 8), buffer, send_index);
}

void comm_can_update_pid_pos_offset(int id, float angle_now, bool store) {
	int32_t send_index = 0;
	uint8_t buffer[8];

	buffer_append_float32(buffer, angle_now, 1e4, &send_index);
	buffer[send_index++] = store;

	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_UPDATE_PID_POS_OFFSET << 8), buffer, send_index);
}

void comm_can_conf_current_limits(uint8_t controller_id, float min, float max) {
	int32_t send_index = 0;
	uint8_t buffer[8];

	buffer_append_float32(buffer, min, 1e3, &send_index);
	buffer_append_float32(buffer, max, 1e3, &send_index);

    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_CONF_STORE_CURRENT_LIMITS << 8), buffer, send_index);
}

void comm_can_conf_current_limits_in(uint8_t controller_id, float min, float max) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float32(buffer, min, 1e3, &send_index);
	buffer_append_float32(buffer, max, 1e3, &send_index);

    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN << 8), buffer, send_index);
}

void comm_can_get_mcconf_temp(uint8_t controller_id) {
    int32_t send_index = 0;
	uint8_t buffer[4];

    buffer[send_index++] = COMM_GET_MCCONF_TEMP;

    comm_can_send_buffer(controller_id, buffer, send_index, 3);
}

void comm_can_set_mcconf_temp(uint8_t controller_id, int store, int forward, int reverse, int divide) {

    int32_t ind = 0;
	uint8_t send_buffer[60];

    send_buffer[ind++] = COMM_SET_MCCONF_TEMP;
    send_buffer[ind++] = store; // 0 temporary - 1 store
    send_buffer[ind++] = forward; // forward can
    send_buffer[ind++] = reverse; // reverse can
    send_buffer[ind++] = divide; // divide by controllers

    buffer_append_float32_auto(send_buffer, mcconf.l_current_min_scale, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.l_current_max_scale, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.l_min_erpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.l_max_erpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.l_min_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.l_max_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.l_watt_min, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.l_watt_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.l_in_current_min, &ind);
	buffer_append_float32_auto(send_buffer, mcconf.l_in_current_max, &ind);

    comm_can_send_buffer(controller_id, send_buffer, ind, 0);
}

