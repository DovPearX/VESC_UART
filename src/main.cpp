#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C"{
#include "commands.h"
#include "comm_uart.h"
#include "utils/mempools.h"
}

void setup() {

    commands_init();
    mempools_init();

    comm_uart_init(17,16,2,115200);
    comm_uart_setup_communication(2);
}

void loop() {

    vTaskDelay(500);
    commands_get_vesc_values();

    Serial.println(values.v_in);    

}