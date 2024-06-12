#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "vesc_uart.h"



void setup() {

    Serial.begin(9600);

    vesc_uart_init(17, 16, 2, 115200);

}

void loop() {

    vTaskDelay(500);

    commands_get_vesc_values();

    Serial.println(values.v_in);    

}

