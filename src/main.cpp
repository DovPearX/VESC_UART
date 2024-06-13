#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "vesc_uart.h"



void setup() {

    Serial.begin(9600);

    vesc_uart_init(17, 16, 2, 115200);

    commands_get_mcconf_temp();

}

void loop() {

    vTaskDelay(500);

    commands_get_vesc_values();

    Serial.println(mcconf.l_current_min_scale);
    Serial.println(mcconf.l_current_max_scale)
    Serial.println(values.v_in);    

}

