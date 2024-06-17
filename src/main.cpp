#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "vesc_uart.h"



void setup() {

    Serial.begin(9600);

    vesc_init();

    delay(100);

    mcconf.l_current_min_scale = 0.5;
    mcconf.l_current_max_scale = 0.2;

    printf("SET l_current_min_scale: %.2f\n", mcconf.l_current_min_scale);
    printf("SET l_current_max_scale: %.2f\n", mcconf.l_current_max_scale);

    comm_uart_set_mcconf_temp(0, 0, 0, 0, 2);

}

void loop() {

    vTaskDelay(500);

    comm_uart_get_vesc_values(2);

    //Serial.println(values.v_in);    

}

