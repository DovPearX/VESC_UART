# VESC_UART

Example use:

#include "vesc_uart.h"

void setup() {

    Serial.begin(9600);

    vesc_uart_init(17, 16, 2, 115200);  //Set vesc uart port to 2;

    vTaskDelay(1000);

    mcconf.l_current_min_scale = 0.5; //Set current min scale to 0.5
    mcconf.l_current_max_scale = 0.2; //Set current max scale to 0.2

    printf("SET l_current_min_scale: %.2f\n", mcconf.l_current_min_scale);
    printf("SET l_current_max_scale: %.2f\n", mcconf.l_current_max_scale);

    commands_set_mcconf_temp(0, 0, 0, 2); //Send changed values

}

void loop() {

    vTaskDelay(500);

    commands_get_vesc_values(2);  //Read data into the values ​​struct
    commands_get_bms_values();  //Read data from BMS connected to VESC via CAN 

    Serial.println(values.v_in);    

}






