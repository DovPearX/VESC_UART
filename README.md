# VESC_UART

Example use:

#include "vesc_uart.h" //LIB

void setup() {

    vesc_uart_init(17, 16, 2, 115200);  //Set vesc uart port to 2;

    commands_get_mcconf_temp(); //Read motor config (temp)

}

----------------------------------------------------------------

commands_get_vesc_values(); //Read data into the values ​​struct
//Serial.printl(values.v_in);

----------------------------------------------------------------

commands_get_bms_values();  //Read data from BMS connected to VESC via CAN 
//Serial.printl(bms.v_tot);

----------------------------------------------------------------

mcconf.l_current_min_scale = 0.5;  //Set current min scale to 0.5

commands_set_mcconf_temp(0,0,0,0); //Send changed values






