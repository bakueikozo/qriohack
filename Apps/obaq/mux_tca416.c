// TCA6416A module
// Copyright 2019 Håkon Nessjøen
// https://github.com/haakonnessjoen/TCA6416A/


#include "bleprofile.h"
#include "bleapp.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "bleappconfig.h"
#include "cfa.h"


#include "mux_tca416.h"

void tca_begin(UINT8 address){
    ble_trace0("tca_begin\n");
	tca_i2caddr = 0x20 | address;
	tca_i2caddr =tca_i2caddr<<1 ;
	tca_i2cwidth = 2;

	tca_port_read();
	tca_mode_read();
    ble_trace0("tca_end\n");
}
void tca_pin_mode(UINT8 pinNum, int mode){
	UINT16 mask = 1 << pinNum;
    ble_trace0("tca_pin_mode\n");

	if (mode == TCA_PINMODE_INPUT) {
		tca_pinModes |= mask;
	} else {
		tca_pinModes &= ~mask;
	}

	tca_mode_write(tca_pinModes);
}


void tca_pin_write(UINT8 pinNum, UINT8 level){
	UINT16 mask = 1 << pinNum;
    ble_trace0("tca_pin_write\n");

	if (level == TCA_OUTPUT_HIGH) {
		tca_pinState |= mask;
	} else {
		tca_pinState &= ~mask;
	}

	tca_port_write(tca_pinState);
}
INT16  tca_pin_read(UINT8 pinNum){
	UINT16 mask = 1 << pinNum;
    ble_trace0("tca_pin_read\n");

	tca_port_read();

	if ((tca_pinState & mask) == mask) {
		return 1;
	} else {
		return 0;
	}
}
void tca_port_write(UINT16 i2cportval){

    CFA_BSC_STATUS read_status;
    UINT8 return_value = 0;

    UINT8 reg_bytes_to_write[3];
    reg_bytes_to_write[0] = TCAREG_OUTPUT0;
    reg_bytes_to_write[1] = i2cportval & 0xFF;
    reg_bytes_to_write[2] = (i2cportval >> 8) & 0xFF;
    ble_trace0("tca_port_write\n");

    // Invoke the lower level driver. Non-combo transaction, so set offset parameters to NULL/0.
    read_status = cfa_bsc_OpExtended(reg_bytes_to_write, sizeof(reg_bytes_to_write), NULL, 0, tca_i2caddr,
                                                I2C_SLAVE_OPERATION_WRITE);

    switch(read_status)
    {
        case CFA_BSC_STATUS_INCOMPLETE:
            // Transaction did not go through. ERROR. Handle this case.
            break;
        case CFA_BSC_STATUS_SUCCESS:
		    ble_trace0("i2c_write success\n");
            return_value = 1;
            break;
        case CFA_BSC_STATUS_NO_ACK:
            // No slave device with this address exists on the I2C bus. ERROR. Handle this.
        default:
            break;
    }

	tca_pinState = i2cportval;
}
UINT16 tca_port_read(){
// (UINT8 slave_address,UINT8 register_address, INT16* data// 

    CFA_BSC_STATUS read_status;
    UINT8 return_value = 0;

    UINT8 reg_read_bytes_to_write[1];
    UINT8 reg_bytes_to_read[2];
	INT16 data;
    ble_trace0("tca_port_read\n");
    
    reg_read_bytes_to_write[0] = TCAREG_INPUT0;

    // Do a combo write then read operation
    read_status = cfa_bsc_OpExtended(reg_bytes_to_read, sizeof(reg_bytes_to_read), reg_read_bytes_to_write,
                                                        sizeof(reg_read_bytes_to_write), tca_i2caddr,
                                                        I2C_SLAVE_OPERATION_READ);
    switch(read_status)
    {
        case CFA_BSC_STATUS_INCOMPLETE:
            // Transaction did not go through. ERROR. Handle this case.
            break;
        case CFA_BSC_STATUS_SUCCESS:
            // The read was successful.
            data = (INT16)(reg_bytes_to_read[0] | reg_bytes_to_read[1] << 8);
			tca_pinState = (tca_pinState & ~tca_pinModes) | (data & tca_pinModes);

		    ble_trace0("i2c_read success\n");

            return tca_pinState;
        case CFA_BSC_STATUS_NO_ACK:
            // No slave device with this address exists on the I2C bus. ERROR. Handle this.
        default:
            break;
    }

    return -1;

}
void tca_mode_write(UINT16 modes){
    CFA_BSC_STATUS read_status;
    UINT8 return_value = 0;

    UINT8 reg_bytes_to_write[3];
    ble_trace0("tca_mode_write\n");

    reg_bytes_to_write[0] = TCAREG_CONFIG0;
    reg_bytes_to_write[1] = modes & 0xFF;
    reg_bytes_to_write[2] = (modes >> 8) & 0xFF;

    // Invoke the lower level driver. Non-combo transaction, so set offset parameters to NULL/0.
    read_status = cfa_bsc_OpExtended(reg_bytes_to_write, sizeof(reg_bytes_to_write), NULL, 0, tca_i2caddr,
                                                I2C_SLAVE_OPERATION_WRITE);

    switch(read_status)
    {
        case CFA_BSC_STATUS_INCOMPLETE:
            // Transaction did not go through. ERROR. Handle this case.
		    ble_trace0("i2c_write INCOMP\n");
            break;
        case CFA_BSC_STATUS_SUCCESS:
            // The read was successful.
		    ble_trace0("i2c_write success\n");
            return_value = 1;
            break;
        case CFA_BSC_STATUS_NO_ACK:
		    ble_trace0("i2c_write NO_ACK\n");
            // No slave device with this address exists on the I2C bus. ERROR. Handle this.
        default:
		    ble_trace0("default\n");
            break;
    }


	tca_pinModes = modes;
}
UINT16 tca_mode_read(){
    CFA_BSC_STATUS read_status;
    UINT8 return_value = 0;

    UINT8 reg_read_bytes_to_write[1];
    UINT8 reg_bytes_to_read[2];
    ble_trace0("tca_mode_read\n");

    reg_read_bytes_to_write[0] = TCAREG_CONFIG0;

    // Do a combo write then read operation
    read_status = cfa_bsc_OpExtended(reg_bytes_to_read, sizeof(reg_bytes_to_read), reg_read_bytes_to_write,
                                                        sizeof(reg_read_bytes_to_write), tca_i2caddr,
                                                        I2C_SLAVE_OPERATION_READ);
    switch(read_status)
    {
        case CFA_BSC_STATUS_INCOMPLETE:
            // Transaction did not go through. ERROR. Handle this case.
            break;
        case CFA_BSC_STATUS_SUCCESS:
            // The read was successful.
            {
            INT16 data = (INT16)(reg_bytes_to_read[0] | reg_bytes_to_read[1] << 8);
            return_value = 1;

			/*
			tca_pinModes = TW.read();
			tca_pinModes |= TW.read() << 8;*/

			tca_pinModes = data;
		    ble_trace0("i2c_read success\n");
			}		
            break;
        case CFA_BSC_STATUS_NO_ACK:
            // No slave device with this address exists on the I2C bus. ERROR. Handle this.
        default:
            break;
    }


	return tca_pinModes;
}

#define LED_ADDR 0xCC
UINT8 led_write_8_bit_register(UINT8 register_address, UINT8 data)
{
    CFA_BSC_STATUS read_status;
    UINT8 return_value = 0;
    UINT8 reg_data_bytes[2];

    reg_data_bytes[0]= register_address;
    reg_data_bytes[1] = data;

    read_status = cfa_bsc_OpExtended(reg_data_bytes, sizeof(reg_data_bytes), NULL, 0, LED_ADDR,
                                                I2C_SLAVE_OPERATION_WRITE);

    switch(read_status)
    {
        case CFA_BSC_STATUS_INCOMPLETE:
            // Transaction did not go through. ERROR. Handle this case.
            break;
        case CFA_BSC_STATUS_SUCCESS:
            // The read was successful.
            return_value = 1;
            break;
        case CFA_BSC_STATUS_NO_ACK:
            // No slave device with this address exists on the I2C bus. ERROR. Handle this.
        default:
            break;
    }

    return return_value;
}
