// TCA6416A module
// Copyright 2019 Håkon Nessjøen
// https://github.com/haakonnessjoen/TCA6416A/

#define TCA_PINMODE_INPUT 0
#define TCA_PINMODE_OUTPUT 1
#define TCA_OUTPUT_HIGH 1
#define TCA_OUTPUT_LOW 0

// Read operation to the lower level driver is 0.
#define I2C_SLAVE_OPERATION_READ                    (0)

// Write operation to the lower level driver is 1.
#define I2C_SLAVE_OPERATION_WRITE                   (1)


#define TCAREG_INPUT0 0x00
#define TCAREG_INPUT1 0x01
#define TCAREG_OUTPUT0 0x02
#define TCAREG_OUTPUT1 0x03
#define TCAREG_POLARITY0 0x04
#define TCAREG_POLARITY1 0x05
#define TCAREG_CONFIG0 0x06
#define TCAREG_CONFIG1 0x07


UINT8 tca_i2caddr;
UINT8 tca_i2cwidth;
UINT16 tca_pinState;
UINT16 tca_pinModes;
void tca_pin_mode(UINT8 pinNum, int mode);
void tca_pin_write(UINT8 pinNum, UINT8 level);
INT16 tca_pin_read(UINT8 pinNum);
void tca_port_write(UINT16 i2cportval);
UINT16 tca_port_read();
void tca_mode_write(UINT16 i2cportval);
UINT16 tca_mode_read();

