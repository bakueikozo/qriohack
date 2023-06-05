
/*
 * Copyright 2014, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */



#define CONFIG_IN_NVRAM 1
#include "bleprofile.h"
#include "bleapp.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "bleappconfig.h"
#include "cfa.h"
#include "types.h"
#include "bleapp.h"
#include "puart.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "blecen.h"
#include "platform.h"
#include "bleapputils.h"
#include "bleappfwu.h"
#include "devicelpm.h"
#include "spar_utils.h"
#include "stdarg.h"
#include "stdlib.h"
#include "mux_tca416.h"
#include "miadriver.h"
#include "aclk.h"
#include "pwm.h"

#define DRV8830_SPEED_MAX 0x3F
#define DRV8830_SPEED_MIN 0x06
#define DRV8830_SLAVE_ADDRESS 0xc8
#define DRV8830_CONTROL_REG 0x00
#define DRV8830_FAULT_REG 0x01
#define DRV8830_STANBY 0b00
#define DRV8830_REVERSE 0b01
#define DRV8830_FORWARD 0b10
#define DRV8830_BRAKE 0b11
/// @}
UINT8 motor_write_8_bit_register(UINT8 register_address, UINT8 data)
{
    CFA_BSC_STATUS read_status;
    UINT8 return_value = 0;
    UINT8 reg_data_bytes[2];

    reg_data_bytes[0]= register_address;
    reg_data_bytes[1] = data;

    read_status = cfa_bsc_OpExtended(reg_data_bytes, sizeof(reg_data_bytes), NULL, 0, DRV8830_SLAVE_ADDRESS,
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

void motor_drive(UINT8 direction, UINT8 speed) {
  UINT8 param = speed << 2 | direction;   
  motor_write_8_bit_register(DRV8830_CONTROL_REG, param);
}

// Absolute macro if not available.
#ifndef ABS
#define ABS(a)                                      (((a) < 0) ? -(a) : (a))
#endif

/******************************************************
 *               Function Prototypes
 ******************************************************/
static void  obaq_create(void);
static void  obaq_timeout( UINT32 count );
static void  obaq_fine_timeout( UINT32 finecount );
static UINT8 obaq_read_16_bit_register(UINT8 register_address, INT16* temperature_data);
static UINT8 obaq_write_16_bit_register(UINT8 register_address, INT16 temperature_data);
static UINT8 obaq_read_8_bit_register(UINT8 register_address, UINT8* config_value);
static UINT8 obaq_write_8_bit_register(UINT8 register_address, UINT8 register_value);
static UINT8 i2c_devices_init(void);
static INT32 obaq_gcd ( INT32 a, INT32 b );
static void  obaq_read_and_print_temperature_data(void);


static void ble_connection_up( void );
static void ble_connection_down( void );
static void ble_advertisement_stopped( void );
static void ble_smp_bond_result( LESMP_PARING_RESULT result );
static void ble_encryption_changed( HCI_EVT_HDR *evt );
static void ble_send_message( void );
static int  ble_write_handler( LEGATTDB_ENTRY_HDR *p );
static void ble_indication_cfm( void );
static void ble_interrupt_handler( UINT8 value );
static void ble_puart_interrupt_callback(void* unused);



/******************************************************
 *               Variables Definitions
 ******************************************************/

/*
 * This is the GATT database for the Temperature Sensor application.  It is
 * currently empty with only required GAP and GATT services.
 */
const UINT8 obaq_gatt_database[]=
{
    // Handle 0x01: GATT service
    PRIMARY_SERVICE_UUID16 (0x0001, UUID_SERVICE_GATT),

    // Handle 0x02: characteristic Service Changed, handle 0x03 characteristic value
    CHARACTERISTIC_UUID16  (0x0002, 0x0003, UUID_CHARACTERISTIC_SERVICE_CHANGED, LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_NONE, 4), 
        0x00, 0x00, 0x00, 0x00,

    // Handle 0x14: GAP service
    PRIMARY_SERVICE_UUID16 (0x0014, UUID_SERVICE_GAP),
    
    // Handle 0x15: characteristic Device Name, handle 0x16 characteristic value
    CHARACTERISTIC_UUID16 (0x0015, 0x0016, UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 16),
        'O','B','A','Q',' ','L','o','c','k','e','r',0,0,0,0,0, 

    // Handle 0x17: characteristic Appearance, handle 0x18 characteristic value
    CHARACTERISTIC_UUID16 (0x0017, 0x0018, UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2),
        BIT16_TO_8(APPEARANCE_GENERIC_BLOOD_PRESSURE),

    // Handle 0x28: Blood Pressure Service
    PRIMARY_SERVICE_UUID16 (0x0028, UUID_SERVICE_BLOOD_PRESSURE),

    // Handle 0x29: characteristic Blood Pressure Measurement, handle 0x2a characteristic value
    CHARACTERISTIC_UUID16 (0x0029, 0x002a, UUID_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT, 
                           LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_NONE, 9),
        0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

    // Handle 0x2b: Client Configuration descriptor
    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x002b, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                      LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

    CHARACTERISTIC_UUID16 (0x002c, 0x002d, UUID_CHARACTERISTIC_INTERMEDIATE_BLOOD_PRESSURE, 
                           LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_NONE, 9),
        0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x002e, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                      LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
        0x00, 0x00,



    // Handle 0x2f: characteristic Blood Pressure Feature, handle 0x30 characteristic value
    CHARACTERISTIC_UUID16 (0x002f, 0x0030, UUID_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE, 
        LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2),
        0x00,0x00,

    // Handle 0x3d: Device Info service
    PRIMARY_SERVICE_UUID16 (0x003d, UUID_SERVICE_DEVICE_INFORMATION),

    // Handle 0x3e: characteristic Manufacturer Name, handle 0x3f characteristic value
    CHARACTERISTIC_UUID16 (0x003e, 0x003f, UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        'H','o','n','e','y','L','a','b',

    // Handle 0x40: characteristic Model Number, handle 0x41 characteristic value
    CHARACTERISTIC_UUID16 (0x0040, 0x0041, UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        '1','2','3','4',0x00,0x00,0x00,0x00,

    // Handle 0x42: characteristic System ID, handle 0x43 characteristic value
    CHARACTERISTIC_UUID16 (0x0042, 0x0043, UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        0x00,0x01,0x02,0x03,0x4,0x5,0x6,0x7,
        
    CHARACTERISTIC_UUID16 (0x0044, 0x0045, UUID_CHARACTERISTIC_DIGITAL, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ, 8),
        0x00,0x03,0x02,0x01,0x3,0x2,0x1,0x7
        


    
};

/*
 * This is the application configuration.
 * */
const BLE_PROFILE_CFG obaq_cfg =
{
        /*.fine_timer_interval            =*/ 100, // ms
        /*.default_adv                    =*/  HIGH_UNDIRECTED_DISCOVERABLE, 
        /*.button_adv_toggle              =*/ 0,    // pairing button make adv toggle (if 1) or always on (if 0)
        /*.high_undirect_adv_interval     =*/ 32,   // slots
        /*.low_undirect_adv_interval      =*/ 2048, // slots
        /*.high_undirect_adv_duration     =*/ 30,   // seconds
        /*.low_undirect_adv_duration      =*/ 300,  // seconds
        /*.high_direct_adv_interval       =*/ 0,    // seconds
        /*.low_direct_adv_interval        =*/ 0,    // seconds
        /*.high_direct_adv_duration       =*/ 0,    // seconds
        /*.low_direct_adv_duration        =*/ 0,    // seconds
        /*.local_name                     =*/ "OBAQ Locker",  // [LOCAL_NAME_LEN_MAX];
        /*.cod                            =*/ BIT16_TO_8(APPEARANCE_GENERIC_BLOOD_PRESSURE), 0,  // [COD_LEN];
        /*.ver                            =*/ "1.00",         // [VERSION_LEN];
        /*.encr_required                  =*/ 0,    // data encrypted and device sends security request on every connection
        /*.disc_required                  =*/ 0,    // if 1, disconnection after confirmation
        /*.test_enable                    =*/ 0,    // TEST MODE is enabled when 1
        /*.tx_power_level                 =*/ 0x04, // dbm
        /*.con_idle_timeout               =*/ 5,    // second  0-> no timeout
        /*.powersave_timeout              =*/ 0,    // second  0-> no timeout
    /*.hdl                            =*/ {0x002a, 0x002d, 0x0030, 0x00, 0x00}, // [HANDLE_NUM_MAX];   //GATT HANDLE number
    /*.serv                           =*/ {UUID_SERVICE_BLOOD_PRESSURE, UUID_SERVICE_BLOOD_PRESSURE, UUID_SERVICE_BLOOD_PRESSURE, 0x00, 0x00},
    /*.cha                            =*/ {UUID_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT, UUID_CHARACTERISTIC_INTERMEDIATE_BLOOD_PRESSURE,
                                            UUID_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE, 0x00, 0x00},
        /*.findme_locator_enable          =*/ 0,    // if 1 Find me locator is enable
        /*.findme_alert_level             =*/ 0,    // alert level of find me
        /*.client_grouptype_enable        =*/ 0,    // if 1 grouptype read can be used
        /*.linkloss_button_enable         =*/ 0,    // if 1 linkloss button is enable
        /*.pathloss_check_interval        =*/ 0,    // second
        /*.alert_interval                 =*/ 0,    // interval of alert
        /*.high_alert_num                 =*/ 0,    // number of alert for each interval
        /*.mild_alert_num                 =*/ 0,    // number of alert for each interval
        /*.status_led_enable              =*/ 0,    // if 1 status LED is enable
        /*.status_led_interval            =*/ 0,    // second
        /*.status_led_con_blink           =*/ 0,    // blink num of connection
        /*.status_led_dir_adv_blink       =*/ 0,    // blink num of dir adv
        /*.status_led_un_adv_blink        =*/ 0,    // blink num of undir adv
        /*.led_on_ms                      =*/ 0,    // led blink on duration in ms
        /*.led_off_ms                     =*/ 0,    // led blink off duration in ms
        /*.buz_on_ms                      =*/ 0,    // buzzer on duration in ms
        /*.button_power_timeout           =*/ 0,    // seconds
        /*.button_client_timeout          =*/ 0,    // seconds
        /*.button_discover_timeout        =*/ 0,    // seconds
        /*.button_filter_timeout          =*/ 0,    // seconds
    #ifdef BLE_UART_LOOPBACK_TRACE
        /*.button_uart_timeout            =*/ 15,   // seconds
    #endif
};

// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG obaq_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ 24,
    /*.rxpin      =*/ 25,
};



#define GPIO_PIN_ADCIN_BATB         	ADC_INPUT_P13
#define GPIO_PIN_ADCEN_BATB         	2
#define GPIO_SETTINGS_ADCEN_BATB    	(GPIO_OUTPUT | GPIO_INIT_HIGH)

#define GPIO_PIN_ADCIN_BATA         	ADC_INPUT_P8
#define GPIO_PIN_ADCEN_BATA         	3
#define GPIO_SETTINGS_ADCEN_BATA    	(GPIO_OUTPUT | GPIO_INIT_HIGH)


#define GPIO_PIN_ADCIN_POTA         	ADC_INPUT_P0
#define GPIO_PIN_ADCIN_POTB         	ADC_INPUT_P15

#undef GPIO_PIN_BUZZER
#undef GPIO_SETTINGS_BUZZER

#define GPIO_PIN_BUZZER         14
#define GPIO_SETTINGS_BUZZER	(GPIO_OUTPUT | GPIO_INIT_LOW | GPIO_BUZ)

// Following structure defines GPIO configuration used by the application
const BLE_PROFILE_GPIO_CFG obaq_gpio_cfg =
{
    /*.gpio_pin =*/
    {
        GPIO_PIN_WP,      // This need to be used to enable/disable NVRAM write protect
        GPIO_PIN_BUTTON,  // Button GPIO is configured to trigger either direction of interrupt
        GPIO_PIN_LED,     // LED GPIO, optional to provide visual effects
        GPIO_PIN_BATTERY, // Battery monitoring GPIO. When it is lower than particular level, it will give notification to the application
        GPIO_PIN_BUZZER,  // Buzzer GPIO, optional to provide audio effects
        GPIO_PIN_ADCEN_BATA,
        GPIO_PIN_ADCEN_BATB,
        GPIO_PIN_ADCIN_BATA,
        GPIO_PIN_ADCIN_BATB,
		GPIO_PIN_ADCIN_POTA,
		GPIO_PIN_ADCIN_POTB,
        -1, -1, -1, -1, -1 // other GPIOs are not used
    },
    /*.gpio_flag =*/
    {
        GPIO_SETTINGS_WP,
        GPIO_SETTINGS_BUTTON,
        GPIO_SETTINGS_LED,
        GPIO_SETTINGS_BATTERY,
        GPIO_SETTINGS_BUZZER,
        GPIO_SETTINGS_ADCEN_BATA,
        GPIO_SETTINGS_ADCEN_BATB, 
        GPIO_INPUT  | GPIO_INIT_LOW,
        GPIO_INPUT  | GPIO_INIT_LOW,
        GPIO_INPUT  | GPIO_INIT_LOW,
        GPIO_INPUT  | GPIO_INIT_LOW,
         0, 0, 0, 0, 0
    }
};

// Remote address when connected.
BD_ADDR ble_sensor_remote_addr        = {0, 0, 0, 0, 0, 0};
/******************************************************
 *               Function Definitions
 ******************************************************/

// Application initialization

APPLICATION_INIT()
{
    bleapp_set_cfg((UINT8 *)obaq_gatt_database,
                   sizeof(obaq_gatt_database),
                   (void *)&obaq_cfg,
                   (void *)&obaq_puart_cfg,
                   (void *)&obaq_gpio_cfg,
                   obaq_create);
}
unsigned int tones_current_state=0;
unsigned int bFirst=1;
unsigned int bEnterSound=0;
#define NVRAM_ID_POSINFO					0x10

#pragma pack(1)
//host information for NVRAM
typedef PACKED struct
{
	UINT32 POTA_ADC_LOCKED;
	UINT32 POTB_ADC_LOCKED;
	UINT32 POTA_ADC_UNLOCKED;
	UINT32 POTB_ADC_UNLOCKED;
	INT32 ROTATION;
}  POSINFO;
POSINFO posinfo;
#pragma pack()
void tones_enter(){
	unsigned int tones_freq[12]={ 262,277,294,311,330,349,367,392,415,440,466,494, };
	unsigned int tones_width[12];
							 
	// 14.64msec=1024step
	// 14.296875 usec = 1step
	
		
	// 262Hz = 3.81679389312977 msec
	// 3816usec*10 /14.3 *10  =266 step
	
	
	int c;
	for(c=0;c<12;c++){
		tones_freq[c]=tones_freq[c];
		tones_width[c]=1000000/tones_freq[c];
		tones_width[c]=tones_width[c]*10 / 143;
//		ble_trace1("tonewidth=%d\n",tones_width[c]);
	}
	
	// F# D A_ D E A E_ E F# E A_ D
	unsigned int FS=tones_width[6];
	unsigned int D=tones_width[2];
	unsigned int A=tones_width[9];
	unsigned int E=tones_width[4];

	{
		unsigned int score[14] = {  
			FS , D , A*2 , D , E , A , A , E*2,
			E , FS , E , A *2 , D , D,  
		};
		if( (tones_current_state%3)==0 ){
			int step=tones_current_state/3;
			if (step < 14 ){
				pwm_start(PWM2, LHL_CLK , 1023-score[step]/2 , 1023-score[step] );
			}else{
			    gpio_configurePin((GPIO_PIN_BUZZER) / 16, (GPIO_PIN_BUZZER) % 16, GPIO_INPUT, 0);
			    bEnterSound=0;
			}
		}

	}
	// Get init and toggle counts for Buzzer and trace it.
//	ble_trace2("Buzzer init count: 0x%03X, Toggle Count: 0x%03X\n", pwm_getInitValue(PWM2), pwm_getToggleCount(PWM2));

    // Output enable the buzzer GPIO.

}

// Plays the tone at the current frequency.
void tones_play_tone(void)
{
	unsigned int tones_freq[12]={ 262,277,294,311,330,349,367,392,415,440,466,494, };
	unsigned int tones_width[12];
							 
	// 14.64msec=1024step
	// 14.296875 usec = 1step
	
		
	// 262Hz = 3.81679389312977 msec
	// 3816usec*10 /14.3 *10  =266 step
	

	int c;
	for(c=0;c<12;c++){
		tones_freq[c]=tones_freq[c]/3;
		tones_width[c]=1000000/tones_freq[c];
		tones_width[c]=tones_width[c]*10 / 143;
//		ble_trace1("tonewidth=%d\n",tones_width[c]);
	}
	
		
	{
		unsigned int bb=tones_width[10];
		unsigned int b=tones_width[11];
		unsigned int score[21] = {  b*2 ,b , bb*2 ,bb,
								b*2 ,b , bb*2 ,bb ,
								b*2 ,b , bb*2 ,bb ,
								b*2 ,b , bb*2 ,bb ,
								tones_width[4] ,tones_width[4],tones_width[4],bb*2,bb*2
								 };

		if (tones_current_state < 21 ){
		    gpio_configurePin((GPIO_PIN_BUZZER) / 16, (GPIO_PIN_BUZZER) % 16, PWM2_OUTPUT_ENABLE_P14, 0);
			pwm_start(PWM2, LHL_CLK , 1023-score[tones_current_state]/2 , 1023-score[tones_current_state] );
		}else{
		    gpio_configurePin((GPIO_PIN_BUZZER) / 16, (GPIO_PIN_BUZZER) % 16, GPIO_INPUT, 0);
		    bFirst=0;
		}

	}
	// Get init and toggle counts for Buzzer and trace it.
//	ble_trace2("Buzzer init count: 0x%03X, Toggle Count: 0x%03X\n", pwm_getInitValue(PWM2), pwm_getToggleCount(PWM2));

    // Output enable the buzzer GPIO.
}

int bstop=0;
void obaq_create(void)
{
    extern UINT32 blecm_configFlag;

    blecm_configFlag |= BLECM_DBGUART_LOG;

    ble_trace0("OBAQ Locker()\n");

    // dump the database to debug uart.
//    legattdb_dumpDb();

 
    if (!mia_isResetReasonPor())
    {
    	ble_trace0("Waking from deep sleep because the timer went off or a GPIO triggered while waiting for timer to expire.");
    }
    else
    {
    	ble_trace0("Not a timed wake.");
    }


    bleprofile_Init(bleprofile_p_cfg);
    bleprofile_GPIOInit(bleprofile_gpio_p_cfg);
    
    i2c_devices_init();

    blebpm_Create();

    bleprofile_KillTimer();
    legattdb_regWriteHandleCb((LEGATTDB_WRITE_CB)ble_write_handler);

    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_UP,ble_connection_up);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_DOWN, ble_connection_down);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ADV_TIMEOUT, ble_advertisement_stopped);
    aclk_configure(512000, ACLK1, ACLK_FREQ_24_MHZ);

    bleprofile_regTimerCb(obaq_fine_timeout, obaq_timeout);
    bleprofile_StartTimer();
//      bleprofile_Discoverable(bleprofile_p_cfg->default_adv, NULL);

    gpio_configurePin((GPIO_PIN_BUZZER) / 16, (GPIO_PIN_BUZZER) % 16, PWM2_OUTPUT_ENABLE_P14, 0);

	gpio_configurePin(0, 10, GPIO_INPUT_DISABLE, 0);
	gpio_configurePin(0, 11, GPIO_INPUT_DISABLE, 0);
	gpio_configurePin(0, 12, GPIO_INPUT_DISABLE, 0);
	gpio_configurePin(1, 10, GPIO_INPUT_DISABLE, 0);
	gpio_configurePin(1, 11, GPIO_INPUT_DISABLE, 0);

    bleapputils_changeLPOSource(LPO_32KHZ_OSC, FALSE, 250);
    bleprofile_ReadNVRAM(NVRAM_ID_POSINFO, sizeof(posinfo), (UINT8 *)&posinfo);
    
    ble_trace0("Load NVRAM\n");
    ble_trace4("{ %d - %d } - { %d - %d }\n",posinfo.POTA_ADC_LOCKED,posinfo.POTB_ADC_LOCKED,posinfo.POTA_ADC_UNLOCKED,posinfo.POTB_ADC_UNLOCKED);

	GetCurrentPositionLocked();
}
#define TCA_INTERRUPT_PORT             (0)
#define TCA_INTERRUPT_PIN              (4)
UINT16 last_tca=0;
int pin_reading=0;
int lastrotate=0;
void tca_interrupt_handler(void* parameter, UINT8 arg)
{
    UINT8 sensor_status = 0, sensor_configuration;
    gpio_clearPinInterruptStatus(TCA_INTERRUPT_PORT, TCA_INTERRUPT_PIN);
	if(pin_reading==0){
		pin_reading=1;
	    ble_trace0("Interrupt!\n");
	    
	    UINT16 r=tca_port_read();
	    
	    if( last_tca != 0 ){
			UINT16 x=last_tca ^ r;
				ble_trace3("last=%04x current=%04x diff=%04x\n",last_tca,r,x);
			
			if( x ){
			}else{
			
			}
			if( x & 0x0800 ){
				if( bstop){
					ble_trace0("bstop\n");
					motor_drive(DRV8830_STANBY, 0);
					bstop=0;
				}
			}
			if( x & 0x0400 ){
				if( bstop){
					ble_trace0("bstop\n");
					motor_drive(DRV8830_STANBY, 0);
					bstop=0;
				}
			}
			if( x & 0x0200 ){
				ble_trace0("Left turned\n");
				lastrotate=-1;
			}

			if( x & 0x0100 ){
				ble_trace0("Right turned\n");
				lastrotate=1;
			}
		}else{
			
		}
		last_tca=r;
		pin_reading=0;
	}else{
		
	}
	
}

int sec=0;
// One second timer expired. Read the sensor data.
void obaq_timeout(UINT32 arg)
{
//    ble_trace0("timer callback\n");
//	led_write_8_bit_register(3,0x03<<((sec%3)*2));
	/*
	motor_drive(((sec/2)%2)?DRV8830_FORWARD:DRV8830_REVERSE, DRV8830_SPEED_MAX);
    next_step(sec%4);*/
	puart_write('B');
    sec++;
    
}


unsigned int mode_lock=0;
unsigned int mode_unlock=0;
int lock_rotation=0;
unsigned int pota_stop=0;
unsigned int potb_stop=0;
unsigned int stopOnPot=0;
int potrotation=0;
UINT32 adcpot_last=0;
void obaq_fine_timeout(UINT32 arg)
{

	
	if( bFirst ){
	    tones_play_tone();
	    tones_current_state=(tones_current_state+1);
	
	}
	if( bEnterSound ){
	   	tones_enter();
	    tones_current_state=(tones_current_state+1);
	}


	if( stopOnPot ){
		UINT32 adc_a,adc_b;
		int diff;
	    ble_trace4("POSINFO { %d - %d } - { %d - %d }\n",posinfo.POTA_ADC_LOCKED,posinfo.POTB_ADC_LOCKED,posinfo.POTA_ADC_UNLOCKED,posinfo.POTB_ADC_UNLOCKED);
		ble_trace2("Move to %d - %d\n",pota_stop,potb_stop);
	    adc_a = adc_readVoltage(GPIO_PIN_ADCIN_POTA);
	    adc_b = adc_readVoltage(GPIO_PIN_ADCIN_POTB);
	    int diffa=pota_stop-adc_a;
	    int diffb=potb_stop-adc_b;
		diff=ABS(diffa)+ABS(diffb);
		ble_trace1("diff=%d\n",diff);

		if( diff < 200 ){
			stopOnPot=0;
			motor_drive(DRV8830_STANBY, 0);
			ble_trace1("STOP!%d\n",diff);
		}
	}
/*
	{
		static UINT32 num_timeouts_since_boot = 0;

		if (num_timeouts_since_boot++ >= 100)
		{
			// If its been ~10s of ADV, configure timed wake and
			// enter deep sleep right now.
			ble_trace0("Entering deep sleep.");

			gpio_configurePin(0, 0, 0x100, 0);

			// Configure the low power manager to enter deep sleep.
			devLpmConfig.disconnectedLowPowerMode = DEV_LPM_DISC_LOW_POWER_MODES_HID_OFF;

			// Configure the wake time in mS.
			devLpmConfig.wakeFromHidoffInMs = 5000;

			// Configure the reference clock to use.

			// Use the external 32k.
			devLpmConfig.wakeFromHidoffRefClk = HID_OFF_TIMED_WAKE_CLK_SRC_32KHZ;

			gpio_configurePin(0, 0, 0x100, 0);

			// Enter deep-sleep now. Will not return.
			devlpm_enterLowPowerMode();
		}
	}*/
	
}
// POTA ->> CCW
// POTB ->> CW 
// –ñ270“x‚ÅˆêŽü
static int posmode=0;

static int posadca=0;
static int posadcb=0;
static int posadca2=0;
static int posadcb2=0;


void SetStopOnPot(unsigned int a,unsigned int b){
	pota_stop=a;
	potb_stop=b;
	stopOnPot=1;
	
}
void do_unlock(){
	SetStopOnPot(posinfo.POTA_ADC_UNLOCKED,posinfo.POTB_ADC_UNLOCKED);
	motor_drive(posinfo.ROTATION==-1?DRV8830_FORWARD:DRV8830_REVERSE, DRV8830_SPEED_MAX);
	
}
void do_lock(){
	SetStopOnPot(posinfo.POTA_ADC_LOCKED,posinfo.POTB_ADC_LOCKED);
	motor_drive(posinfo.ROTATION==1?DRV8830_FORWARD:DRV8830_REVERSE, DRV8830_SPEED_MAX);

}

int GetCurrentPositionLocked(){
	UINT32 adc_a,adc_b;
	int diff;
    adc_a = adc_readVoltage(GPIO_PIN_ADCIN_POTA);
    adc_b = adc_readVoltage(GPIO_PIN_ADCIN_POTB);
    
    int diffa=posinfo.POTA_ADC_UNLOCKED-adc_a;
    int diffb=posinfo.POTB_ADC_UNLOCKED-adc_b;
	diff=ABS(diffa)+ABS(diffb);

	if( diff < 200 ){
		ble_trace0("Unlocked position\n");
		return -1;
	}
	
	diffa=posinfo.POTA_ADC_LOCKED-adc_a;
    diffb=posinfo.POTB_ADC_LOCKED-adc_b;
	diff=ABS(diffa)+ABS(diffb);

	if( diff < 200 ){
		ble_trace0("Locked position\n");
		return 1;
	}
	
	ble_trace0("Position Error!!!\n");
	return 0;
}

// Application thread context uart interrupt handler.
// unused - Unused parameter.
static void puart_control_rx_callback(void* unused)
 {
    // There can be at most 16 bytes in the HW FIFO.
    char  readbytes[17];
    UINT8 number_of_bytes_read = 0, i;
    char  readbyte;
    int   bytes_read = 0;
    int   done = FALSE;
    UINT8 writtenbyte;
    while (puart_rxFifoNotEmpty() && puart_read(&readbyte))
    {
    	bytes_read++;

		if( readbyte=='P' ){
		    UINT32 adc3 = adc_readVoltage(GPIO_PIN_ADCIN_POTA);
		    UINT32 adc4 = adc_readVoltage(GPIO_PIN_ADCIN_POTB);
		    ble_trace2("POTA=%d[mV] POTBB=%d[mV]\n",adc3,adc4);
		    
		    if( posmode==0 ){
			    ble_trace0("Make move thumb-turn and Re-Enter \'P\'\n");
			    posadca=adc3;
			    posadcb=adc4;
			    posmode=1;
			}else if( posmode == 1 ){
			    posadca2=adc3;
			    posadcb2=adc4;
			    posmode=0;
			    ble_trace4("set positions { %d - %d } - { %d - %d }\n",posadca,posadcb,posadca2,posadcb2 );
			    ble_trace1("rotation=%d\n",lastrotate);
			    lock_rotation=lastrotate;
			    
			    /*
			    //host information for NVRAM
				typedef PACKED struct
				{
					UINT32 POTA_ADC_LOCKED;
					UINT32 POTB_ADC_LOCKED;
					UINT32 POTA_ADC_UNLOCKED;
					UINT32 POTB_ADC_UNLOCKED;
					INT32 ROTATION;
				}  POSINFO;
			    */
			 	posinfo.POTA_ADC_LOCKED=posadca;
			 	posinfo.POTB_ADC_LOCKED=posadcb;

			 	posinfo.POTA_ADC_UNLOCKED=posadca2;
			 	posinfo.POTB_ADC_UNLOCKED=posadcb2;
			 	posinfo.ROTATION=lock_rotation;
			 	
			    ble_trace5("{ %d - %d } - { %d - %d } ROT=%d\n",
			    posinfo.POTA_ADC_LOCKED,posinfo.POTB_ADC_LOCKED,posinfo.POTA_ADC_UNLOCKED,posinfo.POTB_ADC_UNLOCKED,posinfo.ROTATION);
		 		writtenbyte = bleprofile_WriteNVRAM(NVRAM_ID_POSINFO, sizeof(posinfo), (UINT8 *)&posinfo);
			    ble_trace1("NVRAM write:%04x\n", writtenbyte);
			    lastrotate=0;
			    
			}
		}
		if( posmode==1 ) continue;

		//puart_control.p_command->data[puart_control.uart_command_offset++] = readbyte;//
		ble_trace1("[%c]\n",readbyte);
		if( readbyte=='F' ){
			motor_drive(DRV8830_FORWARD, DRV8830_SPEED_MAX);
			bstop=1;
		}
		if( readbyte=='R' ){
			motor_drive(DRV8830_REVERSE, DRV8830_SPEED_MAX);
			bstop=1;
		}
		if( readbyte=='S' ){
			motor_drive(DRV8830_STANBY, 0);
		}
		if( readbyte=='L' ){
			do_lock();

		}
		if( readbyte=='U' ){
		    gpio_configurePin((GPIO_PIN_BUZZER) / 16, (GPIO_PIN_BUZZER) % 16, PWM2_OUTPUT_ENABLE_P14, 0);
			tones_current_state=0;
			bEnterSound=1;
			do_unlock();
		}
		

    }
    
    // clear the interrupt
    P_UART_INT_CLEAR(P_UART_ISR_RX_AFF_MASK);

    // enable UART interrupt in the Main Interrupt Controller and RX Almost Full in the UART Interrupt Controller
    P_UART_INT_ENABLE |= P_UART_ISR_RX_AFF_MASK;

	/*
    if (done)
    {
    	puart_control.uart_rx_state = PUART_CONTROL_UART_STATE_IDLE;
    	if (!puart_control_process_rx_command(puart_control.p_command))
    	{
            cfa_mm_Free(puart_control.p_command);
            puart_control.p_command = NULL;
    	}
    	return;
    }*/
/*
    if (bytes_read % PUART_CONTROL_READ_CHUNK_SIZE == 0)
    {
    	test_puart_write(PUART_CONTROL_EVENT_CONTINUE);
    }
    return;*/
}


// Callback called by the FW when ready to sleep/deep-sleep. Disable both by returning 0
// when there is an active download ongoing.
UINT32 uart_device_lpm_queriable(LowPowerModePollType type, UINT32 context)
{
    // Disable sleep.
    return 0;
}


static void tw_puart_init(UINT8 rxPortPin, UINT8 txPortPin, UINT32 bdRate)
 {
    extern puart_UartConfig puart_config;

    // Set the baud rate we want to use. Default is 115200.
    puart_config.baudrate = bdRate;

    // Select the uart pins for RXD, TXD and optionally CTS and RTS.
    // If hardware flow control is not required like here, set these
    // pins to 0x00. See Table 1 and Table 2 for valid options.
    puart_selectUartPads(rxPortPin, txPortPin, 0x00, 0x00);
	ble_trace2("tx=%d rx=%d\n",txPortPin,rxPortPin);
    // Initialize the peripheral uart driver
    puart_init();

    // Since we are not configuring CTS and RTS here, turn off
    // hardware flow control. If HW flow control is used, then
    // puart_flowOff should not be invoked.
    puart_flowOff();

	// Since we are not using any flow control, disable sleep.
	// If HW flow control is configured or app uses its own flow control mechanism,
	// this is not required.
    devlpm_registerForLowPowerQueries(uart_device_lpm_queriable, 0);
    
    // BEGIN - puart interrupt
    //  The following lines enable interrupt when one (or more) bytes
    //  are received over the peripheral uart interface. This is optional.
    //  In the absense of this, the app is expected to poll the peripheral
    //  uart to pull out received bytes.

    // clear interrupt
    P_UART_INT_CLEAR(P_UART_ISR_RX_AFF_MASK);

    // set watermark to 1 byte - will interrupt on every byte received.
    P_UART_WATER_MARK_RX_LEVEL(1);

    // enable UART interrupt in the Main Interrupt Controller and RX Almost Full in the UART
    // Interrupt Controller
    P_UART_INT_ENABLE |= P_UART_ISR_RX_AFF_MASK;

    // Set callback function to app callback function.
    puart_rxCb = puart_control_rx_callback;
    
    // Enable the CPU level interrupt
    puart_enableInterrupt();

	/* END - puart interrupt */
}
#define TCA_PIN_SENSOR_VOLTAGE 15
#define TCA_PIN_LED_EN	17


UINT8 i2c_devices_init(void)
{
	int n;
    ble_trace0("i2c_devices_init()\n");

	tca_begin(0);


	tca_pin_mode(000, TCA_PINMODE_OUTPUT);
	tca_pin_mode(001, TCA_PINMODE_OUTPUT);
	tca_pin_mode(002, TCA_PINMODE_OUTPUT);
	tca_pin_mode(003, TCA_PINMODE_OUTPUT);
	
	tca_pin_mode(006, TCA_PINMODE_OUTPUT);
	tca_pin_mode(007, TCA_PINMODE_OUTPUT);

	tca_pin_mode(015, TCA_PINMODE_OUTPUT);
	tca_pin_mode(017, TCA_PINMODE_OUTPUT);

	tca_pin_mode(010, TCA_PINMODE_INPUT);
	tca_pin_mode(011, TCA_PINMODE_INPUT);
	tca_pin_mode(012, TCA_PINMODE_INPUT);
	tca_pin_mode(013, TCA_PINMODE_INPUT);
	
	// LOW_ENABLE
	tca_pin_write(TCA_PIN_SENSOR_VOLTAGE, TCA_OUTPUT_LOW);
	//tca_pin_write(TCA_PIN_SENSOR_VOLTAGE, TCA_OUTPUT_HIGH);

	// HIGH ENABLE
	tca_pin_write(TCA_PIN_LED_EN, TCA_OUTPUT_HIGH);
	//tca_pin_write(TCA_PIN_LED_EN, TCA_OUTPUT_LOW);
	
	tca_pin_write(006, TCA_OUTPUT_HIGH);
	tca_pin_write(007, TCA_OUTPUT_HIGH);

	led_write_8_bit_register(0,0x07);
	led_write_8_bit_register(1,0x07);
	led_write_8_bit_register(2,0x07);

//	led_write_8_bit_register(3,0x03);

    ble_trace0("i2c_devices_init_end()\n");
    
    adc_config();
    adc_SetInputRange(ADC_RANGE_0_3P6V);


    UINT16 interrupt_handler_mask[3] = {0, 0, 0};


    interrupt_handler_mask[TCA_INTERRUPT_PORT] |= (1 << TCA_INTERRUPT_PIN);
    gpio_registerForInterrupt(interrupt_handler_mask, tca_interrupt_handler, NULL);
    gpio_configurePin(TCA_INTERRUPT_PORT, TCA_INTERRUPT_PIN,
            GPIO_EN_INT_FALLING_EDGE | GPIO_PULL_UP, GPIO_PIN_OUTPUT_LOW);

    tw_puart_init(25, 24, 115200);


    return 1;
}

void next_step(int step){
	/*
	switch(step){
		case 0:
		    ble_trace0("tca pin0 high\n");
			tca_pin_write(000, TCA_OUTPUT_HIGH);
			break;
		case 1:
		    ble_trace0("tca pin1 high\n");
			tca_pin_write(001, TCA_OUTPUT_HIGH);
			break;
		case 2:
		    ble_trace0("tca pin2 high\n");
			tca_pin_write(002, TCA_OUTPUT_HIGH);
			break;
		case 3:
		    ble_trace0("tca pin3 high\n");
			tca_pin_write(003, TCA_OUTPUT_HIGH);
		    ble_trace0("all high\n");
			
			break;
	
	}*/
    ble_trace1("step=%d\n",step);
	
    gpio_setPinOutput(GPIO_PIN_ADCEN_BATB >> 4, GPIO_PIN_ADCEN_BATB & 0x0F, 1);
    gpio_setPinOutput(GPIO_PIN_ADCEN_BATA >> 4, GPIO_PIN_ADCEN_BATA & 0x0F, 1);
    UINT32 adc1 = adc_readVoltage(GPIO_PIN_ADCIN_BATA);
    UINT32 adc2 = adc_readVoltage(GPIO_PIN_ADCIN_BATB);
    
    
    ble_trace2("A=%d[mV] B=%d[mV]\n",adc1*28/10,adc2*28/10);
    
    gpio_setPinOutput(GPIO_PIN_ADCEN_BATB >> 4, GPIO_PIN_ADCEN_BATB & 0x0F, 0);
    gpio_setPinOutput(GPIO_PIN_ADCEN_BATA >> 4, GPIO_PIN_ADCEN_BATA & 0x0F, 0);

/*    
	{	int n;
	for(n=0;n<10;n++){
	    UINT32 adc3 = adc_readVoltage(GPIO_PIN_ADCIN_POTA);
	    UINT32 adc4 = adc_readVoltage(GPIO_PIN_ADCIN_POTB);
	    ble_trace2("POTA=%d[mV] POTBB=%d[mV]\n",adc3,adc4);
	
	}
	}*/
	return;
}

int ble_write_handler(LEGATTDB_ENTRY_HDR *p)
{
    UINT8  writtenbyte;
    UINT16 handle   = legattdb_getHandle(p);
    int    len      = legattdb_getAttrValueLen(p);
    UINT8  *attrPtr = legattdb_getAttrValue(p);


    ble_trace1("hello_sensor_write_handler: handle %04x\n", handle);
	
	char data[8];
	memcpy(data,attrPtr,len);
	int c;
	for(c=0;c<len;c++){
		ble_trace1("[%x]\n",data[c]);
	}
	
	switch(data[0]){
		case 1:
			motor_drive(DRV8830_FORWARD, DRV8830_SPEED_MAX);
			bstop=1;
			break;
		case 2:
			motor_drive(DRV8830_REVERSE, DRV8830_SPEED_MAX);
			bstop=1;
			break;
		case 'U':
			do_unlock();
			break;
		case 'L':
			do_lock();
			break;
		default:
			motor_drive(DRV8830_STANBY, 0);
			break;
	}
	/*

	  // do some noise
    bleprofile_BUZBeep(bleprofile_p_cfg->buz_on_ms);

    // make sure that it is the paired device which is trying to write
    // read BDADDR of the "paired device" from the NVRAM and compare with connected
    bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(hello_sensor_hostinfo), (UINT8 *)&hello_sensor_hostinfo);

    if (memcmp(hello_sensor_remote_addr, hello_sensor_hostinfo.bdaddr, 6) != 0)
    {
        ble_trace1("hello_sensor_write_handler: wrong host handle %04x\n", handle);
        return 0;

    }

    // By writing into Characteristic Client Configuration descriptor
    // peer can enable or disable notification or indication
    if ((len == 2) && (handle == HANDLE_HELLO_SENSOR_CLIENT_CONFIGURATION_DESCRIPTOR))
    {
        hello_sensor_hostinfo.characteristic_client_configuration = attrPtr[0] + (attrPtr[1] << 8);
        ble_trace1("hello_sensor_write_handler: client_configuration %04x\n", hello_sensor_hostinfo.characteristic_client_configuration);

        // Save update to NVRAM.  Client does not need to set it on every connection.
        writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(hello_sensor_hostinfo), (UINT8 *)&hello_sensor_hostinfo);
        ble_trace1("hello_sensor_write_handler: NVRAM write:%04x\n", writtenbyte);
    }
    // User can change number of blinks to send when button is pushed
    else if ((len == 1) && (handle == HANDLE_HELLO_SENSOR_CONFIGURATION))
    {
        hello_sensor_hostinfo.number_of_blinks = attrPtr[0];
    	if (hello_sensor_hostinfo.number_of_blinks != 0)
    	{
    	    bleprofile_LEDBlink(250, 250, hello_sensor_hostinfo.number_of_blinks);
    	}
        // Save update to NVRAM.  Client does not need to set it on every connection.
        writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(hello_sensor_hostinfo), (UINT8 *)&hello_sensor_hostinfo);
        ble_trace1("hello_sensor_write_handler: NVRAM write:%04x\n", writtenbyte);
    }
    else
    {
        ble_trace2("hello_sensor_write_handler: bad write len:%d handle:0x%x\n", len, handle);
		return 0x80;
    }*/
    ble_trace0("write_handler\n");
    
   	return 0;
}

// This function will be called on every connection establishmen
void ble_connection_up(void)
{
	    // Stop advertising
    bleprofile_Discoverable(NO_DISCOVERABLE, NULL);

    bleprofile_StopConnIdleTimer();
	/*
    UINT8 writtenbyte;
    UINT8 *bda;

    hello_sensor_connection_handle = (UINT16)emconinfo_getConnHandle();

    // save address of the connected device and print it out.
    memcpy(hello_sensor_remote_addr, (UINT8 *)emconninfo_getPeerAddr(), sizeof(hello_sensor_remote_addr));

    ble_trace3("hello_sensor_connection_up: %08x%04x %d\n",
                (hello_sensor_remote_addr[5] << 24) + (hello_sensor_remote_addr[4] << 16) + 
                (hello_sensor_remote_addr[3] << 8) + hello_sensor_remote_addr[2],
                (hello_sensor_remote_addr[1] << 8) + hello_sensor_remote_addr[0],
                hello_sensor_connection_handle);

    // Stop advertising
    bleprofile_Discoverable(NO_DISCOVERABLE, NULL);

    bleprofile_StopConnIdleTimer();

    // as we require security for every connection, we will not send any indications until
    // encryption is done.
    if (bleprofile_p_cfg->encr_required != 0)
    {
        lesmp_sendSecurityRequest();
        return;
    }
    // saving bd_addr in nvram

    bda =(UINT8 *)emconninfo_getPeerAddr();

    memcpy(hello_sensor_hostinfo.bdaddr, bda, sizeof(BD_ADDR));
    hello_sensor_hostinfo.characteristic_client_configuration = 0;
    hello_sensor_hostinfo.number_of_blinks = 0;

    writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(hello_sensor_hostinfo), (UINT8 *)&hello_sensor_hostinfo);
    ble_trace1("NVRAM write:%04x\n", writtenbyte);

    hello_sensor_encryption_changed(NULL);*/
    
    ble_trace0("connected\n");
}

// This function will be called when connection goes down
void ble_connection_down(void)
{
	/*
    ble_trace3("hello_sensor_connection_down:%08x%04x handle:%d\n",
                (hello_sensor_remote_addr[5] << 24) + (hello_sensor_remote_addr[4] << 16) +
                (hello_sensor_remote_addr[3] << 8) + hello_sensor_remote_addr[2],
                (hello_sensor_remote_addr[1] << 8) + hello_sensor_remote_addr[0],
                hello_sensor_connection_handle);

	memset (hello_sensor_remote_addr, 0, 6);
	hello_sensor_connection_handle = 0;

    // If we are configured to stay connected, disconnection was caused by the
    // peer, start low advertisements, so that peer can connect when it wakes up.
    if (hello_sensor_stay_connected)
    {
        bleprofile_Discoverable(LOW_UNDIRECTED_DISCOVERABLE, hello_sensor_hostinfo.bdaddr);

        ble_trace2("ADV start: %08x%04x\n",
                      (hello_sensor_hostinfo.bdaddr[5] << 24 ) + (hello_sensor_hostinfo.bdaddr[4] <<16) +
                      (hello_sensor_hostinfo.bdaddr[3] << 8 ) + hello_sensor_hostinfo.bdaddr[2],
                      (hello_sensor_hostinfo.bdaddr[1] << 8 ) + hello_sensor_hostinfo.bdaddr[0]);
    }*/
    ble_trace0("connection down\n");
}

void ble_advertisement_stopped(void)
{
    ble_trace0("ADV stop!!!!");

/*
    // If we are configured to stay connected, disconnection was caused by the
    // peer, start low advertisements, so that peer can connect when it wakes up.
    if (hello_sensor_stay_connected)
    {
        bleprofile_Discoverable(LOW_UNDIRECTED_DISCOVERABLE, hello_sensor_hostinfo.bdaddr);

        ble_trace2("ADV start: %08x%04x\n",
                      (hello_sensor_hostinfo.bdaddr[5] << 24 ) + (hello_sensor_hostinfo.bdaddr[4] <<16) +
                      (hello_sensor_hostinfo.bdaddr[3] << 8 ) + hello_sensor_hostinfo.bdaddr[2],
                      (hello_sensor_hostinfo.bdaddr[1] << 8 ) + hello_sensor_hostinfo.bdaddr[0]);
    }*/
}


