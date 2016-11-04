#define HEATER_PINS_INVERTED 0

/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define ORIG_X_STEP_PIN     0
#define ORIG_X_DIR_PIN      2
#define ORIG_X_MIN_PIN      -1
#define ORIG_X_MAX_PIN      -1
#define ORIG_X_ENABLE_PIN   -1

#define ORIG_Y_STEP_PIN     4
#define ORIG_Y_DIR_PIN      5
#define ORIG_Y_MIN_PIN      -1
#define ORIG_Y_MAX_PIN      -1
#define ORIG_Y_ENABLE_PIN   -1

#define ORIG_Z_STEP_PIN     12
#define ORIG_Z_DIR_PIN      13
#define ORIG_Z_MIN_PIN      -1
#define ORIG_Z_MAX_PIN      -1
#define ORIG_Z_ENABLE_PIN   -1

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN     14
// Due analog pin #54
#define TEMP_0_PIN       A0

#define HEATER_1_PIN     -1
// Due analog pin #55
#define TEMP_1_PIN       -1

#define HEATER_2_PIN     -1
// Due analog pin #56
#define TEMP_2_PIN       -1

#define HEATER_3_PIN     -1
// Due analog pin #57
#define TEMP_3_PIN       -1

// Due analog pin #58
#define TEMP_4_PIN       -1

#define ORIG_E0_STEP_PIN    15
#define ORIG_E0_DIR_PIN     16
#define ORIG_E0_ENABLE_PIN  -1

#define ORIG_E1_STEP_PIN    -1
#define ORIG_E1_DIR_PIN     -1
#define ORIG_E1_ENABLE_PIN  -1

#define ORIG_E2_STEP_PIN    -1
#define ORIG_E2_DIR_PIN     -1
#define ORIG_E2_ENABLE_PIN  -1

#define SDSUPPORT      false
#define SDPOWER      -1
// 4,10,52 if using HW SPI.
#define SDSS       4 
//#define SDSS       -1
//#define ORIG_SDCARDDETECT   -1
#define SDCARDDETECTINVERTED false
#define LED_PIN      -1
#define ORIG_FAN_PIN     -1
#define ORIG_FAN2_PIN       -1
#define ORIG_PS_ON_PIN      -1
#define KILL_PIN     -1
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN         20    
// 21 or 71
#define SCL_PIN         21    


#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,

//#define TWI_CLOCK_FREQ          400000
// see eeprom device data sheet for the following values these are for 24xx256
//#define EEPROM_SERIAL_ADDR      0x50   // 7 bit i2c address (without R/W bit)
//#define EEPROM_PAGE_SIZE        64     // page write buffer size
//#define EEPROM_PAGE_WRITE_TIME  7      // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
//#define EEPROM_ADDRSZ_BYTES     TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 0

