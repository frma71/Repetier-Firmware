/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.



    Main author: repetier

    Initial port of hardware abstraction layer to Arduino Due: John Silvia
*/

#include "Repetier.h"
#include <malloc.h>

//extern "C" void __cxa_pure_virtual() { }
extern "C" char *sbrk(int i);
extern long bresenham_step();

#define NUM_ADC_SAMPLES 2 + (1 << ANALOG_INPUT_SAMPLE)
#if ANALOG_INPUTS > 0
int32_t osAnalogInputBuildup[ANALOG_INPUTS];
int32_t osAnalogSamples[ANALOG_INPUTS][ANALOG_INPUT_MEDIAN];
int32_t osAnalogSamplesSum[ANALOG_INPUTS];
static int32_t adcSamplesMin[ANALOG_INPUTS];
static int32_t adcSamplesMax[ANALOG_INPUTS];
static int adcCounter = 0, adcSamplePos = 0;
#endif

static   uint32_t  adcEnable = 0;

char HAL::virtualEeprom[EEPROM_BYTES];
bool HAL::wdPinged = true;
volatile uint8_t HAL::insideTimer1 = 0;

HAL::HAL()
{
  //ctor
}

HAL::~HAL()
{
  //dtor
}



#if ANALOG_INPUTS > 0
// Initialize ADC channels
void HAL::analogStart(void)
{
}

#endif

// Print apparent cause of start/restart
void HAL::showStartReason() {
}

// Return available memory
int HAL::getFreeRam() {
  return 1024*1014;
}

// Reset peripherals and cpu
void HAL::resetHardware() {
}

// from http://medialab.freaknet.org/martin/src/sqrt/sqrt.c
uint32_t HAL::integer64Sqrt(uint64_t a_nInput) {
  uint64_t op  = a_nInput;
  uint64_t res = 0;
  uint64_t one = 1uLL << 62; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

  // "one" starts at the highest power of four <= than the argument.
  while (one > op)
    one >>= 2;
  while (one != 0)
  {
    if (op >= res + one)
    {
      op = op - (res + one);
      res = res +  2 * one;
    }
    res >>= 1;
    one >>= 2;
  }
  if (op > res) // Do arithmetic rounding to nearest integer
  {
    res++;
  }
  return res;
}


#ifndef DUE_SOFTWARE_SPI
// hardware SPI
void HAL::spiBegin()
{
}
// spiClock is 0 to 6, relecting AVR clock dividers 2,4,8,16,32,64,128
// Due can only go as slow as AVR divider 32 -- slowest Due clock is 329,412 Hz
void HAL::spiInit(uint8_t spiClock)
{
}
// Write single byte to SPI
void HAL::spiSend(byte b) {
}
void HAL::spiSend(const uint8_t* buf , size_t n)
{
}

// Read single byte from SPI
uint8_t HAL::spiReceive()
{
}

// Read from SPI into buffer
void HAL::spiReadBlock(uint8_t*buf, uint16_t nbyte)
{
}

// Write from buffer to SPI

void HAL::spiSendBlock(uint8_t token, const uint8_t* buf)
{
}
#endif

/****************************************************************************************
 Setting for I2C Clock speed. needed to change  clock speed for different peripherals
****************************************************************************************/

void HAL::i2cSetClockspeed(uint32_t clockSpeedHz)
  
{
}

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void HAL::i2cInit(unsigned long clockSpeedHz)
{
}


/*************************************************************************
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char HAL::i2cStart(unsigned char address_and_direction)
{
}


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready

 Input:   address and transfer direction of I2C device
*************************************************************************/
void HAL::i2cStartWait(unsigned char address_and_direction)
{
}

/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 Also specifies internal address of device

 Input:   address and transfer direction of I2C device, internal address
*************************************************************************/
void HAL::i2cStartAddr(unsigned char address_and_direction, unsigned int pos)
{
}

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void HAL::i2cStop(void)
{
}

/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
  Return:   0 write successful
            1 write failed
*************************************************************************/
void HAL::i2cWrite( uint8_t data )
{
}

/*************************************************************************
 Read one byte from the I2C device, request more data from device
 Return:  byte read from I2C device
*************************************************************************/
uint8_t HAL::i2cReadAck(void)
{
}

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition

 Return:  byte read from I2C device
*************************************************************************/
uint8_t HAL::i2cReadNak(void)
{
}


#if FEATURE_SERVO
// may need further restrictions here in the future
#if defined (__SAM3X8E__)
unsigned int HAL::servoTimings[4] = {0, 0, 0, 0};
static uint8_t servoIndex = 0;
unsigned int servoAutoOff[4] = {0, 0, 0, 0};
void HAL::servoMicroseconds(uint8_t servo, int microsec, uint16_t autoOff) {
  if (microsec < 500) microsec = 0;
  if (microsec > 2500) microsec = 2500;
  servoTimings[servo] = (unsigned int)(((F_CPU_TRUE / SERVO_PRESCALE) /
                                        1000000) * microsec);
  servoAutoOff[servo] = (microsec) ? (autoOff / 20) : 0;
}


// ================== Interrupt handling ======================

// Servo timer Interrupt handler
void SERVO_COMPA_VECTOR ()
{
  InterruptProtectedBlock noInt;
  static uint32_t     interval;

  // apparently have to read status register
  //TC_GetStatus(SERVO_TIMER, SERVO_TIMER_CHANNEL);

  switch (servoIndex) {
    case 0:
      if (HAL::servoTimings[0]) {
#if SERVO0_PIN > -1
        WRITE(SERVO0_PIN, HIGH);
#endif
        interval =  HAL::servoTimings[0];
      } else
        interval = SERVO2500US;
      //TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, interval);
      break;
    case 1:
#if SERVO0_PIN > -1
      WRITE(SERVO0_PIN, LOW);
#endif
      //TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO5000US - interval);
      break;
    case 2:
      if (HAL::servoTimings[1]) {
#if SERVO1_PIN > -1
        WRITE(SERVO1_PIN, HIGH);
#endif
        interval =  HAL::servoTimings[1];
      } else
        interval = SERVO2500US;
      //TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, interval);
      break;
    case 3:
#if SERVO1_PIN > -1
      WRITE(SERVO1_PIN, LOW);
#endif
      //TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO5000US - interval);
      break;
    case 4:
      if (HAL::servoTimings[2]) {
#if SERVO2_PIN > -1
        WRITE(SERVO2_PIN, HIGH);
#endif
        interval =  HAL::servoTimings[2];
      } else
        interval = SERVO2500US;
      //TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, interval);
      break;
    case 5:
#if SERVO2_PIN > -1
      WRITE(SERVO2_PIN, LOW);
#endif
      //TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO5000US - interval);
      break;
    case 6:
      if (HAL::servoTimings[3]) {
#if SERVO3_PIN > -1
        WRITE(SERVO3_PIN, HIGH);
#endif
        interval =  HAL::servoTimings[3];
      } else
        interval = SERVO2500US;
      //TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, interval);
      break;
    case 7:
#if SERVO3_PIN > -1
      WRITE(SERVO3_PIN, LOW);
#endif
      //TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO5000US - interval);
      break;
  }
  if (servoIndex & 1)
  {
    uint8_t nr = servoIndex >> 1;
    if (servoAutoOff[nr])
    {
      servoAutoOff[nr]--;
      if (servoAutoOff[nr] == 0) HAL::servoTimings[nr] = 0;
    }
  }
  servoIndex++;
  if (servoIndex > 7) servoIndex = 0;
  return interval;
}
#else
#error No servo support for your board, please diable FEATURE_SERVO
#endif
#endif

//TcChannel *stepperChannel = (TIMER1_TIMER->TC_CHANNEL + TIMER1_TIMER_CHANNEL);
#ifndef STEPPERTIMER_EXIT_TICKS
#define STEPPERTIMER_EXIT_TICKS 105 // at least 2,5us pause between stepper calls
#endif

/** \brief Timer interrupt routine to drive the stepper motors.
*/
uint32_t TIMER1_COMPA_VECTOR ()
{
  static long startLine = 0;
  // apparently have to read status register
  // stepperChannel->TC_SR;
  //stepperChannel->TC_RC = 1000000;
  uint32_t delay;
  if (PrintLine::hasLines())
  {
    delay = PrintLine::bresenhamStep();
    if(startLine == 0)
       startLine = millis();
  }
  else if (Printer::zBabystepsMissing != 0) {
    Printer::zBabystep();
    delay = Printer::interval;
  } else {
    if(startLine) {
      Serial.print("Done: ");
      Serial.println(millis() - startLine);
      startLine = 0;
    }
    if (waitRelax == 0)
    {
#if USE_ADVANCE
      if (Printer::advanceStepsSet)
      {
        Printer::extruderStepsNeeded -= Printer::advanceStepsSet;
#if ENABLE_QUADRATIC_ADVANCE
        Printer::advanceExecuted = 0;
#endif
        Printer::advanceStepsSet = 0;
      }
      if ((!Printer::extruderStepsNeeded) && (DISABLE_E))
        Extruder::disableCurrentExtruderMotor();
#else
      if (DISABLE_E) Extruder::disableCurrentExtruderMotor();
#endif
    }
    else waitRelax--;
    
    delay = 1000000;
  }
  #if 0
  if(delay < 1000)
      delay = 1000;
  #endif
  return delay;
}

#if !defined(HEATER_PWM_SPEED)
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED < 0
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED > 2
#define HEATER_PWM_SPEED 2
#endif

#if HEATER_PWM_SPEED == 0
#define HEATER_PWM_STEP 1
#define HEATER_PWM_MASK 255
#elif HEATER_PWM_SPEED == 1
#define HEATER_PWM_STEP 2
#define HEATER_PWM_MASK 254
#else
#define HEATER_PWM_STEP 4
#define HEATER_PWM_MASK 252
#endif

#if !defined(COOLER_PWM_SPEED)
#define COOLER_PWM_SPEED 0
#endif
#if COOLER_PWM_SPEED < 0
#define COOLER_PWM_SPEED 0
#endif
#if COOLER_PWM_SPEED > 2
#define COOLER_PWM_SPEED 2
#endif

#if COOLER_PWM_SPEED == 0
#define COOLER_PWM_STEP 1
#define COOLER_PWM_MASK 255
#elif COOLER_PWM_SPEED == 1
#define COOLER_PWM_STEP 2
#define COOLER_PWM_MASK 254
#else
#define COOLER_PWM_STEP 4
#define COOLER_PWM_MASK 252
#endif

#define pulseDensityModulate( pin, density,error,invert) {uint8_t carry;carry = error + (invert ? 255 - density : density); WRITE(pin, (carry < error)); error = carry;}

/**
This timer is called 3906 times per second. It is used to update
pwm values for heater and some other frequent jobs.
*/
int PWM_TIMER_VECTOR ()
{
#if 1
  //InterruptProtectedBlock noInt;
  // apparently have to read status register
  //TC_GetStatus(PWM_TIMER, PWM_TIMER_CHANNEL);

  static uint8_t pwm_count_cooler = 0;
  static uint8_t pwm_count_heater = 0;
  static uint8_t pwm_pos_set[NUM_PWM];
  static uint8_t pwm_cooler_pos_set[NUM_EXTRUDER];

  if (pwm_count_heater == 0 && !PDM_FOR_EXTRUDER)
  {
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1
    if ((pwm_pos_set[0] = (pwm_pos[0] & HEATER_PWM_MASK)) > 0) WRITE(EXT0_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1 && !MIXING_EXTRUDER
    if ((pwm_pos_set[1] = (pwm_pos[1] & HEATER_PWM_MASK)) > 0) WRITE(EXT1_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2 && !MIXING_EXTRUDER
    if ((pwm_pos_set[2] = (pwm_pos[2] & HEATER_PWM_MASK)) > 0) WRITE(EXT2_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3 && !MIXING_EXTRUDER
    if ((pwm_pos_set[3] = (pwm_pos[3] & HEATER_PWM_MASK)) > 0) WRITE(EXT3_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4 && !MIXING_EXTRUDER
    if ((pwm_pos_set[4] = (pwm_pos[4] & HEATER_PWM_MASK)) > 0) WRITE(EXT4_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5 && !MIXING_EXTRUDER
    if ((pwm_pos_set[5] = (pwm_pos[5] & HEATER_PWM_MASK)) > 0) WRITE(EXT5_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if HEATED_BED_HEATER_PIN > -1 && HAVE_HEATED_BED
    if ((pwm_pos_set[NUM_EXTRUDER] = pwm_pos[NUM_EXTRUDER]) > 0) WRITE(HEATED_BED_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
  }
  if (pwm_count_cooler == 0 && !PDM_FOR_COOLER)
  {
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN > -1
    if ((pwm_cooler_pos_set[0] = (extruder[0].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT0_EXTRUDER_COOLER_PIN, 1);
#endif
#if !SHARED_COOLER && defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
#if EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN
    if ((pwm_cooler_pos_set[1] = (extruder[1].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT1_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2
#if EXT2_EXTRUDER_COOLER_PIN>-1
    if ((pwm_cooler_pos_set[2] = (extruder[2].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT2_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3
#if EXT3_EXTRUDER_COOLER_PIN>-1
    if ((pwm_cooler_pos_set[3] = (extruder[3].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT3_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4
#if EXT4_EXTRUDER_COOLER_PIN>-1
    if ((pwm_cooler_pos_set[4] = (extruder[4].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT4_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5
#if EXT5_EXTRUDER_COOLER_PIN>-1
    if ((pwm_cooler_pos_set[5] = (extruder[5].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT5_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if FAN_BOARD_PIN > -1 && SHARED_COOLER_BOARD_EXT == 0
        if((pwm_pos_set[PWM_BOARD_FAN] = (pwm_pos[PWM_BOARD_FAN] & COOLER_PWM_MASK)) > 0) WRITE(FAN_BOARD_PIN,1);
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
        if((pwm_pos_set[PWM_FAN1] = (pwm_pos[PWM_FAN1] & COOLER_PWM_MASK)) > 0) WRITE(FAN_PIN,1);
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
		if((pwm_pos_set[PWM_FAN2] = (pwm_pos[PWM_FAN2] & COOLER_PWM_MASK)) > 0) WRITE(FAN2_PIN,1);
#endif
#if defined(FAN_THERMO_PIN) && FAN_THERMO_PIN > -1
		if((pwm_pos_set[PWM_FAN_THERMO] = (pwm_pos[PWM_FAN_THERMO] & COOLER_PWM_MASK)) > 0) WRITE(FAN_THERMO_PIN,1);
#endif
  }
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1
#if PDM_FOR_EXTRUDER
  pulseDensityModulate(EXT0_HEATER_PIN, pwm_pos[0], pwm_pos_set[0], HEATER_PINS_INVERTED);
#else
  if (pwm_pos_set[0] == pwm_count_heater && pwm_pos_set[0] != HEATER_PWM_MASK) WRITE(EXT0_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if EXT0_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
  pulseDensityModulate(EXT0_EXTRUDER_COOLER_PIN, extruder[0].coolerPWM, pwm_cooler_pos_set[0], false);
#else
  if (pwm_cooler_pos_set[0] == pwm_count_cooler && pwm_cooler_pos_set[0] != COOLER_PWM_MASK) WRITE(EXT0_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
  pulseDensityModulate(EXT1_HEATER_PIN, pwm_pos[1], pwm_pos_set[1], HEATER_PINS_INVERTED);
#else
  if (pwm_pos_set[1] == pwm_count_heater && pwm_pos_set[1] != HEATER_PWM_MASK) WRITE(EXT1_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN
#if PDM_FOR_COOLER
  pulseDensityModulate(EXT1_EXTRUDER_COOLER_PIN, extruder[1].coolerPWM, pwm_cooler_pos_set[1], false);
#else
  if (pwm_cooler_pos_set[1] == pwm_count_cooler && pwm_cooler_pos_set[1] != COOLER_PWM_MASK) WRITE(EXT1_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
  pulseDensityModulate(EXT2_HEATER_PIN, pwm_pos[2], pwm_pos_set[2], HEATER_PINS_INVERTED);
#else
  if (pwm_pos_set[2] == pwm_count_heater && pwm_pos_set[2] != HEATER_PWM_MASK) WRITE(EXT2_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT2_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
  pulseDensityModulate(EXT2_EXTRUDER_COOLER_PIN, extruder[2].coolerPWM, pwm_cooler_pos_set[2], false);
#else
  if (pwm_cooler_pos_set[2] == pwm_count_cooler && pwm_cooler_pos_set[2] != COOLER_PWM_MASK) WRITE(EXT2_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
  pulseDensityModulate(EXT3_HEATER_PIN, pwm_pos[3], pwm_pos_set[3], HEATER_PINS_INVERTED);
#else
  if (pwm_pos_set[3] == pwm_count_heater && pwm_pos_set[3] != HEATER_PWM_MASK) WRITE(EXT3_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT3_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
  pulseDensityModulate(EXT3_EXTRUDER_COOLER_PIN, extruder[3].coolerPWM, pwm_cooler_pos_set[3], false);
#else
  if (pwm_cooler_pos_set[3] == pwm_count_cooler && pwm_cooler_pos_set[3] != COOLER_PWM_MASK) WRITE(EXT3_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
  pulseDensityModulate(EXT4_HEATER_PIN, pwm_pos[4], pwm_pos_set[4], HEATER_PINS_INVERTED);
#else
  if (pwm_pos_set[4] == pwm_count_heater && pwm_pos_set[4] != HEATER_PWM_MASK) WRITE(EXT4_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT4_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
  pulseDensityModulate(EXT4_EXTRUDER_COOLER_PIN, extruder[4].coolerPWM, pwm_cooler_pos_set[4], false);
#else
  if (pwm_cooler_pos_set[4] == pwm_count_cooler && pwm_cooler_pos_set[4] != COOLER_PWM_MASK) WRITE(EXT4_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
  pulseDensityModulate(EXT5_HEATER_PIN, pwm_pos[5], pwm_pos_set[5], HEATER_PINS_INVERTED);
#else
  if (pwm_pos_set[5] == pwm_count_heater && pwm_pos_set[5] != HEATER_PWM_MASK) WRITE(EXT5_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT5_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
  pulseDensityModulate(EXT5_EXTRUDER_COOLER_PIN, extruder[5].coolerPWM, pwm_cooler_pos_set[5], false);
#else
  if (pwm_cooler_pos_set[5] == pwm_count_cooler && pwm_cooler_pos_set[5] != COOLER_PWM_MASK) WRITE(EXT5_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if FAN_BOARD_PIN > -1  && SHARED_COOLER_BOARD_EXT == 0
#if PDM_FOR_COOLER
    pulseDensityModulate(FAN_BOARD_PIN, pwm_pos[PWM_BOARD_FAN], pwm_pos_set[PWM_BOARD_FAN], false);
#else
    if(pwm_pos_set[PWM_BOARD_FAN] == pwm_count_cooler && pwm_pos_set[NUM_EXTRUDER + 1] != COOLER_PWM_MASK) WRITE(FAN_BOARD_PIN,0);
#endif
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    if(fanKickstart == 0)
    {
#if PDM_FOR_COOLER
        pulseDensityModulate(FAN_PIN, pwm_pos[PWM_FAN1], pwm_pos_set[PWM_FAN1], false);
#else
        if(pwm_pos_set[PWM_FAN1] == pwm_count_cooler && pwm_pos_set[PWM_FAN1] != COOLER_PWM_MASK) WRITE(FAN_PIN,0);
#endif
    }
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
if(fan2Kickstart == 0)
{
	#if PDM_FOR_COOLER
	pulseDensityModulate(FAN2_PIN, pwm_pos[PWM_FAN2], pwm_pos_set[PWM_FAN2], false);
	#else
	if(pwm_pos_set[PWM_FAN2] == pwm_count_cooler && pwm_pos_set[PWM_FAN2] != COOLER_PWM_MASK) WRITE(FAN2_PIN,0);
	#endif
}
#endif
#if defined(FAN_THERMO_PIN) && FAN_THERMO_PIN > -1
	#if PDM_FOR_COOLER
	pulseDensityModulate(FAN_THERMO_PIN, pwm_pos[PWM_FAN_THERMO], pwm_pos_set[PWM_FAN_THERMO], false);
	#else
	if(pwm_pos_set[PWM_FAN_THERMO] == pwm_count_cooler && pwm_pos_set[PWM_FAN_THERMO] != COOLER_PWM_MASK) WRITE(FAN_THERMO_PIN,0);
	#endif
#endif
#if HEATED_BED_HEATER_PIN > -1 && HAVE_HEATED_BED
#if PDM_FOR_EXTRUDER
  pulseDensityModulate(HEATED_BED_HEATER_PIN, pwm_pos[NUM_EXTRUDER], pwm_pos_set[NUM_EXTRUDER], HEATER_PINS_INVERTED);
#else
  if (pwm_pos_set[NUM_EXTRUDER] == pwm_count_heater && pwm_pos_set[NUM_EXTRUDER] != HEATER_PWM_MASK) WRITE(HEATED_BED_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#endif
  //noInt.unprotect();
  counterPeriodical++; // Appxoimate a 100ms timer
  if (counterPeriodical >= 390) //  (int)(F_CPU/40960))
  {
    counterPeriodical = 0;
    executePeriodical = 1;
#if FEATURE_FAN_CONTROL
    if (fanKickstart) fanKickstart--;
#endif
#if FEATURE_FAN2_CONTROL
    if (fan2Kickstart) fan2Kickstart--;
#endif
  }
  // read analog values -- only read one per interrupt
#if ANALOG_INPUTS > 0
  // conversion finished?
  //if ((ADC->ADC_ISR & adcEnable) == adcEnable)
  {
    adcCounter++;
    for (int i = 0; i < ANALOG_INPUTS; i++) {
      int32_t cur = analogRead(osAnalogInputChannels[i]);
      osAnalogInputBuildup[i] += cur;
      adcSamplesMin[i] = RMath::min(adcSamplesMin[i], cur);
      adcSamplesMax[i] = RMath::max(adcSamplesMax[i], cur);
      if (adcCounter >= NUM_ADC_SAMPLES)     // store new conversion result
      {
        // Strip biggest and smallest value and round correctly
        osAnalogInputBuildup[i] = osAnalogInputBuildup[i] + (1 << (ANALOG_INPUT_SAMPLE - 1)) - (adcSamplesMin[i] + adcSamplesMax[i]);
        adcSamplesMin[i] = 100000;
        adcSamplesMax[i] = 0;
        osAnalogSamplesSum[i] -= osAnalogSamples[i][adcSamplePos];
        osAnalogSamplesSum[i] += (osAnalogSamples[i][adcSamplePos] = osAnalogInputBuildup[i] >> ANALOG_INPUT_SAMPLE);
        osAnalogInputValues[i] = osAnalogSamplesSum[i] / ANALOG_INPUT_MEDIAN;
        osAnalogInputBuildup[i] = 0;
      } // adcCounter >= NUM_ADC_SAMPLES
    } // for i
    if (adcCounter >= NUM_ADC_SAMPLES) {
      adcCounter = 0;
      adcSamplePos++;
      if (adcSamplePos >= ANALOG_INPUT_MEDIAN)
        adcSamplePos = 0;
    }
  }
#endif // ANALOG_INPUTS > 0
  pwm_count_cooler += COOLER_PWM_STEP;
  pwm_count_heater += HEATER_PWM_STEP;
  UI_FAST; // Short timed user interface action
#if FEATURE_WATCHDOG
  if(HAL::wdPinged) {
     //WDT->WDT_CR = 0xA5000001;
     HAL::wdPinged = false;
  }
#endif
#endif
  return 80000000/PWM_CLOCK_FREQ;
}

/** \brief Timer routine for extruder stepper.

Several methods need to move the extruder. To get a optimal
result, all methods update the printer_state.extruderStepsNeeded
with the number of additional steps needed. During this
interrupt, one step is executed. This will keep the extruder
moving, until the total wanted movement is achieved. This will
be done with the maximum allowable speed for the extruder.
*/
#if USE_ADVANCE
//TcChannel *extruderChannel = (EXTRUDER_TIMER->TC_CHANNEL + EXTRUDER_TIMER_CHANNEL);
#define SLOW_EXTRUDER_TICKS  (F_CPU_TRUE / 32 / 1000) // 250us on direction change
#define NORMAL_EXTRUDER_TICKS  (F_CPU_TRUE / 32 / EXTRUDER_CLOCK_FREQ) // 500us on direction change
#ifndef ADVANCE_DIR_FILTER_STEPS
#define ADVANCE_DIR_FILTER_STEPS 2
#endif

static int extruderLastDirection = 0;
void HAL::resetExtruderDirection() {
  extruderLastDirection = 0;
}
// EXTRUDER_TIMER IRQ handler
int EXTRUDER_TIMER_VECTOR ()
{
  int delay;
  InterruptProtectedBlock noInt;
  // apparently have to read status register
 // TC_GetStatus(EXTRUDER_TIMER, EXTRUDER_TIMER_CHANNEL);
  //extruderChannel->TC_SR; // faster replacement for above line!
  
  if (!Printer::isAdvanceActivated()) {
    return NORMAL_EXTRUDER_TICKS; // currently no need
  }
  if (!Printer::isAdvanceActivated()) return NORMAL_EXTRUDER_TICKS; // currently no need
  if (Printer::extruderStepsNeeded > 0 && extruderLastDirection != 1)
  {
    if(Printer::extruderStepsNeeded >= ADVANCE_DIR_FILTER_STEPS) {
      Extruder::setDirection(true);
      extruderLastDirection = 1;
      //extruderChannel->TC_RC = SLOW_EXTRUDER_TICKS;
      delay = Printer::maxExtruderSpeed;
    } else { 
      delay = Printer::maxExtruderSpeed;
    }
  }
  else if (Printer::extruderStepsNeeded < 0 && extruderLastDirection != -1)
  {
    if(-Printer::extruderStepsNeeded >= ADVANCE_DIR_FILTER_STEPS) {
      Extruder::setDirection(false);
      extruderLastDirection = -1;
      //extruderChannel->TC_RC = SLOW_EXTRUDER_TICKS;
      delay = Printer::maxExtruderSpeed;
   } else { 
      delay = Printer::maxExtruderSpeed;
   }
  } 
  else if (Printer::extruderStepsNeeded != 0)
  {
    Extruder::step();
    Printer::extruderStepsNeeded -= extruderLastDirection;
    delay = Printer::maxExtruderSpeed;
    Printer::insertStepperHighDelay();
    Extruder::unstep();
  }
  return delay;
}
#endif

// IRQ handler for tone generator
void BEEPER_TIMER_VECTOR () {
}

#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
RFDoubleSerial::RFDoubleSerial() {
}
void RFDoubleSerial::begin(unsigned long baud) {
  RFSERIAL.begin(baud);
  BT_SERIAL.begin(BLUETOOTH_BAUD);
}
void RFDoubleSerial::end() {
  RFSERIAL.end();
  BT_SERIAL.end();
}
int RFDoubleSerial::available(void) {
  int x = RFSERIAL.available();
  if (x > 0) return x;
  return BT_SERIAL.available();
}
int RFDoubleSerial::peek(void) {
  if(RFSERIAL.available())
    return RFSERIAL.peek();
  return BT_SERIAL.peek();
}
int RFDoubleSerial::read(void) {
  if(RFSERIAL.available())
    return RFSERIAL.read();
  return BT_SERIAL.read();
}
void RFDoubleSerial::flush(void) {
  RFSERIAL.flush();
  BT_SERIAL.flush();
}
size_t RFDoubleSerial::write(uint8_t c) {
  size_t r = RFSERIAL.write(c);
  BT_SERIAL.write(c);
  return r;
}
RFDoubleSerial BTAdapter;
#endif

// Dummy function to overload weak arduino function that always disables
// watchdog. We do not need that as we do this our self.
void watchdogSetup(void) {
}


uint32_t nextPwmInt;
uint32_t nextServoInt;
uint32_t nextStepperInt;
uint32_t nextExtruderInt;

int intCount = 0;
int stepperIntCount = 0;
int minStepTimeout = 10000000;
int pwmIntCount = 0;
int extruderIntCount = 0;
static int pinState[17];
static int pinToggles[17];

void dumpDebug(void) {
  Serial.print("Stats: ");
  Serial.print(millis());
  Serial.print(" "); Serial.print(F_CPU);
  Serial.print(" "); Serial.print(intCount);
  Serial.print(" "); Serial.print(pwmIntCount);
  Serial.print(" "); Serial.print(stepperIntCount);
  Serial.print(" "); Serial.print(extruderIntCount);
  Serial.print(" "); Serial.print(minStepTimeout);
  Serial.println("");
  
  for(int i = 0; i <= 16; i++) {
    Serial.print(pinState[i]);
    Serial.print(":");
    Serial.print(pinToggles[i]);
    Serial.print(" ");
  }
  Serial.println("");
}



#define MAX_POSITIVE 80000000U

void interrupt_handler(void) {
  uint32_t nextInt;
  uint32_t now = ESP.getCycleCount();
  intCount++;
#if FEATURE_SERVO
  if(now - nextServoInt < MAX_POSITIVE) {
     nextServoInt += SERVO_COMPA_VECTOR();
  }
#endif
  if(now - nextStepperInt < MAX_POSITIVE) {
    uint32_t t = TIMER1_COMPA_VECTOR();
    if(t < minStepTimeout) minStepTimeout = t;
     nextStepperInt += t;
     stepperIntCount++;
  }
  if(now - nextPwmInt < MAX_POSITIVE) {
     pwmIntCount++;
     nextPwmInt += PWM_TIMER_VECTOR();
   }
#if USE_ADVANCE
  // 60kHz
  if(now - nextExtruderInt < MAX_POSITIVE) {
     extruderIntCount++;
     nextExtruderInt += EXTRUDER_TIMER_VECTOR();
  }
#endif
#if 0
  if(now - nextBeeperInt < MAX_POSITIVE) {
     nextBeeperInt += BEEPER_TIMER_VECTOR();
  }
#endif

   nextInt = nextStepperInt;
   
   if(nextPwmInt - nextInt > MAX_POSITIVE)
      nextInt = nextPwmInt;
   if(nextInt - 100 - ESP.getCycleCount() > MAX_POSITIVE) 
      nextInt = ESP.getCycleCount() + 100;
   timer0_write(nextInt);
}

// Set up all timer interrupts
void HAL::setupTimer() {
  noInterrupts();
  nextPwmInt = ESP.getCycleCount() + 10000;
  nextServoInt = ESP.getCycleCount() + 12000;
  nextStepperInt = ESP.getCycleCount() + 14000;
  nextExtruderInt = ESP.getCycleCount() + 16000;  
  timer0_isr_init();
  timer0_attachInterrupt(interrupt_handler);
  timer0_write(ESP.getCycleCount() + 10);
  interrupts();
}


void myDigitalWrite(int pin, int val) {
  if(pinState[pin] != val) {
    pinState[pin] = val;
    pinToggles[pin]++;
  }
}

