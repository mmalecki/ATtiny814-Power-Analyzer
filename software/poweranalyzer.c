// ===================================================================================
// Project:   Power Analyzer
// Version:   1.1
// Year:      2020
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// Programmable electronic constant current dummy load with two high side voltage
// and current sensors for an automatic analysis of power supplys, DC/DC converters,
// voltage regulators, batteries, chargers, power consumers and others.
// The device can be controlled via USB serial interface using a serial monitor or
// the provided python skripts. Data can be exported to spread sheet programs or
// directly be analyzed by the python skripts.
//
// Wiring:
// -------
//                            +-\/-+
//                      Vcc  1|°   |14  GND
//     Fan --- !SS AIN4 PA4  2|    |13  PA3 AIN3 SCK ---- 
//     NTC ------- AIN5 PA5  3|    |12  PA2 AIN2 MISO --- 
//    Load --- DAC AIN6 PA6  4|    |11  PA1 AIN1 MOSI --- 
//     LED ------- AIN7 PA7  5|    |10  PA0 AIN0 UPDI --- UPDI
//     USB -------- RXD PB3  6|    |9   PB0 AIN11 SCL --- INA219
//     USB ---------TXD PB2  7|    |8   PB1 AIN10 SDA --- INA219
//                            +----+
//
// Compilation Settings:
// ---------------------
// Core:    megaTinyCore (https://github.com/SpenceKonde/megaTinyCore)
// Board:   ATtiny1614/1604/814/804/414/404/214/204
// Chip:    ATtiny1614 or ATtiny814
// Clock:   20 MHz internal
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// Compile and upload the code.
//
// No Arduino core functions or libraries are used. To compile and upload without
// Arduino IDE download AVR 8-bit toolchain at:
// https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
// and extract to tools/avr-gcc. Use the makefile to compile and upload.
//
// Fuse Settings: 0:0x00 1:0x00 2:0x02 4:0x00 5:0xC5 6:0x04 7:0x00 8:0x00
//
// ===================================================================================
// Operating Instructions
// ===================================================================================
// Load Test:
//
// TEST-IN <---- Test Power Supply
// PWR-OUT --X-- unconnected
// PWR-IN  --X-- unconnected
//
// Command: "l <maxloadcurrent[mA: 17..5000]> <minloadvoltage[mV: 0..26000]>"
// Example: "l 1000 2900"
// The Power Analyzer continuously increases the load from 17 mA up to <maxloadcurrent>.
// It stops automatically if the voltage drops below <minloadvoltage>. It continuously
// transmits the measured values via the serial interface in the format: 
// current[mA] voltage[mV] power[mW] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Voltage Regulation Test:
//
// TEST-IN <---- Test Power Supply
// PWR-OUT --X-- unconnected
// PWR-IN  --X-- unconnected
//
// Command: "g <maxloadcurrent[mA: 17..5000]>"
// Example: "g 3000"
// The Power Analyzer changes rapidly the load between 17 mA and <maxloadcurrent>.
// It continuously transmits the measured values via the serial interface in the format: 
// time[ms] current[mA] voltage[mV] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Efficiency Test:
//
// TEST-IN <---- DC/DC Converter Output
// PWR-OUT ----> DC/DC Converter Input
// PWR-IN  <---- Power Source
//
// Command: "e <maxloadcurrent[mA: 17..5000]> <minloadvoltage[mV: 0..26000]>"
// Example: "e 1000 4700"
// The Power Analyzer continuously increases the load from 17 mA up to <maxloadcurrent>.
// It stops automatically if the voltage at TEST-IN drops below <minloadvoltage>. It 
// continuously transmits the measured values via the serial interface in the format: 
// current[mA] voltage[mV] efficiency[% * 10] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Low Frequency Ripple Test:
//
// TEST-IN <---- Test Power Supply
// PWR-OUT --X-- unconnected
// PWR-IN  --X-- unconnected
//
// Command: "f <maxloadcurrent[mA: 17..5000]> <minloadvoltage[mV: 0..26000]>"
// Example: "f 1000 4700"
// The Power Analyzer continuously increases the load from 17 mA up to <maxloadcurrent>.
// It stops automatically if the voltage at TEST-IN drops below <minloadvoltage>. It 
// continuously transmits the measured values via the serial interface in the format: 
// current[mA] peak-to-peak voltage[mV] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Battery Discharge Test:
//
// TEST-IN <---- Battery
// PWR-OUT --X-- unconnected
// PWR-IN  --X-- unconnected
//
// Command: "b <maxloadcurrent[mA: 17..5000]> <minloadvoltage[mV: 0..26000]>"
// Example: "l 1000 2900"
// The Power Analyzer sets a constant current load of <maxloadcurrent>. If the voltage
// drops below <minloadvoltage> it constantly decreases the load to maintain
// <minloadvoltage>. It stops automatically if the load current drops to 0mA. It
// continuously transmits the measured values via the serial interface in the format: 
// time[s] current[mA] voltage[mV] capacity[mAh] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Long-Term Multimeter:
//
// TEST-IN --X-- unconnected
// PWR-OUT ----> Power Consumer
// PWR-IN  <---- Power Source
//
// Command: "m <interval[ms: 2..65535]> <duration[s: 1..65535]>"
// Example: "m 1000 600"
// The Power Analyzer measures voltage, current and power delivered to the test device at
// every <interval> for a total of <duration>. It continuously transmits the measured
// values via the serial interface in the format: 
// time[ms] current[mA] voltage[mV] (seperated by the SEPERATOR string).
// ---------------------------------------------------------------------------------------
// Calibration:
//
// Set all calibration values in the sketch to 1, compile and upload.
// Connect a power supply to PWR-IN and connect PWR-OUT to TEST-IN with an ammeter in 
// between the positive line. Connect the voltmeter to the TEST-IN terminals. Make sure
// not to exceed the maximum load power.
//
// TEST-IN <---- |Multi-
// PWR-OUT ----> |meter
// PWR-IN  <---- Power Source
//
// Command: "c <loadcurrent[mA: 17..5000]> <duration[s: 1..65535]>"
// Example: "c 1000 10"
// The Power Analyzer sets a constant current load of <loadcurrent> for <duration>.
// It averages the measurements if both voltage/current sensors and transmits the
// values via the serial interface in the format: 
// current_load[mA] voltage_load[mV] current_power[mA] voltage_power[mV] (seperated by the SEPERATOR string)
// Measure voltage and current with a trusty multimeter during this time and calculate
// the calibration values as follows:
// ILCAL1 = current measured with multimeter / transmitted value of current_load
// ULCAL1 = voltage measured with multimeter / transmitted value of voltage_load
// ILCAL2 = current measured with multimeter / transmitted value of current_power
// ULCAL2 = voltage measured with multimeter / transmitted value of voltage_power
// Change the calibration values in the sketch, compile and upload.
// ---------------------------------------------------------------------------------------
// Commands for Direct Control:
//
// Command                Function
// "i"                    transmit indentification string ("Power Analyzer")
// "v"                    transmit firmware version number
// "x"                    terminate current test program
// "s <loadcurrent[mA]>"  set load to a constant current of <loadcurrent>
// "p <loadpower [mW]>"   set load to a constant power of <loadpower> (not yet implemented)
// "o <resistance [Ohm]>" set load to a constant resistance of <resistance> (nyi)
// "r"                    reset the load to minimum
// "t"                    read current and voltage of both sensors and transmit them
// ---------------------------------------------------------------------------------------
// Notes:
// - Use a good heatsink with a 5V fan for the MOSFET!
// - Be careful with high power loads! Make some tests to figure out what can be achieved
//   with your cooling solution!
// - Due to the limitations of the OpAmp the minimum load current is around 17mA.
//   You can choose a better OpAmp if you like, but for most cases this is not neccessary.
// - The maximum load current is 5A, however for small voltages it might be less.
// - The maximum PWR-IN/PWR-OUT current is 8A.
// - Do not exceed the maximum voltage of 26V on all connectors !
// - In order to make the design much simpler all connectors including USB have a common
//   ground. Keep this in mind when making your test setup in order to avoid ground loops
//   or shorts. Using a USB isolator between the Analyzer and your PC is not a bad idea!  


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <avr/io.h>                     // for GPIO
#include <avr/interrupt.h>              // for interrupts
#include <util/delay.h>                 // for delays
#include <string.h>
#include <math.h>
#include <stdlib.h>

// Pin definitions
#define FAN_PIN     PA4                 // pin the fan is connected to
#define NTC_PIN     PA5                 // pin the temperature sensor is connected to
#define DAC_PIN     PA6                 // DAC output pin, connected to load
#define LED_PIN     PA7                 // pin the status LED is connected to
#define SCL_PIN     PB0                 // I2C serial clock pin, connected to INA219
#define SDA_PIN     PB1                 // I2C serial data pin, connected to INA219
#define TXD_PIN     PB2                 // UART transmit data pin, connected to CH330N
#define RXD_PIN     PB3                 // UART receive data pin, connected to CH330N

// Identifiers
#define VERSION     "2.0"               // version number sent via serial if requested
#define IDENT       "Power Analyzer"    // identifier sent via serial if requested
#define SEPARATOR   '\t'                // seperator string for serial communication
#define DONE        "DONE"              // string sent when operation has finished
#define ERROR       "ERROR"
#define OVERLOAD    "OVERLOAD"

// Calibration values
#define ILCAL1      1                   // linear current calibration factor (load)
#define ULCAL1      1                   // linear voltage calibration factor (load)
#define ILCAL2      1                   // linear current calibration factor (supply)
#define ULCAL2      1                   // linear voltage calibration factor (supply)

// Parameters

// XXX(mmalecki): we're rated to 5 A and technically could do much, much more, given
// an appropriate cooling solution. However, INA219's maximum sense voltage and the
// current shunt resistors in place limit our capabilities.
// We could set a divider (PGA) on INA side, or change out the resistors. For the
// time being, however, limit the maximum current to a unit under, so the controller
// is able to operate.
#define MAXCURRENT  4999
#define MAXPOWER    25000               // maximum power of the load in mW -> turn off load

#define FANONPOWER  700                 // power in mW to turn fan on
#define FANOFFPOWER 500                 // power in mW to turn fan off
#define MAXTEMP     85                  // max temperature
#define FANONTEMP   50                  // fan-on temperature
#define FANOFFTEMP  45                  // fan-off temperature

#undef CONTROLLER_DEBUG

#define DAC_MIN 0
#define DAC_MAX 255

#define DAC_LOW_CURRENT_P   0.075
#define DAC_LOW_CURRENT     1000
#define DAC_HIGH_CURRENT_P  0.05

/* #define DAC_MAX_CURRENT 7500 */
#define DAC_MAX_CHANGE 20.0

// NTC probe
#define _K 273.15
#define C_TO_K(X) (X + 273.15)
#define K_TO_C(X) (X - 273.15)

#define NTC_BETA    3977.0
#define NTC_R0      10000.0
#define NTC_T0      C_TO_K(25.0) // reference temperature [K]
#define NTC_DIV_R1  10000.0
#define ADC_MAX     1023.0

#define STEP        50    // control loop time step, ms

#define ADJ_STEP    500  // adjustment step, ms: how often to adjust current in test programs
#define CURRENT_ADJ 10

#define REPORT_STEP 50  // report time step, ms: how often to report

#define BLINK_SHORT_INFO  250
#define BLINK_INFO        500
#define BLINK_ERROR       1250

// INA219 register values
#define INA1ADDR    0b10000000          // I2C write address of INA on the load side 
#define INA2ADDR    0b10000010          // I2C write address of INA on the power side

#define INA1CONFIG  0b0010101001100111  // INA config register according to datasheet
#define INA2CONFIG  0b0010111001100111  // INA config register according to datasheet
#define INAFASTBUS  0b0010110001000110  // INA config register according to datasheet
#define INAFASTBOTH 0b0010110001000111  // INA config register according to datasheet
#define INACALIB    5120                // INA calibration register according to R_SHUNT
#define CONFIG_REG  0x00                // INA configuration register address
#define CALIB_REG   0x05                // INA calibration register address
#define SHUNT_REG   0x01                // INA shunt voltage register address
#define VOLTAGE_REG 0x02                // INA bus voltage register address
#define POWER_REG   0x03                // INA power register address
#define CURRENT_REG 0x04                // INA current register address

// Pin manipulation macros
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB3};    // enumerate pin designators
#define pinInput(x)       (&VPORTA.DIR)[((x)&8)>>1] &= ~(1<<((x)&7))  // set pin to INPUT
#define pinOutput(x)      (&VPORTA.DIR)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to OUTPUT
#define pinLow(x)         (&VPORTA.OUT)[((x)&8)>>1] &= ~(1<<((x)&7))  // set pin to LOW
#define pinHigh(x)        (&VPORTA.OUT)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to HIGH
#define pinToggle(x)      (&VPORTA.IN )[((x)&8)>>1] |=  (1<<((x)&7))  // TOGGLE pin
#define pinRead(x)        ((&VPORTA.IN)[((x)&8)>>1] &   (1<<((x)&7))) // READ pin
#define pinDisable(x)     (&PORTA.PIN0CTRL)[(((x)&8)<<2)+((x)&7)] |= PORT_ISC_INPUT_DISABLE_gc
#define pinPullup(x)      (&PORTA.PIN0CTRL)[(((x)&8)<<2)+((x)&7)] |= PORT_PULLUPEN_bm
#define pinAIN(x)         ((x)<8 ? (x) : (19-(x)))                    // convert pin to ADC port

#define MAX(a, b)         (a > b ? a : b)
#define MIN(a, b)         (a < b ? a : b)

#define ledBlink(INTERVAL) ledToggle(); \
                           _delay_ms(INTERVAL); \
                           ledToggle(); \
                           _delay_ms(INTERVAL);

#define ledBusy() pinHigh(LED_PIN)
#define ledReady() pinLow(LED_PIN)
#define ledToggle() pinToggle(LED_PIN)

typedef enum { FAN_AUTO, FAN_MANUAL } fan_control_t;
typedef enum { DAC_AUTO, DAC_MANUAL } dac_control_t;

// Global variables (voltages in mV, currents in mA, power in mW, shunts in 0.01 mV)
uint16_t current_desired = 0;
uint16_t voltage_load, current_load, voltage_power, current_power, shunt1, shunt2, power2, loadtemp = 0;

bool query = false;
uint16_t argument1, argument2;
uint16_t loadpower = 0;
char     cmd;

fan_control_t fan_control = FAN_AUTO;
dac_control_t dac_control = DAC_AUTO;

// ===================================================================================
// UART Implementation - Low Level Functions (8N1, no calibration, with RX-interrupt)
// ===================================================================================

// UART definitions and macros
#define UART_BAUD         115200
#define UART_BAUD_RATE    ((float)(F_CPU * 64 / (16 * (float)UART_BAUD)) + 0.5)
#define UART_ready()      (USART0.STATUS & USART_DREIF_bm)

#undef UART_LOOPBACK
#undef UART_DEBUG

// UART init
void UART_init(void) {
  pinInput(RXD_PIN);                              // set RX pin as input
  pinOutput(TXD_PIN);                             // set TX pin as output
  USART0.BAUD   = UART_BAUD_RATE;                 // set BAUD
  USART0.CTRLA  = USART_RXCIE_bm;                 // enable RX interrupt
  USART0.CTRLB  = USART_RXEN_bm                   // enable RX
                | USART_TXEN_bm;                  // enable TX
}

// UART transmit data byte
void UART_write(char data) {
  while(!UART_ready());                           // wait until ready for next data
  USART0.TXDATAL = data;                          // send data byte
}

// ===================================================================================
// UART Implementation - String Functions
// ===================================================================================

// UART print string
void UART_print(const char *str) {
  for (size_t i = 0; i < strlen(str); i++)  { UART_write(str[i]); }
}

// UART print string with new line
void UART_println(const char *str) {
  UART_print(str);                                // print string
  UART_write('\r');                               // send new line command
  UART_write('\n');                               // send new line command
}

// UART print 16-bit integer as decimal (BCD conversion by substraction method)
void UART_printInt(uint16_t value) {
  static uint16_t divider[5] = {10000, 1000, 100, 10, 1};
  uint8_t leadflag = 0;                           // flag for leading spaces
  for(uint8_t digit = 0; digit < 5; digit++) {    // 5 digits
    uint8_t digitval = 0;                         // start with digit value 0
    while(value >= divider[digit]) {              // if current divider fits into the value
      leadflag = 1;                               // end of leading spaces
      digitval++;                                 // increase digit value
      value -= divider[digit];                    // decrease value by divider
    }
    if(leadflag || (digit == 4)) UART_write(digitval + '0'); // print the digit
  }
}

// ===================================================================================
// UART Implementation - Command Buffer
// ===================================================================================

// UART command buffer and pointer
#define CMD_BUF_LEN 16                            // command buffer length
volatile uint8_t CMD_buffer[CMD_BUF_LEN] = { 0 };         // command buffer
volatile uint8_t CMD_ptr = 0;                     // buffer pointer for writing
volatile uint8_t CMD_compl = 0;                   // command completely received flag

void ledError() {
  ledBlink(BLINK_ERROR);
  ledBlink(BLINK_ERROR);
}

// UART RXC interrupt service routine (read command via UART)
ISR(USART0_RXC_vect) {
  uint8_t data = USART0.RXDATAL;                  // read received data byte

#ifdef UART_DEBUG
  // 1 blink for each character in the buffer:
  for (int i = 0; i <= CMD_ptr; i++) ledBlink(BLINK_INFO);
#endif

  if(!CMD_compl) {                                // command still incomplete?
    if(data != '\n' && data != '\r') {            // not command end?
#ifdef UART_LOOPBACK
      UART_write(data);
#endif

      CMD_buffer[CMD_ptr] = data;                 // write received byte to buffer
      if(CMD_ptr < (CMD_BUF_LEN-1)) CMD_ptr++;    // increase and limit pointer
    } else if(CMD_ptr > 0) {                      // received at least one byte?
#ifdef UART_DEBUG
      // 3 short blinks for end of command:
      ledBlink(BLINK_SHORT_INFO);
      ledBlink(BLINK_SHORT_INFO);
      ledBlink(BLINK_SHORT_INFO);
#endif
      CMD_compl = 1;                              // set command complete flag
      CMD_buffer[CMD_ptr] = 0;                    // write string terminator
      CMD_ptr = 0;                                // reset pointer
#ifdef UART_LOOPBACK
      UART_write('\r');
      UART_write('\n');
#endif
    }
  }  
}

// ===================================================================================
// I2C Master Implementation (Read/Write, Conservative)
// ===================================================================================

#define I2C_FREQ  400000                          // I2C clock frequency in Hz
#define I2C_BAUD  ((F_CPU / I2C_FREQ) - 10) / 2;  // simplified BAUD calculation

// I2C init function
void I2C_init(void) {
  TWI0.MBAUD   = I2C_BAUD;                        // set TWI master BAUD rate
  TWI0.MCTRLA  = TWI_ENABLE_bm;                   // enable TWI master
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;            // set bus state to idle
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  TWI0.MADDR = addr;                              // start sending address
  while(!(TWI0.MSTATUS&(TWI_WIF_bm|TWI_RIF_bm))); // wait for transfer to complete
}

// I2C restart transmission
void I2C_restart(uint8_t addr) {
  I2C_start(addr);                                // start sending address
}

// I2C stop transmission
void I2C_stop(void) {
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;                 // send stop condition
}

// I2C transmit one data byte to the slave, ignore ACK bit
void I2C_write(uint8_t data) {
  TWI0.MDATA = data;                              // start sending data byte
  while(~TWI0.MSTATUS & TWI_WIF_bm);              // wait for transfer to complete
}

// I2C receive one data byte from slave; ack=0: last byte, ack>0: more bytes to follow
uint8_t I2C_read(uint8_t ack) {
  while(~TWI0.MSTATUS & TWI_RIF_bm);              // wait for transfer to complete
  uint8_t data = TWI0.MDATA;                      // get received data byte
  if(ack) TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;    // ACK:  read more bytes
  else    TWI0.MCTRLB = TWI_ACKACT_NACK_gc;       // NACK: this was the last byte
  return data;                                    // return received byte
}

// ===================================================================================
// INA219 Implementation
// ===================================================================================
// Writes  register value to the INA219
void INA_write(uint8_t addr, uint8_t reg, uint16_t value) {
  I2C_start(addr);
  I2C_write(reg);
  I2C_write(value >> 8);
  I2C_write(value);
  I2C_stop();
}

// Read a register from the INA219
uint16_t INA_read(uint8_t addr, uint8_t reg) {
  I2C_start(addr);
  I2C_write(reg);
  I2C_restart(addr | 1);
  uint16_t result = (uint16_t)(I2C_read(1) << 8) | I2C_read(0);
  I2C_stop();
  return result;
}

// Write inital configuration and calibration values to the INAs
void INA_init(void) {
  INA_write(INA1ADDR, CONFIG_REG, INA1CONFIG);
  INA_write(INA1ADDR, CALIB_REG,  ILCAL1 * INACALIB);
  INA_write(INA2ADDR, CONFIG_REG, INA2CONFIG);
  INA_write(INA2ADDR, CALIB_REG,  ILCAL2 * INACALIB);
}

// ===================================================================================
// Millis Counter Implementation for TCB0
// ===================================================================================

volatile uint32_t MIL_counter = 0;                // millis counter variable

// Init millis counter
void MIL_init(void) {
  TCB0.CCMP    = (F_CPU / 1000) - 1;              // set TOP value (period)
  TCB0.CTRLA   = TCB_ENABLE_bm;                   // enable timer/counter
  TCB0.INTCTRL = TCB_CAPT_bm;                     // enable periodic interrupt
}

// Read millis counter
uint32_t MIL_read(void) {
  cli();                                          // disable interrupt for atomic read
  uint32_t result = MIL_counter;                  // read millis counter
  sei();                                          // enable interrupt again
  return result;                                  // return millis counter value
}

// TCB0 interrupt service routine (every millisecond)
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;                    // clear interrupt flag
  MIL_counter++;                                  // increase millis counter
}

// ===================================================================================
// DAC Implementation and Load Control Functions
// ===================================================================================

// Setup the digital to analog converter (DAC)
void DAC_init(void) {
  // Start up with 1V5, which according to calculations should cover the entire
  // current range well. One could go lower if desired, to achieve higher accuracy
  // over smaller current ranges.
  VREF_CTRLA |= VREF_DAC0REFSEL_1V5_gc;
  VREF_CTRLB |= VREF_DAC0REFEN_bm;                // enable DAC reference
  _delay_us(25);                                  // wait for Vref to start up
  pinDisable(DAC_PIN);                            // disable digital input buffer
  DAC0.DATA = 0;
  DAC0.CTRLA  = DAC_ENABLE_bm                     // enable DAC
              | DAC_OUTEN_bm;                     // enable output buffer
}

void set_current(uint16_t current) {
  dac_control = DAC_AUTO;
  if (current == 0) DAC0.DATA = 0; // short-circuit this particular decision
  current_desired = MIN(current, MAXCURRENT);
}


// ===================================================================================
// ADC Implementation
// ===================================================================================

// ADC init
void ADC_init(void) {
  ADC0.CTRLA  = ADC_ENABLE_bm;                    // enable ADC, 10 bits, single shot
  ADC0.CTRLC  = ADC_SAMPCAP_bm                    // reduced size of sampling capacitance
              | ADC_REFSEL_VDDREF_gc              // set VCC as reference
              | ADC_PRESC_DIV16_gc;               // set prescaler for 1.25 MHz ADC clock
}

// ADC sample and read
uint16_t ADC_read(uint8_t port) {
  ADC0.MUXPOS   = port;                           // set muxer to desired port
  ADC0.INTFLAGS = ADC_RESRDY_bm;                  // clear result ready intflag
  ADC0.COMMAND  = ADC_STCONV_bm;                  // start sampling
  while(~ADC0.INTFLAGS & ADC_RESRDY_bm);          // wait for ADC sampling to complete
  return ADC0.RES;                                // read and return sampling result
}

// ===================================================================================
// Sensors Implementation
// ===================================================================================

void overload() {
  set_current(0);
  fan_control = FAN_AUTO;
  pinHigh(FAN_PIN);
  UART_println(OVERLOAD);
  ledError();
}

// Read all sensors of the electronic load and set the fan
uint8_t updateLoadSensors(void) {
  // Read values from INA on the load side
  shunt1   = INA_read(INA1ADDR, SHUNT_REG);
  if(shunt1 > 4095) shunt1 = 0;
  voltage_load = (INA_read(INA1ADDR, VOLTAGE_REG) >> 1) & 0xfffc;
  voltage_load = ((float)shunt1 / 100 + voltage_load) * ULCAL1 + 0.5;
  current_load = INA_read(INA1ADDR, CURRENT_REG);
  if(current_load > 32767) current_load = 0;

  if (dac_control == DAC_AUTO) {
    if (current_desired == 0) DAC0.DATA = 0;
    else if (current_load != current_desired) {
      int16_t error = current_desired - current_load;
      float p = (current_load < DAC_LOW_CURRENT) ? DAC_LOW_CURRENT_P : DAC_HIGH_CURRENT_P;
      int8_t cs  = (int8_t) MAX(-DAC_MAX_CHANGE, MIN(DAC_MAX_CHANGE, round((float) p * (float) error)));

#ifdef CONTROLLER_DEBUG
      UART_printInt(current_load);
      UART_write(SEPARATOR);
      UART_printInt(current_desired);
      UART_write(SEPARATOR);
      UART_printInt(DAC0.DATA);
      UART_write(SEPARATOR);
      UART_printInt(error);
      UART_write(SEPARATOR);
      UART_printInt(cs);
      UART_println("");
#endif

      DAC0.DATA = MAX(
        DAC_MIN,
        MIN(
          DAC_MAX,
          DAC0.DATA + cs
        )
      );
    }
  }

  // Calculate load power and read temperature of the heatsink
  loadpower= ((uint32_t)voltage_load * current_load + 500) / 1000;

#ifdef NTC_PIN
  uint16_t tempAdc = ADC_read(pinAIN(NTC_PIN));
  float rT = NTC_DIV_R1 * ((ADC_MAX / tempAdc) - 1);
  float temp = 1 / ((1 / NTC_T0) + (log(rT / NTC_R0) / NTC_BETA));
  loadtemp = (uint16_t) round(K_TO_C(temp));

  if (loadtemp > MAXTEMP) {
    overload();
    return 1;
  }
#endif

  if(loadpower > MAXPOWER) {
    overload();
    return 1;
  }

  if (fan_control == FAN_AUTO) {
    // Turn fan on or off depending on load power and temperature
    if((loadpower > FANONPOWER)  || (loadtemp > FANONTEMP))  pinHigh(FAN_PIN);
    if((loadpower < FANOFFPOWER) && (loadtemp < FANOFFTEMP)) pinLow(FAN_PIN); 
  }

  return 0;
}

// Read voltage and current of PWR-IN/PWR-OUT
void updatePowerSensors(void) {
  voltage_power = (INA_read(INA2ADDR, VOLTAGE_REG) >> 1) & 0xfffc;
  voltage_power = (float)voltage_power * ULCAL2 + 0.5;
  current_power = INA_read(INA2ADDR, CURRENT_REG);
  if(current_power > 32767) current_power = 0;
}

// Read all sensors and updates the corresponding variables
uint8_t updateSensors(void) {
  updatePowerSensors();
  return updateLoadSensors();
}

uint8_t wait_for_current(uint16_t current) {
  while (current_load < current_desired) {
    updateLoadSensors();
    _delay_ms(STEP);
  }
  return 0;
}
void transmit_temperature(void) {
  updateLoadSensors();
  UART_printInt(loadtemp);
  UART_println("");
}

void set_fan(void) {
  if (argument1 == 2) {
    fan_control = FAN_AUTO;
    return;
  }
  else {
    if (argument1 == 0) pinLow(FAN_PIN);
    else if (argument1 == 1) pinHigh(FAN_PIN);
    fan_control = FAN_MANUAL;
    return;
  }
  ledError();
}

// ===================================================================================
// Command Buffer Parser
// ===================================================================================

// Wait for, read and parse command string
void CMD_read(void) {
  while(!CMD_compl) { updateLoadSensors(); _delay_ms(STEP); }
  uint8_t i = 0;
  cmd = CMD_buffer[0];
  argument1 = 0; argument2 = 0; query = false;
  if (CMD_buffer[1] == '?') query = true;
  else {
    while(CMD_buffer[++i] == ' ');
    while(CMD_buffer[i] > ' ') argument1 = argument1 * 10 + CMD_buffer[i++] - '0';
    while(CMD_buffer[i] == ' ') i++;
    while(CMD_buffer[i] != 0)  argument2 = argument2 * 10 + CMD_buffer[i++] - '0';
  }
  CMD_compl = 0;
}

// Check if termination command was send during a test program
uint8_t CMD_isTerminated(void) {
  if(CMD_compl) return(CMD_buffer[0] == 'x');
  return 0;
}

// ===================================================================================
// Test Algorithms
// ===================================================================================


// Perform automatic load test according to minimum load voltage and maximum load current
void load_test(void) {
  uint8_t  DACvalue = 0; DAC0.DATA = 0;
  uint16_t maxloadcurrent = MIN(argument1, MAXCURRENT);
  uint16_t minloadvoltage = argument2;

  uint32_t step_time = MIL_read();
  uint32_t next_adj_step = step_time + ADJ_STEP;
  uint32_t next_report_step = step_time + REPORT_STEP;

  set_current(1);
  // Start reporting and stepping up only after we ramp up.
  if (wait_for_current(1) != 0) {
    set_current(0);
    UART_println(ERROR);
    ledError();
    return;
  }

  while(!CMD_isTerminated()) {
    step_time = MIL_read();
    if (step_time >= next_adj_step) {
      next_adj_step = step_time + ADJ_STEP;
      set_current(MIN(current_desired + CURRENT_ADJ, maxloadcurrent));
    }

    bool stop = false;

    if (updateLoadSensors() != 0) stop = true;
    if (voltage_load < minloadvoltage || current_load >= maxloadcurrent )
      stop = true;

    if (step_time >= next_report_step || stop) {
      next_report_step = step_time + REPORT_STEP;
      // Transmit values via serial interface
      UART_printInt(current_load);  UART_write(SEPARATOR);
      UART_printInt(voltage_load);  UART_write(SEPARATOR);
      UART_printInt(loadpower); UART_println("");
    }

    if (stop) break;
    _delay_ms(STEP);                            // give everything a little time to settle
  }

  set_current(0);
  UART_println(DONE);                             // transmit end of test
}

// Transmit sensors, optionally on interval, for a period of time
void transmit_sensors() {
  uint32_t step_time = MIL_read();
  uint32_t next_step = step_time;
  uint32_t last_step = step_time + argument2;
  do {
    updateSensors();
    if ((step_time = MIL_read()) >= next_step) {
      UART_printInt(current_load); UART_write(SEPARATOR);
      UART_printInt(voltage_load); UART_write(SEPARATOR);
      UART_printInt(current_power); UART_write(SEPARATOR);
      UART_printInt(voltage_power);
      UART_println("");
      next_step = step_time + argument1;
    }
    _delay_ms(STEP);
  } while (!CMD_isTerminated() && next_step < last_step);
}

// Perform a voltage regulation test up to the max load current
/* void regulationTest(void) { */
/*   uint16_t maxloadcurrent = argument1; */
/*   uint8_t  profile[] = {0, 2, 5, 7, 10, 10, 5, 0, 10, 0}; */
/*   INA_write(INA1ADDR, CONFIG_REG, INAFASTBOTH);   // speed-up sampling rate of INA */
/*   DAC_setReference(maxloadcurrent); */
/*   uint32_t startmillis = MIL_read(); */
/*   uint32_t nextmillis  = startmillis + 100; */

/*   for(uint8_t i=0; i<10; i++) { */
/*     DAC_set(maxloadcurrent * profile[i] / 10); */
/*     while(MIL_read() < nextmillis) { */
/*       if (updateLoadSensors() != 0) break; */
/*       UART_printInt(MIL_read() - startmillis); UART_write(SEPARATOR); */
/*       UART_printInt(current_load); UART_write(SEPARATOR); */
/*       UART_printInt(voltage_load); UART_println(""); */
/*     } */
/*     nextmillis += 100; */
/*   } */
/*   DAC_resetLoad();                                // reset the load to minimum */
/*   INA_write(INA1ADDR, CONFIG_REG, INA1CONFIG);    // INA back to normal operation */
/*   UART_println(DONE);                             // transmit end of test */
/* } */

/* // Perform automatic efficiency test according to minimum load voltage and maximum load current */
/* void efficiencyTest(void) { */
/*   uint16_t supplypower; */
/*   uint16_t efficiency; */
/*   uint8_t  DACvalue = 0; DAC0.DATA = 0; */
/*   uint16_t maxloadcurrent = argument1; */
/*   uint16_t minloadvoltage = argument2; */
/*   if(maxloadcurrent > 4999) maxloadcurrent = 4999; */
  
/*   DAC_setReference(maxloadcurrent);               // set DAC reference according to max current */
/*   while(++DACvalue) {                             // increase load every cycle */
/*     updateSensors();                              // read all sensor values */
/*     if(loadpower > MAXPOWER) break;               // stop when power reaches maximum */
/*     if(loadtemp  > MAXTEMP)  break;               // stop when heatsink is to hot */
/*     if(voltage_load  < minloadvoltage) break;         // stop when voltage falls below minimum */
/*     if(current_load  > maxloadcurrent) break;         // stop when current reaches maximum */
/*     DAC0.DATA = DACvalue;                         // set load for next measurement */

/*     // Calculate efficiency */
/*     supplypower = ((uint32_t)voltage_power * current_power + 500) / 1000; */
/*     efficiency  = (float)loadpower * 1000 / supplypower + 0.5; */

/*     // Transmit values via serial interface */
/*     UART_printInt(current_load);   UART_write(SEPARATOR); */
/*     UART_printInt(voltage_load);   UART_write(SEPARATOR); */
/*     UART_printInt(efficiency); UART_println(""); */
    
/*     _delay_ms(STEP);                            // give everything a little time to settle */
/*   } */  
/*   DAC_resetLoad();                                // reset the load to minimum */
/*   UART_println(DONE);                             // transmit end of test */
/* } */

/* // Perform low frequency ripple test */
/* void ripple_test(void) { */
/*   uint16_t minvoltage, maxvoltage; */
/*   uint16_t maxloadcurrent = MIN(argument1, MAXCURRENT); */
/*   uint16_t minloadvoltage = argument2; */
  
/*   while(++DACvalue) {                             // increase load every cycle */
/*     if (updateLoadSensors() != 0) break; */
/*     if(voltage_load  < minloadvoltage) break;         // stop when voltage falls below minimum */
/*     if(current_load  > maxloadcurrent) break;         // stop when current reaches maximum */
/*     INA_write(INA1ADDR, CONFIG_REG, INAFASTBUS);  // speed-up sampling rate of INA */
/*     minvoltage = 26000; maxvoltage = 0;           // reset peak voltage values */
/*     for(uint8_t i=255; i; i--) {                  // get 255 voltage readings */
/*       // Read voltage and update peak values */
/*       voltage_load = (INA_read(INA1ADDR, VOLTAGE_REG) >> 1) & 0xfffc; */
/*       if(voltage_load > maxvoltage) maxvoltage = voltage_load; */
/*       if(voltage_load < minvoltage) minvoltage = voltage_load; */
/*     } */
/*     INA_write(INA1ADDR, CONFIG_REG, INA1CONFIG);  // INA back to normal operation */
/*     DAC0.DATA = DACvalue;                         // set load for next measurement */

/*     // Transmit values via serial interface */
/*     UART_printInt(current_load); UART_write(SEPARATOR); */
/*     UART_printInt(maxvoltage - minvoltage); UART_println(""); */
/*     _delay_ms(STEP);                            // give everything a little time to settle */
/*   } */ 
/*   DAC_resetLoad();                                // reset the load to minimum */
/*   UART_println(DONE);                             // transmit end of test */
/* } */

/* // Perform a battery discharge test */
/* void batteryTest(void) { */
/*   uint32_t capacity       = 0; */
/*   uint16_t maxloadcurrent = argument1; */
/*   uint16_t minloadvoltage = argument2; */
/*   uint32_t startmillis    = MIL_read(); */
/*   uint32_t nextmillis     = startmillis + 1000; */
/*   if(maxloadcurrent > 3000) maxloadcurrent = 3000; */
  
/*   DAC_setLoad(maxloadcurrent);                    // set the constant load current */
/*   _delay_ms(STEP);                              // give everything a little time to settle */
/*   while(DAC0.DATA) {                              // repeat until load is zero */
/*     if (updateLoadSensors() != 0) break;         // read all load sensor values, exit on error */
/*     if(CMD_isTerminated()) break;                 // stop if termination command was sent */

/*     // Decrease load if voltage drops below minloadvoltage */
/*     if(voltage_load < minloadvoltage) DAC0.DATA--; */

/*     // Transmit values via serial interface */
/*     UART_printInt((MIL_read() - startmillis) / 1000); UART_write(SEPARATOR); */
/*     UART_printInt(current_load); UART_write(SEPARATOR); */
/*     UART_printInt(voltage_load); UART_write(SEPARATOR); */
/*     UART_printInt(capacity / 3600); UART_println(""); */
    
/*     while(MIL_read() < nextmillis);               // wait for the next cycle (one cycle/second) */
/*     nextmillis += 1000;                           // set end time for next cycle */
/*     capacity   += current_load;                       // calculate capacity */
/*   } */
/*   DAC_resetLoad();                                // reset the load to minimum */
/*   UART_println(DONE);                             // transmit end of test */
/* } */

/* // Set a load for calibration */
/* void calibrateLoad(void) { */
/*   uint8_t  counter     = 0;                       // counts the number of measurements */
/*   uint32_t voltages1   = 0;                       // accumulates all voltage measurements (load) */
/*   uint32_t currents1   = 0;                       // accumulates all current measurements (load) */
/*   uint32_t voltages2   = 0;                       // accumulates all voltage measurements (supply) */
/*   uint32_t currents2   = 0;                       // accumulates all current measurements (supply) */
/*   uint16_t loadcurrent = argument1;               // argument1 is the constant load current */
  
/*   DAC_setLoad(loadcurrent);                       // set the constant load current */
/*   _delay_ms(STEP);                              // a little settle time */
/*   uint32_t endmillis = MIL_read() + (uint32_t)1000 * argument2;  // start the timer */
/*   while(MIL_read() < endmillis) { */
/*     updateSensors();                              // read all sensor values */

/*     // Add up the measurements and increase the counter */
/*     voltages1 += voltage_load; currents1 += current_load; voltages2 += voltage_power; currents2 += current_power; */
/*     counter++; */
    
/*     // Transmit averaged values via serial interface */
/*     UART_printInt(currents1 / counter); UART_write(SEPARATOR); */
/*     UART_printInt(voltages1 / counter); UART_write(SEPARATOR); */
/*     UART_printInt(currents2 / counter); UART_write(SEPARATOR); */
/*     UART_printInt(voltages2 / counter); UART_println(""); */

/*     _delay_ms(100); */
/*   } */
/*   DAC_resetLoad();                                // reset the load to minimum */
/*   UART_println(DONE);                             // transmit end of test */
/* } */

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Setup MCU
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);         // set clock frequency to 20 MHz

  ledBusy();

  // Setup modules
  UART_init();                                    // init UART
  I2C_init();                                     // init I2C
  INA_init();                                     // init INA219
  DAC_init();                                     // init DAC
  ADC_init();                                     // init ADC
  MIL_init();                                     // init millis counter
  sei();                                          // enable interrupts

  // Setup pins
  pinOutput(FAN_PIN);                             // set fan pin as output
  pinOutput(LED_PIN);                             // set LED pin as output

#ifdef NTC_PIN
  pinDisable(NTC_PIN);                            // disable digital input buffer
#endif

  // This delay not necessary, but the reassuring transition isn't visible otherwise:
  _delay_ms(BLINK_INFO);
  ledReady();

  // Loop
  while(1) {
    CMD_read();                                   // wait for and read command
    ledBusy();
    switch(cmd) {
      case 'i':   UART_println(IDENT);   break;   // send identification
      case 'v':   UART_println(VERSION); break;   // send version number
      /* case 'c':   calibrateLoad();       break;   // set load for calibration */
      case 'l':   load_test();           break;   // perform load test
      /* case 'g':   regulationTest();      break;   // perform regulation test */
      /* case 'e':   efficiencyTest();      break;   // perform efficiency test */
      /* case 'r':   ripple_test();          break;   // perform ripple test */
      /* case 'b':   batteryTest();         break;   // perform battery test */
      /* case 'm':   multimeter();          break;   // long-term multimeter */
      case 't':   transmit_sensors();     break;   // read and transmit sensor values
      case 'x':   set_current(0);                // reset the load
                  UART_println(DONE);    break;
      case 's':   set_current(argument1);        // set load current
                  UART_println(DONE);    break; 
      case 'd':
        if (query) {
          UART_printInt(DAC0.DATA);
          UART_println("");
        } else {
          DAC0.DATA = argument1;
          dac_control = DAC_MANUAL;
        }
        break;
      case 'p':   transmit_temperature(); break;   // transmit NTC temp
      case 'w':   set_fan(); break;
      default:    ledError(); CMD_ptr = 0; break;
    }
    ledReady();
  }
}
