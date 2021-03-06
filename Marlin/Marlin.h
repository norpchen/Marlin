// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define VERSION_STRING  "1.1.4"

#define  HardwareSerial_h // trick to disable the standard HWserial

#define  FORCE_INLINE __attribute__((always_inline)) inline


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include  <avr/wdt.h>
#include  <avr/interrupt.h>
#include <SPI.h>


#if ARDUINO >= 100
#if defined(__AVR_ATmega644P__)
#include "WProgram.h"
#else
#include "Arduino.h"
#endif
#else
#include "WProgram.h"
#endif

#ifdef MCP23017_LCD
#include <Wire.h>
#include <LiquidTWI2.h>
#endif
#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#if defined(__AVR_AT90USB1286__)||defined(__AVR_AT90USB1287__)
#define MYSERIAL Serial
#else
#define MYSERIAL MSerial
#endif

#include "Configuration.h"

#include "Utility.h"

#include "fastio.h"

#include "pins.h"
#include "State.h"
#include "job.h"


//this is a unfinsihed attemp to removes a lot of warning messages, see:
// http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=57011
//typedef char prog_char PROGMEM;
// //#define PSTR    (s )        ((const PROGMEM char *)(s))
// //# define MYPGM(s) (__extension__({static prog_char __c[] = (s); &__c[0];}))
// //#define MYPGM(s) ((const prog_char *g PROGMEM=s))
#define MYPGM(s) PSTR(s)
//#define MYPGM(s)  (__extension__({static char __c[] __attribute__((__progmem__)) = (s); &__c[0];}))  //This is the normal behaviour
//#define MYPGM(s)  (__extension__({static prog_char __c[]  = (s); &__c[0];})) //this does not work but hides the warnings


#define SERIAL_PROTOCOL(x) MYSERIAL.print(x);
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y);
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(MYPGM(x));
#define SERIAL_PROTOCOLLN(x) {MYSERIAL.print(x);MYSERIAL.write('\n');}
#define SERIAL_PROTOCOLLNPGM(x) {serialprintPGM(MYPGM(x));MYSERIAL.write('\n');}


const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";
#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic);
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);


//things to write to serial from Programmemory. saves 400 to 2k of RAM.
#define SerialprintPGM(x) serialprintPGM(MYPGM(x))
void serialprintPGM(const char *str);

void get_command();
void process_commands();

void manage_inactivity();

#if X_ENABLE_PIN > -1
#define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
#define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
#define enable_x() ;
#define disable_x() ;
#endif

#if Y_ENABLE_PIN > -1
#define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
#define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
#define enable_y() ;
#define disable_y() ;
#endif

#if Z_ENABLE_PIN > -1
#ifdef Z_DUAL_STEPPER_DRIVERS
#define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
#define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); }
#else
#define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
#define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
#endif
#else
#define enable_z() ;
#define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
#define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
#define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e0()  /* nothing */
#define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
#define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
#define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e1()  /* nothing */
#define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
#define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
#define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e2()  /* nothing */
#define disable_e2() /* nothing */
#endif


enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void kill(int fatal=1);
void Stop();

bool IsStopped();

void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
#define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern float current_head_position[NUM_AXIS] ;
extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];
extern unsigned char FanSpeed;

// Handling multiple extruders pins
extern uint8_t active_extruder;


extern unsigned long time_of_last_command; 

// data logging for statistics and maintenance
extern unsigned long total_on_time ;
extern unsigned long total_printing_time ;
extern unsigned long total_extruder_time0 ;
extern unsigned long total_extruder_time1 ;
extern unsigned long total_bed_time ;
extern float total_distance[NUM_AXIS];

extern float extruder1_degree_seconds;
extern float extruder0_degree_seconds;
extern float bed_degree_seconds;
extern float total_filament;

void manage_other_tasks();
void DisableAllSteppers();

extern 	float BBLookAheadHot ;
extern float BBLookAheadCool ;

extern unsigned long now;			// cached value of millis() -- should be reasonably recent

extern volatile int feedmultiply; //100->1 200->2
extern volatile int extrudemultiply; //
extern long gcode_N; //

extern long save_gcode_start;

#endif
