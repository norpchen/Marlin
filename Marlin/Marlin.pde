#include <SPI.h>

/* -*- c++ -*- */

/*
Reprap firmware based on Sprinter and grbl.
Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Marlin.h"
#include "Utility.h"
#ifdef MCP23017_LCD
#include <Wire.h>
#include <LiquidTWI2.h>
#endif

#include "Job.h"
#include "State.h"


#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "EEPROMwrite.h"
#include "language.h"
#include "pins_arduino.h"
#ifdef BLINK_M
#include "BlinkM_funcs.h"
#endif

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M42  - Change pin status via gcode
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Output current position to serial port
// M115	- Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M999 - Restart after being stopped by error

//	added by Lars 4/20/2013
// M420 - Set RGB mood light
//			R E B to set RGB values (note: E instead of G!)
//			P to run BlinkM script #
// M300 - Beep!
// M70   display messager (replicator)
// M72  play selected song / tone
//		P  = ID
// M73 set the % done
//		Q= percent done (0-100)

//Stepper Movement Variables
float quality = 1.0;

//===========================================================================
//=============================imported variables============================
//===========================================================================

//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
volatile int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
volatile bool feedmultiplychanged=false;
volatile int extrudemultiply=100; //100->1 200->2
float current_head_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]= {0,0,0};
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
uint8_t active_extruder = 0;
unsigned char FanSpeed=0;

#ifdef FWRETRACT
bool autoretract_enabled=true;
bool retracted=false;
float retract_length=3, retract_feedrate=17*60, retract_zlift=0.8;
float retract_recover_length=0, retract_recover_feedrate=8*60;
#endif

//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
long save_gcode_start ;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
static bool relative_mode_e = false;  //Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

// data logging for statistics and maintenance
unsigned long total_on_time =0;
unsigned long total_printing_time=0 ;
unsigned long total_extruder_time0 =0;
unsigned long total_extruder_time1 =0;
unsigned long total_bed_time =0;
float total_distance[NUM_AXIS];

float extruder1_degree_seconds=0;
float extruder0_degree_seconds=0;
float bed_degree_seconds=0;
float total_filament=0;

#define Stopped (state==ERROR)
// bool Stopped=false;

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;	// 15min -- THIS IS BROKEN!!!
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long time_of_last_command=-1;
unsigned long now = millis();
static uint8_t tmp_extruder;

static int counter = EEPROM_UPDATE_RATE ;
static unsigned long last_clock_update = millis();

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serialprintPGM(const char *str)
{
	char ch=pgm_read_byte(str);
	while(ch)
		{
			MYSERIAL.write(ch);
			ch=pgm_read_byte(++str);
		}
}

//-------------------------------------------------------------------------------------------------------------------
void serial_echopair_P(const char *s_P, float v)
{
	serialprintPGM(s_P);
	SERIAL_ECHO(v);
}
//-------------------------------------------------------------------------------------------------------------------
void serial_echopair_P(const char *s_P, double v)
{
	serialprintPGM(s_P);
	SERIAL_ECHO(v);
}
//-------------------------------------------------------------------------------------------------------------------
void serial_echopair_P(const char *s_P, unsigned long v)
{
	serialprintPGM(s_P);
	SERIAL_ECHO(v);
}

//-------------------------------------------------------------------------------------------------------------------
//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
	VISIT;
	if(buflen < BUFSIZE)
		{
			//this is dangerous if a mixing of serial and this happsens
			strcpy(&(cmdbuffer[bufindw][0]),cmd);
			SERIAL_ECHO_START;
			SERIAL_ECHOPGM("enqueing \"");
			SERIAL_ECHO(cmdbuffer[bufindw]);
			SERIAL_ECHOLNPGM("\"");
			bufindw= (bufindw + 1)%BUFSIZE;
			buflen += 1;
		}
}

//-------------------------------------------------------------------------------------------------------------------
void setup_killpin()
{
#if( KILL_PIN>-1 )
	pinMode(KILL_PIN,INPUT);
	WRITE(KILL_PIN,HIGH);
#endif
}

//-------------------------------------------------------------------------------------------------------------------
void setup_photpin()
{
#ifdef PHOTOGRAPH_PIN
#if (PHOTOGRAPH_PIN > -1)
	SET_OUTPUT(PHOTOGRAPH_PIN);
	WRITE(PHOTOGRAPH_PIN, LOW);
#endif
#endif
}

//-------------------------------------------------------------------------------------------------------------------
void setup_powerhold()
{
#ifdef SUICIDE_PIN
#if (SUICIDE_PIN> -1)
	SET_OUTPUT(SUICIDE_PIN);
	WRITE(SUICIDE_PIN, HIGH);
#endif
#endif
}

//-------------------------------------------------------------------------------------------------------------------
void suicide()
{
#ifdef SUICIDE_PIN
#if (SUICIDE_PIN> -1)
	SET_OUTPUT(SUICIDE_PIN);
	WRITE(SUICIDE_PIN, LOW);
#endif
#endif
}

//-------------------------------------------------------------------------------------------------------------------
void setup()
{
	setup_killpin();
	setup_powerhold();
	MYSERIAL.begin(BAUDRATE);

	SERIAL_PROTOCOLLNPGM("start");
	SERIAL_ECHO_START;




	SERIAL_ECHOPGM(MSG_MARLIN);
	SERIAL_ECHOLNPGM(VERSION_STRING);
#ifdef STRING_VERSION_CONFIG_H
#ifdef STRING_CONFIG_H_AUTHOR
	SERIAL_ECHO_START;
	SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
	SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
	SERIAL_ECHOPGM(MSG_AUTHOR);
	SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
#endif
#endif
	SERIAL_ECHO_START;
	SERIAL_ECHOPGM(MSG_FREE_MEMORY);
	SERIAL_ECHO(freeMemory());
	SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
	SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
	for(int8_t i = 0; i < BUFSIZE; i++)
		{
			fromsd[i] = false;
		}

	EEPROM_RetrieveSettings(); // loads data from EEPROM if available

	for(int8_t i=0; i < NUM_AXIS; i++)
		{
			axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
		}

	tp_init();    // Initialize temperature loop
	plan_init();  // Initialize planner;
	st_init();    // Initialize stepper;
	watchdog_init();
	job.init();
	state.init();

	setup_photpin();
	LCD_INIT;
	SetStatusLEDColor(LED_COLOR_WHITE);
#ifdef BLINK_M
	BlinkM_begin();
#endif
	// Check startup - does nothing if bootloader sets MCUSR to 0
	byte mcu = MCUSR;
	if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
	if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
	if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
	if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
	if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
	MCUSR=0;


	if ((mcu & (1 << PORF)) == 0 && lastvisit != NULL)
		{
		SERIAL_ECHOLNPGM("Last visited: ");
		SERIAL_ECHOLN(lastvisit);
#ifdef ULTRA_LCD
		lcd_alertstatuspgm(lastvisit);
#endif
		} 

	pinMode(6, OUTPUT);
	digitalWrite (6,1);
// 	pinMode(5, OUTPUT);
// 	digitalWrite (5,1);
	pinMode(4, OUTPUT);
	digitalWrite (4,1);
	pinMode(2, OUTPUT);
	digitalWrite (2,1);
	pinMode(3, OUTPUT);
	digitalWrite (3,1);
//	pinMode(40, OUTPUT);
//	digitalWrite (40,1);
// 	pinMode(9, OUTPUT);
// 	digitalWrite (9,1);
	pinMode(7, OUTPUT);
	digitalWrite (7,1);
	pinMode(8, OUTPUT);
	digitalWrite (8,1);
// 	pinMode(MOSI_PIN, OUTPUT);
// 	digitalWrite (MOSI_PIN,1);
// 	pinMode(SCK_PIN, OUTPUT);
// 	digitalWrite (SCK_PIN,1);
// 	pinMode(SPI_DISPLAY_CS, OUTPUT);
// 	digitalWrite (SPI_DISPLAY_CS,0);
// 	

}


boolean loop_blinker = false;

//-------------------------------------------------------------------------------------------------------------------
void loop()
{
pinMode(6, OUTPUT);
	digitalWrite (6,loop_blinker?0:1);
	loop_blinker = !loop_blinker;

	if(buflen < (BUFSIZE-1))
		get_command();
#ifdef SDSUPPORT
	card.checkautostart(false);
#endif
	if(buflen)
		{
#ifdef SDSUPPORT
			if(card.saving)
				{
					if(strstr(cmdbuffer[bufindr],"M29") == NULL)
						{
							card.write_command(cmdbuffer[bufindr]);
							SERIAL_PROTOCOLLNPGM(MSG_OK);
						}
					else
						{
							card.closefile();
							SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
							LCD_MESSAGE_CLEAR;
							job.Stop();
							state = DEBUG;
							state = IDLE;
							LCD_MESSAGEPGMPRI ("Upload completed", USER_MESSAGE_PRIORITY);
						}
				}
			else
				{
					process_commands();
				}
#else
			process_commands();
#endif //SDSUPPORT
			buflen = (buflen-1);
			bufindr = (bufindr + 1)%BUFSIZE;
		}
	checkHitEndstops();
	manage_other_tasks();
}

//-------------------------------------------------------------------------------------------------------------------
void get_command()
{
	while( MYSERIAL.available() > 0  && buflen < BUFSIZE)
		{
			VISIT;
			serial_char = MYSERIAL.read();
			state.SetController (SERIAL_HOST);
			time_of_last_command = millis();
			if(serial_char == '\n' ||
					serial_char == '\r' ||
					(serial_char == ':' && comment_mode == false) ||
					serial_count >= (MAX_CMD_SIZE - 1) )
				{
					if(!serial_count)   //if empty line
						{
							comment_mode = false; //for new command
							return;
						}
					cmdbuffer[bufindw][serial_count] = 0; //terminate string
					if(!comment_mode)
						{
							comment_mode = false; //for new command
							fromsd[bufindw] = false;
							if(strstr(cmdbuffer[bufindw], "N") != NULL)
								{
									strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
									gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
									if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], "M110") == NULL) )
										{
											SERIAL_ERROR_START;
											SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
											SERIAL_ERRORLN(gcode_LastN);
											//Serial.println(gcode_N);
											FlushSerialRequestResend();
											serial_count = 0;
											return;
										}

									if(strstr(cmdbuffer[bufindw], "*") != NULL)
										{
											byte checksum = 0;
											byte count = 0;
											while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
											strchr_pointer = strchr(cmdbuffer[bufindw], '*');

											if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum)
												{
													SERIAL_ERROR_START;
													SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
													SERIAL_ERRORLN(gcode_LastN);
													FlushSerialRequestResend();
													serial_count = 0;
													return;
												}
											//if no errors, continue parsing
										}
									else
										{
											SERIAL_ERROR_START;
											SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
											SERIAL_ERRORLN(gcode_LastN);
											FlushSerialRequestResend();
											serial_count = 0;
											return;
										}

									gcode_LastN = gcode_N;
									if (gcode_N<=1) job.Start(true);
									//if no errors, continue parsing
								}
							else  // if we don't receive 'N' but still see '*'
								{
									if((strstr(cmdbuffer[bufindw], "*") != NULL))
										{
											SERIAL_ERROR_START;
											SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
											SERIAL_ERRORLN(gcode_LastN);
											serial_count = 0;
											return;
										}

									/*	 this didn't work.  :(

									if((strstr(cmdbuffer[bufindw], "M117") != NULL))		// some hosts sent M117 without checksum or line number  thank you very much
									{
									gcode_LastN++;		// advance the line number by one to avoid the line number mismatch error handling on next line.
									}*/
								}
							if((strstr(cmdbuffer[bufindw], "G") != NULL))
								{
									strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
									switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL))))
										{
											case 0:
											case 1:
											case 2:
											case 3:
												if(!Stopped)   // If printer is stopped by an error the G[0-3] codes are ignored.
													{
#ifdef SDSUPPORT
														if(card.saving)
															break;
#endif //SDSUPPORT
														SERIAL_PROTOCOLLNPGM(MSG_OK);
													}
												else
													{
														// 							SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
														// 							LCD_MESSAGEPGM(MSG_STOPPED);
														Warning (MSG_STOPPED);
													}
												break;
											default:
												break;
										}
								}
							bufindw = (bufindw + 1)%BUFSIZE;
							buflen += 1;
						}
					serial_count = 0; //clear buffer
				}
			else
				{
					if(serial_char == ';') comment_mode = true;
					if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
				}
		}
#ifdef SDSUPPORT
	if(!card.sdprinting || serial_count!=0)
		{
			return;
		}
	while( !card.eof()  && buflen < BUFSIZE)
		{
			int16_t n=card.get();
			serial_char = (char)n;
			if(serial_char == '\n' ||
					serial_char == '\r' ||
					(serial_char == ':' && comment_mode == false) ||
					serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
				{
					if(card.eof())
						{
							SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
							SERIAL_ECHO_START;
							SERIAL_ECHOLN(EchoTimeSpan (job.JobTime()));
							card.printingHasFinished();
							/* JobDone(); */
							card.checkautostart(true);
						}
					if(!serial_count)
						{
							comment_mode = false; //for new command
							return; //if empty line
						}
					cmdbuffer[bufindw][serial_count] = 0; //terminate string
					//      if(!comment_mode){
					fromsd[bufindw] = true;
					buflen += 1;
					bufindw = (bufindw + 1)%BUFSIZE;
					//      }
					comment_mode = false; //for new command
					serial_count = 0; //clear buffer
				}
			else
				{
					if(serial_char == ';') comment_mode = true;
					if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
				}
		}

#endif //SDSUPPORT
}

//-------------------------------------------------------------------------------------------------------------------
float code_value()
{
	return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

//-------------------------------------------------------------------------------------------------------------------
long code_value_long()
{
	return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

//-------------------------------------------------------------------------------------------------------------------
bool code_seen(char code_string[]) //Return True if the string was found
{
	return (strstr(cmdbuffer[bufindr], code_string) != NULL);
}

//-------------------------------------------------------------------------------------------------------------------
bool code_seen(char code)
{
	strchr_pointer = strchr(cmdbuffer[bufindr], code);
	return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)		\
	static inline type pgm_read_any(const type *p)	\
	{ return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG)	\
	static const PROGMEM type array##_P[3] =		\
	{ X_##CONFIG, Y_##CONFIG, Z_##CONFIG };		\
	static inline type array(int axis)			\
	{ return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

//-------------------------------------------------------------------------------------------------------------------
static void axis_is_at_home(int axis)
{
	current_head_position[axis] = base_home_pos(axis) + add_homeing[axis];
	min_pos[axis] =          base_min_pos(axis) + add_homeing[axis];
	max_pos[axis] =          base_max_pos(axis) + add_homeing[axis];
}

//-------------------------------------------------------------------------------------------------------------------
static void homeaxis(int axis)
{
#define HOMEAXIS_DO(LETTER) \
	((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

	if (axis==X_AXIS ? HOMEAXIS_DO(X) :
			axis==Y_AXIS ? HOMEAXIS_DO(Y) :
			axis==Z_AXIS ? HOMEAXIS_DO(Z) :
			0)
		{
			quality = 1.0;

			current_head_position[axis] = 0;
			plan_set_position(current_head_position[X_AXIS], current_head_position[Y_AXIS], current_head_position[Z_AXIS], current_head_position[E_AXIS]);
			destination[axis] = 1.5 * max_length(axis) * home_dir(axis);
			feedrate = homing_feedrate[axis];
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
			st_synchronize();

			current_head_position[axis] = 0;
			plan_set_position(current_head_position[X_AXIS], current_head_position[Y_AXIS], current_head_position[Z_AXIS], current_head_position[E_AXIS]);
			destination[axis] = -home_retract_mm(axis) * home_dir(axis);
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
			st_synchronize();

			destination[axis] = 2*home_retract_mm(axis) * home_dir(axis);
			feedrate = homing_feedrate[axis]/2 ;
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
			st_synchronize();

			axis_is_at_home(axis);
			destination[axis] = current_head_position[axis];
			feedrate = 0.0;
			endstops_hit_on_purpose();
		}
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

//-------------------------------------------------------------------------------------------------------------------
void process_commands()
{
	unsigned long codenum; //throw away variable
	char *starpos = NULL;
	VISIT;
	// if (echo)
	// {
	// SERIAL_ECHO_START;
	// SERIAL_ECHOLN(cmdbuffer[bufindr]);
	// }
	if(code_seen('G'))
		{
			int cv = (int)code_value();
			switch(cv)
				{
					case 0: // G0 -> G1
					case 1: // G1
						// 			SERIAL_ECHO_START;
						// 			SERIAL_ECHOLN("G0/G1");

						if(!Stopped)
							{
								get_coordinates(); // For X Y Z E F
								prepare_move();
								//ClearToSend();
								return;
							}
						else
							{
								LCD_MESSAGEPGM(MSG_STOPPED);
							}
						break;
						//break;
					case 2: // G2  - CW ARC
						// 			SERIAL_ECHO_START;
						// 			SERIAL_ECHOLN("G2");
						if(!Stopped)
							{
								get_arc_coordinates();
								prepare_arc_move(true);
								return;
							}
						else
							{
								LCD_MESSAGEPGM(MSG_STOPPED);
							}

						break;
					case 3: // G3  - CCW ARC
						// 			SERIAL_ECHO_START;
						// 			SERIAL_ECHOLN("C3");
						if(!Stopped)
							{
								get_arc_coordinates();
								prepare_arc_move(false);
								return;
							}
						else
							{
								LCD_MESSAGE(MSG_STOPPED);
							}

						break;
					case 4: // G4 dwell
						state = PAUSED;
						LCD_MESSAGEPGM(MSG_DWELL);
						codenum = 0;
						if(code_seen('P')) codenum = code_value(); // milliseconds to wait
						if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

						st_synchronize();
						codenum += now;  // keep track of when we started waiting
						previous_millis_cmd = now;
						while(now  < codenum )
							{
								progress_string[0] = EchoTimeSpan (codenum-now,true);
								manage_other_tasks();	// now is updated in here!
								VISIT;
							}
						break;
#ifdef FWRETRACT
					case 10: // G10 retract
						state = RETRACT;
						if(!retracted)
							{
								destination[X_AXIS]=current_head_position[X_AXIS];
								destination[Y_AXIS]=current_head_position[Y_AXIS];
								destination[Z_AXIS]=current_head_position[Z_AXIS];
								current_head_position[Z_AXIS]+=-retract_zlift;
								destination[E_AXIS]=current_head_position[E_AXIS]-retract_length;
								feedrate=retract_feedrate;
								retracted=true;
								prepare_move();
							}

						break;
					case 11: // G10 retract_recover
						state = EXTRUDE;
						if(!retracted)
							{
								destination[X_AXIS]=current_head_position[X_AXIS];
								destination[Y_AXIS]=current_head_position[Y_AXIS];
								destination[Z_AXIS]=current_head_position[Z_AXIS];

								current_head_position[Z_AXIS]+=retract_zlift;
								current_head_position[E_AXIS]+=-retract_recover_length;
								feedrate=retract_recover_feedrate;
								retracted=false;
								prepare_move();
							}
						break;
#endif //FWRETRACT
					case 28: //G28 Home all Axis one at a time
						LCD_MESSAGEPGM("HOME");
						state = MOVING;
						saved_feedrate = feedrate;
						saved_feedmultiply = feedmultiply;
						feedmultiply = 100;
						previous_millis_cmd = millis();
						enable_endstops(true);
						for(int8_t i=0; i < NUM_AXIS; i++)
							{
								destination[i] = current_head_position[i];
							}
						feedrate = 0.0;
						home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

#if Z_HOME_DIR > 0                      // If homing away from BED do Z first
						if((home_all_axis) || (code_seen(axis_codes[Z_AXIS])))
							{
								HOMEAXIS(Z);
							}
#endif

#ifdef QUICK_HOME
						if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
							{
								current_head_position[X_AXIS] = 0;
								current_head_position[Y_AXIS] = 0;

								plan_set_position(current_head_position[X_AXIS], current_head_position[Y_AXIS], current_head_position[Z_AXIS], current_head_position[E_AXIS]);
								destination[X_AXIS] = 1.5 * X_MAX_LENGTH * X_HOME_DIR;
								destination[Y_AXIS] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
								feedrate = homing_feedrate[X_AXIS];
								if(homing_feedrate[Y_AXIS]<feedrate)
									feedrate =homing_feedrate[Y_AXIS];
								plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
								st_synchronize();

								axis_is_at_home(X_AXIS);
								axis_is_at_home(Y_AXIS);
								plan_set_position(current_head_position[X_AXIS], current_head_position[Y_AXIS], current_head_position[Z_AXIS], current_head_position[E_AXIS]);
								destination[X_AXIS] = current_head_position[X_AXIS];
								destination[Y_AXIS] = current_head_position[Y_AXIS];
								plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
								feedrate = 0.0;
								st_synchronize();
								endstops_hit_on_purpose();
							}
#endif

						if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
							{
								HOMEAXIS(X);
							}

						if((home_all_axis) || (code_seen(axis_codes[Y_AXIS])))
							{
								HOMEAXIS(Y);
							}

#if Z_HOME_DIR < 0                      // If homing towards BED do Z last
						if((home_all_axis) || (code_seen(axis_codes[Z_AXIS])))
							{
								HOMEAXIS(Z);
							}
#endif

						if(code_seen(axis_codes[X_AXIS]))
							{
								if(code_value_long() != 0)
									{
										current_head_position[X_AXIS]=code_value()+add_homeing[0];
									}
							}

						if(code_seen(axis_codes[Y_AXIS]))
							{
								if(code_value_long() != 0)
									{
										current_head_position[Y_AXIS]=code_value()+add_homeing[1];
									}
							}

						if(code_seen(axis_codes[Z_AXIS]))
							{
								if(code_value_long() != 0)
									{
										current_head_position[Z_AXIS]=code_value()+add_homeing[2];
									}
							}
						plan_set_position(current_head_position[X_AXIS], current_head_position[Y_AXIS], current_head_position[Z_AXIS], current_head_position[E_AXIS]);

#ifdef ENDSTOPS_ONLY_FOR_HOMING
						enable_endstops(false);
#endif

						feedrate = saved_feedrate;
						feedmultiply = saved_feedmultiply;
						previous_millis_cmd = millis();
						endstops_hit_on_purpose();
						break;
					case 90: // G90
						relative_mode = false;
						break;
					case 91: // G91
						relative_mode = true;
						break;
					case 92: // G92
						if(!code_seen(axis_codes[E_AXIS]))
							st_synchronize();
						for(int8_t i=0; i < NUM_AXIS; i++)
							{
								if(code_seen(axis_codes[i]))
									{
										if(i == E_AXIS)
											{
												current_head_position[i] = code_value();
												plan_set_e_position(current_head_position[E_AXIS]);
											}
										else
											{
												current_head_position[i] = code_value()+add_homeing[i];
												plan_set_position(current_head_position[X_AXIS], current_head_position[Y_AXIS], current_head_position[Z_AXIS], current_head_position[E_AXIS]);
											}
									}
							}
						break;
				}
		}

	else
		if(code_seen('M'))
			{
				int cv = (int)code_value();
				switch( cv )
					{
#ifdef ULTRA_LCD
						case 0: // M0 - Unconditional stop - Wait for user button press on LCD
						case 1: // M1 - Conditional stop - Wait for user button press on LCD
						case 226:
							{
								STATES prev_state = state;
								state = PAUSED;

								codenum = 0;
								if(code_seen('P')) codenum = code_value(); // milliseconds to wait
								if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

								st_synchronize();
								previous_millis_cmd = now;
								if (codenum > 0)
									{
										codenum +=now;  // keep track of when we started waiting
										while(now  < codenum && !CLICKED)
											{
												LCD_MESSAGEPGM("Click, or will resume in ");
												progress_string[0] = EchoTimeSpan ((codenum - now )/1000, true);
												progress_string[1] = String("M") + String(cv,DEC);
												manage_other_tasks();
											}
									}
								else
									{
										while(!CLICKED)
											{
												LCD_MESSAGEPGM(MSG_USERWAIT);
												progress_string[0] = EchoTimeSpan ((now - previous_millis_cmd)/1000, true);
												progress_string[1] = "M" + String(cv,DEC);
												manage_other_tasks();
											}
									}
								state = prev_state;
							}
							break;
#endif
						case 17:
							state = IDLE;
							LCD_MESSAGEPGM(MSG_NO_MOVE);
							enable_x();
							enable_y();
							enable_z();
							enable_e0();
							enable_e1();
							enable_e2();
							break;

#ifdef SDSUPPORT
						case 20: // M20 - list SD card
							SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
							card.ls();
							SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
							break;
						case 21: // M21 - init SD card

							card.initsd();
							SERIAL_ECHO_START;
							SERIAL_PROTOCOLPGM ("Volume size: ");
							SERIAL_PROTOCOLLN (card.Volsize());

							break;
						case 22: //M22 - release SD card
							card.release();

							break;
						case 23: //M23 - Select file
							starpos = (strchr(strchr_pointer + 4,'*'));
							if(starpos!=NULL)
								*(starpos-1)='\0';
							card.openFile(strchr_pointer + 4,true);
							break;
						case 24: //M24 - Start SD print
							state = PRINTING;
							state.SetController ( SDCARD);
							job.Start(true);
							card.startFileprint();
							break;
						case 25: //M25 - Pause SD print
							card.pauseSDPrint();
							state = PAUSED;
							LCD_MESSAGEPGM("SD PAUSED");
							break;
						case 26: //M26 - Set SD index
							if(card.cardOK && code_seen('S'))
								{
									card.setIndex(code_value_long());
								}
							break;
						case 27: //M27 - Get SD status
							card.getStatus();
							break;
						case 28: //M28 - Start SD write
							state = SAVING;
							job.Start(true);

							starpos = (strchr(strchr_pointer + 4,'*'));
							if(starpos != NULL)
								{
									char* npos = strchr(cmdbuffer[bufindr], 'N');
									strchr_pointer = strchr(npos,' ') + 1;
									*(starpos-1) = '\0';
								}
							card.openFile(strchr_pointer+4,false);
							break;
						case 29: //M29 - Stop SD write
							//processed in write to file routine above
							state = IDLE;

							//card,saving = false;
							break;
						case 30: //M30 <filename> Delete File
							if (card.cardOK)
								{
									card.closefile();
									starpos = (strchr(strchr_pointer + 4,'*'));
									if(starpos != NULL)
										{
											char* npos = strchr(cmdbuffer[bufindr], 'N');
											strchr_pointer = strchr(npos,' ') + 1;
											*(starpos-1) = '\0';
										}
									card.removeFile(strchr_pointer + 4);
								}
							break;
						case 928: //M928 - Start SD write
							starpos = (strchr(strchr_pointer + 5,'*'));
							if(starpos != NULL)
								{
									char* npos = strchr(cmdbuffer[bufindr], 'N');
									strchr_pointer = strchr(npos,' ') + 1;
									*(starpos-1) = '\0';
								}
							card.openLogFile(strchr_pointer+5);
							break;

#endif //SDSUPPORT

						case 31: //M31 take time since the start of the SD print or an M109 command
							{
								SERIAL_ECHO_START;
								SERIAL_ECHOLN(EchoTimeSpan (job.JobTime()));

								autotempShutdown();  // why are we doing this?  Do we really want to do anything because we queried the time since printing started?
							}
							break;
						case 42: //M42 -Change pin status via gcode
							if (code_seen('S'))
								{
									int pin_status = code_value();
									if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
										{
											int pin_number = code_value();
											for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
												{
													if (sensitive_pins[i] == pin_number)
														{
															pin_number = -1;
															SERIAL_ECHO_START;
															SERIAL_ECHOLNPGM("Cannot change pin value,  that pin is important!");
															break;
														}
												}

											if (pin_number > -1)
												{
													pinMode(pin_number, OUTPUT);
													digitalWrite(pin_number, pin_status);
													analogWrite(pin_number, pin_status);
												}
										}
								}
							break;
						case 104: // M104
							if(setTargetedHotend(104))
								{
									break;
								}
							if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
							setWatch();
							break;
						case 140: // M140 set bed temp
							if (code_seen('S')) setTargetBed(code_value());
							break;
						case 105 : // M105
							if(setTargetedHotend(105))
								{
									break;
								}
#if (TEMP_0_PIN > -1)
							SERIAL_PROTOCOLPGM("ok T:");
							SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
							SERIAL_PROTOCOLPGM(" /");
							SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
#if TEMP_BED_PIN > -1
							SERIAL_PROTOCOLPGM(" B:");
							SERIAL_PROTOCOL_F(degBed(),1);
							SERIAL_PROTOCOLPGM(" /");
							SERIAL_PROTOCOL_F(degTargetBed(),1);
#endif //TEMP_BED_PIN
#else
							SERIAL_ERROR_START;
							SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
#endif

							SERIAL_PROTOCOLPGM(" @:");
							SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));

							SERIAL_PROTOCOLPGM(" B@:");
							SERIAL_PROTOCOL(getHeaterPower(-1));

							SERIAL_PROTOCOLLN("");
							return;
							break;
						case 109:
							{
								job.Start(true);				//					echo = true;
								state = HEATING;
								// M109 - Wait for extruder heater to reach target.
								if(setTargetedHotend(109))
									{
										break;
									}

#ifdef AUTOTEMP
								autotemp_enabled=false;
#endif
								if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
#ifdef AUTOTEMP
								if (code_seen('S')) autotemp_min=code_value();
								if (code_seen('B')) autotemp_max=code_value();
								if (code_seen('F'))
									{
										autotemp_factor=code_value();
										autotemp_enabled=true;
									}
#endif
								progress_string[2] = String ("  EXTR ") + String((int) tmp_extruder);

								setWatch();
								codenum = millis();

								/* See if we are heating up or cooling down */
								bool target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

#ifdef TEMP_RESIDENCY_TIME
								long residencyStart;
								residencyStart = -1;
								/* continue to loop until we have reached the target temp
								_and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
								while((residencyStart == -1) ||
										(residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) )
									{
#else
								while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) )
									{
#endif //TEMP_RESIDENCY_TIME
										if( (millis() - codenum) > 1000UL )
											{
												//Print Temp Reading and remaining time every 1 second while heating up/cooling down
												SERIAL_PROTOCOLPGM("T:");
												SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
												SERIAL_PROTOCOLPGM(" E:");
												SERIAL_PROTOCOL( (int)tmp_extruder );
#ifdef TEMP_RESIDENCY_TIME
												SERIAL_PROTOCOLPGM(" W:");
												if(residencyStart > -1)
													{
														codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
														SERIAL_PROTOCOLLN( codenum );
													}
												else
													{
														SERIAL_PROTOCOLLN( "?" );
													}
#else
												SERIAL_PROTOCOLLN("");
#endif
												codenum = millis();
												VISIT;
											}
										manage_other_tasks();
										LCD_MESSAGEPGMPRI(MSG_HEATING,DEFAULT_MESSAGE_PRIORITY/2);
#ifdef TEMP_RESIDENCY_TIME
										/* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
										or when current temp falls outside the hysteresis after target temp was reached */
										if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
												(residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
												(residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
											{
												residencyStart = millis();
											}
										VISIT;
#endif //TEMP_RESIDENCY_TIME
									}
								LCD_MESSAGEPGMPRI(MSG_HEATING_COMPLETE, DEFAULT_MESSAGE_PRIORITY+1);
								job.Start(true);
								SERIAL_ECHO_START;
								SERIAL_ECHOLNPGM("Heating done, let's go! ");
								previous_millis_cmd = millis();
							}
							break;
						case 190: // M190 - Wait for bed heater to reach target.
#if TEMP_BED_PIN > -1
							job.Start(true);

							state = HEATING;
							progress_string[2] = String ("   BED   ") ;

							if (code_seen('S')) setTargetBed(code_value());
							codenum = millis();
							while(isHeatingBed())
								{
									if(( now - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
										{
											float tt=degHotend(active_extruder);
											SERIAL_PROTOCOLPGM("T:");
											SERIAL_PROTOCOL(tt);
											SERIAL_PROTOCOLPGM(" E:");
											SERIAL_PROTOCOL( (int)active_extruder );
											SERIAL_PROTOCOLPGM(" B:");
											SERIAL_PROTOCOL_F(degBed(),1);
											SERIAL_PROTOCOLLN("");
											codenum =now;
											LCD_MESSAGEPGMPRI(MSG_BED_HEATING,DEFAULT_MESSAGE_PRIORITY/2);
										}
									manage_other_tasks();
									VISIT;
								}
							LCD_MESSAGEPGM(MSG_BED_DONE);
							previous_millis_cmd = now;
#endif
							break;

#if FAN_PIN > -1
						case 106: //M106 Fan On
							if (code_seen('S'))
								{
									FanSpeed=constrain(code_value(),0,255);
								}
							else
								{
									FanSpeed=255;
								}
							break;
						case 107: //M107 Fan Off
							FanSpeed = 0;
							break;
#endif //FAN_PIN

#if (PS_ON_PIN > -1)
						case 80: // M80 - ATX Power On
							SET_OUTPUT(PS_ON_PIN); //GND
							WRITE(PS_ON_PIN, LOW);
							break;
#endif

						case 81: // M81 - ATX Power Off
							state = SLEEPING;
							LCD_MESSAGEPGMPRI("POWER OFF",USER_MESSAGE_PRIORITY);
#if defined SUICIDE_PIN && SUICIDE_PIN > -1
							st_synchronize();
							suicide();
#elif (PS_ON_PIN > -1)
							SET_OUTPUT(PS_ON_PIN);
							WRITE(PS_ON_PIN, HIGH);
#endif
							break;

						case 82:
							axis_relative_modes[3] = false;
							break;
						case 83:
							axis_relative_modes[3] = true;
							break;
						case 18: //compatibility
						case 84: // M84
							if (!state.isActiveState()) state = IDLE; else job.Stop();
							if(code_seen('S'))
								{
									stepper_inactive_time = code_value() * 1000;
								}
							else
								{
									bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
									if(all_axis)
										{
											st_synchronize();
											disable_e0();
											disable_e1();
											disable_e2();
											finishAndDisableSteppers();
											LCD_MESSAGEPGM(MSG_STEPPER_RELEASED);
										}
									else
										{
											String message = "Steppers ";
											st_synchronize();
											if(code_seen('X')) { disable_x(); message+="X";};
											if(code_seen('Y')) { disable_y(); message+="Y";};
											if(code_seen('Z')) { disable_z(); message+="Z";};
#if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
											if(code_seen('E'))
												{
													disable_e0();
													disable_e1();
													disable_e2();
													message+="E";
												}
#endif
											message += " released";
											LCD_MESSAGE(message);
										}
								}
							break;
						case 85: // M85what does this do?
							code_seen('S');
							max_inactive_time = code_value() * 1000;
							break;
						case 92: // M92
							for(int8_t i=0; i < NUM_AXIS; i++)
								{
									if(code_seen(axis_codes[i]))

										if(i == 3)   // E
											{
												float value = code_value();
												if(value < 20.0)
													{
														float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
														max_e_jerk *= factor;
														max_feedrate[i] *= factor;
														axis_steps_per_sqr_second[i] *= factor;
													}
												axis_steps_per_unit[i] = value;
											}
										else
											{
												axis_steps_per_unit[i] = code_value();
											}
								}
							break;
						case 115: // M115 Report firmware string
							SerialprintPGM(MSG_M115_REPORT);
							state.SetController (SERIAL_HOST);
							EEPROM_printHistory();
							LCD_MESSAGEPGM("USB host connected");
							if (state==WELCOME) state = IDLE;
							break;
						case 70:  // M70 display message (replicator)
						case 117: // M117 display message
							starpos = (strchr(strchr_pointer + 4,'*'));
							if(starpos!=NULL)
								*(starpos-1)='\0';
							if (strchr_pointer[0]==0)
								LCD_MESSAGE_CLEAR;
							else
								LCD_MESSAGEPRI(strchr_pointer, USER_MESSAGE_PRIORITY);
							break;
						case 114: // M114  report current position
							SERIAL_PROTOCOLPGM("X:");
							SERIAL_PROTOCOL(current_head_position[X_AXIS]);
							SERIAL_PROTOCOLPGM("Y:");
							SERIAL_PROTOCOL(current_head_position[Y_AXIS]);
							SERIAL_PROTOCOLPGM("Z:");
							SERIAL_PROTOCOL(current_head_position[Z_AXIS]);
							SERIAL_PROTOCOLPGM("E:");
							SERIAL_PROTOCOL(current_head_position[E_AXIS]);

							SERIAL_PROTOCOLPGM(MSG_COUNT_X);
							SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
							SERIAL_PROTOCOLPGM("Y:");
							SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
							SERIAL_PROTOCOLPGM("Z:");
							SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

							SERIAL_PROTOCOLLN("");
							break;
						case 120: // M120
							enable_endstops(false) ;
							break;
						case 121: // M121
							enable_endstops(true) ;
							break;
						case 119: // M119
							SERIAL_PROTOCOLLN(MSG_M119_REPORT);
#if (X_MIN_PIN > -1)
							SERIAL_PROTOCOLPGM(MSG_X_MIN);
							SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (X_MAX_PIN > -1)
							SERIAL_PROTOCOLPGM(MSG_X_MAX);
							SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (Y_MIN_PIN > -1)
							SERIAL_PROTOCOLPGM(MSG_Y_MIN);
							SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (Y_MAX_PIN > -1)
							SERIAL_PROTOCOLPGM(MSG_Y_MAX);
							SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (Z_MIN_PIN > -1)
							SERIAL_PROTOCOLPGM(MSG_Z_MIN);
							SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MINENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if (Z_MAX_PIN > -1)
							SERIAL_PROTOCOLPGM(MSG_Z_MAX);
							SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAXENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
							break;
							//TODO: update for all axis, use for loop
						case 201: // M201
							for(int8_t i=0; i < NUM_AXIS; i++)
								{
									if(code_seen(axis_codes[i]))
										{
											max_acceleration_units_per_sq_second[i] = code_value();
											axis_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
										}
								}
							break;
#if 0 // Not used for Sprinter/grbl gen6
						case 202: // M202
							for(int8_t i=0; i < NUM_AXIS; i++)
								{
									if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
								}
							break;
#endif
						case 203: // M203 max feedrate mm/sec
							for(int8_t i=0; i < NUM_AXIS; i++)
								{
									if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
								}
							break;
						case 204: // M204 acclereration S normal moves T filmanent only moves
							{
								if(code_seen('S')) acceleration = code_value() ;
								if(code_seen('T')) retract_acceleration = code_value() ;
							}
							break;
						case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
							{
								if(code_seen('S')) minimumfeedrate = code_value();
								if(code_seen('T')) mintravelfeedrate = code_value();
								if(code_seen('B')) minsegmenttime = code_value() ;
								if(code_seen('X')) max_xy_jerk = code_value() ;
								if(code_seen('Z')) max_z_jerk = code_value() ;
								if(code_seen('E')) max_e_jerk = code_value() ;
							}
							break;
						case 206: // M206 additional homeing offset
							for(int8_t i=0; i < 3; i++)
								{
									if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
								}
							break;
#ifdef FWRETRACT
						case 207: //M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
							{
								if(code_seen('S'))
									{
										retract_length = code_value() ;
									}
								if(code_seen('F'))
									{
										retract_feedrate = code_value() ;
									}
								if(code_seen('Z'))
									{
										retract_zlift = code_value() ;
									}
							}
							break;
						case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
							{
								if(code_seen('S'))
									{
										retract_recover_length = code_value() ;
									}
								if(code_seen('F'))
									{
										retract_recover_feedrate = code_value() ;
									}
							}
							break;

						case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
							{
								if(code_seen('S'))
									{
										int t= code_value() ;
										switch(t)
											{
												case 0:
													autoretract_enabled=false;
													retracted=false;
													break;
												case 1:
													autoretract_enabled=true;
													retracted=false;
													break;
												default:
													SERIAL_ECHO_START;
													SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
													SERIAL_ECHO(cmdbuffer[bufindr]);
													SERIAL_ECHOLNPGM("\"");
											}
									}
							} break;
#endif // FWRETRACT
#if EXTRUDERS > 1
						case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
							{
								if(setTargetedHotend(218))
									{
										break;
									}
								if(code_seen('X'))
									{
										extruder_offset[X_AXIS][tmp_extruder] = code_value();
									}
								if(code_seen('Y'))
									{
										extruder_offset[Y_AXIS][tmp_extruder] = code_value();
									}
								SERIAL_ECHO_START;
								SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
								for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
									{
										SERIAL_ECHO(" ");
										SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
										SERIAL_ECHO(",");
										SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
									}
								SERIAL_ECHOLN("");
							} break;
#endif
						case 220: // M220 S<factor in percent>- set speed factor override percentage
							{
								if(code_seen('S'))
									{
										feedmultiply = code_value() ;
										if(feedmultiply<10) feedmultiply=10;
										if(feedmultiply>250) feedmultiply=250;
										feedmultiplychanged=true;
									}
							}
							break;
						case 221: // M221 S<factor in percent>- set extrude factor override percentage
							{
								if(code_seen('S'))
									{
										extrudemultiply = code_value() ;
										if(extrudemultiply<10)	extrudemultiply=10;
										if(extrudemultiply>250)	extrudemultiply=250;
									}
							}
							break;

#if defined(LARGE_FLASH) && LARGE_FLASH == true && defined(BEEPER) && BEEPER > -1
						case 300: // M300
							{
								int beepS = 1;
								int beepP = 1000;
								if(code_seen('S')) beepS = code_value();
								if(code_seen('P')) beepP = code_value();
								tone(BEEPER, beepS);
								delay(beepP);
								noTone(BEEPER);
							}
							break;
#endif // M300

#ifdef PIDTEMP
						case 301: // M301
							{
								if(code_seen('P')) Kp = code_value();
								if(code_seen('I')) Ki = scalePID_i(code_value());
								if(code_seen('D')) Kd = scalePID_d(code_value());

#ifdef PID_ADD_EXTRUSION_RATE
								if(code_seen('C')) Kc = code_value();
#endif
								updatePID();
								SERIAL_PROTOCOLPGM(MSG_OK);
								SERIAL_PROTOCOLPGM(" p:");
								SERIAL_PROTOCOL(Kp);
								SERIAL_PROTOCOL(" i:");
								SERIAL_PROTOCOL(unscalePID_i(Ki));
								SERIAL_PROTOCOL(" d:");
								SERIAL_PROTOCOL(unscalePID_d(Kd));
#ifdef PID_ADD_EXTRUSION_RATE
								SERIAL_PROTOCOLPGM(" c:");
								SERIAL_PROTOCOL(Kc);
#endif
								SERIAL_PROTOCOLLN("");
							}
							break;
#endif //PIDTEMP
#ifdef PIDTEMPBED
						case 304: // M304
							{
								if(code_seen('P')) bedKp = code_value();
								if(code_seen('I')) bedKi = scalePID_i(code_value());
								if(code_seen('D')) bedKd = scalePID_d(code_value());

								updatePID();
								SERIAL_PROTOCOLPGM(MSG_OK);
								SERIAL_PROTOCOLPGM(" p:");
								SERIAL_PROTOCOL(bedKp);
								SERIAL_PROTOCOL(" i:");
								SERIAL_PROTOCOL(unscalePID_i(bedKi));
								SERIAL_PROTOCOL(" d:");
								SERIAL_PROTOCOL(unscalePID_d(bedKd));
								SERIAL_PROTOCOLLN("");
							}
							break;
#endif //PIDTEMP
						case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
							{
#ifdef PHOTOGRAPH_PIN
#if (PHOTOGRAPH_PIN > -1)
								const uint8_t NUM_PULSES=16;
								const float PULSE_LENGTH=0.01524;
								for(int i=0; i < NUM_PULSES; i++)
									{
										WRITE(PHOTOGRAPH_PIN, HIGH);
										_delay_ms(PULSE_LENGTH);
										WRITE(PHOTOGRAPH_PIN, LOW);
										_delay_ms(PULSE_LENGTH);
									}
								delay(7.33);
								for(int i=0; i < NUM_PULSES; i++)
									{
										WRITE(PHOTOGRAPH_PIN, HIGH);
										_delay_ms(PULSE_LENGTH);
										WRITE(PHOTOGRAPH_PIN, LOW);
										_delay_ms(PULSE_LENGTH);
									}
#endif
#endif
							}
							break;

						case 302: // allow cold extrudes
							{
								allow_cold_extrudes(true);
							}
							break;
						case 303: // M303 PID autotune
							{
								float temp = 150.0;
								int e=0;
								int c=5;
								if (code_seen('E')) e=code_value();
								if (e<0)
									temp=70;
								if (code_seen('S')) temp=code_value();
								if (code_seen('C')) c=code_value();
								LCD_MESSAGE_CLEAR;
								VISIT;
								LCD_MESSAGEPGM("TUNING PID");
								PID_autotune(temp, e, c);
								LCD_MESSAGEPGM("PID TUNE DONE");
							}
							break;
						case 310:
							if (code_seen('C')) BBLookAheadCool=code_value();
							if (code_seen('H')) BBLookAheadHot=code_value();
							break;
						case 400: // M400 finish all moves
							{
								st_synchronize();
							}
							break;
#if 0 
						case 401: // Memory position
							printer_state.memoryX = printer_state.currentPositionSteps[0];
							printer_state.memoryY = printer_state.currentPositionSteps[1];
							printer_state.memoryZ = printer_state.currentPositionSteps[2];
							break;
						case 402: // Go to stored position
							{
								bool all = !(GCODE_HAS_X(com) && GCODE_HAS_Y(com) && GCODE_HAS_Z(com));
								move_steps((all || GCODE_HAS_X(com) ? printer_state.memoryX-printer_state.currentPositionSteps[0] : 0)
										   ,(all || GCODE_HAS_Y(com) ? printer_state.memoryY-printer_state.currentPositionSteps[1] : 0)
										   ,(all || GCODE_HAS_Z(com) ? printer_state.memoryZ-printer_state.currentPositionSteps[2] : 0)
										   ,0,(GCODE_HAS_F(com) ? com->F : printer_state.feedrate),false,ALWAYS_CHECK_ENDSTOPS);
							}
							break;

#endif 

						case 500: // Store settings in EEPROM
							{
								EEPROM_StoreSettings();
							}
							break;
						case 501: // Read settings from EEPROM
							{
								EEPROM_RetrieveSettings();
							}
							break;
						case 502: // Revert to default settings
							{
								EEPROM_RetrieveSettings(true);
							}
							break;
						case 503: // print settings currently in memory
							{
								EEPROM_printSettings();
							}
							break;
						case 504:
							ResetMetrics();
							break;



#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
						case 540:
							{
								if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
							}
							break;
#endif
#ifdef ULTRA_LCD

#ifdef FILAMENTCHANGEENABLE
						case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
							{
								float target[4];
								float lastpos[4];
								target[X_AXIS]=current_position[X_AXIS];
								target[Y_AXIS]=current_position[Y_AXIS];
								target[Z_AXIS]=current_position[Z_AXIS];
								target[E_AXIS]=current_position[E_AXIS];
								lastpos[X_AXIS]=current_position[X_AXIS];
								lastpos[Y_AXIS]=current_position[Y_AXIS];
								lastpos[Z_AXIS]=current_position[Z_AXIS];
								lastpos[E_AXIS]=current_position[E_AXIS];
								//retract by E
								if(code_seen('E'))
									{
										target[E_AXIS]+= code_value();
									}
								else
									{
#ifdef FILAMENTCHANGE_FIRSTRETRACT
										target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
#endif
									}
								plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

								//lift Z
								if(code_seen('Z'))
									{
										target[Z_AXIS]+= code_value();
									}
								else
									{
#ifdef FILAMENTCHANGE_ZADD
										target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
#endif
									}
								plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

								//move xy
								if(code_seen('X'))
									{
										target[X_AXIS]+= code_value();
									}
								else
									{
#ifdef FILAMENTCHANGE_XPOS
										target[X_AXIS]= FILAMENTCHANGE_XPOS ;
#endif
									}
								if(code_seen('Y'))
									{
										target[Y_AXIS]= code_value();
									}
								else
									{
#ifdef FILAMENTCHANGE_YPOS
										target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
#endif
									}

								plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

								if(code_seen('L'))
									{
										target[E_AXIS]+= code_value();
									}
								else
									{
#ifdef FILAMENTCHANGE_FINALRETRACT
										target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
#endif
									}

								plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

								//finish moves
								st_synchronize();
								//disable extruder steppers so filament can be removed
								disable_e0();
								disable_e1();
								disable_e2();
								delay(100);
								LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
								uint8_t cnt=0;
								while(!LCD_CLICKED)
									{
										cnt++;
										manage_heater();
										manage_inactivity();
										lcd_update();

#if BEEPER > -1
										if(cnt==0)
											{
												SET_OUTPUT(BEEPER);

												WRITE(BEEPER,HIGH);
												delay(3);
												WRITE(BEEPER,LOW);
												delay(3);
											}
#endif
									}

								//return to normal
								if(code_seen('L'))
									{
										target[E_AXIS]+= -code_value();
									}
								else
									{
#ifdef FILAMENTCHANGE_FINALRETRACT
										target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
#endif
									}
								current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
								plan_set_e_position(current_position[E_AXIS]);
								plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
								plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
								plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
								plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
							}
							break;
#endif //FILAMENTCHANGEENABLE
#endif
						case 999: // Restart after being stopped
							state= IDLE;
							gcode_LastN = Stopped_gcode_LastN;
							job.Stop();
							LCD_MESSAGE_CLEAR;
							FlushSerialRequestResend();
							LCD_MESSAGEPGM("RESET");
							break;

							// extensions start here:
						case 300:	// beep the beeper for a beep
#ifdef ULTRA_LCD
							beep();
#endif
							break;

						case 420: 	// set RGB "mood light"
							{
								if (code_seen('P'))
									{
#ifdef BLINK_M
										BlinkM_playScript(BLINK_M_ADDR,code_value(),0,0);
#endif
										break;
									}
								int r,g,b=0;
								if (code_seen('R')) r=code_value();
								if (code_seen('E')) g=code_value();
								// yes, this is an E, not a G
								if (code_seen('B')) b=code_value();
								SetStatusLEDColor(r,g,b);
							}
							break;
						case 73:	// set progress
							if (code_seen ('Q'))		// ack, this comes in as an int...oh well....
								{
									float p = code_value();
									if (p==0.0) job.Start(true);		// setting percent done to 0 will also reset the job start point
									else job.SetPercent(p);
								}
							break;

							// 		case 72: // play song
							// 			break;
					}
			}

		else
			if(code_seen('T'))
				{
					tmp_extruder = code_value();
					if(tmp_extruder >= EXTRUDERS)
						{
							SERIAL_ECHO_START;
							SERIAL_ECHO("T");
							SERIAL_ECHO(tmp_extruder);
							SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
						}
					else
						{
							boolean make_move = false;
							if(code_seen('F'))
								{
									make_move = true;
									next_feedrate = code_value();
									if(next_feedrate > 0.0)
										{
											feedrate = next_feedrate;
										}
								}
#if EXTRUDERS > 1
							if(tmp_extruder != active_extruder)
								{
									// Save current position to return to after applying extruder offset
									memcpy(destination, current_position, sizeof(destination));
									// Offset extruder (only by XY)
									int i;
									for(i = 0; i < 2; i++)
										{
											current_position[i] = current_position[i] -
																  extruder_offset[i][active_extruder] +
																  extruder_offset[i][tmp_extruder];
										}
									// Set the new active extruder and position
									active_extruder = tmp_extruder;
									plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
									// Move to the old position if 'F' was in the parameters
									if(make_move && Stopped == false)
										{
											prepare_move();
										}
								}
#endif
							SERIAL_ECHO_START;
							SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
							SERIAL_PROTOCOLLN((int)active_extruder);
						}
				}

			else
				{
					SERIAL_ECHO_START;
					SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
					SERIAL_ECHO(cmdbuffer[bufindr]);
					SERIAL_ECHOLNPGM("\"");
				}

	ClearToSend();
}

//-------------------------------------------------------------------------------------------------------------------
void FlushSerialRequestResend()
{
	//char cmdbuffer[bufindr][100]="Resend:";
	MYSERIAL.flush();
	SERIAL_PROTOCOLPGM(MSG_RESEND);
	SERIAL_PROTOCOLLN(gcode_LastN + 1);
	ClearToSend();
}

//-------------------------------------------------------------------------------------------------------------------
void ClearToSend()
{
	previous_millis_cmd = millis();
#ifdef SDSUPPORT
	if(fromsd[bufindr])
		return;
#endif //SDSUPPORT
	SERIAL_PROTOCOLLNPGM(MSG_OK);
}

//-------------------------------------------------------------------------------------------------------------------
void get_coordinates()
{
	bool seen[4]= {false,false,false,false};
	for(int8_t i=0; i < NUM_AXIS; i++)
		{
			if(code_seen(axis_codes[i]))
				{
					destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_head_position[i];
					seen[i]=true;
				}
			else destination[i] = current_head_position[i]; //Are these else lines really needed?
		}

	// does this move have an extract?
	if (state != SAVING)
		{
			if (seen[E_AXIS])
				{
					if (!seen[0] && !seen[1] && !seen[2])
						{
							if (destination[E_AXIS] > current_head_position[E_AXIS] )
								state = EXTRUDE;
							else
								state = RETRACT;
							// 				progress_string[1] = (ftostr32a(destination[E_AXIS] - current_head_position[E_AXIS]));
							// 				progress_string[1] += "mm";
						}
					else
						state = PRINTING;
				}
			else state = MOVING;
		}

	if(code_seen('F'))
		{
			next_feedrate = code_value();
			if(next_feedrate > 0.0) feedrate = next_feedrate;
		}
	if(code_seen('Q'))
		{
		float next_quality = code_value();
		if(next_quality < 0.001) next_quality = 0.001;
		if(next_quality > 100) next_quality = 100;
		quality = next_quality;
		}


#ifdef FWRETRACT
	if(autoretract_enabled)
		if( !(seen[X_AXIS] || seen[Y_AXIS] || seen[Z_AXIS]) && seen[E_AXIS])
			{
				float echange=destination[E_AXIS]-current_head_position[E_AXIS];
				if(echange<-MIN_RETRACT) //retract
					{
						if(!retracted)
							{
								destination[Z_AXIS]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
								//if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
								float correctede=-echange-retract_length;
								//to generate the additional steps, not the destination is changed, but inversely the current position
								current_head_position[E_AXIS]+=-correctede;
								feedrate=retract_feedrate;
								retracted=true;
							}
					}
				else
					if(echange>MIN_RETRACT) //retract_recover
						{
							if(retracted)
								{
									//current_position[Z_AXIS]+=-retract_zlift;
									//if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
									float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
									current_head_position[E_AXIS]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
									feedrate=retract_recover_feedrate;
									retracted=false;
								}
						}
			}
#endif //FWRETRACT
}

//-------------------------------------------------------------------------------------------------------------------
void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
	bool relative_mode_backup = relative_mode;
	relative_mode = true;
#endif
	get_coordinates();
#ifdef SF_ARC_FIX
	relative_mode=relative_mode_backup;
#endif

	if(code_seen('I'))
		{
			offset[0] = code_value();
		}
	else
		{
			offset[0] = 0.0;
		}
	if(code_seen('J'))
		{
			offset[1] = code_value();
		}
	else
		{
			offset[1] = 0.0;
		}
}

//-------------------------------------------------------------------------------------------------------------------
void clamp_to_software_endstops(float target[3])
{
	if (min_software_endstops)
		{
			if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
			if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
			if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
		}

	if (max_software_endstops)
		{
			if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
			if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
			if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
		}
}


//-------------------------------------------------------------------------------------------------------------------
void prepare_move()
{
	clamp_to_software_endstops(destination);

	previous_millis_cmd = millis();
	// Do not use feedmultiply for E or Z only moves
	if( (current_head_position[X_AXIS] == destination [X_AXIS]) && (current_head_position[Y_AXIS] == destination [Y_AXIS]))
		{
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		}
	else
		{
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder,quality);
		}

	// record the net movement of the filaments
	total_filament +=  destination[E_AXIS]-current_head_position[E_AXIS] ;

	for(int8_t i=0; i < NUM_AXIS; i++)
		{
			// record all motion, forward or backwards, into the total movement
			total_distance[i] += fabs (current_head_position[i] -  destination[i]);
			current_head_position[i] = destination[i];
		}
}

//-------------------------------------------------------------------------------------------------------------------
void prepare_arc_move(char isclockwise)
{
	float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

	// Trace the arc
	mc_arc(current_head_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

	// As far as the parser is concerned, the position is now == target. In reality the
	// motion control system might still be processing the action and the real tool position
	// in any intermediate location.
	for(int8_t i=0; i < NUM_AXIS; i++)
		{
			current_head_position[i] = destination[i];
		}
	previous_millis_cmd = millis();
}

#ifdef CONTROLLERFAN_PIN
unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
	if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
		{
			lastMotorCheck = millis();

			if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN)
#if EXTRUDERS > 2
					|| !READ(E2_ENABLE_PIN)
#endif
#if EXTRUDER > 1
					|| !READ(E2_ENABLE_PIN)
#endif
					|| !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
				{
					lastMotor = millis(); //... set time to NOW so the fan will turn on
				}

			if ((millis() - lastMotor) >= (CONTROLLERFAN_SEC*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
				{
					WRITE(CONTROLLERFAN_PIN, LOW); //... turn the fan off
				}
			else
				{
					WRITE(CONTROLLERFAN_PIN, HIGH); //... turn the fan on
				}
		}
}
#endif

#ifdef EXTRUDERFAN_PIN
unsigned long lastExtruderCheck = 0;

void extruderFan()
{
	if ((millis() - lastExtruderCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
		{
			lastExtruderCheck = millis();

			if (degHotend(active_extruder) < EXTRUDERFAN_DEC)
				{
					WRITE(EXTRUDERFAN_PIN, LOW); //... turn the fan off
				}
			else
				{
					WRITE(EXTRUDERFAN_PIN, HIGH); //... turn the fan on
				}
		}
}
#endif
void DisableAllSteppers()
{
	disable_x();
	disable_y();
	disable_z();
	disable_e0();
	disable_e1();
	disable_e2();
}

//-------------------------------------------------------------------------------------------------------------------
void manage_inactivity()
{
	if( (now - previous_millis_cmd) >  max_inactive_time )
		if(max_inactive_time)
			kill(0);
	if(stepper_inactive_time)
		{
			if( (now - previous_millis_cmd) >  stepper_inactive_time )
				{
					if(blocks_queued() == false)
						{
							DisableAllSteppers();
						}
				}
		}
#if( KILL_PIN>-1 )
	if( 0 == READ(KILL_PIN) )
		kill();
#endif
#ifdef CONTROLLERFAN_PIN
	controllerFan(); //Check if fan should be turned on to cool stepper drivers down
#endif
#ifdef EXTRUDER_RUNOUT_PREVENT
	if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 )
		if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
			{
				bool oldstatus=READ(E0_ENABLE_PIN);
				enable_e0();
				float oldepos=current_head_position[E_AXIS];
				float oldedes=destination[E_AXIS];
				plan_buffer_line(current_head_position[X_AXIS], current_head_position[Y_AXIS], current_head_position[Z_AXIS],
								 current_head_position[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS],
								 EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
				current_head_position[E_AXIS]=oldepos;
				destination[E_AXIS]=oldedes;
				plan_set_e_position(oldepos);
				previous_millis_cmd=millis();
				st_synchronize();
				WRITE(E0_ENABLE_PIN,oldstatus);
			}
#endif
	check_axes_activity();
}

//-------------------------------------------------------------------------------------------------------------------
void kill(int fatal)
{
	//	if (state.isActiveState ()) job.Stop();
	SERIAL_ERROR_START;
	if (fatal)
		{
			state = ERROR;
			//	Error(MSG_ERR_KILLED);
		}
	else
		state = SLEEPING;
	disable_heater();
	DisableAllSteppers();
	cli(); // Stop interrupts
	if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);
	suicide();
#if !defined(__AVR_AT90USB1286__) && !defined(__AVR_AT90USB1287__) // this will kill the usb serial so the messages aren't seen
	while(1); // Wait for reset
#endif
}

//-------------------------------------------------------------------------------------------------------------------
void Stop()
{
	disable_heater();
	if(!Stopped)
		{
			Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
			state = ERROR;
			Error(MSG_ERR_STOPPED);
		}
}

//-------------------------------------------------------------------------------------------------------------------
bool IsStopped()
{
	return Stopped;
};

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
	val &= 0x07;
	switch(digitalPinToTimer(pin))
		{
#if defined(TCCR0A)
			case TIMER0A:
			case TIMER0B:
				//         TCCR0B &= ~(CS00 | CS01 | CS02);
				//         TCCR0B |= val;
				break;
#endif

#if defined(TCCR1A)
			case TIMER1A:
			case TIMER1B:
				//         TCCR1B &= ~(CS10 | CS11 | CS12);
				//         TCCR1B |= val;
				break;
#endif

#if defined(TCCR2)
			case TIMER2:
			case TIMER2:
				TCCR2 &= ~(CS10 | CS11 | CS12);
				TCCR2 |= val;
				break;
#endif

#if defined(TCCR2A)
			case TIMER2A:
			case TIMER2B:
				TCCR2B &= ~(CS20 | CS21 | CS22);
				TCCR2B |= val;
				break;
#endif

#if defined(TCCR3A)
			case TIMER3A:
			case TIMER3B:
			case TIMER3C:
				TCCR3B &= ~(CS30 | CS31 | CS32);
				TCCR3B |= val;
				break;
#endif

#if defined(TCCR4A)
			case TIMER4A:
			case TIMER4B:
			case TIMER4C:
				TCCR4B &= ~(CS40 | CS41 | CS42);
				TCCR4B |= val;
				break;
#endif

#if defined(TCCR5A)
			case TIMER5A:
			case TIMER5B:
			case TIMER5C:
				TCCR5B &= ~(CS50 | CS51 | CS52);
				TCCR5B |= val;
				break;
#endif
		}
}
#endif //FAST_PWM_FAN

//-------------------------------------------------------------------------------------------------------------------
bool setTargetedHotend(int code)
{
	tmp_extruder = active_extruder;
	if(code_seen('T'))
		{
			tmp_extruder = code_value();
			if(tmp_extruder >= EXTRUDERS)
				{
					SERIAL_ECHO_START;
					switch(code)
						{
							case 104:
								Error(MSG_M104_INVALID_EXTRUDER);
								break;
							case 105:
								Error(MSG_M105_INVALID_EXTRUDER);
								break;
							case 109:
								Error(MSG_M109_INVALID_EXTRUDER);
								break;
						}
					SERIAL_ECHOLN(tmp_extruder);
					return true;
				}
		}
	return false;
}

//-------------------------------------------------------------------------------------------------------------------
extern unsigned long I2C_timeouts;
unsigned long last_update = millis();

void manage_other_tasks()
{
	now = millis();
	manage_heater();
	manage_inactivity();
	state.Update();
	LCD_STATUS;
	unsigned long dt = now - last_update ;
	if (dt > 500)
		{
			SERIAL_ECHO_START;
			SERIAL_ECHOPAIR ("Time:" , (unsigned long) (dt));
			SERIAL_ECHOLNPGM ("Excess time between updates!");
		}

	if (I2C_timeouts > 0 )
		{
			SERIAL_ECHO_START;
			SERIAL_ECHOPAIR ("Errors:" , I2C_timeouts);
			SERIAL_ECHOLNPGM ("I2C errors");
			I2C_timeouts=0;
		}

	last_update = now;

	dt = (now - last_clock_update);

	if (dt > 1000)
		{
			dt /=1000;
			total_on_time += dt;
			last_clock_update = now;
			if (degTargetBed() > 10)
				{
					total_bed_time+=dt;
					bed_degree_seconds += degBed() * dt;
				}
			if (degTargetHotend0() > 0)
				{
					total_extruder_time0+=dt;
					extruder0_degree_seconds += degHotend0() * dt;
				}
#if EXTRUDERS>1
			if (degTargetHotend1() > 0)
				{
					total_extruder_time1+=dt;
					extruder1_degree_seconds += degTargetHotend1() * dt;
				}
#endif
			if (state.isActiveState() && state!=SAVING) total_printing_time += dt;

			counter -= dt;
			if (counter<=0)
				{
					EEPROM_StoreFrequentSettings();
					counter = EEPROM_UPDATE_RATE ;
				}
		}
}
