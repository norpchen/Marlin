// State.h

#ifndef _STATE_h
#define _STATE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Marlin.h"


enum STATES
{
	IDLE=0,
	PRINTING,
	SAVING,
	MOVING,
	HEATING,
	RETRACT,
	EXTRUDE,
	ERROR,
	SLEEPING,
	DONE,
	PAUSED,
	WELCOME,
	DEBUG,
	MAX_STATES
};


enum CONTROLLER 
{
	PANEL=0,
	SERIAL_HOST,
	SDCARD,
	MAX_CONTROLLERS
};


const char * const CONTROLLER_STRINGS[MAX_CONTROLLERS] =
{
	"UI",
	"USB",
	"SD"
};

const char STATE_STRINGS[MAX_STATES][10]=
{
	" IDLE   ",
	"PRINTING",
	"SD SAVE ",
	"MOVING  ",
	"HEATING ",
	"RETRACT ",
	"EXTRUDE ",
	" ERROR  ",
	" SLEEP  ",
	" DONE   ",
	" PAUSED ",
	"  ON  " ,
	"  DEBUG "
	
};


extern String progress_string[3];


class State
{
 private:
	 STATES current_state;
	 CONTROLLER controller;
	 unsigned long last_block_refresh;

 public:
	void init();
	void Update ();
	State() { init();} 
	State & operator=(STATES newstate);
	operator STATES() const { return current_state; }
	CONTROLLER Controller() const { return controller; }
	void SetController(CONTROLLER val) { controller = val; }

	bool isActiveState() 
	{ 
		return (current_state < ERROR && current_state > IDLE);
	};
};

extern State state;
;;

#endif

