//
//
//
#include "Marlin.h"
#include "State.h"
#include "Utility.h"
#include "cardreader.h"
#include "ultralcd.h"
#include "BlinkM_funcs.h"
#include "Configuration.h"
#include "planner.h"

extern unsigned long time_of_last_command;

#ifdef SDSUPPORT
extern CardReader card;
#endif

State state;

String progress_string[3];

void State::init()
{
	current_state = WELCOME;
	controller  = PANEL;
	last_block_refresh=-1;
}

//-------------------------------------------------------------------------------------------------------------------
void State::Update ()
{
	int time_since_command = (now - time_of_last_command )/1000;
	// we haven't heard from the host in a while...or ever?
	if (time_of_last_command<0 || time_since_command > 10) controller = PANEL;
	if (card.saving) controller = SERIAL_HOST;
	if (card.sdprinting) controller = SDCARD;

	int r,g,b =0 ;
	// way of tracking the screen updates  to not update when not needed
	static int last_phase = -1;

	// every 1024 ms  --- don't want to do this too often, as we are actively printing and need to keep  things light
	bool refresh_lcd_messages=(now>>10) != last_block_refresh;

	switch (current_state)
	{
		// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	case SLEEPING:
	case IDLE :
		progress_string[1] = "Last Cmd";
		progress_string[2] = EchoTimeSpan((now - time_of_last_command)/1000,true);
		break;

		// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	case WELCOME:
		{
#ifdef SAVE_DEVICE_METRICS
			progress_string[1]=String(" LIFETIME");
#else
			progress_string[1]=String(" SESSION ");
#endif
			switch ((now/3000) % 10)							// yah, maybe a little slower, but we're not doing anything in this state, so who cares?
			{
			case 0: progress_string[1]=String("ver ") + VERSION_STRING;
				progress_string[0]=__DATE__;
				progress_string[2]=" "  __TIME__;
				break;
			case 1: progress_string[0]=String(" FREE MEM  ");
				progress_string[1]="SYSTEM   ";
				progress_string[2]=String (freeMemory()) +String("     ");;
				break;
			case 4:
				progress_string[0]=" FILAMENT ";
				progress_string[2]=ftostr(total_filament/1000,5,2)+String ("m");
				break;
			case 3:
				progress_string[0]=" PRINTING ";
				progress_string[2]=String (EchoTimeSpan(total_printing_time,true))+String(" ");
				break;
			case 2:
				progress_string[0]="POWERED ON ";
				progress_string[2]=String (EchoTimeSpan(total_on_time,true));
				break;
			case 5:
				progress_string[0]="EXTRD 0 ON ";
				progress_string[2]=String (EchoTimeSpan(total_extruder_time0,true))+String(" ");
				break;
			case 6:
				progress_string[0]="BED HTR ON ";
				progress_string[2]=String (EchoTimeSpan(total_bed_time,true));
				break;
			case 7:
				progress_string[0]="EXT AVGTEMP";
				if (total_extruder_time0<1)total_extruder_time0 = 1;
				progress_string[2]=String(" ") +ftostr(extruder0_degree_seconds/total_extruder_time0,3,1)+String("\001  ");
				break;
			case 8:
				progress_string[0]="BED AVGTEMP";
				if (total_bed_time<1)total_bed_time = 1;
				progress_string[2]=String(" ") +ftostr(bed_degree_seconds/total_bed_time,3,1)+String("\001  ");
				break;
			case 9:  progress_string[0]=String("  TIME ON  ");
				progress_string[1]="SYSTEM   ";
				progress_string[2]=EchoTimeSpan(now / 1000,true);
				break;
			}
			if ((now & 0x3f) ==0) 
		//	if (refresh_lcd_messages)
			{
				int v = 32+(now & 0x0f)<<2;

			     int r = random(v);
			     int g = random(v);
			     int  b = max(v - r - g,0);
				SetLEDColor(r,g,b, false);
			}

		}
		break;

		// ------------------------------------------------------------------------------------------------------------------------------------------------------------
		// Active States
		// cycle through four relevant but infrequently changed status variables:
		// speed (feedrate) %
		// flowrate %
		// Z height
		// fan speed %


		// gotta be careful not to do too much chatter over I2C or SD card access or whatever else while printing 
		// as it can cause a freeze up
	case MOVING:
	case PRINTING:
		{
			if (current_state == MOVING) b = 255; 
			else 
			{
				g = 255; 
				if (job.JobState()!=Job::RUNNING) job.Start();
			};
			int phase = (job.JobTime()>>1) & 0x03;
			if (phase != last_phase)
			{
				switch (phase)
				{
				case 0:
					progress_string[1] = String("FEED:");
					progress_string[1]+=String (feedmultiply);
					progress_string[1]+="%  ";
					break;
				case 1:
					progress_string[1] = String("FLOW:");
					progress_string[1]+=String (extrudemultiply);
					progress_string[1]+="%  ";
					break;
				case 2:
					progress_string[1] = String(ftostr((current_head_position[2]/(job.JobTime())*3600),3,1)+String("mm/h"));
					break;
				case 3:
					progress_string[1] = String("FAN:");
					if (FanSpeed==0)
						progress_string[1]+="OFF    ";
					else
					{
						progress_string[1]+=String (100*FanSpeed/255);
						progress_string[1]+=" % ";
					}
					break;
				}
			}
			last_phase = phase;
		}

		// case roll thru

	case RETRACT:
	case EXTRUDE:
		// return to idle state if we were in an active moving state, our buffer is empty and haven't done anything in 3 seconds
		r = 255;
		if (blocks_queued()==NULL && time_since_command > 3)
		{
			current_state = IDLE;
			return;
		}
		else
		{
			if (refresh_lcd_messages)
			{
				LCD_MESSAGEPRI(String("MOVES: ") + String(itostr2(get_buffer_depth())) + String(" ") + String(ftostr(job.Speed(),3,2))+String("mm/s"),DEFAULT_MESSAGE_PRIORITY);
				last_block_refresh = now;
			}
		}

		// case roll thru

	case HEATING:
	case SAVING:
		{
			if (!refresh_lcd_messages) return;
			job.Update();

			if (current_state==SAVING)
			{
				int color = (gcode_N & 0x3)<<2;
				SetLEDColor(color,color,color, false);
			} 
			else
			{
			//	if (current_state!=HEATING) SetLEDColor(((int) current_head_position[X_AXIS] * 100) & 0xff ,((int) current_head_position[Y_AXIS]*100) & 0xff,((int) current_head_position[E_AXIS]*1000) & 0xff, false);

#ifdef SET_LED_COLOR_BY_ACTION
				if (current_state!=HEATING) 
					SetLEDColor(r,g,b,false);
#endif
			}


			// everything after here is updated 1x per sec

			progress_string[0] = String ("E") ;
			progress_string[0] += EchoTimeSpan(job.JobTime(),true);

			if (current_state==HEATING) return;
			if (current_state==SAVING)
			{
				progress_string[1] =String(((gcode_N-save_gcode_start)/job.JobTime())) + String(" N/S");
			}
			// are we printing from the SD card?  Then we know the percent done, which we can use to
			// predict ending time.
			if (controller == SDCARD || job.KnowPercentage())
			{
				static unsigned long last_job_time =0 ;
				if (!job.KnowPercentage() &&( job.JobTime() - last_job_time > 2))
				{
					job.SetSDPercent(card.percentDone());
					last_job_time = job.JobTime();
				}

				if (current_state!=HEATING && ((job.JobTime()) & 2) && job.Percent() > 1.0)		// alternate every second between ETA and % done
				{
					progress_string[2] = String("R");
					progress_string[2] += EchoTimeSpan(	job.CalculateRemainingTime (),true);
				}
				else
				{
					progress_string[2] = String(ftostr(job.Percent(),2,2)) + String(" %  ");
				}
				return;		// bail out
			}

			// we aren't printing from SD card
			// we don't know how big the file is, so all we can do is echo the line number we're on.
			progress_string[2] = String ("N:") + String (gcode_N-save_gcode_start);
			break;
		}

		// ------------------------------------------------------------------------------------------------------------------------------------------------------------

	case DONE:
		if (!refresh_lcd_messages) return;

		switch (((now / 2000)) & 0x03)
		{
		case 0: progress_string[1] = String ("X:") + String(ftostr(job.GetDistanceTravelled(X_AXIS)/1000,3,2)+String("m")); break;
		case 1: progress_string[1] = String ("Y:") + String(ftostr(job.GetDistanceTravelled(Y_AXIS)/1000,3,2)+String("m")); break;
		case 2: progress_string[1] = String ("Z:") + String(ftostr(job.GetDistanceTravelled(Z_AXIS)/1000,2,3)+String("m")); break;
		case 3: progress_string[1] = String ("E:") + String(ftostr(job.GetDistanceTravelled(E_AXIS)/1000,2,3) +String("m")); break;
		}

		// case roll thru

	case ERROR:
	case PAUSED:

		// blink the attention LED
#if STATUS_LED_PIN>=0
		pinMode(STATUS_LED_PIN, OUTPUT);
		digitalWrite(STATUS_LED_PIN, ((now >>7) & 1));
#endif

		break ;
	}
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------
State & State::operator=( STATES newstate )
{
	if (newstate==current_state) return *this;
	last_block_refresh=-1;
	// old state ending
	STATES previous_state = current_state;
	switch (current_state)
	{
		// don't  go to sleep or idle state from DONE
		case DONE: 
			if (newstate == SLEEPING || newstate == IDLE)	return *this;
			break;
	}
	current_state= newstate;

	// new state starting
	switch(current_state)
	{
		// -------------------------------------------------------------------------------------------------------------------------

	case ERROR:
		job.Stop(true);
		Error ("Printer halted. KILLED!!");
#ifdef BLINK_M
		BlinkM_playScript(BLINK_M_ADDR,BLINK_M_ERROR_SCRIPT,0,0);
#else
		SetLEDColor(ERROR_COLOR);
#endif
		break;

		// -------------------------------------------------------------------------------------------------------------------------

	case SLEEPING:
		SERIAL_ERRORLNPGM("Printer going to sleep");
		LCD_MESSAGEPGM("...sleeping... ");
#ifdef BLINK_M
		BlinkM_playScript(BLINK_M_ADDR,BLINK_M_SLEEP_SCRIPT,0,0);
#else
		SetLEDColor(SLEEP_COLOR);
#endif
		// deliberate roll thru case
	case IDLE :
		progress_string[1] = "Last Cmd";
		if (previous_state!=SAVING) progress_string[0] = "          "; else progress_string[0] =  EchoTimeSpan (job.JobTime(),1);
#ifdef BLINK_M
		BlinkM_playScript(BLINK_M_ADDR,BLINK_M_IDLE_SCRIPT,0,0);
#endif

		break;
	case HEATING :
#ifdef BLINK_M
		BlinkM_playScript(BLINK_M_ADDR,BLINK_M_HEATING_SCRIPT,0,0);
#endif
		break;
	case WELCOME:
#ifdef BLINK_M
	//	BlinkM_playScript(BLINK_M_ADDR,BLINK_M_WELCOME_SCRIPT,0,0);
#endif
		break;

		// -------------------------------------------------------------------------------------------------------------------------
	case DONE:
		job.Stop();
		progress_string[0] = EchoTimeSpan (job.JobTime(),1);
		progress_string[1] = "         ";//String ("Z:") + String(ftostr31(current_position[Z_AXIS]));

		progress_string[2] = ftostr ((job.GetFilamentUsed())/1000,2,3) + String("m");

		LCD_MESSAGEPGM ("JOB DONE");
		LCD_STATUS;
		SERIAL_ECHO_START
		SERIAL_PROTOCOLPGM("Done, print time: " );
		SERIAL_PROTOCOLLN(progress_string[0]);
		SERIAL_PROTOCOLPGM("Filament used: ");
		SERIAL_PROTOCOLLN(progress_string[2]);
		progress_string[2] = String ("F:") +progress_string[2];
#ifdef BLINK_M
		BlinkM_playScript(BLINK_M_ADDR,BLINK_M_DONE_SCRIPT,0,0);
#endif
		break;

		// -------------------------------------------------------------------------------------------------------------------------

	case PAUSED:
#ifdef BLINK_M
		BlinkM_playScript(BLINK_M_ADDR, BLINK_M_PAUSE_SCRIPT,0,0);
#endif
		break;

	case SAVING:
		break;

		// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	default:
		break;
	}

	return *this;
}