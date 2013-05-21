//
//
//

#include "Job.h"
#include "Utility.h"
#include "ultralcd.h"
#include "cardreader.h"
#include "language.h"

#ifdef SDSUPPORT
extern CardReader card;
#endif

Job job;
static unsigned long last_speed_check = 0;


void Job::init()
{
	starttime=0;
	stoptime=0;
	pausedtime =0;
	last_time_estimate=1;
	job_start_filament=total_filament;
	sdpercentage = percent=0;
	we_have_gcode_progress = false;
	jobstate = STOPPED;
	count =0;
	for (int a=0;a<NUM_AXIS;a++)
		job_distance[a]=total_distance[a];

}

void Job::Start( bool force/*=false*/ )
{
	if (current_head_position[Z_AXIS] < last_z || current_head_position[Z_AXIS]  < 0.5)
		force = true;
	last_z = current_head_position[Z_AXIS] ;
	if (jobstate == RUNNING && !force) return;
	last_speed_check = millis();
	stoptime = starttime = millis()/1000;
	save_gcode_start = gcode_N;
	job_start_filament = total_filament;
	int a;
	for (a=0;a<NUM_AXIS;a++)
		job_distance[a]=total_distance[a];
	last_time_estimate = 1;
	speed =0;
	last_percent =-1;
	pausedtime=0;
	sdpercentage=percent = 0;
	we_have_gcode_progress = false;
	jobstate = RUNNING;
	LCD_MESSAGE_CLEARPRI(USER_MESSAGE_PRIORITY);
	LCD_MESSAGEPGM (JOB_START_MESSAGE);
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM (JOB_START_MESSAGE);
}

void Job::Stop(bool cancelled)
{
	if (jobstate == STOPPED) return;
	count++;
	state = DONE;
	jobstate = STOPPED;
	percent = 100;
	sdpercentage= 100;
	stoptime=millis()/1000;
	last_percent =-1;
	last_time_estimate = 0;
	LCD_MESSAGE_CLEARPRI(USER_MESSAGE_PRIORITY-1);
	if (cancelled)
	{
		LCD_MESSAGEPGM (JOB_CANCEL_MESSAGE);
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM (JOB_CANCEL_MESSAGE);
	}

	else
	{
		LCD_MESSAGEPGM (JOB_DONE_MESSAGE);
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM (JOB_DONE_MESSAGE);
	}

	// play a happy tune on the beeper!
}

unsigned long Job::CalculateRemainingTime()
{
	
	float t = Percent();
	if (t<0.25)
	{
		last_time_estimate =0 ;
		return 0;
	}
	if (last_percent == t ) return last_time_estimate;
	last_percent= t;
	constrain (last_percent,0.0,100.0);
	unsigned long dt = stoptime-starttime;
	unsigned long total_time = dt / ( last_percent * 0.01);
	last_time_estimate = max (total_time - dt,0);
	return last_time_estimate;
}

void Job::Pause()
{
	if (jobstate==RUNNING)
	{
		pausedtime +=stoptime - starttime;
		jobstate = PAUSED;
	}
}

void Job::Resume()
{
	if (jobstate == PAUSED)
	{
		starttime = stoptime = millis() / 1000;
		jobstate==RUNNING;
		return;
	}
	if (jobstate == STOPPED)
	{
		// can't resume from stopped.
	}
}

float Job::GetFilamentUsed() const
{
	return	total_filament - job_start_filament;
}

void Job::Update()
{
	if (jobstate == STOPPED || (jobstate == PAUSED))
		return;
	static int counter =0;
	unsigned long m =  millis();
	stoptime = m / 1000;

#ifdef SDSUPPORT
	if ((now & 0xfff) ==0 && card.sdprinting)
		{
			SetSDPercent(card.percentDone());
		}
#endif
}

float Job::Percent() const
{
#ifdef SDSUPPORT
	if (we_have_gcode_progress && card.sdprinting) return ((percent + sdpercentage) / 2.0);
#endif
	if (we_have_gcode_progress) return percent;
	return sdpercentage;
}

float Job::GetDistanceTravelled( int axis ) const
{
	return total_distance[axis] - job_distance[axis];
}

void Job::SetPercent( float val )
{
	percent = val; 
	if (percent<0.0f) percent =0.0f;
	if (percent>100.0f) percent =100.0f;

	we_have_gcode_progress = true; 
	if (!card.sdprinting)
			sdpercentage=percent ;
}

void Job::SetSDPercent( float val )
{
	if (sdpercentage<0.0f) sdpercentage =0.0f;
	if (sdpercentage>100.0f) sdpercentage =100.0f;
	sdpercentage = val;
}