//
//
//

#include "Job.h"
#include "Utility.h"
#include "ultralcd.h"
#include "cardreader.h"

#ifdef SDSUPPORT
extern CardReader card;
#endif

Job job;

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
	if (jobstate == RUNNING && !force) return;
	stoptime = starttime = millis()/1000;
		save_gcode_start = gcode_N;
	job_start_filament = total_filament;
	int a;
	for (a=0;a<NUM_AXIS;a++)
		job_distance[a]=total_distance[a];
	last_time_estimate = 1;
	sdpercentage=percent = 0;
	we_have_gcode_progress = false;
	jobstate = RUNNING;
	LCD_MESSAGE_CLEARPRI(USER_MESSAGE_PRIORITY);
	LCD_MESSAGEPGM ("JOB STARTED");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM ("JOB STARTED");
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
	last_time_estimate = 0;
	LCD_MESSAGE_CLEARPRI(USER_MESSAGE_PRIORITY-1);
	if (cancelled)
	{
		LCD_MESSAGEPGM ("JOB CANCELLED");
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM ("JOB CANCELLED");
	}

	else
	{
		LCD_MESSAGEPGM ("JOB DONE");
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM ("JOB DONE");
	}

	// play a happy tune on the beeper!
}

unsigned long Job::CalculateRemainingTime()
{
	if (percent<0.1) return 0;
	static float last_percent = -1;
	if (last_percent == (percent+sdpercentage)/2 ) return last_time_estimate;
	last_percent= (percent+sdpercentage)/2;
	unsigned long total_time = JobTime() / ( last_percent / 100.0);
	last_time_estimate = total_time - JobTime();
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
	stoptime = millis() / 1000;
	
#ifdef SDSUPPORT
	if (counter++ % 100 ==0 )
		sdpercentage = card.percentDone();
#endif
}

float Job::Percent() const
{
#ifdef SDSUPPORT
	if (we_have_gcode_progress && card.sdprinting) return (percent + sdpercentage) / 2.0;
#endif
	if (we_have_gcode_progress) return percent;
	return sdpercentage;
}

float Job::GetDistanceTravelled( int axis ) const
{
	return total_distance[axis] - job_distance[axis];
}