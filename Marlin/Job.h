// Job.h

#ifndef _JOB_h
#define _JOB_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Marlin.h"


class Job
{
 private:
// job completion
	 float percent;
	 float sdpercentage;
	 bool we_have_gcode_progress;
	 unsigned long starttime;
	 unsigned long stoptime;
	 unsigned long pausedtime;
	 unsigned long last_time_estimate;
	 int count;
	 // job metrics
	 float job_start_filament;
	 float job_distance[NUM_AXIS];
public:
	 enum JOBSTATE { STOPPED, PAUSED, RUNNING};
private: 
	JOBSTATE jobstate;
public:
	int Count() const { return count; }
	void init();
	void Start(bool force=false);
	void Stop(bool cancelled = false);
	void Pause();
	void Resume();
	unsigned long JobTime() const { int rv = stoptime-starttime+pausedtime; if (rv >0) return rv; else return 1; };
	unsigned long CalculateRemainingTime () ;
	float Percent() const;
	void SetPercent(float val) { percent = val; we_have_gcode_progress = true;}
	void SetSDPercent(float val) { sdpercentage = val; }
	Job::JOBSTATE JobState() const { return jobstate; }
	void SetJobState(Job::JOBSTATE val) { jobstate = val; }
	float GetFilamentUsed() const;
	void Update();
	bool KnowPercentage () { return we_have_gcode_progress; }
	float GetDistanceTravelled (int axis) const;
};

extern Job job;
;;

#endif

