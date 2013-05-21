// 
// 
// 

#include "EEPROMwrite.h"
#include "Marlin.h"
#include "ultralcd.h"

int plaPreheatHotendTemp;
int plaPreheatHPBTemp;
int plaPreheatFanSpeed;

int absPreheatHotendTemp;
int absPreheatHPBTemp;
int absPreheatFanSpeed;

int frequently_updated_offset = -1;

void EEPROM_StoreSettings() 
{
#ifdef EEPROM_SETTINGS
	char ver[4]= "000";
	int i=EEPROM_OFFSET;
	EEPROM_writeAnything(i,ver); // invalidate data first 
	EEPROM_writeAnything(i,axis_steps_per_unit);  
	EEPROM_writeAnything(i,max_feedrate);  
	EEPROM_writeAnything(i,max_acceleration_units_per_sq_second);
	EEPROM_writeAnything(i,acceleration);
	EEPROM_writeAnything(i,retract_acceleration);
	EEPROM_writeAnything(i,minimumfeedrate);
	EEPROM_writeAnything(i,mintravelfeedrate);
	EEPROM_writeAnything(i,minsegmenttime);
	EEPROM_writeAnything(i,max_xy_jerk);
	EEPROM_writeAnything(i,max_z_jerk);
	EEPROM_writeAnything(i,max_e_jerk);
	EEPROM_writeAnything(i,add_homeing);
	EEPROM_writeAnything(i,plaPreheatHotendTemp);
	EEPROM_writeAnything(i,plaPreheatHPBTemp);
	EEPROM_writeAnything(i,plaPreheatFanSpeed);
	EEPROM_writeAnything(i,absPreheatHotendTemp);
	EEPROM_writeAnything(i,absPreheatHPBTemp);
	EEPROM_writeAnything(i,absPreheatFanSpeed);
#ifdef PIDTEMP
	EEPROM_writeAnything(i,Kp);
	EEPROM_writeAnything(i,Ki);
	EEPROM_writeAnything(i,Kd);
#else
	EEPROM_writeAnything(i,3000);
	EEPROM_writeAnything(i,0);
	EEPROM_writeAnything(i,0);
#endif
	frequently_updated_offset=i;
	EEPROM_StoreFrequentSettings();


	char ver2[4]=EEPROM_VERSION;
	i=EEPROM_OFFSET;
	EEPROM_writeAnything(i,ver2); // validate data
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Settings Stored");
#endif //EEPROM_SETTINGS
}



// these are settings that change frequently -- wrtite only those
void EEPROM_StoreFrequentSettings() 
{
#ifdef SAVE_DEVICE_METRICS
#ifdef EEPROM_SETTINGS
	char ver[4]= "000";
	int i=frequently_updated_offset;
	if (frequently_updated_offset>0)
	{
		EEPROM_writeAnything(i,total_distance);
		EEPROM_writeAnything(i,total_on_time);
		EEPROM_writeAnything(i,total_printing_time);
		EEPROM_writeAnything(i,extruder0_degree_seconds);
		EEPROM_writeAnything(i,extruder1_degree_seconds);
		EEPROM_writeAnything(i,bed_degree_seconds);
		EEPROM_writeAnything(i,total_extruder_time0);
		EEPROM_writeAnything(i,total_extruder_time1);
		EEPROM_writeAnything(i,total_bed_time);
		EEPROM_writeAnything(i,total_filament);
	}
	else 
	{
		EEPROM_StoreSettings();

	}

#endif //EEPROM_SETTINGS
#endif

}


void EEPROM_printSettings()
{  // if def=true, the default values will be used
	//  #ifdef EEPROM_SETTINGS  
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Steps per unit:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[0]);
	SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[1]);
	SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[2]);
	SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3]);
	SERIAL_ECHOLN("");

	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M203 X",max_feedrate[0]);
	SERIAL_ECHOPAIR(" Y",max_feedrate[1] ); 
	SERIAL_ECHOPAIR(" Z", max_feedrate[2] ); 
	SERIAL_ECHOPAIR(" E", max_feedrate[3]);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[0] ); 
	SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[1] ); 
	SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[2] );
	SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3]);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M204 S",acceleration ); 
	SERIAL_ECHOPAIR(" T" ,retract_acceleration);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum xY jerk (mm/s),  Z=maximum Z jerk (mm/s)");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M205 S",minimumfeedrate ); 
	SERIAL_ECHOPAIR(" T" ,mintravelfeedrate ); 
	SERIAL_ECHOPAIR(" B" ,minsegmenttime ); 
	SERIAL_ECHOPAIR(" X" ,max_xy_jerk ); 
	SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
	SERIAL_ECHOPAIR(" E" ,max_e_jerk);
	SERIAL_ECHOLN(""); 
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Home offset (mm):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M206 X",add_homeing[0] );
	SERIAL_ECHOPAIR(" Y" ,add_homeing[1] );
	SERIAL_ECHOPAIR(" Z" ,add_homeing[2] );
	SERIAL_ECHOLN("");
#ifdef PIDTEMP
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("PID settings:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("   M301 P",Kp); 
	SERIAL_ECHOPAIR(" I" ,Ki/PID_dT); 
	SERIAL_ECHOPAIR(" D" ,Kd*PID_dT);
	SERIAL_ECHOLN(""); 
#endif

}

void EEPROM_printHistory()
{
	//	  SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Device Metrics:");	  
	SERIAL_ECHOPGM("Total Time On: "); 
	SERIAL_ECHOLN(EchoTimeSpan(total_on_time)); 
	SERIAL_ECHOPGM("Total Print Time: "); 
	SERIAL_ECHO(EchoTimeSpan(total_printing_time)); 
	SERIAL_ECHOPAIR("  ( ",100.0*total_printing_time/total_on_time); 
	SERIAL_ECHOLNPGM  (" %) " ); 

	SERIAL_ECHOPGM("Extr 0 Time On: "); 
	SERIAL_ECHOLN(EchoTimeSpan(total_extruder_time0)); 
	SERIAL_ECHOPAIR("Extr 0 Degree-Hours: ",extruder0_degree_seconds/ 3600); 
	if (total_extruder_time0!=0)
		{
			SERIAL_ECHOPAIR(" (avg ",extruder0_degree_seconds/total_extruder_time0); 
			SERIAL_ECHOLNPGM  (" degrees)" ); 
		}
#if EXTRUDERS > 1
	SERIAL_ECHOLNPGM("Extruder 1 Time On: "); 
	SERIAL_ECHOLN(EchoTimeSpan(total_extruder_time1)); 
	SERIAL_ECHOPAIR("Extruder 1 Degree-Minutes: ",extruder1_degree_seconds/ 60); 
	if (total_extruder_time1!=0)
	{
		SERIAL_ECHOPAIR("  ( avg temp = ",extruder1_degree_seconds/total_extruder_time1); 
		SERIAL_ECHOLN  ("  degrees) " ); #endif

	}
#endif

	SERIAL_ECHOPGM("Bed Time On: "); 
	SERIAL_ECHOLN(EchoTimeSpan(total_bed_time)); 
	SERIAL_ECHOPAIR("Bed Degree-Hours: ",bed_degree_seconds/ 3600); 
	if (total_bed_time!=0) 
		{
			SERIAL_ECHOPAIR(" (avg ",bed_degree_seconds/total_bed_time); 
			SERIAL_ECHOLNPGM  (" degrees) " ); 
	}


	SERIAL_ECHOLN("Total Travel (meters)");
	SERIAL_ECHOPAIR("X: ",total_distance[X_AXIS]/1000.0); 
	SERIAL_ECHOLN("");
	SERIAL_ECHOPAIR("Y: ",total_distance[Y_AXIS]/1000.0); 
	SERIAL_ECHOLN("");
	SERIAL_ECHOPAIR("Z: ",total_distance[Z_AXIS]/1000.0); 
	SERIAL_ECHOLN("");

	SERIAL_ECHOPAIR("Extruder 0: ",total_distance[E_AXIS]/1000.0); 
	SERIAL_ECHOLN("");
	SERIAL_ECHOPAIR("Filament Extruded: ",total_filament/1000.0); 
	SERIAL_ECHOLNPGM  ("" ); 

	//  #endif
} 

void ResetMetrics() 
{
	for (short i=0;i<4;i++) 
	{
		total_distance[i]=0;
	}
	total_on_time=0;
	total_printing_time=0;
	total_bed_time=0;
	total_extruder_time1=0;
	total_extruder_time0=0;
	extruder1_degree_seconds=0;
	extruder0_degree_seconds=0;
	bed_degree_seconds=0;
	total_filament=0;
	EEPROM_StoreFrequentSettings();
}

void EEPROM_RetrieveSettings(bool def)
{  // if def=true, the default values will be used
#ifdef EEPROM_SETTINGS
	int i=EEPROM_OFFSET;
	char stored_ver[4];
	char ver[4]=EEPROM_VERSION;
	EEPROM_readAnything(i,stored_ver); //read stored version
	//  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
	if ((!def)&&(strncmp(ver,stored_ver,3)==0)) 
	{   // version number match
		EEPROM_readAnything(i,axis_steps_per_unit);  
		EEPROM_readAnything(i,max_feedrate);  
		EEPROM_readAnything(i,max_acceleration_units_per_sq_second);
		EEPROM_readAnything(i,acceleration);
		EEPROM_readAnything(i,retract_acceleration);
		EEPROM_readAnything(i,minimumfeedrate);
		EEPROM_readAnything(i,mintravelfeedrate);
		EEPROM_readAnything(i,minsegmenttime);
		EEPROM_readAnything(i,max_xy_jerk);
		EEPROM_readAnything(i,max_z_jerk);
		EEPROM_readAnything(i,max_e_jerk);
		EEPROM_readAnything(i,add_homeing);
		EEPROM_readAnything(i,plaPreheatHotendTemp);
		EEPROM_readAnything(i,plaPreheatHPBTemp);
		EEPROM_readAnything(i,plaPreheatFanSpeed);
		EEPROM_readAnything(i,absPreheatHotendTemp);
		EEPROM_readAnything(i,absPreheatHPBTemp);
		EEPROM_readAnything(i,absPreheatFanSpeed);
#ifndef PIDTEMP
		float Kp,Ki,Kd;
#endif
		EEPROM_readAnything(i,Kp);
		EEPROM_readAnything(i,Ki);
		EEPROM_readAnything(i,Kd);
#ifdef SAVE_DEVICE_METRICS
		EEPROM_readAnything(i,total_distance);
		EEPROM_readAnything(i,total_on_time);
		EEPROM_readAnything(i,total_printing_time);

		EEPROM_readAnything(i,extruder0_degree_seconds);
		EEPROM_readAnything(i,extruder1_degree_seconds);
		EEPROM_readAnything(i,bed_degree_seconds);
		EEPROM_readAnything(i,total_extruder_time0);
		EEPROM_readAnything(i,total_extruder_time1);
		EEPROM_readAnything(i,total_bed_time);
		EEPROM_readAnything(i,total_filament);
#else
		ResetMetrics();

#endif

		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM("Stored settings retreived:");
	}
	else 
#endif
	{
		float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
		float tmp2[]=DEFAULT_MAX_FEEDRATE;
		long tmp3[]=DEFAULT_MAX_ACCELERATION;
		for (short i=0;i<4;i++) 
		{
			axis_steps_per_unit[i]=tmp1[i];  
			max_feedrate[i]=tmp2[i];  
			max_acceleration_units_per_sq_second[i]=tmp3[i];
		}
		acceleration=DEFAULT_ACCELERATION;
		retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
		minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
		minsegmenttime=DEFAULT_MINSEGMENTTIME;       
		mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
		max_xy_jerk=DEFAULT_XYJERK;
		max_z_jerk=DEFAULT_ZJERK;
		max_e_jerk=DEFAULT_EJERK;
		add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;
		ResetMetrics();

		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM("Using Default settings:");
#ifdef ULTIPANEL
		plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
		plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
		plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
		absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
		absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
		absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
		LCD_MESSAGEPGM("Defaults loaded");
#endif
	}
#ifdef EEPROM_CHITCHAT
	EEPROM_printSettings();
#endif
}  