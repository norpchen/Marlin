// Utility.h

#ifndef _UTILITY_h
#define _UTILITY_h

#include "Marlin.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

char *ftostr3(const float &x);
char *itostr2(const uint8_t &x);
char *ftostr31(const float &x);
char *ftostr32(const float &x);
char *itostr31(const int &xx);
char *itostr3(const int &xx);
char *itostr4(const int &xx);
char *ftostr51(const float &x);
char *ftostr52(const float &x);
char *ftostr32a(const float &x);
char *ftostr(float x,int pre,int post, bool sign=false);
char *itostr5(const int &xx);

void SetLEDColor (byte r, byte g, byte b, bool fade=true);

unsigned long CalculateRemainingTime (float percent_complete,unsigned long elapsed_time);
char* EchoTimeSpan (long t, bool shortform=false);

void MyTone(int pin, int freq, int dur);

void error_beep(); ;

void warning_beep(); ;

void Error (char * message);
void Warning (char * message);


//-------------------------------------------------------------------------------------------------------------------
extern "C"
{
	int freeMemory();
};

#endif
