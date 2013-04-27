#ifndef EEPROM_H
#define EEPROM_H

#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
//#include <EEPROM.h>

extern int plaPreheatHotendTemp;
extern int plaPreheatHPBTemp;
extern int plaPreheatFanSpeed;

extern int absPreheatHotendTemp;
extern int absPreheatHPBTemp;
extern int absPreheatFanSpeed;

template <class T> int EEPROM_writeAnything(int &ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < (int)sizeof(value); i++)
    eeprom_write_byte((unsigned char *)ee++, *p++);
  return i;
}

template <class T> int EEPROM_readAnything(int &ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < (int)sizeof(value); i++)
    *p++ = eeprom_read_byte((unsigned char *)ee++);
  return i;
}
//======================================================================================

inline void EEPROM_StoreFrequentSettings() ;


#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION "V08"

void EEPROM_StoreSettings() ;


// these are settings that change frequently -- wrtite only those
void EEPROM_StoreFrequentSettings() ;

void EEPROM_printSettings();

void EEPROM_printHistory();


void EEPROM_RetrieveSettings(bool def=false);

void ResetMetrics();


#endif


