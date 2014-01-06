#ifndef WATCHDOG_H
#define WATCHDOG_H


/* Locals */

extern const char *lastvisit;// __attribute__ ((section (".noinit")));

/* http://gcc.gnu.org/onlinedocs/cpp/Stringification.html */
#define VISITSTR1(fn, ln) ;
 // VISITSTR2(fn, ln)
#define VISITSTR2(fn, ln)  ;
 // fn ":" #ln
#define VISIT ;
//(lastvisit = VISITSTR1(__FILE__, __LINE__)) 

#include "Marlin.h"

#ifdef USE_WATCHDOG
  // intialise watch dog with a 1 sec interrupt time
  void watchdog_init();
  // pad the dog/reset watchdog. MUST be called at least every second after the first watchdog_init or avr will go into emergency procedures..
  void watchdog_reset();
#else
  //If we do not have a watchdog, then we can have empty functions which are optimized away.
  FORCE_INLINE void watchdog_init() {};
  FORCE_INLINE void watchdog_reset() {};
#endif

#endif
