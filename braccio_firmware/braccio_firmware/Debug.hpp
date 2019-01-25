#ifndef DEBUG_H
#define DEBUG_H

#define DEBUG_MODE

#include <Arduino.h>

#ifdef DEBUG_MODE
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#endif /* DEBUG_H */
