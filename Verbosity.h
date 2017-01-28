// All the Serial.print statements add up. Removing all of them from a full build cuts down almost 4K.
// define VERBOSITY 0 removes all of them.
// define VERBOSITY 1 includes only the highest level statements, like program identification and errors.
// define VERBOSITY 2 includes statements like program configuration and mode changes.
// define VERBOSITY 3 includes everything.
// Keep these levels in mind when adding new statements. Always use one of the _printN and _printlnN 
// variants rather than calling Serial.print[ln]() directly.
#if VERBOSITY >= 1
#define _print1(x) Serial.print(x)
#define _println1(x) Serial.println(x)
#else
#define _print1(x)
#define _println1(x)
#endif
#if VERBOSITY >= 2
#define _print2(x) Serial.print(x)
#define _println2(x) Serial.println(x)
#else
#define _print2(x)
#define _println2(x)
#endif
#if VERBOSITY >= 3
#define _print3(x) Serial.print(x)
#define _println3(x) Serial.println(x)
#else
#define _print3(x)
#define _println3(x)
#endif
#if VERBOSITY >= 4
#define _print4(x) Serial.print(x)
#define _println4(x) Serial.println(x)
#else
#define _print4(x)
#define _println4(x)
#endif
