// All the Serial.print statements add up. Removing all of them from a full build cuts down almost 4K.
// define VERBOSITY 0 removes all of them.
// define VERBOSITY 1 includes only the highest level statements, like program identification and errors.
// define VERBOSITY 2 includes statements like program configuration and mode changes.
// define VERBOSITY 3 includes everything.
// Keep these levels in mind when adding new statements. Always use one of the _printN and _printlnN 
// variants rather than calling Serial.print[ln]() directly.
#if VERBOSITY >= 1
#define _print1(...) Serial.print(__VA_ARGS__)
#define _println1(...) Serial.println(__VA_ARGS__)
#else
#define _print1(...)
#define _println1(...)
#endif
#if VERBOSITY >= 2
#define _print2(...) Serial.print(__VA_ARGS__)
#define _println2(...) Serial.println(__VA_ARGS__)
#else
#define _print2(...)
#define _println2(...)
#endif
#if VERBOSITY >= 3
#define _print3(...) Serial.print(__VA_ARGS__)
#define _println3(...) Serial.println(__VA_ARGS__)
#else
#define _print3(...)
#define _println3(...)
#endif
#if VERBOSITY >= 4
#define _print4(...) Serial.print(__VA_ARGS__)
#define _println4(...) Serial.println(__VA_ARGS__)
#else
#define _print4(...)
#define _println4(...)
#endif
