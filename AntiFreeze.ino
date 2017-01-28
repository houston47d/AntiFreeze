#define VERSION "0.2"

// For testing purposes, it is useful ta have the calls active HIGH since when not connected
// to a real furnace all of the calls read back active (which is a highly unusual state for
// the furnace and not one which I'm trying to modify the behavior of). Selecting CALLS_ACTIVE_LOW
// as false makes it better behaved in this case.
#define CALLS_ACTIVE_LOW true
// #define CALLS_ACTIVE_LOW false

#define DIAG_STANDALONE 0

// All the Serial.print statements add up. Removing all of them from a full build cuts down almost 4K.
// define VERBOSITY 0 removes all of them.
// define VERBOSITY 1 includes only the highest level statements, like program identification and errors.
// define VERBOSITY 2 includes statements like program configuration and mode changes.
// define VERBOSITY 3 includes everything.
// Keep these levels in mind when adding new statements. Always use one of the _printN and _printlnN 
// variants rather than calling Serial.print[ln]() directly.
#define VERBOSITY 3
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

#include <SPI.h>
#include <SD.h>
#include <MCP23S17.h>
#include <SimpleTimer.h>
#include <Sleep_n0m1.h>
#include <avr/pgmspace.h>

#define sdCardCs 7
#define ioExpCs 10
#define ioExpInt 3

#define tempPin A0

Sleep sleep;
bool abortSleep = false;

SimpleTimer timer;

bool sdCardPresent = false;
char filename[] = "LOGGER00.CSV";
File logfile;

bool ioExpPresent = false;
MCP ioExp0( 0, ioExpCs );

#define SECONDStoMS(s) ((s)*1000L)
#define MINUTEStoMS(m) ((m)*60*1000L)
#define MINUTEStoS(m)  ((m)*60L)
#define HOURStoS(h)    ((h)*3600L)

uint16_t s_currentCycleS = 0;
char s_currentCycleHMS[10];
#if DIAG_STANDALONE
uint16_t s_digitalIo = -1;
uint16_t s_ioExp = 0;
#endif

#if DIAG_STANDALONE
unsigned long measurementIntervalMs = SECONDStoMS(15);
unsigned long callOutputDurationMs = SECONDStoMS(2);
unsigned long callOutputToValveTimeoutMs = MINUTEStoMS(5);
unsigned long zone2callOutputDurationMs = SECONDStoMS(10);
uint16_t zone2CallIntervalMinS = 30;
uint16_t zone2CallIntervalMaxS = 45;
#else
// measurementIntervalMs - how often it runs in the absence of any input change. Intended for
// the interval at which analog thnigs are measured.
unsigned long measurementIntervalMs = MINUTEStoMS(15);
// callOutputDurationMs - how long to assert a call output after the corresponding valve has
// opened.
unsigned long callOutputDurationMs = SECONDStoMS(2);
// callOutputToValveTimeoutMs - how long to wait for the corresponding valve to open after
// asserting the call output. Set to cover the case of a cold furnace having to heat up to 
// service the call.
unsigned long callOutputToValveTimeoutMs = MINUTEStoMS(5);
// zone2callOutputDurationMs - how long to assert the call after the valve opens for zone 2,
// as the purpose in this case is to get hot water all the way through the loop.
// zone2CallIntervalMinS - time after which to start cycle zone 2 if the furnace is already
// servicing another zone.
// zone2CallIntervalMaxS - time after which to start cycle zone 2 even if the furnace is not
// servicing another zone (means we will trigger the burner, but oh well).
unsigned long zone2callOutputDurationMs = MINUTEStoMS(2);
uint16_t zone2CallIntervalMinS = MINUTEStoS(30);
uint16_t zone2CallIntervalMaxS = MINUTEStoS(45);
#endif

enum Source { eNone, eDigitalIo, eAnalogIo, eIoExpander };
struct SignalBase {
    Source source;
    uint8_t pin;
    uint8_t type;
    const PROGMEM char* name;
    bool priorValid;
    unsigned long priorTransition;
    bool changed;
    SignalBase( Source _source, uint8_t _pin, uint8_t _type, const PROGMEM char* _name ) 
      : source( _source ), pin( _pin ), type( _type ), name( _name ), priorValid( false ), priorTransition( 0 ), changed( false ) {}
};
struct BoolSignal : public SignalBase {
    bool activeLow;
    bool currentState;
    void init() {
      switch( source ) {
      case eDigitalIo:
        pinMode( pin, type );
        if( type == OUTPUT )
          digitalWrite( pin, activeLow ? HIGH : LOW );
        break;
      case eIoExpander:
        ioExp0.pinMode( pin, type );
        if( type == OUTPUT )
          ioExp0.digitalWrite( pin, activeLow ? HIGH : LOW );
        break;
      default:
        break;
      }
    }
    void write( bool value, bool force = false ) {
      if( type == OUTPUT && (value != currentState || force) ) {
        switch( source ) {
        case eDigitalIo:
          digitalWrite( pin, value ^ activeLow ? HIGH : LOW );
          break;
        case eIoExpander:
          ioExp0.digitalWrite( pin, value ^ activeLow ? HIGH : LOW );
          break;
        default:
          break;
        }
        currentState = value;
      }
    }
    BoolSignal( Source _source, uint8_t _pin, uint8_t _type, const PROGMEM char* _name, bool _activeLow )
      : SignalBase( _source, _pin, _type, _name ), activeLow( _activeLow ), currentState( false ) {}
};
static const char activeName[] PROGMEM = { "Active" };
BoolSignal active( eNone, 0, 0, activeName, false );
static const char activeAFName[] PROGMEM = { "AntiFreeze" };
BoolSignal activeAF( eNone, 0, 0, activeAFName, false );

// The pushbutton is between the pin and GND, so LOW means the button was pushed.
static const char pushButtonName[] PROGMEM = { "PushButton" };
BoolSignal pushButton( eDigitalIo, 2, INPUT_PULLUP, pushButtonName, true );
// The two LEDs are tied to +V through a resistor, so 'on' is LOW (and thus activeLow = true).
static const char greenLedName[] PROGMEM = { "GreenLed" };
BoolSignal greenLed( eDigitalIo, A4, OUTPUT, greenLedName, true );
static const char redLedName[] PROGMEM = { "RedLed" };
BoolSignal redLed( eDigitalIo, A5, OUTPUT, redLedName, true );
// There are 13 inputs covering the calls into the controller and the valve controls
// out of the controller. The calls are active low because the thermostat grounds the
// signal to call for heat.
static const char zoneHwCallName[] PROGMEM = { "ZoneHwCall" };
BoolSignal zoneHwCall( eIoExpander, 15, INPUT, zoneHwCallName, CALLS_ACTIVE_LOW );
static const char zone1CallName[] PROGMEM = { "Zone1Call" };
BoolSignal zone1Call( eIoExpander, 14, INPUT, zone1CallName, CALLS_ACTIVE_LOW );
static const char zone2CallName[] PROGMEM = { "Zone2Call" };
BoolSignal zone2Call( eIoExpander, 13, INPUT, zone2CallName, CALLS_ACTIVE_LOW );
static const char zone3CallName[] PROGMEM = { "Zone3Call" };
BoolSignal zone3Call( eIoExpander, 12, INPUT, zone3CallName, CALLS_ACTIVE_LOW );
static const char zone4CallName[] PROGMEM = { "Zone4Call" };
BoolSignal zone4Call( eIoExpander, 11, INPUT, zone4CallName, CALLS_ACTIVE_LOW );

// The following are all INPUT_PULLUP and active low only because they are not currently connected.
static const char zoneHwValveName[] PROGMEM = { "ZoneHwValve" };
BoolSignal zoneHwValve( eIoExpander, 7, INPUT, zoneHwValveName, false );
static const char zone1ValveName[] PROGMEM = { "Zone1Valve" };
BoolSignal zone1Valve( eIoExpander, 6, INPUT, zone1ValveName, false );
static const char zone2ValveName[] PROGMEM = { "Zone2Valve" };
BoolSignal zone2Valve( eIoExpander, 5, INPUT, zone2ValveName, false );
static const char zone3ValveName[] PROGMEM = { "Zone3Valve" };
BoolSignal zone3Valve( eIoExpander, 4, INPUT, zone3ValveName, false );
static const char zone4ValveName[] PROGMEM = { "Zone4Valve" };
BoolSignal zone4Valve( eIoExpander, 3, INPUT, zone4ValveName, false );
static const char inductorName[] PROGMEM = { "InductorOn" };
BoolSignal inductor( eIoExpander, 2, INPUT, inductorName, false );
static const char circulatorName[] PROGMEM = { "CirculatorOn" };
BoolSignal circulator( eIoExpander, 1, INPUT, circulatorName, false );
static const char burnerName[] PROGMEM = { "BurnerOn" };
BoolSignal burner( eIoExpander, 0, INPUT_PULLUP, burnerName, true );

// There are three call out signals that (currently) activate the respective zone call.
// The relays are normally opened, and a high output closes the relay and grounds the
// respective signal.
static const char zone1CallOutName[] PROGMEM = { "Zone1CallOut" };
BoolSignal zone1CallOut( eIoExpander, 10, OUTPUT, zone1CallOutName, false );
static const char zone2CallOutName[] PROGMEM = { "Zone2CallOut" };
BoolSignal zone2CallOut( eIoExpander, 9, OUTPUT, zone2CallOutName, false );
static const char zone3CallOutName[] PROGMEM = { "Zone3CallOut" };
BoolSignal zone3CallOut( eIoExpander, 8, OUTPUT, zone3CallOutName, false );

// The following array of pointers just makes it easier to handle the signals in a
// generic fashion when appropriate.
static BoolSignal* const s_Signals[] PROGMEM = {
    &pushButton,
    &greenLed,
    &redLed,
    &zoneHwCall,
    &zone1Call,
    &zone2Call,
    &zone3Call,
    &zone4Call,
    &zoneHwValve,
    &zone1Valve,
    &zone2Valve,
    &zone3Valve,
    &zone4Valve,
    &inductor,
    &circulator,
    &burner,
    &zone1CallOut,
    &zone2CallOut,
    &zone3CallOut
};
byte s_NumSignals = sizeof( s_Signals ) / sizeof( s_Signals[0] );
BoolSignal* s_CallSignals[] = {
    &zoneHwCall,
    &zone1Call,
    &zone2Call,
    &zone3Call,
    &zone4Call
};
BoolSignal* s_ValveSignals[] = {
    &zoneHwValve,
    &zone1Valve,
    &zone2Valve,
    &zone3Valve,
    &zone4Valve
};
byte s_NumCallSignals = sizeof( s_CallSignals ) / sizeof( s_CallSignals[0] );

#define ReadTemperature 0
float temperatureC = 0.0;

void showCapabilities() {
  _println1( F("Furnace Sidekick " VERSION " - " __DATE__ " " __TIME__) );
#if DIAG_STANDALONE
  _println1( F("Standalone Mode") );
#endif // DIAG_STANDALONE
  _println2( F("") );
  _print2( F("IoExpander CS on pin ") ); _println2( ioExpCs );
  _print2( F("IoExpander is ") ); _println2( ioExpPresent ? F("present") : F("ABSENT") );
  _print2( F("SD Card CS on pin ") ); _println2( sdCardCs );
  _println2( F("Signals:") );
  for( byte i = 0; i < s_NumSignals; ++i ) {
    const BoolSignal* sig( (const BoolSignal*) pgm_read_word_near( &s_Signals[i] ) );
    _print2( F("  ") );
    _print2( reinterpret_cast<const __FlashStringHelper *>( sig->name ) );
    _print2( F(", ") );
    _print2( sig->source == eIoExpander ? F("Expander ") : sig->source == eAnalogIo ? F("Analog ") : F("Digital ") );
    _println2( sig->pin );
  }
}

bool s_pushButtonPressed = false;
void switchHandler() {
  s_pushButtonPressed = true;
  abortSleep = true;
}
bool s_ioExpanderInterrupt = false;
void ioExpHandler() {
  ioExp0.wordRead(INTCAPA);
  s_ioExpanderInterrupt = true;
  abortSleep = true;
}

int s_greenLedFlashTimer = -1;
int s_redLedFlashTimer = -1;
void ledFlashExpired(void* param) {
  BoolSignal* led( reinterpret_cast<BoolSignal*>( param ) );
  led->write( !led->currentState );
  led->priorValid = true;
  led->priorTransition = s_currentCycleS;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

#if 0
  _print3( F("sizeof(unsigned long) ") ); _println3( sizeof(unsigned long) );
  _print3( F("sizeof(unsigned int) ") ); _println3( sizeof(unsigned int) );
  _print3( F("sizeof(unsigned short) ") ); _println3( sizeof(unsigned short) );
#endif

#if DIAG_STANDALONE
  timer.setTimeout( 1000, processTestScript );
#endif
  s_greenLedFlashTimer = timer.setInterval( 1000, ledFlashExpired, &greenLed );
  timer.enable( s_greenLedFlashTimer, false );
  s_redLedFlashTimer = timer.setInterval( 1000, ledFlashExpired, &redLed );
  timer.enable( s_redLedFlashTimer, false );

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  ioExpPresent = ioExp0.begin();
  if( ioExpPresent ) {
    for( byte i = 0; i < s_NumSignals; ++i ) {
      BoolSignal* sig( (BoolSignal*) pgm_read_word_near( &s_Signals[i] ) );
      sig->init();
    }
    // Set MIRROR (shared interrupt pin), ensure ODR and INTPOL are clear.
    uint8_t ioCon = ioExp0.byteRead(IOCON);
    ioExp0.byteWrite(IOCON, (ioCon | 0x40) & ~0x06);
    unsigned int ioDirRead = ioExp0.wordRead(IODIRA);
    // Generate interrupt when input changes from previous value.
    ioExp0.wordWrite(INTCONA, 0);
    // Enable change interrupt on all input channels.
    ioExp0.wordWrite(GPINTENA, ioDirRead);

    // The input is configured as a push-pull active low, so no pullup required.
    pinMode( ioExpInt, INPUT );
    attachInterrupt(digitalPinToInterrupt( ioExpInt ), ioExpHandler, FALLING); 
  }
    
  // Note that 'FALLING' will not work for lower power states - that requires 'LOW'.
  attachInterrupt(digitalPinToInterrupt( pushButton.pin ), switchHandler, FALLING); 

  showCapabilities();
}

#if 0
void writeLogEntry( const PROGMEM char* message ) {
  if( logfile ) {      
    logfile.print( reinterpret_cast<const __FlashStringHelper *>( message ) );
  }
  _println2( reinterpret_cast<const __FlashStringHelper *>( message ) );
}
#endif

void generateHMS( char* buffer, unsigned long seconds ) {
  byte hour = 0;
  if( seconds >= 3600 ) {
    hour = (byte) (seconds / 3600);
    if( hour > 99 ) 
      hour = 99;
    seconds = seconds % 3600;
  }
  byte minute = 0;
  if( seconds >= 60 ) {
    minute = (byte) (seconds / 60);
    seconds = seconds % 60;
  }
  byte second = (byte) seconds;
  buffer[0] = (hour / 10 + '0');
  buffer[1] = (hour % 10 + '0');
  buffer[2] = ':';
  buffer[3] = (minute / 10 + '0');
  buffer[4] = (minute % 10 + '0');
  buffer[5] = ':';
  buffer[6] = (second / 10 + '0');
  buffer[7] = (second % 10 + '0');
  buffer[8] = 0;
}

void writeLogEntry( const __FlashStringHelper* message ) {
  if( logfile ) {      
    logfile.print( s_currentCycleHMS );
    logfile.print( F(",") );
    logfile.print( message );
  }
  _print3( s_currentCycleHMS );
  _print3( F(",") );
  _println2( message );
}
  
void writeLogEntry( struct BoolSignal& signal ) {
  char durationHMS[10];
  if( signal.priorValid )
    generateHMS( durationHMS, s_currentCycleS - signal.priorTransition );
  else
    strcpy( durationHMS, "--:--:--" );

  if( logfile ) {      
    // replace millis() with the real-time-clock when available.
    logfile.print( s_currentCycleHMS );
    logfile.print( F(",") );
    logfile.print( durationHMS );
    logfile.print( F(",") );
    logfile.print( reinterpret_cast<const __FlashStringHelper *>( signal.name ) );
    logfile.print( F(",") );
    logfile.println( signal.currentState ? 1 : 0 );
  }
  _print3( s_currentCycleHMS );
  _print3( F(",") );
  _print3( durationHMS );
  _print3( F(",") );
  _print3( reinterpret_cast<const __FlashStringHelper *>( signal.name ) );
  _print3( F(",") );
  _println3( signal.currentState ? 1 : 0 );
}

void writeLogEntry( const char* signal, float value ) {
  if( logfile ) {      
    // replace millis() with the real-time-clock when available.
    logfile.print( s_currentCycleS);
    logfile.print( F(",") );
    logfile.print( signal );
    logfile.print( F(",") );
    logfile.println( value );
  }
  _print3( s_currentCycleS );
  _print3( F(",") );
  _print3( signal );
  _print3( F(",") );
  _println3( value );
}

void readSignalAndLogChange( struct BoolSignal& signal ) {
  bool stateNow = ((digitalRead( signal.pin ) == HIGH) ^ signal.activeLow);
  if( stateNow != signal.currentState ) {
    signal.currentState = stateNow;
    writeLogEntry( signal );
    signal.priorValid = true;
    signal.priorTransition = s_currentCycleS;
  }
}

float readTemp() {
  // read the voltage, and scale from 0..1023 -> 0.0..5.0.
  float voltage = (analogRead( tempPin ) * 5.0 / 1024.0);
  // 500 mV == 0 degrees C, and 10 mV / degree C.
  float tempC = (voltage - .5) * 100.0;
  return tempC;
}

struct zoneCallInfo {
    struct BoolSignal* zoneCallOut;
    struct BoolSignal* zoneValve;
    int waitForValveTimer;
    int holdTimer;
    uint16_t holdDurationMs;
    bool active() const { return( waitForValveTimer >= 0 || holdTimer >= 0 ); }
};

void zoneCallWaitForValveTimout( void* param ) {
  struct zoneCallInfo* info( reinterpret_cast<struct zoneCallInfo*>( param ) );
  info->zoneCallOut->write( false );
  writeLogEntry( *(info->zoneCallOut) );
  info->zoneCallOut->priorValid = true;
  info->zoneCallOut->priorTransition = s_currentCycleS;
  writeLogEntry( F("Error: expected zone X valve, but never detected") );
  info->waitForValveTimer = -1;
}
void zoneCallExpired( void* param) {
  struct zoneCallInfo* info( reinterpret_cast<struct zoneCallInfo*>( param ) );
  _println3( F("  + zone X call expired") );
  info->zoneCallOut->write( false );
  writeLogEntry( *(info->zoneCallOut) );
  info->zoneCallOut->priorValid = true;
  info->zoneCallOut->priorTransition = s_currentCycleS;
  info->holdTimer = -1;
}
void setupZoneCallOut( struct zoneCallInfo& info ) {
  _println3( F("  + triggering zone X") );
  info.zoneCallOut->write( true );
  writeLogEntry( *(info.zoneCallOut) );
  info.zoneCallOut->priorValid = true;
  info.zoneCallOut->priorTransition = s_currentCycleS;
  if( info.waitForValveTimer >= 0 ) // should never happen...
    timer.deleteTimer( info.waitForValveTimer );
  info.waitForValveTimer = timer.setTimeout( callOutputToValveTimeoutMs, zoneCallWaitForValveTimout, &info );
  if( info.waitForValveTimer == -1 ) {
    writeLogEntry( F("Error: failed call to setTimeout (1)") );
  }
}
void checkForValveOpen( struct zoneCallInfo& info ) {
  if( info.waitForValveTimer >= 0 && info.zoneValve->currentState ) {
    // we generated a call on zone 1 and the valve has now come on, so we set a new timer
    // for the duration that we want it open.
    _println3( F("  + zone X valve is now open") );
    timer.deleteTimer( info.waitForValveTimer );
    info.waitForValveTimer = -1;
    if( info.holdTimer >= 0 ) // should never happen...
      timer.deleteTimer( info.holdTimer );
    info.holdTimer = timer.setTimeout( info.holdDurationMs, zoneCallExpired, &info );
    if( info.holdTimer == -1 ) {
      writeLogEntry( F("Error: failed call to setTimeout (2)") );
    }
  }
}
void cancelZoneCallOut( struct zoneCallInfo& info ) {
  if( info.waitForValveTimer >= 0 )
    timer.deleteTimer( info.waitForValveTimer );
  info.waitForValveTimer = -1;
  if( info.holdTimer >= 0 )
    timer.deleteTimer( info.holdTimer );
  info.holdTimer = -1;
  info.zoneCallOut->write( false );
}
struct zoneCallInfo s_zoneCallOut[] = {
  { &zone1CallOut, &zone1Valve, -1, -1, callOutputDurationMs },
  { &zone2CallOut, &zone2Valve, -1, -1, zone2callOutputDurationMs }
};

void processPushButton() {
  // Do some debouncing and detect a long-press. Interrupt is on FALLING (not change),
  // so we know we are looking for the active state. So we first wait for the signal to
  // stabilize on 'active' then for it to stabilize on 'inactive'. If it never stabilizes
  // on active, we ignore it. If the duration from the initial interrupt until it stabilizes
  // inactive again exceeds 1 second, then it is a long press. Otherwise, a short press.
  unsigned long start = millis();
  bool stateNow = false;
  byte countSeqActive = 0;
  byte countSeqInactive = 0;
  byte count = 0;
  while( count < 20 && countSeqActive < 3 ) {
#if DIAG_STANDALONE
    uint16_t digitalIoInputs = s_digitalIo;
#else
    uint16_t digitalIoInputs = (((uint16_t) PINB) << 8) | PIND;
#endif
    stateNow = ((digitalIoInputs & (1 << pushButton.pin)) != 0) ^ pushButton.activeLow;
    countSeqActive = (stateNow ? countSeqActive + 1 : 0); 
    ++count;
    delay( 10 );
    timer.run();
  }
  if( countSeqActive >= 3 ) {
    while( count < 200 && countSeqInactive < 3 ) {
#if DIAG_STANDALONE
      uint16_t digitalIoInputs = s_digitalIo;
#else
      uint16_t digitalIoInputs = (((uint16_t) PINB) << 8) | PIND;
#endif
      stateNow = ((digitalIoInputs & (1 << pushButton.pin)) != 0) ^ pushButton.activeLow;
      countSeqInactive = (!stateNow ? countSeqInactive + 1 : 0);
      delay( 10 );
      timer.run();
    }
    unsigned long end = millis();
    bool longPress = (end - start) > 1000;
    if( active.currentState ) {
      if( longPress ) {
        active.currentState = false;
        activeAF.currentState = false;
      }
      else
        activeAF.currentState = !activeAF.currentState;
    }
    else
      active.currentState = true;
  }
}

void processActiveStateChange() {
  if( active.currentState ) {
    // do the SD.begin() here in case the card has been removed/reinserted
    sdCardPresent = SD.begin( sdCardCs );
    
    // create a new file
    if( sdCardPresent ) {
      for (uint8_t i = 0; i < 100; i++) {
        filename[6] = i/10 + '0';
        filename[7] = i%10 + '0';
        if (! SD.exists(filename)) {
          // only open a new file if it doesn't exist
          logfile = SD.open(filename, FILE_WRITE); 
          break;  // leave the loop!
        }
      }
      _print1( logfile ? F("Logging to file ") : F("Error: failed to open file ") ); _println1( filename );
    }
    else {
      _println1( F("Error: no SD card found") );
      _print1( F("  errorCode ") ); _println1( SD.errorCode() );
      _print1( F("  errorData ") ); _println1( SD.errorData() );
    }
  }

  writeLogEntry( active );
  active.priorValid = true;
  active.priorTransition = s_currentCycleS;
  
  greenLed.write( active.currentState );
  redLed.write( false );
  if( s_redLedFlashTimer >= 0 ) timer.enable( s_redLedFlashTimer, active.currentState && !logfile );
  
  if( !active.currentState ) {
    if( logfile ) {
      logfile.flush();
      logfile.close();
    }
    cancelZoneCallOut( s_zoneCallOut[0] );
    if( sdCardPresent ) {
      SD.end();
      sdCardPresent = false;
    }
    
    // setting false will force them to be logged when activated if they are high.
    for( byte i = 0; i < s_NumSignals; ++i ) {
      BoolSignal* sig( (BoolSignal*) pgm_read_word_near( &s_Signals[i] ) );
      if( sig->type != OUTPUT ) {
        sig->currentState = false;
      }
      else {
        sig->write( false );
      }
      sig->priorValid = false;
      sig->priorTransition = 0;
    }
  }
}        

void loop() {
  bool wasActive = active.currentState;
  bool wasActiveAF = activeAF.currentState;
  bool flashState = false;
  uint8_t zone1Signaled = 0;
  uint8_t zone2Signaled = 0;

  while( true ) {

    if( s_pushButtonPressed ) {
      _println3( F("  + Push Button interrupt") );
      processPushButton();
      s_pushButtonPressed = false;
    }
    if( s_ioExpanderInterrupt ) {
      _println3( F("  + Expander interrupt") );
      s_ioExpanderInterrupt = false;
    }

    unsigned long currentCycleS = (millis() / 1000);
    s_currentCycleS = (uint16_t) currentCycleS;
    generateHMS( s_currentCycleHMS, currentCycleS );

    timer.run();
   
    if( wasActive != active.currentState ) {
      processActiveStateChange();
      wasActive = active.currentState;
      flashState = false;
    }

    if( wasActiveAF != activeAF.currentState ) {
      writeLogEntry( activeAF );
      if( s_greenLedFlashTimer >= 0 )
        timer.enable( s_greenLedFlashTimer, activeAF.currentState );
      if( !activeAF.currentState )
        cancelZoneCallOut( s_zoneCallOut[1] );
      wasActiveAF = activeAF.currentState;
    }

    // Go through and read all of our inputs, recording changes in the log file.
    if( active.currentState ) {
#if DIAG_STANDALONE
      uint16_t ioExpInputs = s_ioExp;
      uint16_t digitalIoInputs = s_digitalIo;
#else
      uint16_t ioExpInputs = ioExp0.digitalRead();
      uint16_t digitalIoInputs = (((uint16_t) PINB) << 8) | PIND;
#endif

      bool anyInputsChanged = false;
      for( byte i = 0; i < s_NumSignals; ++i ) {
        BoolSignal* sig( (BoolSignal*) pgm_read_word_near( &s_Signals[i] ) );
        if( sig->type != OUTPUT ) {
          bool stateNow = 
            sig->source == eIoExpander ? ((ioExpInputs & (1 << sig->pin)) != 0) ^ sig->activeLow :
            ((digitalIoInputs & (1 << sig->pin)) != 0) ^ sig->activeLow;
          sig->changed = (stateNow != sig->currentState);
          if( sig->changed ) {
            sig->currentState = stateNow;
            writeLogEntry( *sig );
            sig->priorValid = true;
            sig->priorTransition = s_currentCycleS;
            anyInputsChanged = true;
          }
        }
      }
      
#if ReadTemperature
      temperatureC = readTemp();
      writeLogEntry( "tempC", temperatureC );
#endif

      checkForValveOpen( s_zoneCallOut[0] );
      checkForValveOpen( s_zoneCallOut[1] );

      // Calculate a new state for our outputs (if any)
      bool zoneHwTurnedOff = (!zoneHwCall.currentState && zoneHwCall.changed);
      uint16_t timeSinceZone2 = zone2Call.priorValid ? s_currentCycleS - zone2Call.priorTransition : -1;
      bool zone2NeedsCall = (
          !zone2Call.currentState &&
          timeSinceZone2 != -1 && 
          timeSinceZone2 > zone2CallIntervalMinS);
      bool zone2ReallyNeedsCall = (
          !zone2Call.currentState &&
          timeSinceZone2 != -1 && 
          timeSinceZone2 > zone2CallIntervalMaxS);
      bool zoneTurnedOff = false;
      byte numZonesCalling = 0;
      byte numValvesOpen = 0;
      for( byte i = 0; i < s_NumCallSignals; ++i ) {
        zoneTurnedOff = zoneTurnedOff || (!s_CallSignals[i]->currentState && s_CallSignals[i]->changed);
        if( s_CallSignals[i]->currentState )
          ++numZonesCalling;
        if( s_ValveSignals[i]->currentState )
          ++numValvesOpen;
      }
      bool anyZonesCalling = (numZonesCalling > 0);
      bool noZonesCalling = (numZonesCalling == 0);
      // We consider zone1 to be 'active' if it has called in the last 6 hours.
      uint16_t timeSinceZone1 = zone1Call.priorValid ? s_currentCycleS - zone1Call.priorTransition : -1;
      bool zone1Active = zone1Call.currentState || (timeSinceZone1 != -1 && timeSinceZone1 < HOURStoS( 6 ));

      bool anyOutputsChanged = false;

      if( activeAF.currentState ) {
        if( !s_zoneCallOut[1].active() && 
            ((anyZonesCalling && zone2NeedsCall) || zone2ReallyNeedsCall) )
        {
          // make the call. Then wait up to 10 minutes for the valve to be open (should be plenty even
          // if the furnace has to heat up). The expectation is that the function will never get called.
          setupZoneCallOut( s_zoneCallOut[1] );
          anyOutputsChanged = true;
        }
      }
      
      // If zone2 (upstairs) just turned off and no (other) zones are calling, then the
      // furnace is going to keep dumping heat to the upstairs. Don't like that - the zone
      // absorbes heat slowly so it takes a long time and the zone is small so the amount
      //
      // I have no way to do that. So we simply signal a short call on zone1 so the furnace 
      // dumps the heat there.
      // For many of the same reasons, we want to dump the heat following the hot water 
      // running into the house rather than making the hot water hotter.
      if( !s_zoneCallOut[0].active() &&       // if we haven't already called for zone1
          zone1Active )                       // if zone 1 has called 'recently'
      {
        if( numZonesCalling == 0 &&           // nobody is calling now
            circulator.currentState &&        // and the circulator is still on
            numValvesOpen == 1 &&             // and there is a single valve open
            !zone1Valve.currentState )        // and it isn't zone 1 (our preferred 'purge' zone)
        {
          setupZoneCallOut( s_zoneCallOut[0] );
          anyOutputsChanged = true;
        }
          // (zoneTurnedOff && noZonesCalling && zone1Active && !zone1Valve.currentState) ) {
      }

#if 0
      // If more than three zones are calling and one of them is the Hw, then suppress it 
      // for the time being.
      if( numZonesCalling > 3 &&
          zoneHwCall.currentState )
      {
        _println2( F("Priority: (would be) suppressing Hw due to load") );
        anyOutputsChanged = true;
      }
#endif
#if 0
      if( anyInputsChanged || anyOutputsChanged ) {
        _print3( F("  - timeSinceZone2 ") ); _print3( timeSinceZone2 ); _print3( F(", zone2NeedsCall ") ); _print3( zone2NeedsCall ); _print3( F(", really ") ); _println3( zone2ReallyNeedsCall );
        _print3( F("  - zoneTurnedOff ") ); _print3( zoneTurnedOff ); _print3( F(", numZonesCalling ") ); _print3( numZonesCalling ); _print3( F(", valves ") ); _println3( numValvesOpen );
        _print3( F("  - timeSinceZone1 ") ); _print3( timeSinceZone1 ); _print3( F(", zone1Active ") ); _println3( zone1Active );
      }
#endif

      // Write the outputs to their respective pins.
    }

    // Since we are storing the priorTransition as an uint16_t, it has a range of a bit over 18 hours.
    // Since we expect to be running for many, many days, we will encounter many wrap-arounds. So once
    // the current time starts approaching the priorTransition from the other side, we set the priorTransition
    // time to invalid (we really just need to do it based on the maximim time between iterations so we 
    // could probably stretch this out to 17 hours, but this is simple for now).
    for( byte i = 0; i < s_NumSignals; ++i ) {
      BoolSignal* sig( (BoolSignal*) pgm_read_word_near( &s_Signals[i] ) );
      if( sig->priorValid ) {
        // by casting this to a signed value, a negative result indicates we are now
        // more than half our maximum time past the event (about 9 hours), and we mark it 
        // as invalid.
        int16_t relative = (int16_t) (s_currentCycleS - sig->priorTransition);
        if( relative < 0 )
          sig->priorValid = false;
      }
    }

    // every 10 seconds if there are timers active, and the measurement interval otherwise.
    int timeout = (timer.getNumEnabledTimers() > 0 ? 1000 : measurementIntervalMs);

#if !DIAG_STANDALONE
    // flushing the serial uart does nothing if nothing was written, but ensures it gets
    // out before sleeping if data was written.
    Serial.flush();
    // using 'idle' mode keeps the timers running, which I need since I don't currently
    // have a real-time-clock. Once I have an RTC, I'll switch it.
    sleep.idleMode();
    // sleep.adcMode();
    abortSleep = false;
    sleep.sleepDelay( timeout, abortSleep );
    // delay( timeout );
#else
    delay( 10 );
#endif
  }
}

#if DIAG_STANDALONE
// In DIAG_STANDALONE, the cmdQueue is filled from the following 'script'.
// For each entry, it waits the specified amout of time, and then places the supplied
// bytes into the queue.

// This 'normal' nothing calling state is from the 'calls' being active low and the 'valves'
// being active high and the three output bits being zero and bit zero unused (defaults high).
#define INACTIVE 0xf801
#define ZHWC 0x8000
#define Z1C  0x4000
#define Z2C  0x2000
#define Z3C  0x1000
#define Z4C  0x0800
#define ZHWV 0x0080
#define Z1V  0x0040
#define Z2V  0x0020
#define Z3V  0x0010
#define Z4V  0x0008
#define CIRC 0x0002

static const struct {
  unsigned long delay;
  uint16_t digital;
  uint16_t ioexp;
} testScript[] PROGMEM = {
  {             1000,  0x0004, INACTIVE },  // activate
  {              100,  0x0004, 0 },
  { SECONDStoMS( 5  ), 0x0000, ZHWC },    // HwCall
  { SECONDStoMS( 5  ), 0x0000, ZHWV | CIRC },
  { SECONDStoMS( 10 ), 0x0000, ZHWC },
  { SECONDStoMS(  5 ), 0x0000, ZHWV | CIRC  },
  { SECONDStoMS( 5 ),  0x0000, Z2C | Z2V | CIRC }, // Zone2Call
  { SECONDStoMS( 10 ), 0x0000, Z2C },
  { SECONDStoMS( 10 ), 0x0000, Z2V | CIRC },
  { SECONDStoMS( 5 ),  0x0000, Z3C },    // Zone3Call
  { SECONDStoMS( 5  ), 0x0000, Z3V | CIRC },
  { SECONDStoMS( 5 ),  0x0000, Z4C },    // Zone4Call
  { SECONDStoMS( 5  ), 0x0000, Z4V },
  { SECONDStoMS( 10 ), 0x0000, Z4C | Z4V },
  { SECONDStoMS( 10 ), 0x0000, Z3C },
  { SECONDStoMS( 5  ), 0x0000, Z3V | CIRC },
  { SECONDStoMS( 5 ),  0x0000, Z1C },    // Zone1Call
  { SECONDStoMS( 5  ), 0x0000, Z1V | CIRC },
  { SECONDStoMS( 10 ), 0x0000, Z1C },
  { SECONDStoMS( 5  ), 0x0000, Z1V | CIRC },
  // Now that zone1 is 'active', these should result in zone1 being called to purge.
  { SECONDStoMS( 5  ), 0x0000, ZHWC },    // HwCall
  { SECONDStoMS( 5  ), 0x0000, ZHWV | CIRC },
  { SECONDStoMS( 10 ), 0x0000, ZHWC },
  { SECONDStoMS( 1 ),  0x0000, Z1C | Z1V | ZHWV },
  { SECONDStoMS( 1 ),  0x0000, Z1C },
  { SECONDStoMS( 9 ),  0x0000, Z1V | CIRC },

  { SECONDStoMS( 5 ),  0x0000, Z2C | Z2V | CIRC }, // Zone2Call
  { SECONDStoMS( 10 ), 0x0000, Z2C },
  { SECONDStoMS( 1 ),  0x0000, Z1C | Z1V | Z2V },
  { SECONDStoMS( 1 ),  0x0000, Z1C },
  { SECONDStoMS( 9 ),  0x0000, Z1V | CIRC },
  
  { SECONDStoMS( 5 ),  0x0000, Z3C },    // Zone3Call
  { SECONDStoMS( 5  ), 0x0000, Z3V | CIRC },
  { SECONDStoMS( 5 ),  0x0000, Z4C },    // Zone4Call
  { SECONDStoMS( 5  ), 0x0000, Z4V },
  { SECONDStoMS( 10 ), 0x0000, Z4C | Z4V },
  { SECONDStoMS( 10 ), 0x0000, Z3C },
  { SECONDStoMS( 1 ),  0x0000, Z1C | Z1V | Z3V },
  { SECONDStoMS( 1 ),  0x0000, Z1C },
  { SECONDStoMS( 9 ),  0x0000, Z1V | CIRC },

  { SECONDStoMS( 5 ),  0x0000, Z2C | Z2V | CIRC }, // Zone2Call
  { SECONDStoMS( 5 ),  0x0000, Z2C | Z2V | CIRC },
  {               800, 0x0004, 0 },           // activate Anti-freeze
  {               200, 0x0004, 0 },
  { SECONDStoMS( 14 ), 0x0000, Z1C },         // 15 seconds since - should not trigger.
  { SECONDStoMS( 10 ), 0x0000, Z1C },
  { SECONDStoMS( 10 ), 0x0000, Z1C | Z1V | CIRC },         // 35 seconds since - should trigger.
  {               100, 0x0000, Z2C | Z2V },
  {              2900, 0x0000, Z2C | Z2V },
  { SECONDStoMS( 10 ), 0x0000, Z1C },
  { SECONDStoMS( 10 ), 0x0000, Z1V | CIRC },
  
  { SECONDStoMS( 26 ), 0x0000, Z2C },         // 45 seconds since - forces the call.
  { SECONDStoMS( 10 ), 0x0000, Z2V | CIRC },  // slower since the furnace fires.
  { SECONDStoMS( 10 ), 0x0000, Z2C },         
  { SECONDStoMS( 1 ),  0x0000, Z1C | Z1V | Z2V },
  { SECONDStoMS( 1 ),  0x0000, Z1C },
  { SECONDStoMS( 9 ),  0x0000, Z1V | CIRC },

  {              1000, 0x0004, 0 },           // deactivate Anti-freeze
  {               200, 0x0004, 0 },
  {              1000, 0x0004, 0 },
  {              1200, 0x0004, INACTIVE },    // deactivate
  { MINUTEStoMS( 30 ), 0x0000, 0 }
};
byte testScriptIndex = 0;

void processTestScript(void*)
{
  unsigned long delay = 0;
  do {
    uint16_t toggle = pgm_read_word_near( &testScript[testScriptIndex].digital );
    s_digitalIo = s_digitalIo ^ toggle;
    toggle = pgm_read_word_near( &testScript[testScriptIndex].ioexp );
    s_ioExp = s_ioExp ^ toggle;
    if( (s_digitalIo & 0x0004) == 0 )
      s_pushButtonPressed = true;
      
    ++testScriptIndex;
    if( testScriptIndex >= sizeof( testScript ) / sizeof( testScript[0] ) )
      testScriptIndex = 0;
    delay = pgm_read_word_near( &testScript[testScriptIndex].delay );
    delay |= (unsigned long) pgm_read_word_near( ((uint16_t*) &testScript[testScriptIndex].delay) + 1 ) << 16;
  } while( delay == 0 );
  timer.setTimeout( delay, processTestScript );
}
#endif // DIAG_STANDALONE

