#define VERSION "1.6"

// VERSION 1.6 - addition (for real) of the RTC and logging of real-time.
// VERSION 1.5 - separation of monitor, purge, and anti-freeze modes. Purge only zones 2 and 3.
// VERSION 1.4 - SRAM use reduction (to make room for the RTC), zone1Active only looks at thermostat calls.
// VERSION 1.3 - addition of logging parameters, real-time-clock.
// VERSION 1.2 - restructuring of the files, addition of the watchdog timer and the use of the 
//   EEPROM to remember what state it was in.
// VERSION 1.1 - diagnostics added to investigate issues in 1.0, mainly related to what appeared
//   to be communication failures with the Io Expander.
// VERSION 1.0 - first working version, ready for live test.
// VERSION < 1.0 - investigations.

// For testing purposes, it is useful ta have the calls active HIGH since when not connected
// to a real furnace all of the calls read back active (which is a highly unusual state for
// the furnace and not one which I'm trying to modify the behavior of). Selecting CALLS_ACTIVE_LOW
// as false makes it better behaved in this case.
#define CALLS_ACTIVE_LOW true
// #define CALLS_ACTIVE_LOW false

#define DIAG_STANDALONE 0
#define VERBOSITY 3
#define USE_RTC 1

// no idea why this helps, but removing it causes functions from Utilities to not be defined.
extern void generateHMS( char* buffer, unsigned long seconds );

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

#include <SPI.h>
#include <SD.h>
#if USE_RTC
#include <Wire.h>
#include <RTClib.h>
#endif
#include <MCP23S17.h>
#include <SimpleTimer.h>
// #include <Sleep_n0m1.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#define sdCardCs 7
#define ioExpCs 10
#define ioExpInt 3

#define SECONDStoMS(s) ((s)*1000U)
#define MINUTEStoMS(m) ((m)*60*1000U)
#define MINUTEStoS(m)  ((m)*60U)
#define HOURStoS(h)    ((h)*3600U)

#if DIAG_STANDALONE
#define callOutputDurationMs SECONDStoMS(2)
#define callOutputToValveTimeoutMs MINUTEStoMS(5)
#define zone2callOutputDurationMs SECONDStoMS(10)
#define zone2CallIntervalMinS 30
#define zone2CallIntervalMaxS 45
#else
// callOutputDurationMs - how long to assert a call output after the corresponding valve has
// opened.
#define callOutputDurationMs SECONDStoMS(2)
// callOutputToValveTimeoutMs - how long to wait for the corresponding valve to open after
// asserting the call output. Set to cover the case of a cold furnace having to heat up to 
// service the call.
#define callOutputToValveTimeoutMs MINUTEStoMS(5)
// zone2callOutputDurationMs - how long to assert the call after the valve opens for zone 2,
// as the purpose in this case is to get hot water all the way through the loop.
// zone2CallIntervalMinS - time after which to start cycle zone 2 if the furnace is already
// servicing another zone.
// zone2CallIntervalMaxS - time after which to start cycle zone 2 even if the furnace is not
// servicing another zone (means we will trigger the burner, but oh well).
#define zone2callOutputDurationMs MINUTEStoMS(1)
#define zone2CallIntervalMinS MINUTEStoS(60)
#define zone2CallIntervalMaxS HOURStoS(6)
// #define zone2CallIntervalMinS MINUTEStoS(30)
// #define zone2CallIntervalMaxS MINUTEStoS(45)
#endif

#define tempPin A0

#undef USE_SLEEP
#if USE_SLEEP
Sleep sleep;
bool abortSleep = false;
#endif

SimpleTimer timer;

bool sdCardPresent = false;
char filename[] = "LOGGER00.CSV";
File logfile;

#if USE_RTC
bool rtcPresent = false;
RTC_DS1307 rtc;
#endif

bool ioExpPresent = false;
MCP ioExp0( 0, ioExpCs );

uint16_t s_currentCycleS = 0;
char s_currentCycleTime[22];  // DD-MMM-YYYY HH:MM:SS or HH:MM:SS

void updateCurrentTime() {
  unsigned long currentCycleS = (millis() / 1000);
  s_currentCycleS = (uint16_t) currentCycleS;
  s_currentCycleTime[0] = 0;
}
char* getCurrentTime() {
  if( s_currentCycleTime[0] == 0 ) {
#if USE_RTC
    if( rtcPresent )
      generateDateTimeRTC( s_currentCycleTime );
    else
#endif
    {
      unsigned long currentCycleS = (millis() / 1000);
      generateHMS( s_currentCycleTime, currentCycleS );
    }
  }
  return( s_currentCycleTime );
}

#if DIAG_STANDALONE
uint16_t s_digitalIo = -1;
uint16_t s_ioExp = 0;
#endif

#define EEPROM_ADDR_SIG 0x000
#define EEPROM_ADDR_ACTIVE (EEPROM_ADDR_SIG+1)
#define EEPROM_ADDR_PURGE (EEPROM_ADDR_SIG+2)
#define EEPROM_ADDR_ANTIFREEZE (EEPROM_ADDR_SIG+3)
#define EEPROM_ADDR_LOG (EEPROM_ADDR_SIG+4)
#define EEPROM_SIG 0xae

void watchdogResetFunc() {
  wdt_reset();
}

enum Source { eNone, eDigitalIo, eAnalogIo, eIoExpander };
struct BoolSignal {
    uint8_t source : 2;
    uint8_t type : 2;
    uint8_t activeLow : 1;
    uint8_t logChanges : 1;
    uint8_t priorValid : 1;
    uint8_t currentState : 1;
    uint8_t pin;
    const PROGMEM char* name;
    uint16_t priorTransition;
    
    void init();
    void write( bool value, bool force = false );
    uint16_t timeSincePrior() const {
      return( priorValid ? s_currentCycleS - priorTransition : -1 );
    }
    BoolSignal( Source _source, uint8_t _pin, uint8_t _type, const PROGMEM char* _name, bool _activeLow, bool _log  )
      : source( _source ), pin( _pin ), type( _type ), name( _name ), logChanges( _log ), priorValid( false ), priorTransition( 0 ), activeLow( _activeLow ), currentState( false ) {}
};

static const char activeName[] PROGMEM = { "Active" };
BoolSignal active( eNone, 0, 0, activeName, false, false );
static const char activePrgName[] PROGMEM = { "Purge" };
BoolSignal activePrg( eNone, 0, 0, activePrgName, false, true );
static const char activeAFName[] PROGMEM = { "AntiFreeze" };
BoolSignal activeAF( eNone, 0, 0, activeAFName, false, true );
static const char zone2NeedsCallName[] PROGMEM = { "Zone2NeedsCall" };
BoolSignal zone2NeedsCall( eNone, 0, 0, zone2NeedsCallName, false, false );
static const char zone2ReallyNeedsCallName[] PROGMEM = { "Zone2ReallyNeedsCall" };
BoolSignal zone2ReallyNeedsCall( eNone, 0, 0, zone2ReallyNeedsCallName, false, false );
static const char anyZonesCallingName[] PROGMEM = { "AnyZonesCalling" };
BoolSignal anyZonesCalling( eNone, 0, 0, anyZonesCallingName, false, false );
static const char zone1CallThermName[] PROGMEM = { "Zone1CallTherm" };
BoolSignal zone1CallTherm( eNone, 0, 0, zone1CallThermName, false, false );
static const char zone1ActiveName[] PROGMEM = { "Zone1Active" };
BoolSignal zone1Active( eNone, 0, 0, zone1ActiveName, false, false );

// The pushbutton is between the pin and GND, so LOW means the button was pushed.
static const char pushButtonName[] PROGMEM = { "PushButton" };
BoolSignal pushButton( eDigitalIo, 2, INPUT_PULLUP, pushButtonName, true, false );
// The two LEDs are tied to +V through a resistor, so 'on' is LOW (and thus activeLow = true).
static const char greenLedName[] PROGMEM = { "GreenLed" };
BoolSignal greenLed( eDigitalIo, A4, OUTPUT, greenLedName, true, false );
static const char redLedName[] PROGMEM = { "RedLed" };
BoolSignal redLed( eDigitalIo, A5, OUTPUT, redLedName, true, false );
// There are 13 inputs covering the calls into the controller and the valve controls
// out of the controller. The calls are active low because the thermostat grounds the
// signal to call for heat.
static const char zoneHwCallName[] PROGMEM = { "ZoneHwCall" };
BoolSignal zoneHwCall( eIoExpander, 15, INPUT, zoneHwCallName, CALLS_ACTIVE_LOW, true );
static const char zone1CallName[] PROGMEM = { "Zone1Call" };
BoolSignal zone1Call( eIoExpander, 14, INPUT, zone1CallName, CALLS_ACTIVE_LOW, true );
static const char zone2CallName[] PROGMEM = { "Zone2Call" };
BoolSignal zone2Call( eIoExpander, 13, INPUT, zone2CallName, CALLS_ACTIVE_LOW, true );
static const char zone3CallName[] PROGMEM = { "Zone3Call" };
BoolSignal zone3Call( eIoExpander, 12, INPUT, zone3CallName, CALLS_ACTIVE_LOW, true );
static const char zone4CallName[] PROGMEM = { "Zone4Call" };
BoolSignal zone4Call( eIoExpander, 11, INPUT, zone4CallName, CALLS_ACTIVE_LOW, true );

// The following are all INPUT_PULLUP and active low only because they are not currently connected.
static const char zoneHwValveName[] PROGMEM = { "ZoneHwValve" };
BoolSignal zoneHwValve( eIoExpander, 7, INPUT, zoneHwValveName, false, true );
static const char zone1ValveName[] PROGMEM = { "Zone1Valve" };
BoolSignal zone1Valve( eIoExpander, 6, INPUT, zone1ValveName, false, true );
static const char zone2ValveName[] PROGMEM = { "Zone2Valve" };
BoolSignal zone2Valve( eIoExpander, 5, INPUT, zone2ValveName, false, true );
static const char zone3ValveName[] PROGMEM = { "Zone3Valve" };
BoolSignal zone3Valve( eIoExpander, 4, INPUT, zone3ValveName, false, true );
static const char zone4ValveName[] PROGMEM = { "Zone4Valve" };
BoolSignal zone4Valve( eIoExpander, 3, INPUT, zone4ValveName, false, true );
static const char inductorName[] PROGMEM = { "InductorOn" };
BoolSignal inductor( eIoExpander, 2, INPUT, inductorName, false, true );
static const char circulatorName[] PROGMEM = { "CirculatorOn" };
BoolSignal circulator( eIoExpander, 1, INPUT, circulatorName, false, true );
static const char burnerName[] PROGMEM = { "BurnerOn" };
BoolSignal burner( eIoExpander, 0, INPUT_PULLUP, burnerName, true, true );

// There are three call out signals that (currently) activate the respective zone call.
// The relays are normally opened, and a high output closes the relay and grounds the
// respective signal.
static const char zone1CallOutName[] PROGMEM = { "Zone1CallOut" };
BoolSignal zone1CallOut( eIoExpander, 10, OUTPUT, zone1CallOutName, false, true );
static const char zone2CallOutName[] PROGMEM = { "Zone2CallOut" };
BoolSignal zone2CallOut( eIoExpander, 9, OUTPUT, zone2CallOutName, false, true );
static const char zone3CallOutName[] PROGMEM = { "Zone3CallOut" };
BoolSignal zone3CallOut( eIoExpander, 8, OUTPUT, zone3CallOutName, false, true );

// The following array of pointers just makes it easier to handle the signals in a
// generic fashion when appropriate.
BoolSignal* const s_Signals[] PROGMEM = {
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
    &zone3CallOut,
    // internal state
    &active,
    &activePrg,
    &activeAF,
    &zone2NeedsCall,
    &zone2ReallyNeedsCall,
    &anyZonesCalling,
    &zone1CallTherm,
    &zone1Active,
};
byte s_NumSignals = sizeof( s_Signals ) / sizeof( s_Signals[0] );
BoolSignal* const s_CallSignals[] PROGMEM = {
    &zoneHwCall,
    &zone1Call,
    &zone2Call,
    &zone3Call,
    &zone4Call
};
BoolSignal* const s_ValveSignals[] PROGMEM = {
    &zoneHwValve,
    &zone1Valve,
    &zone2Valve,
    &zone3Valve,
    &zone4Valve
};
byte s_NumCallSignals = sizeof( s_CallSignals ) / sizeof( s_CallSignals[0] );

void checkForSecondsOverflow() {
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
      // int16_t relative = (int16_t) (s_currentCycleS - sig->priorTransition);
      // if( relative < 0 )
      //  sig->priorValid = false;
      uint16_t relative = (s_currentCycleS - sig->priorTransition);
      if( relative > HOURStoS( 17 ) )
        sig->priorValid = false;
    }
  }
}

#define ReadTemperature 0
#if ReadTemperature
float temperatureC = 0.0;
#endif

void showCapabilities() {
  _println1( F("Furnace Sidekick " VERSION " - " __DATE__ " " __TIME__) );
#if DIAG_STANDALONE
  _println1( F("Standalone Mode") );
#endif // DIAG_STANDALONE
  _println2( F("") );
#if USE_RTC
  bool rtcRunning = rtc.isrunning();
  _print2( F("Real-time clock is ") ); _print2( rtcPresent ? F("present") : F("ABSENT") ); _print2( F(" and ") ); _println2( rtcRunning ? F("running") : F("STOPPED") );
#endif
  _print2( F("IoExpander CS on pin ") ); _print2( ioExpCs ); _print2( F(" is ") ); _println2( ioExpPresent ? F("present") : F("ABSENT") );
  _print2( F("SD Card CS on pin ") ); _println2( sdCardCs );
  _println2( F("Signals:") );
  for( byte i = 0; i < s_NumSignals; ++i ) {
    const BoolSignal* sig( (const BoolSignal*) pgm_read_word_near( &s_Signals[i] ) );
    _print2( F("  ") );
    _print2( reinterpret_cast<const __FlashStringHelper *>( sig->name ) );
    _print2( F(", ") );
    _print2( sig->source == eIoExpander ? F("Expander ") : sig->source == eAnalogIo ? F("Analog ") : sig->source == eDigitalIo ? F("Digital ") : F("None ") );
    _println2( sig->pin );
  }
}

bool s_pushButtonPressed = false;
void switchHandler() {
  s_pushButtonPressed = true;
#if USE_SLEEP
  abortSleep = true;
#endif
}
bool s_ioExpanderInterrupt = false;
void ioExpHandler() {
  ioExp0.wordRead(INTCAPA);
  s_ioExpanderInterrupt = true;
#if USE_SLEEP
  abortSleep = true;
#endif
}

uint8_t s_ledTimer = -1;
uint8_t s_ledCounter = 0;
void ledTimerExpired() {
  ++s_ledCounter;
  bool greenState = false;
  bool redState = active.currentState && !logfile;
  if( active.currentState ) {
    if( activeAF.currentState ) {
      // if purge + antifreeze, then fast flash.
      greenState = (s_ledCounter & 0x1) != 0;
    }
    else if( activePrg.currentState ) {
      // if purge, then slow flash.
      greenState = (s_ledCounter & 0x2) != 0;
    }
    else
      // if monitoring only, solid green.
      greenState = true;
  }
  else {
    greenState = false;
  }

  greenLed.write( greenState );
  redLed.write( redState );
}

void setup() {
  Serial.begin(115200);

#if DIAG_STANDALONE
  timer.setTimeout( 1000, processTestScript );
#endif
  s_ledTimer = timer.setInterval( 500, ledTimerExpired );

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);

#if USE_RTC
  rtcPresent = rtc.begin();
#endif

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
    // pinMode( ioExpInt, INPUT );
    // attachInterrupt(digitalPinToInterrupt( ioExpInt ), ioExpHandler, FALLING); 
  }
    
  // Note that 'FALLING' will not work for lower power states - that requires 'LOW'.
  attachInterrupt(digitalPinToInterrupt( pushButton.pin ), switchHandler, FALLING); 

  showCapabilities();

  if( EEPROM.read( EEPROM_ADDR_SIG ) == EEPROM_SIG ) {
    _println3( F("  + valid settings found in EEPROM") );
#if !DIAG_STANDALONE
    // In standalone mode, it assumes the initial state is inactive.
    active.currentState = EEPROM.read( EEPROM_ADDR_ACTIVE );
    activePrg.currentState = EEPROM.read( EEPROM_ADDR_PURGE );
    activeAF.currentState = EEPROM.read( EEPROM_ADDR_ANTIFREEZE );
#endif
  }

  // Setup a timer to invalid the times stored in the signals before the timer wraps around.
  timer.setInterval( MINUTEStoS(60), checkForSecondsOverflow, true );

  // Enable the hardware watchdog timer for a 2 second timeout, and setup a timer to 
  // reset the timer every 1/4 second. Using the timer ensures that it runs any time
  // that the timer is run.
  timer.setInterval( 250, watchdogResetFunc );
  wdt_enable( WDTO_2S );

  // This is the function that performs the bulk of our processing, reading inputs and 
  // determining a new state for the outputs.
  timer.setInterval( SECONDStoMS(1), processMonitorTimer );
}

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
        active.write( false );
        activePrg.write( false );
        activeAF.write( false );
      }
      // cycle through the three states of both off, purge on, both on.
      else if( !activePrg.currentState && !activeAF.currentState ) {
        activePrg.write( true );
      }
      else if( activePrg.currentState && !activeAF.currentState ) {
        activeAF.write( true );
      }
      else {
        activePrg.write( false );
        activeAF.write( false );
      }
    }
    else
      active.write( true );
  }
}

void processActiveStateChange() {
  if( active.currentState ) {
    // This can take several seconds if the card is not present.
    wdt_disable();

    // do the SD.begin() here in case the card has been removed/reinserted
    sdCardPresent = SD.begin( sdCardCs );

    wdt_enable( WDTO_2S );

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
      _print1( logfile ? F("  + Logging to file ") : F("  + Error: failed to open file ") ); _println1( filename );
    }
    else {
      _print1( F("  + Error: no SD card found") ); _print1( F(", code ") ); _print1( SD.errorCode() ); _print1( F(" ") ); _print1( F("  errorData ") ); _println1( SD.errorData() );
    }
  }

  if( !active.currentState ) {
    if( logfile ) {
      logfile.flush();
      logfile.close();
    }
    activeChanged( active.currentState, false, false );
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
  bool wasActive = false;
  bool wasActivePrg = false;
  bool wasActiveAF = false;

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

    timer.run();
   
    if( wasActive != active.currentState ) {
      processActiveStateChange();
      wasActive = active.currentState;

      _println3( F("  + Updating current state in EEPROM (1)") );
      EEPROM.update( EEPROM_ADDR_SIG, EEPROM_SIG );
      EEPROM.update( EEPROM_ADDR_ACTIVE, active.currentState );  
    }

    if( wasActivePrg != activePrg.currentState 
        || wasActiveAF != activeAF.currentState ) {
      activeChanged( active.currentState, activePrg.currentState, activeAF.currentState );
      wasActivePrg = activePrg.currentState;
      wasActiveAF = activeAF.currentState;

      _println3( F("  + Updating current state in EEPROM (2)") );
      EEPROM.update( EEPROM_ADDR_PURGE, activePrg.currentState );  
      EEPROM.update( EEPROM_ADDR_ANTIFREEZE, activeAF.currentState );  
    }

#if 0
#if !DIAG_STANDALONE
    // flushing the serial uart does nothing if nothing was written, but ensures it gets
    // out before sleeping if data was written.
    Serial.flush();
    // not sure about the best approach - keeping it open and flushing, or just opening/closing
    // it each time an entry is written.
    if( logfile )
      logfile.flush();
#if USE_SLEEP
    // using 'idle' mode keeps the timers running, which I need since I don't currently
    // have a real-time-clock. Once I have an RTC, I'll switch it.
    sleep.idleMode();
    // sleep.adcMode();
    abortSleep = false;
    sleep.sleepDelay( timeout, abortSleep );
#endif

    // delay( timeout );
#else
    delay( 10 );
#endif
#endif
  }
}


