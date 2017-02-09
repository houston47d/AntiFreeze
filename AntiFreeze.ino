#define VERSION "1.9"

// VERSION 1.9 - use of avr/sleep.h. USE_SLEEP works with DIAG_STANDALONE.
// VERSION 1.8 - refinements of sleep, watchdog, etc.
// VERSION 1.7 - completion of RTC usage. Ready for Winter 2016-2017.
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

// DIAG_STANDALONE runs the system through a test script in which the input signals are prescribed.
// To get it to match the ReferenceLog.csv, CALLS_ACTIVE_LOW needs to be true (the default) and USE_RTC
// needs to be 0 (so the times are consistent).
#define DIAG_STANDALONE 0
#define VERBOSITY 1
#define USE_RTC 1
// running standalone is incompatible with sleeping because it relies on the internal timers running.
// #define USE_SLEEP (!DIAG_STANDALONE && 1)
#define USE_SLEEP 1
// the sleep library uses the WDT to periodically wake the processor from sleep states,
// so it is incompatible with actually using it as a watchdog.
#define USE_WDT (!USE_SLEEP && 1)

#include "Verbosity.h"

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <MCP23S17.h>
#include <SimpleTimer.h>
#include <EEPROM.h>
#if USE_SLEEP
#include <avr/interrupt.h>
#include <avr/sleep.h>
#endif
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#define sdCardCs 7
// If running on the UNO with the Adafruit Music Make shield, SD card is on pin 4.
// #define sdCardCs 4
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

SimpleTimer timer;

bool sdCardPresent = false;
File logfile;
// Want to periodically sync the log file, which forces writing out the directory information,
// in case of a loss-of-power. Otherwise, the directory entry indicates a zero length and you 
// get nothing in the log file. But it is expensive, so don't want to do it too often. The 
// 'File' implementation of flush simply calls sync - it doesn't seem quite right, but I have 
// no other access to the underlying SdFile class it is wrapping.
void syncLogFile() {
  if(logfile) {
    logfile.flush();  
  }
}

bool rtcPresent = false;
RTC_DS1307 rtc;
DateTime bootTime;
// this function is provided to the SD card in order to put real timestamps on the files.
void myDateTimeCallback(uint16_t* date, uint16_t* time) {
  DateTime now = rtc.now();
  *date = FAT_DATE(now.year(), now.month(), now.day());
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

bool ioExpPresent = false;
MCP ioExp0( 0, ioExpCs );

unsigned long s_currentTimeS = 0;
#define s_currentCycleS ((uint16_t) s_currentTimeS)
char s_currentCycleTime[22];  // DD-MMM-YYYY HH:MM:SS or HH:MM:SS
void updateCurrentTime() {
  if( USE_RTC && rtcPresent ) {
    TimeSpan timeSinceBoot = rtc.now() - bootTime;
    s_currentTimeS = timeSinceBoot.totalseconds();
  } else {
    // due to the divide, this will produce a discontinuity in the time values when millis()
    // wraps around, approximately every 49 days. Since the purpose of the RTC is to address
    // this, I'm ok with it here.
    s_currentTimeS = (millis() / 1000);
  }
  s_currentCycleTime[0] = 0;
}
// This is only installed if rtcPresent, so not tested here again.
unsigned long myCourseTimerFunc() {
  // TimeSpan timeSinceBoot = rtc.now() - bootTime;
  // return( timeSinceBoot.totalseconds() );
  return( s_currentTimeS );
}
char* getCurrentTime() {
  if( s_currentCycleTime[0] == 0 ) {
    if( USE_RTC && rtcPresent )
      generateDateTimeRTC( s_currentCycleTime );
    else
      generateHMS( s_currentCycleTime, s_currentCycleS );
  }
  return( s_currentCycleTime );
}

#if DIAG_STANDALONE
volatile uint16_t s_digitalIo = -1;
volatile uint16_t s_ioExp = 0;
#endif

#define EEPROM_ADDR_SIG 0x000
#define EEPROM_ADDR_ACTIVE (EEPROM_ADDR_SIG+1)
#define EEPROM_ADDR_PURGE (EEPROM_ADDR_SIG+2)
#define EEPROM_ADDR_ANTIFREEZE (EEPROM_ADDR_SIG+3)
#define EEPROM_ADDR_LOG (EEPROM_ADDR_SIG+4)
#define EEPROM_SIG 0xae

#if USE_WDT
void watchdogResetFunc() {
  wdt_reset();
}
#endif
volatile bool s_wdtFired = false;
ISR(WDT_vect) {
  wdt_disable();
  s_wdtFired = true;
}

enum Source { eNone, eDigitalIo, eAnalogIo, eIoExpander };
struct BoolSignal {
    uint8_t source : 2;
    uint8_t type : 2;
    uint8_t activeLow : 1;
    uint8_t logChanges : 1;
    // uint8_t reserved0 : 2;
    uint8_t priorValid : 1;
    uint8_t currentState : 1;
    // uint8_t reserved1 : 6;
    uint8_t pin;
    const PROGMEM char* name;
    uint16_t priorTransition;
    
    void init();
    void write( bool value, bool force = false );
    uint16_t timeSincePrior() const {
      return( priorValid ? s_currentCycleS - priorTransition : -1 );
    }
    void setLogChanges(bool enable);
    BoolSignal( Source _source, uint8_t _pin, uint8_t _type, const PROGMEM char* _name, bool _activeLow, bool _log  )
      : source( _source ), pin( _pin ), type( _type ), name( _name ), logChanges( _log ), priorValid( false ), priorTransition( 0 ), activeLow( _activeLow ), currentState( false ) {}
};

static const char activeName[] PROGMEM = { "Active" };
BoolSignal active( eNone, 0, 0, activeName, false, true );
static const char activePrgName[] PROGMEM = { "Purge" };
BoolSignal activePrg( eNone, 0, 0, activePrgName, false, true );
static const char activeAFName[] PROGMEM = { "AntiFreeze" };
BoolSignal activeAF( eNone, 0, 0, activeAFName, false, true );
static const char zone2NeedsCallName[] PROGMEM = { "Zone2NeedsCall" };
BoolSignal zone2NeedsCall( eNone, 0, 0, zone2NeedsCallName, false, true );
static const char zone2ReallyNeedsCallName[] PROGMEM = { "Zone2ReallyNeedsCall" };
BoolSignal zone2ReallyNeedsCall( eNone, 0, 0, zone2ReallyNeedsCallName, false, true );
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
BoolSignal greenLed( eDigitalIo, A2, OUTPUT, greenLedName, true, false );
static const char redLedName[] PROGMEM = { "RedLed" };
BoolSignal redLed( eDigitalIo, A3, OUTPUT, redLedName, true, false );
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
BoolSignal burner( eIoExpander, 0, INPUT_PULLUP, burnerName, true, false ); // burner is not currently connected, so don't log.

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

volatile bool s_pushButtonPressed = false;
volatile bool s_pushButtonIntInstalled = false;
void switchHandler() {
  s_pushButtonPressed = true;
  detachInterrupt(digitalPinToInterrupt( pushButton.pin ));
  s_pushButtonIntInstalled = false;
}
bool s_ioExpanderInterrupt = false;
void ioExpHandler() {
  ioExp0.wordRead(INTCAPA);
  s_ioExpanderInterrupt = true;
}

uint8_t s_ledTimer = -1;
void ledTimerExpired() {
  uint8_t remaining = timer.remaining( s_ledTimer );
  
  bool greenState = false;
  bool redState = false;
  if( remaining == 0 ) {
    // turn both LEDs off.
    s_ledTimer = -1; // we will be deleted
  }
  else {
    // Let's set the red if either the IO expander is not present or the logfile couldn't be opened.
    redState = !ioExpPresent || active.currentState && !logfile;
    if( active.currentState ) {
      if( activeAF.currentState ) {
        // if purge + antifreeze, then fast flash.
        greenState = (remaining & 0x1) != 0;
      }
      else if( activePrg.currentState ) {
        // if purge, then slow flash.
        greenState = (remaining & 0x2) != 0;
      }
      else
        // if monitoring only, solid green.
        greenState = true;
    }
  }
  greenLed.write( greenState );
  redLed.write( redState );
}

void setup() {
  Serial.begin(115200);

  // Not (currently) using the ADC, so turn it off.
  _print4(F("ADCSRA started as 0x")); _println4(ADCSRA, HEX);
  ADCSRA = 0;
  // Turn off power to peripherals that we are not using. TWI is used for RTC, Timer0 for millis,
  // SPI used for IO Expander and SD card, and USART0 for Serial.
  _print4(F("PRR started as 0x")); _println4(PRR, HEX);
  PRR = _BV(PRTIM2) | _BV(PRTIM1) | _BV(PRADC);
  _print4(F("CLKPR is 0x")); _println4(CLKPR, HEX);
  
#if DIAG_STANDALONE
  timer.setTimeout( 1000, processTestScript );
#endif

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);

#if USE_RTC
  // The begin() function always returns true - no indication of whether the clock
  // is actually present.
  rtc.begin();
  uint8_t origData = rtc.readnvram( 0 );
  _print4( "origData " ); _println4( origData );
  rtc.writenvram( 0, origData ^ 0xff );
  _print4( "wroteData " ); _println4( origData ^ 0xff );
  uint8_t readData = rtc.readnvram( 0 );
  _print4( "readData " ); _println4( readData );
  rtc.writenvram( 0, origData );
  rtcPresent = (readData == (origData ^ 0xff));
  if( rtcPresent ) {
    bootTime = rtc.now();
    SdFile::dateTimeCallback( myDateTimeCallback );
    timer.setCourseTimeCallback( myCourseTimerFunc );
  }
#endif

  ioExpPresent = ioExp0.begin();
  
  for( byte i = 0; i < s_NumSignals; ++i ) {
    BoolSignal* sig( (BoolSignal*) pgm_read_word_near( &s_Signals[i] ) );
    sig->init();
  }
  if( ioExpPresent ) {
    // Set MIRROR (shared interrupt pin), ensure ODR and INTPOL are clear.
    uint8_t ioCon = ioExp0.byteRead(IOCON);
    ioExp0.byteWrite(IOCON, (ioCon | 0x40) & ~0x06);
    unsigned int ioDirRead = ioExp0.wordRead(IODIRA);
    // Generate interrupt when input changes from previous value.
    ioExp0.wordWrite(INTCONA, 0);
    // Enable change interrupt on all input channels.
    ioExp0.wordWrite(GPINTENA, ioDirRead);

    // The input is configured as a push-pull active low, so no pullup required. The IoExp asserts the 
    // interrupt until a read of INTCAPA, done in the interrupt handler. Has to be LOW for it to wake
    // us from sleep modes.
    pinMode( ioExpInt, INPUT );
    attachInterrupt(digitalPinToInterrupt( ioExpInt ), ioExpHandler, LOW); 
  }
    
  // Note that 'FALLING' will not work for lower power states - that requires 'LOW'.
  s_pushButtonIntInstalled = true;
  attachInterrupt(digitalPinToInterrupt( pushButton.pin ), switchHandler, LOW); 

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

  // Setup a timer to invalidate the times stored in the signals before the timer wraps around.
  timer.setInterval( MINUTEStoS(60), checkForSecondsOverflow, true );
  // Setup a timer to sync the log file, if open. This updates the directory entry to reflect
  // current file state. It has the side effect of flushing the file if the directory entry
  // is dirty (since it has to load in the directory entry).
  timer.setInterval( MINUTEStoS(60), syncLogFile, true );

#if USE_WDT
  // Enable the hardware watchdog timer for a 2 second timeout, and setup a timer to 
  // reset the timer every 1/4 second. Using the timer ensures that it runs any time
  // that the timer is run.
  timer.setInterval( 250, watchdogResetFunc );
  wdt_enable( WDTO_2S );
#endif

#if !USE_SLEEP
  // When not using sleep, we periodically want to update our 'course' time. During sleep, this 
  // is done each time we wake up. But without sleep, this would otherwise run constantly.
  timer.setInterval( SECONDStoMS(1), updateCurrentTime );
#endif

  // Run the monitor for the first time to establish the current state of all signals.
  processMonitorTimer();
}

void processPushButton() {
  // Do some debouncing and detect a long-press. Interrupt is on LOW (not change),
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
  // we have waited for the pushbutton to stabilize active, and then to stabilize inactive. So we should now
  // be inactive and can install the interrupt handler again.
  s_pushButtonIntInstalled = true;
  attachInterrupt(digitalPinToInterrupt( pushButton.pin ), switchHandler, LOW); 
}

void processActiveStateChange() {
  if( active.currentState ) {
#if USE_WDT
    // This can take several seconds if the card is not present.
    wdt_disable();
#endif

    // do the SD.begin() here in case the card has been removed/reinserted
    sdCardPresent = SD.begin( sdCardCs );

#if USE_WDT
    wdt_enable( WDTO_2S );
#endif

    // create a new file
    if( sdCardPresent ) {
      char filename[13]; // bbbbbbbb.eeen
      generateBaseFilename(filename);
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

      // Since the log file wasn't open when we switched to the active state, force it to be 
      // logged now.
      writeLogEntry( active );
    }
    else {
      _print1( F("  + Error: no SD card found") ); _print1( F(", code ") ); _print1( SD.errorCode() ); _print1( F(" ") ); _print1( F("  errorData ") ); _println1( SD.errorData() );
    }
  }

  if( !active.currentState ) {
    if( logfile ) {
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
      s_pushButtonPressed = false;
      _println4( F("  + Push Button interrupt") );
      processPushButton();
    }
    if( s_ioExpanderInterrupt ) {
      s_ioExpanderInterrupt = false;
      _println4( F("  + Expander interrupt") );
      processMonitorTimer();
    }

    timer.run();

    bool someStateChanged = false;
    if( wasActive != active.currentState ) {
      processActiveStateChange();
      wasActive = active.currentState;
      someStateChanged = true;

      _println3( F("  + Updating current state in EEPROM (1)") );
      EEPROM.update( EEPROM_ADDR_SIG, EEPROM_SIG );
      EEPROM.update( EEPROM_ADDR_ACTIVE, active.currentState );  
    }

    if( wasActivePrg != activePrg.currentState 
        || wasActiveAF != activeAF.currentState ) {
      activeChanged( active.currentState, activePrg.currentState, activeAF.currentState );
      wasActivePrg = activePrg.currentState;
      wasActiveAF = activeAF.currentState;
      someStateChanged = true;

      _println3( F("  + Updating current state in EEPROM (2)") );
      EEPROM.update( EEPROM_ADDR_PURGE, activePrg.currentState );  
      EEPROM.update( EEPROM_ADDR_ANTIFREEZE, activeAF.currentState );  
    }

    if( someStateChanged ) {
      if( s_ledTimer >= 0 )
        timer.deleteTimer( s_ledTimer );
      s_ledTimer = timer.setTimer( 500, ledTimerExpired, 20 );
      ledTimerExpired(); // run the first time immediately.
    }
    
#if USE_SLEEP
    // flushing the serial uart does nothing if nothing was written, but ensures it gets
    // out before sleeping if data was written.
    Serial.flush();

    // using 'idle' mode keeps the timers running, which I need since I don't currently
    // have a real-time-clock. Once I have an RTC, I'll switch it.
    uint16_t timeout;
    uint8_t fineTimers = timer.getNumEnabledTimers( false );
    if( rtcPresent && fineTimers == 0 ) {
      set_sleep_mode(SLEEP_MODE_STANDBY); // not using ADC, so this is better.
      timeout = timer.getDelayTillNext(true);
      // Since we want to convert to milliseconds, use maximum if we will overflow.
      timeout = (timeout >= 0x40 ? 0xffff : timeout * 1024);
    }
    else {
      set_sleep_mode(SLEEP_MODE_IDLE);
      timeout = timer.getDelayTillNext(false);
    }

    // Find the longest watchdog delay that is less than the desired timout. 
    // Longest time is really 8192, but 8000 looks better and makes it more likely that we will
    // sleep a bit longer then requested so it is actually triggered when we wake up.
    uint8_t wdTimeoutReg = WDTO_8S;
    uint16_t wdTimeoutMs = 8000; 
    while( wdTimeoutReg > WDTO_15MS && wdTimeoutMs > timeout ) {
      wdTimeoutMs /= 2;
      wdTimeoutReg -= 1;
    }
    
    cli();
    if( !s_pushButtonPressed && !s_ioExpanderInterrupt ) {
      s_wdtFired = false;
      // Can't use wdt_enable because it clears the WDIE (interrupt enable) bit which we require 
      // for our ISR to be called. This routine is copied from wdt.h/wdt_enable with the WDIE bit
      // enabled and the SREG manipulations removed because I know I'm already in a cli()/sei() pair.
      __asm__ __volatile__ (
          "wdr" "\n\t"
          "sts %0, %1" "\n\t"
          "sts %0, %2" "\n \t"
          : /* no outputs */
          : "n" (_SFR_MEM_ADDR(_WD_CONTROL_REG)),
          "r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDIE) | _BV(WDE))),
          "r" ((uint8_t) ((wdTimeoutReg & 0x08 ? _WD_PS3_MASK : 0x00) | _BV(WDIE) | _BV(WDE) | (wdTimeoutReg & 0x07)) )
      );
      sleep_enable();
      // Why the while loop? Because SLEEP_MODE_IDLE is woken by (among other thnigs) the Timer0 and USART interrupts.
      // So in the absence of the while loop, this block will exit in rather short order. And don't want to disable them
      // since running the timers is the whole point of using IDLE mode.
      while( !s_pushButtonPressed && !s_ioExpanderInterrupt && !s_wdtFired ) {
        sleep_bod_disable();
        sei();
        sleep_cpu();
        cli();
      }
      sleep_disable();
      // In case we exited due to one of our interrupts. 
      wdt_disable();
    }
    sei();

    updateCurrentTime();
#endif
  }
}


