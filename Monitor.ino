
void BoolSignal::init() {
  switch( source ) {
  case eDigitalIo:
    _print4( F("setting Digital ") ); _print4( pin ); _print4( F(" to ") ); _println4( type );
    pinMode( pin, type );
    if( type == OUTPUT )
      digitalWrite( pin, activeLow ? HIGH : LOW );
    break;
  case eIoExpander:
    if( ioExpPresent ) {
      ioExp0.pinMode( pin, type );
      if( type == OUTPUT )
         ioExp0.digitalWrite( pin, activeLow ? HIGH : LOW );
    }
    break;
  default:
    break;
  }
}
void BoolSignal::write( bool value, bool force ) {
  if( value != currentState || force ) {
    if( type == OUTPUT ) {
      switch( source ) {
      case eDigitalIo:
        digitalWrite( pin, value ^ activeLow ? HIGH : LOW );
        break;
      case eIoExpander:
        if( ioExpPresent ) {
          ioExp0.digitalWrite( pin, value ^ activeLow ? HIGH : LOW );
        }
        break;
      default:
        break;
      }
    }
    currentState = value;
    if( logChanges )
      writeLogEntry( *this );
    priorValid = true;
    priorTransition = s_currentCycleS;
  }
}

void BoolSignal::setLogChanges(bool enable) {
  if( enable ) {
    // if enabling logging, then force is current state to be logged.
    logChanges = enable;
    write( currentState, currentState );
  }
  else {
    // if disabling logging, then force the signal to be logged as false.
    write( false, false );
    logChanges = enable;
  }
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
  writeLogEntry( F("Error: expected zone X valve, but never detected") );
  info->waitForValveTimer = -1;
}
void zoneCallExpired( void* param ) {
  struct zoneCallInfo* info( reinterpret_cast<struct zoneCallInfo*>( param ) );
  _println3( F("  + zone X call expired") );
  info->zoneCallOut->write( false );
  info->holdTimer = -1;
}
void setupZoneCallOut( struct zoneCallInfo& info ) {
  _println3( F("  + triggering zone X") );
  info.zoneCallOut->write( true );
  if( info.waitForValveTimer >= 0 ) // should never happen...
    timer.deleteTimer( info.waitForValveTimer );
  info.waitForValveTimer = timer.setTimeout( callOutputToValveTimeoutMs, zoneCallWaitForValveTimout, &info, false );
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
    info.holdTimer = timer.setTimeout( info.holdDurationMs, zoneCallExpired, &info, false );
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

void activeChanged( bool active, bool activePurge, bool antifreeze ) {
  if( !antifreeze )
    cancelZoneCallOut( s_zoneCallOut[1] );
  if( !activePurge )
    cancelZoneCallOut( s_zoneCallOut[0] );
}
void readSignals() {
#if DIAG_STANDALONE
  uint16_t ioExpInputs = s_ioExp;
  uint16_t digitalIoInputs = s_digitalIo;
#else
  // If the IO Expander is not present, the reads come back as 1's. Which is ok.
  uint16_t ioExpInputs = ioExp0.digitalRead();
  uint16_t digitalIoInputs = (((uint16_t) PINB) << 8) | PIND;
#endif

  bool anyInputsChanged = false;
  for( byte i = 0; i < s_NumSignals; ++i ) {
    BoolSignal* sig( (BoolSignal*) pgm_read_word_near( &s_Signals[i] ) );
    if( sig->source != eNone && sig->type != OUTPUT ) {
      bool stateNow = 
        sig->source == eIoExpander ? ((ioExpInputs & (1 << sig->pin)) != 0) ^ sig->activeLow :
        ((digitalIoInputs & (1 << sig->pin)) != 0) ^ sig->activeLow;
      if( stateNow != sig->currentState ) {
        sig->currentState = stateNow;
        if( sig->logChanges )
          writeLogEntry( *sig );
        sig->priorValid = true;
        sig->priorTransition = s_currentCycleS;
        anyInputsChanged = true;
      }
    }
#if !DIAG_STANDALONE
    else if( sig->source == eIoExpander ) {
      bool stateNow = ((ioExpInputs & (1 << sig->pin)) != 0) ^ sig->activeLow;
      if( stateNow != sig->currentState ) {
        writeLogEntry( F("Output signal found in unexpected state") );            
      }
    }
#endif
  }
#if ReadTemperature
  temperatureC = readTemp();
  writeLogEntry( "tempC", temperatureC );
#endif

  checkForValveOpen( s_zoneCallOut[0] );
  checkForValveOpen( s_zoneCallOut[1] );
}

bool timeSinceZone1CallValid = false;
uint16_t timeSinceZone1Call = 0;

void determineActions() {
  // Calculate a new state for our outputs (if any)
  uint16_t timeSinceZone2 = zone2Call.timeSincePrior();
  // For zone 2, we consider an invalid interval to be greater than zone2CallIntervalMinS and less than
  // zone2CallIntervalMaxS - so we will basically trigger it the first time the boiler is hot, but don't
  // consider it so critical to fire up the burner.
  zone2NeedsCall.write( !zone2Call.currentState &&
      (timeSinceZone2 == -1 || 
        timeSinceZone2 > zone2CallIntervalMinS) );
  zone2ReallyNeedsCall.write( !zone2Call.currentState &&
      timeSinceZone2 != -1 && 
      timeSinceZone2 > zone2CallIntervalMaxS );
  byte numZonesCalling = 0;
  byte numValvesOpen = 0;
  for( byte i = 0; i < s_NumCallSignals; ++i ) {
    const BoolSignal* callSig( (const BoolSignal*) pgm_read_word_near( &s_CallSignals[i] ) );
    if( callSig->currentState )
      ++numZonesCalling;
    const BoolSignal* valveSig( (const BoolSignal*) pgm_read_word_near( &s_ValveSignals[i] ) );
    if( valveSig->currentState )
      ++numValvesOpen;
  }
  anyZonesCalling.write( numZonesCalling > 0 );
  // We consider zone1 to be 'active' if it has called in the last 6 hours.
  zone1CallTherm.write( zone1Call.currentState && !zone1CallOut.currentState );
  uint16_t timeSinceZone1 = zone1CallTherm.timeSincePrior();
  zone1Active.write( zone1CallTherm.currentState || (timeSinceZone1 != -1 && timeSinceZone1 < HOURStoS( 6 )) );

  bool anyOutputsChanged = false;

  if( activeAF.currentState ) {
    if( !s_zoneCallOut[1].active() && 
        ((anyZonesCalling.currentState && zone2NeedsCall.currentState) || zone2ReallyNeedsCall.currentState) )
    {
      // make the call. Then wait up to 10 minutes for the valve to be open (should be plenty even
      // if the furnace has to heat up). The expectation is that the function will never get called.
      setupZoneCallOut( s_zoneCallOut[1] );
      anyOutputsChanged = true;
    }
  }
  
  if( activePrg.currentState ) {
    // If zone2 (upstairs) just turned off and no (other) zones are calling, then the
    // furnace is going to keep dumping heat to the upstairs. Don't like that - the zone
    // absorbes heat slowly so it takes a long time and the zone is small so the amount
    // of heat in the furnace results in a big temperature swing.
    //
    // Ideally, I would just open additional valves, but I have no way to do that. So we simply 
    // signal a short call on zone1 so the furnace dumps the heat there.
    // Considered the same thing for the hot water zone. But it takes a pretty short time to purge - 
    // about 3 minutes, so seems best to let it do so.
    // Considered the same thing for zone 4. But it tends to call frequently and run for a short time,
    // which results in zone1 being heated more than it should.
    if( !s_zoneCallOut[0].active() &&       // if we haven't already called for zone1
        zone1Active.currentState )          // if zone 1 has called 'recently'
    {
      if( !anyZonesCalling.currentState &&  // nobody is calling now
          circulator.currentState &&        // and the circulator is still on
          numValvesOpen == 1 &&             // and there is a single valve open
          (zone2Valve.currentState || zone3Valve.currentState) ) 
//          !zone1Valve.currentState )        // and it isn't zone 1 (our preferred 'purge' zone)
      {
        setupZoneCallOut( s_zoneCallOut[0] );
        anyOutputsChanged = true;
      }
    }
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
    _print3( F("  - numZonesCalling ") ); _print3( numZonesCalling ); _print3( F(", valves ") ); _println3( numValvesOpen );
    _print3( F("  - timeSinceZone1 ") ); _print3( timeSinceZone1 ); _print3( F(", zone1Active ") ); _println3( zone1Active );
  }
#endif
}


void checkIoExpanderState() {

  ioExp0.wordWrite( DEFVALA, 0xa5a5 );
  ioExp0.wordWrite( DEFVALA, 0x0000 );

  if( ioExpPresent ) {
    // checking for errors only makes sense if the IO Expander is actually present.
    unsigned int errorCountWr = ioExp0.errorCountWr();
    unsigned int errorCountRd = ioExp0.errorCountRd();
    if( errorCountWr > 0 || errorCountRd > 0 ) {
      if( logfile ) {      
        logfile.print( getCurrentTime() );
        logfile.print( F(",--:--:--,") );
        logfile.print( F("Errors detected in ioExp communication,") );
        logfile.print( errorCountWr );
        logfile.print( F(",") );
        logfile.println( errorCountRd );
      }
      _print3( getCurrentTime() );
      _print3( F(",--:--:--,") );
      _print3( F("Errors detected in ioExp communication,") );
      _print3( errorCountWr );
      _print3( F(",") );
      _println3( errorCountRd );
    }

    bool ioExpStateValid = ioExp0.verifyAll();
    if( !ioExpStateValid ) {
      writeLogEntry( F("ioExp state is invalid, attempting recovery") );
      ioExp0.updateAll();
      ioExpStateValid = ioExp0.verifyAll();
      if( !ioExpStateValid )
        writeLogEntry( F("ioExp state is still invalid") );
    }
  }
}

// This is the function that performs the bulk of our processing, reading inputs and 
// determining a new state for the outputs. It gets called from the main loop each
// time an IO Expander interrupt occurs.
void processMonitorTimer() {

#if !USE_SLEEP
  updateCurrentTime();
#endif

  // _println3( digitalRead( pushButton.pin ) == HIGH ? F("HIGH") : F("LOW") );
  // pinMode( pushButton.pin, INPUT_PULLUP );

  if( active.currentState ) {
    // Go through and read all of our inputs, recording changes in the log file.
    readSignals();
    
    // Calculate a new state for our outputs (if any)
    determineActions();
  }

  checkIoExpanderState();
}


