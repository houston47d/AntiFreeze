
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
    logfile.println( message );
  }
  _print3( s_currentCycleHMS );
  _print3( F(",") );
  _println2( message );
}
  
void writeLogEntry( struct BoolSignal& signal ) {
  char durationHMS[10];
  if( signal.priorValid )
    generateHMS( durationHMS, (uint16_t) (s_currentCycleS - signal.priorTransition) );
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
  signal.write( stateNow );
}

#if USE_RTC
void writeLogEntryRtc() {
  DateTime now = rtc.now();
  if( logfile ) {
    logfile.print(now.year(), DEC);
    logfile.print('/');
    logfile.print(now.month(), DEC);
    logfile.print('/');
    logfile.print(now.day(), DEC);
    logfile.print(" ");
    logfile.print(now.hour(), DEC);
    logfile.print(':');
    logfile.print(now.minute(), DEC);
    logfile.print(':');
    logfile.print(now.second(), DEC);
    logfile.println();
  }  
  _print3(now.year());
  _print3('/');
  _print3(now.month());
  _print3('/');
  _print3(now.day());
  _print3(" ");
  _print3(now.hour());
  _print3(':');
  _print3(now.minute());
  _print3(':');
  _print3(now.second());
  _println3();
}
#endif

float readTemp() {
  // read the voltage, and scale from 0..1023 -> 0.0..5.0.
  float voltage = (analogRead( tempPin ) * 5.0 / 1024.0);
  // 500 mV == 0 degrees C, and 10 mV / degree C.
  float tempC = (voltage - .5) * 100.0;
  return tempC;
}

