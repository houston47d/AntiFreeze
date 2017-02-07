
char* generateUint( char* buffer, uint16_t num, uint8_t digits ) {
  for( uint8_t i = digits; i > 0; --i ) {
    buffer[i-1] = '0' + (num % 10);
    num = num / 10;
  }
  return( buffer+digits );
}

void generateDateTimeRTC( char* buffer ) {
#if USE_RTC
  DateTime now = rtc.now();
  uint8_t i = 0;
  buffer = generateUint( buffer, now.year(), 4 );
  *buffer++ = '/';
  buffer = generateUint( buffer, now.month(), 2 );
  *buffer++ = '/';
  buffer = generateUint( buffer, now.day(), 2 );
  *buffer++ = ' ';
  buffer = generateUint( buffer, now.hour(), 2 );
  *buffer++ = ':';
  buffer = generateUint( buffer, now.minute(), 2 );
  *buffer++ = ':';
  buffer = generateUint( buffer, now.second(), 2 );
  *buffer++ = 0;
#endif
}

void generateHMS( char* buffer, unsigned long seconds ) {
  byte hour = 0;
  if( seconds >= 3600 ) {
    hour = (byte) (seconds / 3600);
    hour = hour % 100;
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
    logfile.print( getCurrentTime() );
    logfile.print( F(",") );
    logfile.println( message );
  }
  _print3( getCurrentTime() );
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
    logfile.print( getCurrentTime() );
    logfile.print( F(",") );
    logfile.print( durationHMS );
    logfile.print( F(",") );
    logfile.print( reinterpret_cast<const __FlashStringHelper *>( signal.name ) );
    logfile.print( F(",") );
    logfile.println( signal.currentState ? 1 : 0 );
  }
  _print3( getCurrentTime() );
  _print3( F(",") );
  _print3( durationHMS );
  _print3( F(",") );
  _print3( reinterpret_cast<const __FlashStringHelper *>( signal.name ) );
  _print3( F(",") );
  _println3( signal.currentState ? 1 : 0 );
}

void writeLogEntry( const char* signal, float value ) {
  if( logfile ) {      
    logfile.print( getCurrentTime() );
    logfile.print( F(",,") ); // no duration.
    logfile.print( signal );
    logfile.print( F(",") );
    logfile.println( value );
  }
  _print3( getCurrentTime() );
  _print3( F(",,") ); // no duration.
  _print3( signal );
  _print3( F(",") );
  _println3( value );
}

void readSignalAndLogChange( struct BoolSignal& signal ) {
  bool stateNow = ((digitalRead( signal.pin ) == HIGH) ^ signal.activeLow);
  signal.write( stateNow );
}

float readTemp() {
  // read the voltage, and scale from 0..1023 -> 0.0..5.0.
  float voltage = (analogRead( tempPin ) * 5.0 / 1024.0);
  // 500 mV == 0 degrees C, and 10 mV / degree C.
  float tempC = (voltage - .5) * 100.0;
  return tempC;
}


