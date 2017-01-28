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
  uint16_t delay;
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
  
  // Zone1 is now active, but it is just in monitoring mode, so same behavior.
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
  
  // Activate purge. Since zone1 is 'active', these should result in zone1 being called to purge.
  {               800, 0x0004, 0 },           // activate purge
  {               200, 0x0004, 0 },
  { SECONDStoMS( 5  ), 0x0000, ZHWC },    // HwCall
  { SECONDStoMS( 5  ), 0x0000, ZHWV | CIRC },
  { SECONDStoMS( 10 ), 0x0000, ZHWC },
  { SECONDStoMS(  5 ), 0x0000, ZHWV | CIRC  },
//  { SECONDStoMS( 1 ),  0x0000, Z1C | Z1V | ZHWV }, // not purging zoneHw
//  { SECONDStoMS( 1 ),  0x0000, Z1C },
//  { SECONDStoMS( 9 ),  0x0000, Z1V | CIRC },

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

  {              1000, 0x0004, 0 },           // deactivate Anti-freeze, taking us back to monitor.
  {               200, 0x0004, 0 },
  {              1000, 0x0004, 0 },
  {              1200, 0x0004, INACTIVE },    // deactivate
  {  MINUTEStoS( 30 ), 0x8000, 0 }
};
byte testScriptIndex = 0;

void processTestScript(void*) {
  uint16_t delay = 0;
  bool seconds = false;
  do {
    uint16_t toggle = pgm_read_word_near( &testScript[testScriptIndex].digital );
    s_digitalIo = s_digitalIo ^ toggle;
    if( (toggle & 0x8000) != 0 )
      seconds = true;
    toggle = pgm_read_word_near( &testScript[testScriptIndex].ioexp );
    s_ioExp = s_ioExp ^ toggle;
    if( (s_digitalIo & 0x0004) == 0 )
      s_pushButtonPressed = true;
      
    ++testScriptIndex;
    if( testScriptIndex >= sizeof( testScript ) / sizeof( testScript[0] ) )
      testScriptIndex = 0;
    delay = pgm_read_word_near( &testScript[testScriptIndex].delay );
    // delay |= (unsigned long) pgm_read_word_near( ((uint16_t*) &testScript[testScriptIndex].delay) + 1 ) << 16;
  } while( delay == 0 );
  timer.setTimeout( delay, processTestScript, seconds );
}
#endif

