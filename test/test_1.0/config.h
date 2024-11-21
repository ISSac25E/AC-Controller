// Arduino User Config

/*
  You may enter Pairing Mode by by holding down the button located on the top of the box until the LED starts pulsing quickly
  You may have to hold the button for up to 30 seconds

  To Switch between AP and SMART pairing mode, release the pairing button and Press and hold again until the LED begins to flash at a different pattern
*/

/////////////////////////////////
/////////// TUYA Setup //////////
/////////////////////////////////

#define TUYA_PID "6o1dxveugdesqpdd" // << This can be found in your TUYA account
#define TUYA_MCU_Version "1.0.0"    // << This field does not really matter. It can be a useful tool to keep track of revisions

#define TUYA_ModuleResetPin 7 // << configure pin number for Arduino to reset the TUYA module in the case of mis-communication or module error

#define DPID_UpdateInterval 800 // << set individual DPID update interval in milliseconds (this will only run when necessary)

#define DPID_UpdateAllInterval 10000 // << set update ALL DPID's interval in milliseconds. Keep this as long as possible to prevent app crashing and stuttering

//////////////////////////////////
//////////// Pin Setup ///////////
//////////////////////////////////

#define AC_LedPin 6 // << Status Pin For Tuya, AnalogWrite
#define AC_ButtonPin 3 // << define button pin to pair Tuya

#define AC_RelayPin 11  // << relay Pin DigitalWrite

#define AC_DebugLedPin 13 // << define debug led pin. If debug led is not needed, simply comment out this line. DigitalWrite Only, No Analog

/////////////////////////////////
/////////// DPID Setup //////////
/////////////////////////////////

#define IntervalOn_DPID 101  // << Control DPID, Value Type. Used to control the On interval of Relay in Minutes. Max 1440min (24Hrs)
#define IntervalOff_DPID 102  // << Control DPID, Value Type. Used to control the Off interval of Relay in Minutes. Max 1440min (24Hrs)
#define Control_DPID 103 // << Control DPID, Enum Type. Used to control AC Interval State. On(0), Off(1), Auto(2)
#define IntervalCount_DPID 104 // << Interval Count DPID, Value Type. Used to Report time remaining in current interval. This is a control value also. Interval can be increased or decreased as needed.
