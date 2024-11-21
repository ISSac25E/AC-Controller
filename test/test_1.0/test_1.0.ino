#include "Arduino.h"
#include "EEPROM.h"
#include "config.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\Tuya WiFi MCU Serial SDK\Tuya_WiFi_MCU_SDK_2.0.0\Tuya_WiFi_MCU_SDK\src\TuyaWifi.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Vacuum V3\released\VacuumV3_1.0.0\Core\SequenceBuild\SequenceBuild_1.0.3.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\LedMacro\LedMacro_1.0.0.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\InputMacro\InputMacro_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinDriver\PinDriver_1.0.1.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinPort\PinPort_1.0.2.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\VarPar\VarPar_1.0.1.h"

// generate prototypes for required functions:
unsigned char dp_process(unsigned char, const unsigned char[], unsigned short);
void dp_update_all(void);

////////////////////////////////////
//////////// GPIO Setup ////////////
////////////////////////////////////

// Setup Pairing Button:
PinDriver ButtonInput(AC_ButtonPin);
InputMacro ButtonMacro(HIGH);

// Setup Relay Output Pin:
PinPort RelayOutput(AC_RelayPin);

#ifdef AC_DebugLedPin
// define debug led pin:
PinPort DebugLed(AC_DebugLedPin);
#endif

//////////////////////////////////////////////////////
//////////// Output Macros Setup and Misc ////////////
//////////////////////////////////////////////////////
// setup sequence builder for led output:
SequenceBuild LedBuild;
#ifdef AC_DebugLedPin
// Set up Debug Led Build
SequenceBuild DebugLedBuild;
#endif

#ifdef AC_DebugLedPin
// setup Three macros for Led, Button Output and debug Led:
LedMacro _Macro[3];
LedMacroManager Macro(_Macro, 3);
#else
// setup two macros for Led and Button Output:
LedMacro _Macro[2];
LedMacroManager Macro(_Macro, 2);
#endif

// pwm value of Led:
uint8_t LedVal = 0;

#ifdef AC_DebugLedPin
// define debug led val:
uint8_t DebugLedVal = 0;
#endif

/////////////////////////////////////////
////////////// Relay Setup //////////////
/////////////////////////////////////////

uint32_t IntervalTimer = 0;
uint32_t Interval = 0;
bool RelayState = false;

/////////////////////////////////////////
//////////// TUYA DPID Setup ////////////
/////////////////////////////////////////
/*
  TY_AcControl:
    0 = On
    1 = Off
    2 = Auto
*/
Par_uint8_t TY_AcControl = 0;

Par_uint16_t TY_OnInterval = 0;  // range 0 - (24 * 60)
Par_uint16_t TY_OffInterval = 0; // range 0 - (24 * 60)
Par_uint16_t TY_Interval = 0;    // range 0 - (24 * 60). Current Interval Count

// Update Timers for DPID's
uint32_t dpid_updateAll_timer = millis();
uint32_t dpid_update_timer = millis();

////////////////////////////////////
//////////// TUYA Setup ////////////
////////////////////////////////////
// Tuya Module communication:
TuyaWifi tuya_module((HardwareSerial *)&Serial);

// setup DPID array to pass onto 'TuyaWifi':
unsigned char dpid_array[4][2] = {
    {IntervalOn_DPID, DP_TYPE_VALUE},
    {IntervalOff_DPID, DP_TYPE_VALUE},
    {Control_DPID, DP_TYPE_ENUM},
    {IntervalCount_DPID, DP_TYPE_VALUE}};

// setup module reset pin:
PinPort moduleResetPin(TUYA_ModuleResetPin);

void setup()
{
  // initialize hardware Serial to 9600 for communication with TUYA Module:
  Serial.begin(9600);

  //////// Pin Setup ////////
  {
    // set module reset pin to Input so user can reset Module too without damaging Arduino:
    moduleResetPin.pinMode(INPUT);
    moduleResetPin.digitalWrite(LOW);

    // LED pin setup:
    pinMode(AC_LedPin, OUTPUT);
    analogWrite(AC_LedPin, LedVal);

#ifdef AC_DebugLedPin
    // setup debug led:
    DebugLed.digitalWrite(LOW);
    DebugLed.pinMode(OUTPUT);
#endif
  }

  // Setup Timer Interrupt Interval for LED (100Hz):
  {
    cli(); // stop all interrupts

    // reset Timer 1 Registers:
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 0; // initialize timer  counter value to 0

    // set compare match A register for 100 Hz increments:
    OCR1A = 19999; // = 16000000 / (8(pre-scaler) * 100) - 1 (must be <65536)

    TCCR1B |= (1 << WGM12); // turn on CTC(Clear-Timer(counter)-On-Compare) mode

    // Set CS12, CS11 and CS10 bits for 8 prescaler:
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);

    // enable timer compare interrupt on control mask register:
    TIMSK1 |= (1 << OCIE1A);
    sei(); // allow all interrupts
  }

  //////// set up TUYA Module: ////////
  {
    unsigned char pid[] = {TUYA_PID};
    unsigned char mcu_ver[] = {TUYA_MCU_Version};

    // init TUYA PID and MCU Version:
    tuya_module.init(pid, mcu_ver);

    // Set all TUYA DPID's:
    tuya_module.set_dp_cmd_total(dpid_array, 4);

    // set callbacks:
    tuya_module.dp_process_func_register(dp_process);       // function to proccess incoming commands
    tuya_module.dp_update_all_func_register(dp_update_all); // function to update all DPID states at once
  }

  // load EEPROM:
  EEPROM_load();

  // set initial LED sequence:
  LedBuild.setPrioritySequence(init_led, 0, true);
}

void loop()
{
  // run tuya uart service:
  tuya_module.uart_service();

  // run all handles:
  buttonHandle();
  ledHandle();
  moduleErrorHandle();
  checkUpdateHandle();
  relayHandle();
  EEPROM_handle();

  // check update all timer:
  if (millis() - dpid_updateAll_timer >= DPID_UpdateAllInterval && DPID_UpdateAllInterval)
    dp_update_all();
}

/*
  buttonHandle():
    handle WiFi Mode button, sets TUYA Module to Pairing mode
*/
inline void buttonHandle()
{
  /*
    WiFi_Config:
      0 = don't set WiFi
      1 = Smart_Mode
      2 = AP_Mode
  */
  static uint8_t WiFi_Config = 0;
  static uint32_t WiFi_setTimer = 0;
  const uint16_t WiFi_setInterval = 20000;

  // check for pin state change:
  if (ButtonMacro(ButtonInput))
  {
    if (ButtonMacro) // button released
    {
      WiFi_Config = 0; // stop setting up WiFi
    }
    else // button pressed
    {
      WiFi_setTimer = (millis() - WiFi_setInterval); // reset wifi send timer so it sends immediately next timer
    }
  }

  // check if button has been pushed down for more than 1000ms
  if (!ButtonMacro && !ButtonMacro.triggered() && ButtonMacro.interval() > 1000)
  {
    ButtonMacro.trigger();

    // check if the module is already in Smart Config State:
    if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
      WiFi_Config = 2; // set to AP Mode
    else
      WiFi_Config = 1; // set to smart config
  }

  // set up wifi according to 'WiFi_Config'
  if (WiFi_Config)
  {
    // check if WiFi already done with setup:
    if (WiFi_Config == 1 && tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
      WiFi_Config = 0; // no need to set up WiFi pair anymore
    else if (WiFi_Config == 2 && tuya_module.mcu_get_wifi_work_state() == AP_STATE)
      WiFi_Config = 0; // no need to set up WiFi pair anymore,
    else
    {
      if (millis() - WiFi_setTimer >= WiFi_setInterval)
      {
        WiFi_setTimer = millis(); // reset timer

        if (WiFi_Config == 1) // set to smart config mode
          tuya_module.mcu_set_wifi_mode(SMART_CONFIG);
        else if (WiFi_Config == 2) // set to smart config mode
          tuya_module.mcu_set_wifi_mode(AP_CONFIG);
      }
    }
  }
}

/*
  ledHandle():
    Handle LED macro states
*/
inline void ledHandle()
{
  // determine led sequence:
  if (tuya_module.mcu_get_wifi_work_state() == AP_STATE)
    LedBuild.setSequence(ap_mode_led, 0, true);
  else if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
    LedBuild.setSequence(smart_mode_led, 0, true);
  else if (tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR || tuya_module.mcu_get_wifi_work_state() == WIFI_STATE_UNKNOWN)
    LedBuild.setSequence(error_led, 0, true);
  else
    LedBuild.setSequence(idle_led, 0, true);

#ifdef AC_DebugLedPin
  // Run Debug Led:
  if (tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR)
    DebugLedBuild.setSequence(error_debugLed, 0, true);
  else
    DebugLedBuild.setSequence(idle_debugLed, 0, true);
#endif
}

/*
  moduleErrorHandle():
    handle resetting module when error occurs
*/
inline void moduleErrorHandle()
{
  // keep track of the state for the reset pin. false/LOW = reset, true/HIGH = idle
  static bool resetPinState = true;
  static Par_bool moduleErrorVar = false; // << par var for Error state change
  static uint32_t moduleResetTimer = 0;
  const uint16_t moduleResetInterval = 10000;
  const uint16_t moduleResetHeadStart = 4000;

  // run par var for Error Status:
  moduleErrorVar = (bool)(tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR);

  // check for state change:
  if (moduleErrorVar.change())
  {
    if (moduleErrorVar) // Error has occurred
    {
      // Give timer head start. We dont need Module to be reset immediately
      moduleResetTimer = (millis() - (moduleResetInterval - moduleResetHeadStart));
    }
    else // Error has stopped
    {
      // set reset pin to input:
      moduleResetPin.pinMode(INPUT);
      moduleResetPin.digitalWrite(LOW);

      // set pin state:
      resetPinState = true;
    }
  }

  if (moduleErrorVar)
  {
    if (resetPinState)
    {
      if (millis() - moduleResetTimer >= moduleResetInterval)
      {
        moduleResetTimer = millis(); // reset timer

        // set pin to LOW and OUTPUT:
        moduleResetPin.digitalWrite(LOW);
        moduleResetPin.pinMode(OUTPUT);

        // set pin state:
        resetPinState = false;
      }
    }
    else
    {
      if (millis() - moduleResetTimer >= 300)
      {
        moduleResetTimer = millis(); // reset timer

        // set pin to LOW and INPUT:
        moduleResetPin.pinMode(INPUT);
        moduleResetPin.digitalWrite(LOW);

        // set pin state:
        resetPinState = true;
      }
    }
  }
}

/*
  checkUpdateHandle():
    check Tuya Values and update if needed
*/
inline void checkUpdateHandle()
{
  // only update at an interval:
  if (millis() - dpid_update_timer >= DPID_UpdateInterval && DPID_UpdateInterval)
  {
    if (TY_AcControl.change())
    {
      tuya_module.mcu_dp_update(Control_DPID, TY_AcControl, 1);
      dpid_update_timer = millis(); // << reset update timer
    }
    if (TY_OnInterval.change())
    {
      tuya_module.mcu_dp_update(Control_DPID, TY_OnInterval, 1);
      dpid_update_timer = millis(); // << reset update timer
    }
    if (TY_OffInterval.change())
    {
      tuya_module.mcu_dp_update(Control_DPID, TY_OffInterval, 1);
      dpid_update_timer = millis(); // << reset update timer
    }

    if (TY_Interval.change())
    {
      tuya_module.mcu_dp_update(Control_DPID, TY_Interval, 1);
      dpid_update_timer = millis(); // << reset update timer
    }
  }
}

inline void relayHandle()
{
  if (Interval)
  {
    if (millis() - IntervalTimer >= Interval)
    {
      // interval Reached!
      if (TY_AcControl == 2 /*Auto*/)
      {
        if (RelayState)
        {
          IntervalTimer = millis();
          Interval = (TY_OffInterval * 60 * 1000);
        }
        else
        {
          IntervalTimer = millis();
          Interval = (TY_OnInterval * 60 * 1000);
        }
        RelayState = !RelayState;
      }
      else
      {
        Interval = 0;
        RelayState = !RelayState;

        if (TY_AcControl == 0)
          TY_AcControl = 1;
        else
          TY_AcControl = 0;
      }
    }
  }
  else
  {
    if (TY_AcControl == 2)
    {
      IntervalTimer = millis();
      if (RelayState)
        Interval = (TY_OnInterval * 60 * 1000);
      else
        Interval = (TY_OffInterval * 60 * 1000);
    }
  }

  if (TY_AcControl != 2)
  {
    if (TY_AcControl == 0)
      RelayState = true;
    else
      RelayState = false;
  }

  // update interval count for Tuya:
  if (Interval)
  {
    if (Interval >= millis() - IntervalTimer)
    {
      TY_Interval = (Interval - (millis() - IntervalTimer)) / 60 / 1000;
    }
    else
    {
      TY_Interval = 0;
    }
  }
  else
  {
    TY_Interval = 0;
  }
}

/*
  EEPROM_load():
    run once at the beginning
    runs check sum. If checksum fails, sets default values*/
void EEPROM_load()
{
  uint8_t ac_mode;
  uint16_t intervals[2];
  uint8_t checkSum;

  ac_mode = EEPROM.read(0);

  intervals[0] = EEPROM.read(1);
  intervals[0] |= EEPROM.read(2) << 8;

  intervals[1] = EEPROM.read(3);
  intervals[1] |= EEPROM.read(4) << 8;

  checkSum = 5;

  // check checksum:
  uint8_t calc = ac_mode + intervals[0] + intervals[1] + B01010101;

  // checkSum check Passed
  if (calc == checkSum)
  {
    // checkSum check Passed
    TY_AcControl = ac_mode % 3;

    TY_OnInterval = intervals[0];
    TY_OffInterval = intervals[1];
  }
  else
  {
    // set Default Values
    TY_AcControl = 1; // "off"
    TY_OnInterval = 0;
    TY_OffInterval = 0;
  }
}

void EEPROM_handle()
{
  static uint32_t eepromWriteTimer = millis();
  const uint16_t writeInterval = 5000;

  static Par_uint8_t ac_mode;
  static Par_uint16_t intervals[2];

  static bool boot = false;
  static bool write = false;

  if (!boot)
  {
    boot = true;
    // load values:
    ac_mode = EEPROM.read(0);

    intervals[0] = (EEPROM.read(1) | (EEPROM.read(2) << 8));

    intervals[1] = (EEPROM.read(3) | (EEPROM.read(4) << 8));
  }

  if (ac_mode.change() ||
      intervals[0].change() ||
      intervals[1].change())
  {
    write = true;
    eepromWriteTimer = millis(); // << reset timer
  }

  if (write && millis() - eepromWriteTimer >= writeInterval)
  {
    uint8_t checksum = ac_mode + intervals[0] + intervals[1] + B01010101;

    EEPROM.update(0, ac_mode);

    EEPROM.update(1, intervals[0]);
    EEPROM.update(2, intervals[0] >> 8);
    EEPROM.update(3, intervals[1]);
    EEPROM.update(4, intervals[1] >> 8);

    EEPROM.update(5, checksum);

    write = false;
  }
}

unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
#ifdef AC_DebugLedPin
  DebugLedBuild.setPrioritySequence(dpProccess_debugLed, 0, true); // indicate dp proccess
#endif
  if (dpid == Control_DPID)
  {
    TY_AcControl = tuya_module.mcu_get_dp_download_data(dpid, value, length);
    tuya_module.mcu_dp_update(Control_DPID, TY_AcControl, 1);
  }
  if (dpid == IntervalOn_DPID)
  {
    TY_OnInterval = tuya_module.mcu_get_dp_download_data(dpid, value, length);
    tuya_module.mcu_dp_update(Control_DPID, TY_OnInterval, 1);
  }
  if (dpid == IntervalOff_DPID)
  {
    TY_OffInterval = tuya_module.mcu_get_dp_download_data(dpid, value, length);
    tuya_module.mcu_dp_update(Control_DPID, TY_OffInterval, 1);
  }
  if (dpid == IntervalCount_DPID)
  {
    TY_Interval = tuya_module.mcu_get_dp_download_data(dpid, value, length);
    tuya_module.mcu_dp_update(Control_DPID, TY_Interval, 1);
    if (TY_Interval > 0)
    {
      IntervalTimer = millis();
      Interval = TY_Interval * 60 * 1000;
    }
    else
    {
      Interval = 0;
    }
  }
  // return:
  return TY_SUCCESS;
}

void dp_update_all()
{
  // trigger change interrupt on values and update Tuya Module:
  TY_AcControl.change();
  tuya_module.mcu_dp_update(Control_DPID, TY_AcControl, 1);
  TY_OnInterval.change();
  tuya_module.mcu_dp_update(Control_DPID, TY_OnInterval, 1);
  TY_OffInterval.change();
  tuya_module.mcu_dp_update(Control_DPID, TY_OffInterval, 1);
  TY_Interval.change();
  tuya_module.mcu_dp_update(Control_DPID, TY_Interval, 1);

  // Reset all timers:
  dpid_updateAll_timer = millis();
  dpid_update_timer = millis();
}

/*
  ISR(TIMER1_COMPA_vect):
    Timer 1 Interrupt Vector for LED and Button Output
    Runs 100 timers-per-second (100Hz)

    Writes to LED and Button Outputs also
*/
ISR(TIMER1_COMPA_vect)
{
  // run all necessary handles first:
  LedBuild.run();
  Macro.run();
#ifdef AC_DebugLedPin
  DebugLedBuild.run();
#endif

  // Write to LED Pins:
  analogWrite(AC_LedPin, LedVal);
  RelayOutput.digitalWrite(RelayState);
#ifdef AC_DebugLedPin
  // write debug led val:
  if (DebugLedVal)
  {
    DebugLed.digitalWrite(HIGH);
  }
  else
  {
    DebugLed.digitalWrite(LOW);
  }
#endif
}

/////////////////////////////////////////////
/////////////// LED Sequences ///////////////
/////////////////////////////////////////////

// smart mode led sequence, fast pulse:
SB_FUNCT(smart_mode_led, Macro.ready(LedVal))
SB_STEP(Macro.quadEase(LedVal, 0, 10);)
SB_STEP(Macro.quadEase(LedVal, 255, 10);)
SB_STEP(Macro.delay(LedVal, 100);)
SB_STEP(Macro.quadEase(LedVal, 0, 10);)
SB_STEP(Macro.delay(LedVal, 100);)
SB_STEP(LedBuild.loop(1);)
SB_END

// ap mode led sequence, slow pulse:
SB_FUNCT(ap_mode_led, Macro.ready(LedVal))
SB_STEP(Macro.quadEase(LedVal, 0, 10);)
SB_STEP(Macro.quadEase(LedVal, 255, 10);)
SB_STEP(Macro.delay(LedVal, 1400);)
SB_STEP(Macro.quadEase(LedVal, 0, 10);)
SB_STEP(Macro.delay(LedVal, 1400);)
SB_STEP(LedBuild.loop(1);)
SB_END

// idle led, steady with very soft glow:
SB_FUNCT(idle_led, Macro.ready(LedVal))
SB_STEP(Macro.quadEase(LedVal, 255, 180);)
SB_STEP(Macro.delay(LedVal, 400);)
SB_STEP(Macro.quadEase(LedVal, 100, 180);)
SB_STEP(Macro.delay(LedVal, 400);)
SB_STEP(LedBuild.loop(0);)
SB_END

// error led, rapid flashing:
SB_FUNCT(error_led, Macro.ready(LedVal))
SB_STEP(Macro.set(LedVal, 255, 50);)
SB_STEP(Macro.set(LedVal, 0, 50);)
SB_STEP(Macro.set(LedVal, 255, 50);)
SB_STEP(Macro.set(LedVal, 0, 50);)
SB_STEP(Macro.set(LedVal, 255, 50);)
SB_STEP(Macro.set(LedVal, 0, 1000);)
SB_STEP(LedBuild.loop(0);)
SB_END

// init led, slow fade in and small delay to allow full initialization of Arduino:
SB_FUNCT(init_led, Macro.ready(LedVal))
SB_STEP(Macro.quadEase(LedVal, 255, 120);)
SB_STEP(Macro.delay(LedVal, 1000);)
SB_END

#ifdef AC_DebugLedPin
//////// Debug Led Sequences ////////
// simply keeps debug led on:
SB_FUNCT(idle_debugLed, Macro.ready(DebugLedVal))
SB_STEP(Macro.set(DebugLedVal, 1, 0);)
SB_END

// debug led error. Rapid Triple Flash
SB_FUNCT(error_debugLed, Macro.ready(DebugLedVal))
SB_STEP(Macro.set(DebugLedVal, 1, 50);)
SB_STEP(Macro.set(DebugLedVal, 0, 50);)
SB_STEP(Macro.set(DebugLedVal, 1, 50);)
SB_STEP(Macro.set(DebugLedVal, 0, 50);)
SB_STEP(Macro.set(DebugLedVal, 1, 50);)
SB_STEP(Macro.set(DebugLedVal, 0, 1000);)
SB_STEP(DebugLedBuild.loop(0);)
SB_END

// Quick flash to indicate dp Proccess:
SB_FUNCT(dpProccess_debugLed, Macro.ready(DebugLedVal))
SB_STEP(Macro.set(DebugLedVal, 0, 100);)
SB_STEP(Macro.set(DebugLedVal, 1, 50);)
SB_END
#endif