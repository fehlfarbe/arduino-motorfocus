#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimerOne.h>
#include "DummySerial.h"

#define BTN_IN 7
#define BTN_OUT 8
#define BTN_STEP 32

#define PIN_DRIVER_ENABLE 4
#define PIN_DRIVER_DIR 3
#define PIN_DRIVER_STEP 5

#define ONE_WIRE_BUS 2

#define PERIOD_US 2000

// #define USE_DRIVER

//#define DEBUG
#ifdef DEBUG
SoftwareSerial debugSerial(9, 10);
#else
DummySerial debugSerial;
#endif

// initialize the stepper library
#ifdef USE_DRIVER
AccelStepper stepper(AccelStepper::DRIVER, PIN_DRIVER_STEP, PIN_DRIVER_DIR);
#else
AccelStepper stepper(AccelStepper::FULL4WIRE, 6, 4, 5, 3, false);
#endif

// temperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// multiplier of SPEEDMUX, currently max speed is 480.
int speedFactor = 16;
int speedFactorRaw = 4;
int speedMult = 30;

float t_coeff = 0;

// button
unsigned long currentPosition = 0;
long targetPosition = 0;
unsigned long lastSavedPosition = 0;
long millisLastMove = 0;
const long millisDisableDelay = 15000;
bool isRunning = false;

// read commands
bool eoc = false;
String line;

// function declarations
long hexstr2long(String line);
static void intHandler();

/*************************************
 * SETUP
*************************************/
void setup()
{
  Serial.begin(9600);
  debugSerial.begin(9600);

  // setup pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // initalize motor
  debugSerial.println("init motor driver...");
  stepper.setMaxSpeed(speedFactor * speedMult);
  stepper.setAcceleration(100);
#ifdef USE_DRIVER
  stepper.setEnablePin(PIN_DRIVER_ENABLE);
#endif
  millisLastMove = millis();

  // read saved position from EEPROM
  //EEPROM.put(0, (long)0);
  EEPROM.get(0, currentPosition);
  // prevent negative values if EEPROM is empty
  currentPosition = max(0, currentPosition);

  stepper.setCurrentPosition(currentPosition);
  lastSavedPosition = currentPosition;
  debugSerial.print("Last position in EEPROM...");
  debugSerial.println(lastSavedPosition);

  // init temperature sensor
  sensors.begin();

  // setup buttons
  pinMode(BTN_IN, INPUT_PULLUP);
  pinMode(BTN_OUT, INPUT_PULLUP);

  // init timer
  Timer1.initialize(PERIOD_US);
  Timer1.attachInterrupt(intHandler);
}

/*************************************
 * LOOP
*************************************/
void loop()
{

  // process the command we got
  if (eoc)
  {
    debugSerial.print("Got new command: ");
    debugSerial.println(line);

    if (line.startsWith("2"))
    {
      debugSerial.println("Got Dual focuser command(?) starting with 2. Send values of first motor");
      // remove first character and parse command
      line = line.substring(1);
    }

    String cmd, param;
    int len = line.length();
    if (len >= 2)
    {
      cmd = line.substring(0, 2);
    }
    if (len > 2)
    {
      param = line.substring(2);
    }

    debugSerial.print(cmd);
    debugSerial.print(":");
    debugSerial.println(param);
    debugSerial.println(line);

    line = "";
    eoc = false;

    // LED backlight value, always return "00"
    if (cmd.equalsIgnoreCase("GB"))
    {
      Serial.print("00#");
    }
    // home the motor, hard-coded, ignore parameters since we only have one motor
    if (cmd.equalsIgnoreCase("PH"))
    {
      stepper.setCurrentPosition(8000);
      stepper.moveTo(0);
      isRunning = true;
    }
    // firmware value, always return "10"
    if (cmd.equalsIgnoreCase("GV"))
    {
      Serial.print("10#");
    }
    // get the current motor position
    if (cmd.equalsIgnoreCase("GP"))
    {
      currentPosition = stepper.currentPosition();
      char tempString[6];
      sprintf(tempString, "%04X", currentPosition);
      Serial.print(tempString);
      Serial.print("#");

      debugSerial.print("current motor position: 0x");
      debugSerial.print(tempString);
      debugSerial.print(" = ");
      debugSerial.println(currentPosition);
    }
    // get the new motor position (target)
    if (cmd.equalsIgnoreCase("GN"))
    {
      //pos = stepper.targetPosition();
      char tempString[6];
      sprintf(tempString, "%04X", targetPosition);
      Serial.print(tempString);
      Serial.print("#");

      debugSerial.print("target motor position: ");
      debugSerial.println(tempString);
    }
    // get the current temperature from DS1820 temperature sensor
    if (cmd.equalsIgnoreCase("GT"))
    {
      sensors.requestTemperatures();
      float temperature = sensors.getTempCByIndex(0);
      debugSerial.print("temperature: ");
      debugSerial.println(temperature);
      if (temperature > 100 || temperature < -50)
      {
        // error
        temperature = 0;
      }
      byte t_int = (byte)temperature << 1;
      t_int += round(temperature - (byte)temperature);
      Serial.print(t_int, HEX);
      Serial.print('#');
    }

    // get the temperature coefficient
    if (cmd.equalsIgnoreCase("GC"))
    {
      //Serial.print("02#");
      Serial.print((byte)t_coeff, HEX);
      Serial.print('#');
    }

    // set the temperature coefficient
    if (cmd.equalsIgnoreCase("SC"))
    {
      //debugSerial.println(param);
      if (param.length() > 4)
      {
        param = param.substring(param.length() - 4);
      }
      debugSerial.println(param);

      if (param.startsWith("F"))
      {
        //debugSerial.println("negative");
        //debugSerial.println(strtol("FFFF", NULL, 16));
        //debugSerial.println(strtol(param.c_str(), NULL, 16));
        t_coeff = ((0xFFFF - strtol(param.c_str(), NULL, 16)) / -2.0f) - 0.5f;
      }
      else
      {
        t_coeff = strtol(param.c_str(), NULL, 16) / 2.0f;
      }
      debugSerial.print("t_coeff: ");
      debugSerial.println(t_coeff);
      //Serial.print("02#");
    }

    // get the current motor speed, only values of 02, 04, 08, 10, 20
    if (cmd.equalsIgnoreCase("GD"))
    {
      char tempString[6];
      sprintf(tempString, "%02X", speedFactorRaw);
      Serial.print(tempString);
      Serial.print("#");

      debugSerial.print("current motor speed: ");
      debugSerial.println(tempString);
    }

    // set speed, only acceptable values are 02, 04, 08, 10, 20
    if (cmd.equalsIgnoreCase("SD"))
    {
      speedFactorRaw = hexstr2long(param);

      // SpeedFactor: smaller value means faster
      speedFactor = 32 / speedFactorRaw;
      stepper.setMaxSpeed(speedFactor * speedMult);
    }

    // whether half-step is enabled or not, always return "00"
    if (cmd.equalsIgnoreCase("GH"))
    {
      Serial.print("00#");
    }

    // motor is moving - 01 if moving, 00 otherwise
    if (cmd.equalsIgnoreCase("GI"))
    {
      if (abs(targetPosition - currentPosition) > 0)
      {
        Serial.print("01#");
      }
      else
      {
        Serial.print("00#");
      }
    }
    // set current motor position
    if (cmd.equalsIgnoreCase("SP"))
    {
      currentPosition = hexstr2long(param);
      stepper.setCurrentPosition(currentPosition);
    }
    // set new motor position
    if (cmd.equalsIgnoreCase("SN"))
    {
      //Serial.println(param);
      debugSerial.print("new target position ");
      debugSerial.print(targetPosition);
      debugSerial.print(" -> ");
      targetPosition = hexstr2long(param);
      debugSerial.println(targetPosition);
      //Serial.println(targetPosition);
      //stepper.moveTo(pos);
    }
    // initiate a move
    if (cmd.equalsIgnoreCase("FG"))
    {
      //isRunning = 1;
      //running = true;
      stepper.enableOutputs();
      stepper.moveTo(targetPosition);
    }
    // stop a move
    if (cmd.equalsIgnoreCase("FQ"))
    {
      //isRunning = 0;
      //stepper.moveTo(stepper.currentPosition());
      //stepper.run();
      //running = false;
      stepper.stop();
    }
  }

  int btn_in = digitalRead(BTN_IN);
  int btn_out = digitalRead(BTN_OUT);

  // move motor if not done
  if (stepper.distanceToGo() != 0)
  {
    isRunning = true;
    millisLastMove = millis();
  }

  // handle manual buttons
  else if (btn_in == LOW || btn_out == LOW)
  {
    stepper.enableOutputs();
    while (btn_in == LOW || btn_out == LOW)
    {
      if (btn_in == LOW)
      {
        stepper.move(BTN_STEP);
      }
      else
      {
        stepper.move(-BTN_STEP);
      }
      stepper.runSpeedToPosition();
      btn_in = digitalRead(BTN_IN);
      btn_out = digitalRead(BTN_OUT);
    }
    stepper.stop();
    stepper.disableOutputs();
    millisLastMove = millis();
    currentPosition = stepper.currentPosition();
  }
  else
  {
    isRunning = false;
    if (millis() - millisLastMove > millisDisableDelay)
    {
      // Save current location in EEPROM
      if (lastSavedPosition != currentPosition)
      {
        EEPROM.put(0, currentPosition);
        lastSavedPosition = currentPosition;
        debugSerial.println("Save last position to EEPROM");
      }
      // set motor to sleep state
      stepper.disableOutputs();
      debugSerial.println("Disabled output pins");
    }
  }

  digitalWrite(LED_BUILTIN, isRunning);

  //delay(20);
}

// read the command until the terminating # character
void serialEvent()
{
  // read the command until the terminating # character
  while (Serial.available() && !eoc)
  {
    char c = Serial.read();
    if (c != '#' && c != ':')
    {
      line = line + c;
    }
    else
    {
      if (c == '#')
      {
        eoc = true;
      }
    }
  }
}

long hexstr2long(String line)
{
  char buf[line.length() + 1];
  line.toCharArray(buf, line.length() + 1);
  return strtol(buf, NULL, 16);
}

static void intHandler()
{
  stepper.run();
}
