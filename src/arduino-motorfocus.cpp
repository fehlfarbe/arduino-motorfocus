#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimerOne.h>
#include "DummySerial.h"

// button and button speed config
#ifndef BTN_IN
#define BTN_IN 7
#endif
#ifndef BTN_OUT
#define BTN_OUT 8
#endif
#ifndef BTN_POTI_SPEED
#define BTN_POTI_SPEED A0
#endif

#define BTN_MIN_SPEED 8
#define BTN_MAX_SPEED 512
#define BTN_ACCEL_FACTOR 1.05f

// driver config
#ifndef PIN_DRIVER_SLEEP
#define PIN_DRIVER_SLEEP 4
#endif

#ifndef PIN_DRIVER_DIR
#define PIN_DRIVER_DIR 3
#endif
#ifndef PIN_DRIVER_STEP
#define PIN_DRIVER_STEP 5
#endif

#ifndef PIN_DRIVER_SLEEP_INVERTED
#define PIN_DRIVER_SLEEP_INVERTED false
#endif

#ifndef PIN_DRIVER_DIR_INVERTED
#define PIN_DRIVER_DIR_INVERTED false
#endif

#ifndef PIN_DRIVER_STEP_INVERTED
#define PIN_DRIVER_STEP_INVERTED false
#endif

// one wire bus / temperature sensor config
#ifndef ONE_WIRE_BUS
#define ONE_WIRE_BUS 2
#endif

#define PERIOD_US 2000


// Maximum motor speed multiplicator in steps per second
#define SPEED_MULTIPLICATOR 30
// Motor acceleration in steps per second per second
#define ACCELERATION 100

#ifndef DEBUG_RX
#define DEBUG_RX A1
#endif
#ifndef DEBUG_TX
#define DEBUG_TX A2
#endif
#ifdef DEBUG
SoftwareSerial debugSerial(DEBUG_RX, DEBUG_TX);
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

float tCoeff = 0;

// button
unsigned long currentPosition = 0;
unsigned long targetPosition = 0;
unsigned long lastSavedPosition = 0;
long millisLastMove = 0;
const long millisDisableDelay = 15000;
bool isEnabled = false;
bool isInManualMode = false;

// read commands
bool eoc = false;
String line;

// function declarations
long hexstr2long(String line);
static void intHandler();
int readButtonSpeed();

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
  stepper.setMaxSpeed(speedFactor * SPEED_MULTIPLICATOR);
  stepper.setAcceleration(ACCELERATION);
#ifdef USE_DRIVER
  debugSerial.println("Using A4988 driver");
  stepper.setEnablePin(PIN_DRIVER_SLEEP);
  stepper.setPinsInverted(PIN_DRIVER_DIR_INVERTED, PIN_DRIVER_STEP_INVERTED, PIN_DRIVER_SLEEP_INVERTED);
  stepper.disableOutputs();
  isEnabled = false;
#endif
  millisLastMove = millis();

  // read saved position from EEPROM
  // EEPROM.put(0, (long)0);
  EEPROM.get(0, currentPosition);
  // prevent negative values if EEPROM is empty
  currentPosition = max(0, currentPosition);

  stepper.setCurrentPosition(currentPosition);
  lastSavedPosition = currentPosition;
  targetPosition = currentPosition;
  debugSerial.print("Last position in EEPROM...");
  debugSerial.println(lastSavedPosition);

  // init temperature sensor
  sensors.begin();
  if(sensors.getDeviceCount()){
    debugSerial.println("Found DS18B20 temperature sensor!");
  }

  // setup buttons
  pinMode(BTN_IN, INPUT_PULLUP);
  pinMode(BTN_OUT, INPUT_PULLUP);
  pinMode(BTN_POTI_SPEED, INPUT_PULLUP);

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
    else
    {
      cmd = line.substring(0, 1);
    }
    if (len > 2)
    {
      param = line.substring(2);
    }

    debugSerial.print("Got line: ");
    debugSerial.println(line);
    debugSerial.print("Command: ");
    debugSerial.print(cmd);
    if (param.length())
    {
      debugSerial.print(" Param: ");
      debugSerial.print(param);
    }
    debugSerial.println();

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
    }
    // firmware value, always return "10"
    if (cmd.equalsIgnoreCase("GV"))
    {
      Serial.print("10#");
    }
    // Initiate a temperature conversion the conversion
    // process takes a maximum of 750 milliseconds. The
    // value returned by the :GT# command will not be
    // valid until the conversion process completes.
    if (cmd.equalsIgnoreCase("C"))
    {
      debugSerial.println("No temperature conversion implemented, yet");
      // Serial.print("10#");
    }
    // get the current motor position
    if (cmd.equalsIgnoreCase("GP"))
    {
      currentPosition = stepper.currentPosition();
      char tempString[6];
      sprintf(tempString, "%04lX", currentPosition);
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
      // pos = stepper.targetPosition();
      char tempString[6];
      sprintf(tempString, "%04lX", targetPosition);
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
      debugSerial.print("Current temperature ");
      debugSerial.println(temperature);
      byte t_int = (byte)temperature << 1;
      t_int += round(temperature - (byte)temperature);
      Serial.print(t_int, HEX);
      Serial.print('#');
    }

    // get the temperature coefficient
    if (cmd.equalsIgnoreCase("GC"))
    {
      // Serial.print("02#");
      Serial.print((byte)tCoeff, HEX);
      Serial.print('#');
    }

    // set the temperature coefficient
    if (cmd.equalsIgnoreCase("SC"))
    {
      // debugSerial.println(param);
      if (param.length() > 4)
      {
        param = param.substring(param.length() - 4);
      }
      debugSerial.println(param);

      if (param.startsWith("F"))
      {
        // debugSerial.println("negative");
        // debugSerial.println(strtol("FFFF", NULL, 16));
        // debugSerial.println(strtol(param.c_str(), NULL, 16));
        tCoeff = ((0xFFFF - strtol(param.c_str(), NULL, 16)) / -2.0f) - 0.5f;
      }
      else
      {
        tCoeff = strtol(param.c_str(), NULL, 16) / 2.0f;
      }
      debugSerial.print("t_coeff: ");
      debugSerial.println(tCoeff);
      // Serial.print("02#");
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
      stepper.setMaxSpeed(speedFactor * SPEED_MULTIPLICATOR);
    }

    // whether half-step is enabled or not, always return "00"
    if (cmd.equalsIgnoreCase("GH"))
    {
      Serial.print("00#");
    }

    // motor is moving - 01 if moving, 00 otherwise
    if (cmd.equalsIgnoreCase("GI"))
    {
      if (stepper.distanceToGo() != 0)
      {
        Serial.print("01#");
        debugSerial.print("Motor is moving, target = ");
        debugSerial.print(targetPosition);
        debugSerial.print(" current = ");
        debugSerial.println(currentPosition);
      }
      else
      {
        Serial.print("00#");
        debugSerial.println("Motor is not moving");
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
      // Serial.println(param);
      debugSerial.print("new target position ");
      debugSerial.print(targetPosition);
      debugSerial.print(" -> ");
      targetPosition = hexstr2long(param);
      debugSerial.println(targetPosition);
      // Serial.println(targetPosition);
      // stepper.moveTo(pos);
    }
    // initiate a move
    if (cmd.equalsIgnoreCase("FG"))
    {
      stepper.enableOutputs();
      isEnabled = true;
      stepper.moveTo(targetPosition);
    }
    // stop a move
    if (cmd.equalsIgnoreCase("FQ"))
    {
      stepper.stop();
    }
  }

  int btn_in = digitalRead(BTN_IN);
  int btn_out = digitalRead(BTN_OUT);

  // move motor if not done
  if (stepper.distanceToGo() != 0)
  {
    debugSerial.print("Has distance to go: ");
    debugSerial.println(stepper.distanceToGo());
    millisLastMove = millis();
    currentPosition = stepper.currentPosition();
  }
  // handle manual buttons
  else if (btn_in == LOW || btn_out == LOW)
  {
    isInManualMode = true;
    // enable motor outputs if motor is idle
    if (!isEnabled)
    {
      stepper.enableOutputs();
      isEnabled = true;
    }
    // save current speed settings
    auto tmpMaxSpeed = stepper.maxSpeed();
    auto tmpSpeed = stepper.speed();
    // run loop while buttons are pressed
    float speed = BTN_MIN_SPEED;
    stepper.setSpeed(speed);
    stepper.setMaxSpeed(readButtonSpeed());
    while (btn_in == LOW || btn_out == LOW)
    {
      debugSerial.print("Speed from poti: ");
      debugSerial.println(speed);
      stepper.setMaxSpeed(readButtonSpeed());
      if (btn_in == LOW && speed < 0)
      {
        speed = BTN_MIN_SPEED;
        debugSerial.println("Button forward ");
      }
      else if (btn_out == LOW && speed > 0)
      {
        // prevent negative values
        speed = -BTN_MIN_SPEED;
        debugSerial.println("Button backward ");
      }
      speed = constrain(speed * BTN_ACCEL_FACTOR, -stepper.maxSpeed(), stepper.maxSpeed());
      debugSerial.print("Set speed to ");
      debugSerial.println(speed);
      stepper.setSpeed(speed); // set speed with direction
      delay(25);

      // read new button values
      btn_in = digitalRead(BTN_IN);
      btn_out = digitalRead(BTN_OUT);
    }

    // // stop and ensure the stepper isn't moving anymore
    stepper.moveTo(stepper.currentPosition());
    while (stepper.distanceToGo())
    {
      debugSerial.println("Stopping motor...");
    }

    // reset speed
    stepper.setMaxSpeed(tmpMaxSpeed);
    stepper.setSpeed(tmpSpeed);

    millisLastMove = millis();
    currentPosition = targetPosition = stepper.currentPosition();
    isInManualMode = false;
  }
  else
  {
    // check if motor was'nt moved for several seconds and save position and disable motors
    if (millis() - millisLastMove > millisDisableDelay)
    {
      // Save current location in EEPROM
      if (lastSavedPosition != currentPosition)
      {
        EEPROM.put(0, currentPosition);
        lastSavedPosition = currentPosition;
        debugSerial.print("Save last position ");
        debugSerial.print(lastSavedPosition);
        debugSerial.println(" to EEPROM");
      }
      if (isEnabled)
      {
        // set motor to sleep state
        stepper.disableOutputs();
        isEnabled = false;
        debugSerial.println("Disabled output pins");
      }
    }
  }
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
  bool moving = false;
  if (isInManualMode)
  {
    stepper.runSpeed();
    moving = abs(stepper.speed()) > 0;
  }
  else
  {
    stepper.run();
    moving = stepper.distanceToGo() != 0;
  }

  digitalWrite(LED_BUILTIN, moving);
}

int readButtonSpeed()
{
  // analogRead 0 -> min speed, 1023 -> max speed so if there is nothing connected
  // the motor will accelerate to max speed
  auto speedVal = map(analogRead(BTN_POTI_SPEED), 50, 900, BTN_MIN_SPEED, BTN_MAX_SPEED);
  return max(min(speedVal, BTN_MAX_SPEED), BTN_MIN_SPEED);
}